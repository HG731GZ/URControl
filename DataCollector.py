import os
import time
import json
import copy
import queue
import shutil
import threading
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np
import cv2

ImageFrame = Tuple[Optional[int], int, float, Optional[np.ndarray], Optional[np.ndarray]]


class DataCollector:
    """RL/IL 数据采集器。

    推送式架构——不持有线程、定时器、相机或机器人客户端。
    由外部调用方（如 PyQt5 定时器回调）以自身节奏推送数据。

    用法示例::

        collector = DataCollector(base_dir="data", write_mode="episode")
        collector.register_numeric("JointAngle", ["q1","q2","q3","q4","q5","q6"])
        collector.register_numeric("TcpPose", ["x","y","z","rx","ry","rz"])
        collector.register_image("camera_d435i", "d435i")
        collector.register_image("camera_video", "d435i", storage="video")

        collector.start_episode()
        collector.push_numeric("JointAngle", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        collector.push_image("camera_d435i", rgb_array, depth_array)
        collector.end_episode()
        collector.close()

    write_mode 可选:
        - "episode": 默认值，所有数据先缓存在内存中，end_episode()/flush()/close() 时写盘。
        - "realtime": 每次 push 后尽快写盘。

    async_write:
        - True: 默认值，写盘任务交给后台线程，避免阻塞调用方线程。
        - False: 调用方线程同步写盘。
    """

    def __init__(
        self,
        base_dir: str = "data",
        session_name: Optional[str] = None,
        write_mode: str = "episode",
        async_write: bool = True,
        extra_metadata: Optional[Dict[str, Any]] = None,
    ):
        if write_mode not in ("episode", "realtime"):
            raise ValueError("write_mode 必须是 'episode' 或 'realtime'")

        self._base_dir = os.path.abspath(base_dir)
        self._session_name = session_name or time.strftime("URCollect_%Y%m%d_%H%M%S")
        self._created_at = time.time()
        self._write_mode = write_mode
        self._async_write = async_write
        self._extra_metadata = copy.deepcopy(extra_metadata or {})

        self._session_dir = os.path.join(self._base_dir, self._session_name)

        # 注册信息
        self._numeric_groups: Dict[str, Dict[str, Any]] = {}
        self._image_groups: Dict[str, Dict[str, Any]] = {}
        self._action_group: Optional[Dict[str, Any]] = None

        # 剧集状态
        self._active_episode: Optional[int] = None
        self._next_episode_id: int = 0
        self._episode_dir: Optional[str] = None
        self._step_counter: int = 0
        self._episodes: List[str] = []

        # 线程安全
        self._lock = threading.Lock()
        self._writer_lock = threading.Lock()
        self._write_queue: "queue.Queue[Optional[Tuple[str, Callable[[], None]]]]" = (
            queue.Queue()
        )
        self._writer_thread: Optional[threading.Thread] = None
        self._pending_write_jobs = 0
        self._video_writers: Dict[Tuple[str, str], cv2.VideoWriter] = {}
        self._video_writer_sizes: Dict[Tuple[str, str], Tuple[int, int]] = {}
        self._scan_existing_episodes_unlocked()

    # ---- 属性 ----

    @property
    def episode_active(self) -> bool:
        return self._active_episode is not None

    @property
    def current_episode(self) -> Optional[int]:
        return self._active_episode

    @property
    def is_writing(self) -> bool:
        with self._writer_lock:
            return self._pending_write_jobs > 0

    # ---- 注册 ----

    def register_numeric(
        self,
        group_name: str,
        column_names: Optional[List[str]] = None,
        fmt: str = "%.6f",
        *,
        units: Optional[List[str]] = None,
    ) -> None:
        with self._lock:
            if group_name in self._numeric_groups:
                raise ValueError(
                    f"数值组 '{group_name}' 已注册。"
                    f"已注册组: {list(self._numeric_groups.keys())}"
                )
            self._numeric_groups[group_name] = {
                "column_names": list(column_names) if column_names is not None else [],
                "fmt": fmt,
                "units": list(units) if units is not None else None,
                "buffer": [],
            }

    def update_metadata(self, values: Dict[str, Any]) -> None:
        """更新会话附加元数据，保存本轮控制参数、坐标约定及运行记录。"""
        with self._lock:
            self._extra_metadata.update(copy.deepcopy(values))
            self._write_metadata_unlocked()

    def register_image(
        self,
        group_name: str,
        camera_id: str,
        storage: str = "png",
        video_codec: str = "MJPG",
        video_fps: float = 30.0,
        video_extension: str = "avi",
    ) -> None:
        storage = storage.lower()
        if storage in ("avi", "compressed_video"):
            storage = "video"
        if storage not in ("png", "video"):
            raise ValueError("storage 必须是 'png' 或 'video'")
        if len(video_codec) != 4:
            raise ValueError("video_codec 必须是 4 个字符，例如 'MJPG'")
        if video_fps <= 0:
            raise ValueError("video_fps 必须大于 0")

        with self._lock:
            if group_name in self._image_groups:
                raise ValueError(
                    f"图像组 '{group_name}' 已注册。"
                    f"已注册组: {list(self._image_groups.keys())}"
                )
            self._image_groups[group_name] = {
                "camera_id": camera_id,
                "storage": storage,
                "video_codec": video_codec,
                "video_fps": float(video_fps),
                "video_extension": video_extension.lstrip("."),
                "next_frame_index": 0,
                "buffer": [],
            }

    def register_action(self, column_names: List[str]) -> None:
        with self._lock:
            if self._action_group is not None:
                raise ValueError("动作组已注册")
            self._action_group = {
                "column_names": list(column_names),
                "fmt": "%.6f",
                "buffer": [],
            }

    # ---- 数据推送 ----

    def push_numeric(
        self,
        group_name: str,
        values: Union[List[float], np.ndarray],
        timestamp: Optional[float] = None,
    ) -> None:
        if self._active_episode is None:
            print("[DataCollector] push_numeric: 没有活跃剧集，请先调用 start_episode()")
            return

        if isinstance(values, np.ndarray):
            values = values.tolist()

        with self._lock:
            if group_name not in self._numeric_groups:
                print(
                    f"[DataCollector] push_numeric: 数值组 '{group_name}' 未注册。"
                    f"已注册组: {list(self._numeric_groups.keys())}"
                )
                return
            info = self._numeric_groups[group_name]
            if not info["column_names"]:
                info["column_names"] = [
                    f"{group_name}_{i}" for i in range(1, len(values) + 1)
                ]
            if len(values) != len(info["column_names"]):
                print(
                    f"[DataCollector] push_numeric: 数值组 '{group_name}' "
                    f"期望 {len(info['column_names'])} 个值，实际收到 {len(values)} 个"
                )
                return
            ts = timestamp if timestamp is not None else time.time()
            info["buffer"].append([ts] + list(values))
            if self._write_mode == "realtime":
                self._flush_numeric_unlocked(group_name, "实时数值")

    def push_image(
        self,
        group_name: str,
        rgb: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """推送图像；时间戳使用来源帧的主机接收时间，省略时使用当前时间。"""
        if self._active_episode is None:
            print("[DataCollector] push_image: 没有活跃剧集，请先调用 start_episode()")
            return

        if rgb is None and depth is None:
            print(f"[DataCollector] push_image('{group_name}'): rgb 和 depth 均为 None，跳过")
            return

        with self._lock:
            if group_name not in self._image_groups:
                print(
                    f"[DataCollector] push_image: 图像组 '{group_name}' 未注册。"
                    f"已注册组: {list(self._image_groups.keys())}"
                )
                return
            img_dir = os.path.join(
                self._episode_dir, "images", group_name  # type: ignore[arg-type]
            )
            step = self._step_counter
            timestamp = time.time() if timestamp is None else timestamp
            info = self._image_groups[group_name]
            frame_index = None
            if info["storage"] == "video" and rgb is not None:
                frame_index = info["next_frame_index"]
                info["next_frame_index"] += 1
            frame: ImageFrame = (
                frame_index,
                step,
                timestamp,
                np.array(rgb, copy=True) if rgb is not None else None,
                np.array(depth, copy=True) if depth is not None else None,
            )

            if self._write_mode == "realtime":
                self._write_image_unlocked(
                    self._episode_dir,  # type: ignore[arg-type]
                    img_dir,
                    group_name,
                    self._image_write_options_unlocked(group_name),
                    frame,
                    close_video=False,
                    description=f"实时图像 {group_name} step_{step:06d}",
                )
                return

            self._image_groups[group_name]["buffer"].append(frame)

    def push_action(
        self,
        values: Union[List[float], np.ndarray],
        timestamp: Optional[float] = None,
    ) -> None:
        if self._active_episode is None:
            print("[DataCollector] push_action: 没有活跃剧集，请先调用 start_episode()")
            return
        if self._action_group is None:
            print("[DataCollector] push_action: 动作组未注册，请先调用 register_action()")
            return

        if isinstance(values, np.ndarray):
            values = values.tolist()

        with self._lock:
            if len(values) != len(self._action_group["column_names"]):
                print(
                    f"[DataCollector] push_action: 动作组期望 "
                    f"{len(self._action_group['column_names'])} 个值，实际收到 {len(values)} 个"
                )
                return
            ts = timestamp if timestamp is not None else time.time()
            self._action_group["buffer"].append([ts] + list(values))
            if self._write_mode == "realtime":
                self._flush_action_unlocked("实时动作")

    # ---- 剧集管理 ----

    def start_episode(self, episode_id: Optional[int] = None) -> int:
        with self._lock:
            if self._active_episode is not None:
                self._end_episode_unlocked()

            if episode_id is not None:
                self._active_episode = episode_id
                self._next_episode_id = max(self._next_episode_id, episode_id + 1)
            else:
                self._active_episode = self._next_episode_id
                self._next_episode_id += 1

            self._step_counter = 0
            self._make_episode_dirs_unlocked()
            return self._active_episode

    def step(self) -> int:
        """推进一个逻辑步，返回推进后的 step 编号。

        同一逻辑步内多次调用 push_image（多相机）共享同一步号，
        调用 step() 后才进入下一步。
        """
        with self._lock:
            self._step_counter += 1
            return self._step_counter

    def end_episode(self) -> None:
        with self._lock:
            self._end_episode_unlocked()

    def flush(self) -> None:
        with self._lock:
            if self._active_episode is None:
                return
            for name in self._numeric_groups:
                self._flush_numeric_unlocked(name, "手动刷新数值")
            for name in self._image_groups:
                self._flush_image_unlocked(name, "手动刷新图像")
            if self._action_group is not None:
                self._flush_action_unlocked("手动刷新动作")

    def wait_for_writes(self) -> None:
        self._write_queue.join()

    # ---- 生命周期 ----

    def close(self, wait_for_writes: bool = True) -> None:
        with self._lock:
            if self._active_episode is not None:
                self._end_episode_unlocked()

        if wait_for_writes:
            self.wait_for_writes()

        with self._lock:
            if wait_for_writes:
                self._remove_empty_episodes_unlocked()
            self._write_metadata_unlocked()

        if wait_for_writes:
            self._stop_writer()

    def resume(self) -> None:
        """手动重新扫描已存在的 episode 目录。__init__ 已自动调用，一般无需再调。"""
        with self._lock:
            self._scan_existing_episodes_unlocked()

    # ---- 内部方法 ----

    def _scan_existing_episodes_unlocked(self) -> None:
        """扫描 session_dir 下已有的 episode_xxx 目录，恢复计数和 episodes 列表。"""
        if not os.path.isdir(self._session_dir):
            return
        max_ep = -1
        for name in os.listdir(self._session_dir):
            if name.startswith("episode_") and os.path.isdir(
                os.path.join(self._session_dir, name)
            ):
                try:
                    ep_id = int(name.split("_")[1])
                    max_ep = max(max_ep, ep_id)
                except (ValueError, IndexError):
                    continue
        if max_ep >= 0:
            self._next_episode_id = max_ep + 1
            for i in range(max_ep + 1):
                ep_name = f"episode_{i:03d}"
                ep_dir = os.path.join(self._session_dir, ep_name)
                if os.path.isdir(ep_dir) and ep_name not in self._episodes:
                    self._episodes.append(ep_name)

    def _make_episode_dirs_unlocked(self) -> None:
        ep_name = f"episode_{self._active_episode:03d}"
        self._episode_dir = os.path.join(self._session_dir, ep_name)

        os.makedirs(os.path.join(self._episode_dir, "numeric"), exist_ok=True)
        for img_name in self._image_groups:
            os.makedirs(
                os.path.join(self._episode_dir, "images", img_name), exist_ok=True
            )
            self._image_groups[img_name]["next_frame_index"] = 0
        if any(info["storage"] == "video" for info in self._image_groups.values()):
            os.makedirs(os.path.join(self._episode_dir, "videos"), exist_ok=True)
        if self._action_group is not None:
            os.makedirs(os.path.join(self._episode_dir, "action"), exist_ok=True)

    def _end_episode_unlocked(self) -> None:
        if self._active_episode is None:
            return

        for name in self._numeric_groups:
            self._flush_numeric_unlocked(name, "剧集结束数值")

        for name in self._image_groups:
            self._flush_image_unlocked(name, "剧集结束图像", close_video=True)

        if self._action_group is not None:
            self._flush_action_unlocked("剧集结束动作")

        ep_name = f"episode_{self._active_episode:03d}"
        if ep_name not in self._episodes:
            self._episodes.append(ep_name)

        self._active_episode = None
        self._episode_dir = None
        self._step_counter = 0

    def _flush_numeric_unlocked(self, group_name: str, reason: str) -> None:
        info = self._numeric_groups[group_name]
        buffer = info["buffer"]
        if not buffer:
            return

        filepath = os.path.join(
            self._episode_dir, "numeric", f"{group_name}.csv"  # type: ignore[arg-type]
        )
        rows = np.array(buffer, dtype=np.float64)
        column_names = list(info["column_names"])
        fmt = info["fmt"]
        buffer.clear()

        self._write_or_enqueue(
            f"{reason} {group_name}",
            lambda: self._write_numeric_rows(filepath, column_names, fmt, rows),
        )

    def _write_image_unlocked(
        self,
        episode_dir: str,
        img_dir: str,
        group_name: str,
        options: Dict[str, Any],
        frame: ImageFrame,
        close_video: bool,
        description: str,
    ) -> None:
        self._write_or_enqueue(
            description,
            lambda: self._write_image_batch(
                episode_dir, img_dir, group_name, options, [frame], close_video
            ),
        )

    def _flush_image_unlocked(
        self,
        group_name: str,
        reason: str,
        close_video: bool = False,
    ) -> None:
        info = self._image_groups[group_name]
        buffer = info["buffer"]
        if not buffer and (not close_video or info["storage"] != "video"):
            return

        episode_dir = self._episode_dir  # type: ignore[assignment]
        img_dir = os.path.join(
            self._episode_dir, "images", group_name  # type: ignore[arg-type]
        )
        images = list(buffer)
        options = self._image_write_options_unlocked(group_name)
        buffer.clear()

        self._write_or_enqueue(
            f"{reason} {group_name}",
            lambda: self._write_image_batch(
                episode_dir, img_dir, group_name, options, images, close_video
            ),
        )

    def _flush_action_unlocked(self, reason: str) -> None:
        if self._action_group is None:
            return
        buffer = self._action_group["buffer"]
        if not buffer:
            return

        filepath = os.path.join(
            self._episode_dir, "action", "action.csv"  # type: ignore[arg-type]
        )
        rows = np.array(buffer, dtype=np.float64)
        column_names = list(self._action_group["column_names"])
        fmt = self._action_group["fmt"]
        buffer.clear()

        self._write_or_enqueue(
            f"{reason} action",
            lambda: self._write_numeric_rows(filepath, column_names, fmt, rows),
        )

    def _write_or_enqueue(self, description: str, job: Callable[[], None]) -> None:
        if not self._async_write:
            job()
            print(f"[DataCollector] 写盘完成: {description}")
            return

        self._ensure_writer_started()
        with self._writer_lock:
            self._pending_write_jobs += 1
        self._write_queue.put((description, job))

    def _ensure_writer_started(self) -> None:
        with self._writer_lock:
            if self._writer_thread is not None and self._writer_thread.is_alive():
                return
            self._writer_thread = threading.Thread(
                target=self._writer_loop,
                name="DataCollectorWriter",
                daemon=True,
            )
            self._writer_thread.start()

    def _writer_loop(self) -> None:
        while True:
            item = self._write_queue.get()
            if item is None:
                self._write_queue.task_done()
                return
            description, job = item
            try:
                job()
                print(f"[DataCollector] 写盘完成: {description}")
            except Exception as exc:  # noqa: BLE001
                print(f"[DataCollector] 写盘失败: {description}: {exc}")
            finally:
                with self._writer_lock:
                    self._pending_write_jobs -= 1
                self._write_queue.task_done()

    def _stop_writer(self) -> None:
        with self._writer_lock:
            thread = self._writer_thread
            if thread is None or not thread.is_alive():
                self._writer_thread = None
                return
        self._write_queue.put(None)
        thread.join()
        with self._writer_lock:
            self._writer_thread = None

    def _write_numeric_rows(
        self,
        filepath: str,
        column_names: List[str],
        fmt: str,
        rows: np.ndarray,
    ) -> None:
        file_exists = os.path.isfile(filepath)
        with open(filepath, "a") as f:
            if not file_exists:
                header = "timestamp," + ",".join(column_names)
                f.write(header + "\n")
            np.savetxt(f, rows, delimiter=",", fmt=fmt)

    def _write_image_batch(
        self,
        episode_dir: str,
        img_dir: str,
        group_name: str,
        options: Dict[str, Any],
        images: List[ImageFrame],
        close_video: bool,
    ) -> None:
        if options["storage"] == "video":
            if images:
                self._write_video_frames(
                    episode_dir, img_dir, group_name, options, images
                )
            if close_video:
                self._close_video_writer(episode_dir, group_name)
            return

        for _frame_index, step, _timestamp, rgb, depth in images:
            self._write_image_files(img_dir, step, rgb, depth)

    def _write_image_files(
        self,
        img_dir: str,
        step: int,
        rgb: Optional[np.ndarray],
        depth: Optional[np.ndarray],
    ) -> None:
        step_str = f"step_{step:06d}"
        if rgb is not None:
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(img_dir, f"{step_str}_rgb.png"), bgr)
        if depth is not None:
            np.save(os.path.join(img_dir, f"{step_str}_depth.npy"), depth)

    def _write_video_frames(
        self,
        episode_dir: str,
        img_dir: str,
        group_name: str,
        options: Dict[str, Any],
        images: List[ImageFrame],
    ) -> None:
        videos_dir = os.path.join(episode_dir, "videos")
        os.makedirs(videos_dir, exist_ok=True)
        os.makedirs(img_dir, exist_ok=True)
        csv_path = os.path.join(videos_dir, f"{group_name}_frames.csv")
        csv_exists = os.path.isfile(csv_path)

        with open(csv_path, "a") as index_file:
            if not csv_exists:
                index_file.write("frame,step,timestamp\n")
            for frame_index, step, timestamp, rgb, depth in images:
                if rgb is not None:
                    self._write_video_frame(
                        episode_dir, group_name, options, frame_index, step, timestamp, rgb, index_file
                    )
                if depth is not None:
                    step_str = f"step_{step:06d}"
                    np.save(os.path.join(img_dir, f"{step_str}_depth.npy"), depth)

    def _write_video_frame(
        self,
        episode_dir: str,
        group_name: str,
        options: Dict[str, Any],
        frame_index: Optional[int],
        step: int,
        timestamp: float,
        rgb: np.ndarray,
        index_file: Any,
    ) -> None:
        if frame_index is None:
            raise ValueError(f"视频图像组 '{group_name}' 缺少 frame_index")

        key = (episode_dir, group_name)
        height, width = rgb.shape[:2]
        size = (width, height)
        writer = self._video_writers.get(key)

        if writer is None:
            video_path = os.path.join(
                episode_dir,
                "videos",
                f"{group_name}.{options['video_extension']}",
            )
            fourcc = cv2.VideoWriter_fourcc(*options["video_codec"])
            writer = cv2.VideoWriter(video_path, fourcc, options["video_fps"], size)
            if not writer.isOpened():
                raise RuntimeError(
                    f"无法打开视频写入器: {video_path}, codec={options['video_codec']}"
                )
            self._video_writers[key] = writer
            self._video_writer_sizes[key] = size
        elif self._video_writer_sizes[key] != size:
            raise ValueError(
                f"视频图像组 '{group_name}' 的帧尺寸变化: "
                f"{self._video_writer_sizes[key]} -> {size}"
            )

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
        index_file.write(f"{frame_index},{step},{timestamp:.9f}\n")

    def _close_video_writer(self, episode_dir: str, group_name: str) -> None:
        key = (episode_dir, group_name)
        writer = self._video_writers.pop(key, None)
        self._video_writer_sizes.pop(key, None)
        if writer is not None:
            writer.release()

    def _image_write_options_unlocked(self, group_name: str) -> Dict[str, Any]:
        info = self._image_groups[group_name]
        return {
            "storage": info["storage"],
            "video_codec": info["video_codec"],
            "video_fps": info["video_fps"],
            "video_extension": info["video_extension"],
        }

    def _write_metadata_unlocked(self) -> None:
        os.makedirs(self._session_dir, exist_ok=True)

        metadata = {
            "session_name": self._session_name,
            "created_at": self._created_at,
            "base_dir": self._base_dir,
            "write_mode": self._write_mode,
            "async_write": self._async_write,
            "numeric_groups": {
                name: info["column_names"]
                for name, info in self._numeric_groups.items()
            },
            "numeric_group_units": {
                name: info["units"] for name, info in self._numeric_groups.items()
            },
            "timestamp_unit": "Unix s",
            "extra_metadata": self._extra_metadata,
            "image_groups": {
                name: info["camera_id"] for name, info in self._image_groups.items()
            },
            "image_group_options": {
                name: {
                    "storage": info["storage"],
                    "video_codec": info["video_codec"],
                    "video_fps": info["video_fps"],
                    "video_extension": info["video_extension"],
                }
                for name, info in self._image_groups.items()
            },
            "action_group": (
                self._action_group["column_names"]
                if self._action_group is not None
                else None
            ),
            "episodes": self._episodes,
        }

        filepath = os.path.join(self._session_dir, "metadata.json")
        with open(filepath, "w") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)

    def _remove_empty_episodes_unlocked(self) -> None:
        if not os.path.isdir(self._session_dir):
            return
        for name in os.listdir(self._session_dir):
            if not name.startswith("episode_"):
                continue
            ep_dir = os.path.join(self._session_dir, name)
            if not os.path.isdir(ep_dir):
                continue

            numeric_dir = os.path.join(ep_dir, "numeric")
            images_dir = os.path.join(ep_dir, "images")
            videos_dir = os.path.join(ep_dir, "videos")
            action_dir = os.path.join(ep_dir, "action")

            has_data = False
            for d in [numeric_dir, images_dir, videos_dir, action_dir]:
                if not os.path.isdir(d):
                    continue
                for _root, _dirs, files in os.walk(d):  # noqa: B007
                    if files:
                        has_data = True
                        break
                if has_data:
                    break

            if not has_data:
                shutil.rmtree(ep_dir)
                if name in self._episodes:
                    self._episodes.remove(name)
