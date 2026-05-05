import os
import time
import json
import shutil
import threading
from typing import Any, Dict, List, Optional, Union

import numpy as np
import cv2


class DataCollector:
    """RL/IL 数据采集器。

    推送式架构——不持有线程、定时器、相机或机器人客户端。
    由外部调用方（如 PyQt5 定时器回调）以自身节奏推送数据。

    用法示例::

        collector = DataCollector(base_dir="data")
        collector.register_numeric("JointAngle", ["q1","q2","q3","q4","q5","q6"])
        collector.register_numeric("TcpPose", ["x","y","z","rx","ry","rz"])
        collector.register_image("camera_d435i", "d435i")

        collector.start_episode()
        collector.push_numeric("JointAngle", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        collector.push_image("camera_d435i", rgb_array, depth_array)
        collector.end_episode()
        collector.close()
    """

    def __init__(
        self,
        base_dir: str = "data",
        session_name: Optional[str] = None,
    ):
        self._base_dir = os.path.abspath(base_dir)
        self._session_name = session_name or time.strftime("URCollect_%Y%m%d_%H%M%S")
        self._created_at = time.time()

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

    # ---- 属性 ----

    @property
    def episode_active(self) -> bool:
        return self._active_episode is not None

    @property
    def current_episode(self) -> Optional[int]:
        return self._active_episode

    # ---- 注册 ----

    def register_numeric(
        self,
        group_name: str,
        column_names: List[str],
        fmt: str = "%.6f",
    ) -> None:
        with self._lock:
            if group_name in self._numeric_groups:
                raise ValueError(
                    f"数值组 '{group_name}' 已注册。"
                    f"已注册组: {list(self._numeric_groups.keys())}"
                )
            self._numeric_groups[group_name] = {
                "column_names": list(column_names),
                "fmt": fmt,
                "buffer": [],
            }

    def register_image(self, group_name: str, camera_id: str) -> None:
        with self._lock:
            if group_name in self._image_groups:
                raise ValueError(
                    f"图像组 '{group_name}' 已注册。"
                    f"已注册组: {list(self._image_groups.keys())}"
                )
            self._image_groups[group_name] = {
                "camera_id": camera_id,
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
            if len(values) != len(info["column_names"]):
                print(
                    f"[DataCollector] push_numeric: 数值组 '{group_name}' "
                    f"期望 {len(info['column_names'])} 个值，实际收到 {len(values)} 个"
                )
                return
            ts = timestamp if timestamp is not None else time.time()
            info["buffer"].append([ts] + list(values))

    def push_image(
        self,
        group_name: str,
        rgb: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
    ) -> None:
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
            step_str = f"step_{self._step_counter:06d}"

            if rgb is not None:
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imwrite(os.path.join(img_dir, f"{step_str}_rgb.png"), bgr)
            if depth is not None:
                np.save(os.path.join(img_dir, f"{step_str}_depth.npy"), depth)

            self._step_counter += 1

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

    def end_episode(self) -> None:
        with self._lock:
            self._end_episode_unlocked()

    def flush(self) -> None:
        with self._lock:
            if self._active_episode is None:
                return
            for name in self._numeric_groups:
                self._flush_numeric_unlocked(name)
            if self._action_group is not None:
                self._flush_action_unlocked()

    # ---- 生命周期 ----

    def close(self) -> None:
        with self._lock:
            if self._active_episode is not None:
                self._end_episode_unlocked()
            self._remove_empty_episodes_unlocked()
            self._write_metadata_unlocked()

    def resume(self) -> None:
        with self._lock:
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

    # ---- 内部方法 ----

    def _make_episode_dirs_unlocked(self) -> None:
        ep_name = f"episode_{self._active_episode:03d}"
        self._episode_dir = os.path.join(self._session_dir, ep_name)

        # 如果目录已存在（resume 场景），先清理旧数据
        if os.path.isdir(self._episode_dir):
            numeric_dir = os.path.join(self._episode_dir, "numeric")
            if os.path.isdir(numeric_dir):
                shutil.rmtree(numeric_dir)
            images_dir = os.path.join(self._episode_dir, "images")
            if os.path.isdir(images_dir):
                shutil.rmtree(images_dir)

        os.makedirs(os.path.join(self._episode_dir, "numeric"), exist_ok=True)
        for img_name in self._image_groups:
            os.makedirs(
                os.path.join(self._episode_dir, "images", img_name), exist_ok=True
            )
        if self._action_group is not None:
            os.makedirs(os.path.join(self._episode_dir, "action"), exist_ok=True)

    def _end_episode_unlocked(self) -> None:
        if self._active_episode is None:
            return

        for name in self._numeric_groups:
            self._flush_numeric_unlocked(name)

        if self._action_group is not None:
            self._flush_action_unlocked()

        ep_name = f"episode_{self._active_episode:03d}"
        if ep_name not in self._episodes:
            self._episodes.append(ep_name)

        self._active_episode = None
        self._episode_dir = None
        self._step_counter = 0

    def _flush_numeric_unlocked(self, group_name: str) -> None:
        info = self._numeric_groups[group_name]
        buffer = info["buffer"]
        if not buffer:
            return

        filepath = os.path.join(
            self._episode_dir, "numeric", f"{group_name}.csv"  # type: ignore[arg-type]
        )
        file_exists = os.path.isfile(filepath)

        rows = np.array(buffer, dtype=np.float64)

        with open(filepath, "a") as f:
            if not file_exists:
                header = "timestamp," + ",".join(info["column_names"])
                f.write(header + "\n")
            np.savetxt(f, rows, delimiter=",", fmt=info["fmt"])

        buffer.clear()

    def _flush_action_unlocked(self) -> None:
        if self._action_group is None:
            return
        buffer = self._action_group["buffer"]
        if not buffer:
            return

        filepath = os.path.join(
            self._episode_dir, "action", "action.csv"  # type: ignore[arg-type]
        )
        file_exists = os.path.isfile(filepath)

        rows = np.array(buffer, dtype=np.float64)

        with open(filepath, "a") as f:
            if not file_exists:
                header = "timestamp," + ",".join(self._action_group["column_names"])
                f.write(header + "\n")
            np.savetxt(f, rows, delimiter=",", fmt=self._action_group["fmt"])

        buffer.clear()

    def _write_metadata_unlocked(self) -> None:
        os.makedirs(self._session_dir, exist_ok=True)

        metadata = {
            "session_name": self._session_name,
            "created_at": self._created_at,
            "base_dir": self._base_dir,
            "numeric_groups": {
                name: info["column_names"]
                for name, info in self._numeric_groups.items()
            },
            "image_groups": {
                name: info["camera_id"] for name, info in self._image_groups.items()
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
            action_dir = os.path.join(ep_dir, "action")

            has_data = False
            for d in [numeric_dir, images_dir, action_dir]:
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


# ---- 独立测试 ----

def _test():
    import tempfile

    tmpdir = tempfile.mkdtemp(prefix="datacollector_test_")
    print(f"测试目录: {tmpdir}")

    collector = DataCollector(base_dir=tmpdir, session_name="test_session")

    # 注册
    collector.register_numeric("JointAngle", ["q1", "q2", "q3", "q4", "q5", "q6"])
    collector.register_numeric("TcpPose", ["x", "y", "z", "rx", "ry", "rz"])
    collector.register_image("camera_d435i", "d435i")
    collector.register_action(["a1", "a2", "a3", "a4", "a5", "a6"])

    # 剧集 0
    collector.start_episode()
    assert collector.episode_active
    assert collector.current_episode == 0

    for i in range(10):
        collector.push_numeric(
            "JointAngle", [0.1 * i] * 6, timestamp=1000.0 + i * 0.01
        )
        collector.push_numeric("TcpPose", [0.01 * i] * 6, timestamp=1000.0 + i * 0.01)
        collector.push_action([0.0] * 6, timestamp=1000.0 + i * 0.01)
        rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        depth = np.random.randint(0, 65535, (480, 640), dtype=np.uint16)
        collector.push_image("camera_d435i", rgb=rgb, depth=depth)

    collector.end_episode()
    assert not collector.episode_active

    # 剧集 1
    collector.start_episode()
    assert collector.current_episode == 1
    for i in range(5):
        collector.push_numeric("JointAngle", [0.2 * i] * 6)
    collector.end_episode()

    collector.close()

    # 验证文件结构
    session_dir = os.path.join(tmpdir, "test_session")
    assert os.path.isdir(session_dir)
    assert os.path.isfile(os.path.join(session_dir, "metadata.json"))

    ep0 = os.path.join(session_dir, "episode_000")
    assert os.path.isfile(os.path.join(ep0, "numeric", "JointAngle.csv"))
    assert os.path.isfile(os.path.join(ep0, "numeric", "TcpPose.csv"))
    assert os.path.isfile(os.path.join(ep0, "action", "action.csv"))

    data = np.loadtxt(
        os.path.join(ep0, "numeric", "JointAngle.csv"), delimiter=",", skiprows=1
    )
    assert data.shape == (10, 7), f"期望 (10, 7)，实际 {data.shape}"
    assert np.allclose(data[0, 0], 1000.0)

    img_dir = os.path.join(ep0, "images", "camera_d435i")
    pngs = sorted(f for f in os.listdir(img_dir) if f.endswith("_rgb.png"))
    npys = sorted(f for f in os.listdir(img_dir) if f.endswith("_depth.npy"))
    assert len(pngs) == 10
    assert len(npys) == 10

    ep1 = os.path.join(session_dir, "episode_001")
    data1 = np.loadtxt(
        os.path.join(ep1, "numeric", "JointAngle.csv"), delimiter=",", skiprows=1
    )
    assert data1.shape == (5, 7)

    shutil.rmtree(tmpdir)
    print("全部测试通过!")


if __name__ == "__main__":
    _test()
