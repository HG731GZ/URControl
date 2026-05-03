import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import pyrealsense2 as rs


Resolution = Tuple[int, int]


class CameraError(RuntimeError):
    """RealSense camera initialization or runtime error."""


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    timestamp_ms: float
    frame_number: int
    received_time: float
    timestamp_domain: str


@dataclass(frozen=True)
class StreamConfig:
    width: int
    height: int
    fps: int
    format: object

    @property
    def resolution(self) -> Resolution:
        return self.width, self.height


class Camera:
    """
    Intel RealSense RGB-D camera wrapper for D455, D435i and D405.

    默认选择策略:
    - 未指定分辨率时，优先选择该流支持的最大分辨率。
    - 未指定帧率时，选择该分辨率下支持的最高帧率。

    Parameters
    ----------
    model:
        相机型号，支持 "D455"、"D435i"、"D405"。
    resolution:
        同时应用到 RGB 与 depth 的分辨率，例如 (1280, 720)。
    fps:
        同时应用到 RGB 与 depth 的帧率。
    color_resolution, depth_resolution:
        分别指定 RGB 与 depth 分辨率；优先级高于 resolution。
    color_fps, depth_fps:
        分别指定 RGB 与 depth 帧率；优先级高于 fps。
    align_depth_to_color:
        True 时将深度帧对齐到 RGB 坐标系。
    frame_timeout_ms:
        初始化与后台取帧时的等待超时。
    """

    SUPPORTED_MODELS = {
        "D455": "d455",
        "D435I": "d435i",
        "D405": "d405",
    }
    MULTI_CAMERA_FALLBACK_MODES = (
        (640, 480, 30),
        (848, 480, 30),
        (1280, 720, 15),
        (640, 480, 15),
    )

    def __new__(
            cls,
            model: str,
            resolution: Optional[Resolution] = None,
            fps: Optional[int] = None,
            color_resolution: Optional[Resolution] = None,
            depth_resolution: Optional[Resolution] = None,
            color_fps: Optional[int] = None,
            depth_fps: Optional[int] = None,
            align_depth_to_color: bool = True,
            frame_timeout_ms: int = 5000,
    ):
        instance = super().__new__(cls)
        try:
            instance._initialize(
                model=model,
                resolution=resolution,
                fps=fps,
                color_resolution=color_resolution,
                depth_resolution=depth_resolution,
                color_fps=color_fps,
                depth_fps=depth_fps,
                align_depth_to_color=align_depth_to_color,
                frame_timeout_ms=frame_timeout_ms,
            )
        except Exception as exc:
            try:
                instance.close()
            except Exception:
                pass
            print(f"RealSenseCamera 创建失败: {exc}")
            return None

        return instance

    def __init__(
            self,
            model: str,
            resolution: Optional[Resolution] = None,
            fps: Optional[int] = None,
            color_resolution: Optional[Resolution] = None,
            depth_resolution: Optional[Resolution] = None,
            color_fps: Optional[int] = None,
            depth_fps: Optional[int] = None,
            align_depth_to_color: bool = True,
            frame_timeout_ms: int = 5000,
    ):
        # 初始化在 __new__ 中完成，这样创建失败时 Camera(...) 可以直接返回 None。
        pass

    def _initialize(
            self,
            model: str,
            resolution: Optional[Resolution] = None,
            fps: Optional[int] = None,
            color_resolution: Optional[Resolution] = None,
            depth_resolution: Optional[Resolution] = None,
            color_fps: Optional[int] = None,
            depth_fps: Optional[int] = None,
            align_depth_to_color: bool = True,
            frame_timeout_ms: int = 5000,
    ):
        self.model = self._normalize_model(model)
        self.frame_timeout_ms = frame_timeout_ms
        self.align_depth_to_color = align_depth_to_color

        self._pipeline: Optional[rs.pipeline] = None
        self._profile = None
        self._align = rs.align(rs.stream.color) if align_depth_to_color else None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cond = threading.Condition()
        self._latest_color: Optional[CameraFrame] = None
        self._latest_depth: Optional[CameraFrame] = None
        self._last_error: Optional[BaseException] = None

        self.depth_scale: Optional[float] = None
        self.device_name: Optional[str] = None
        self.serial_number: Optional[str] = None
        self.connected_device_count: int = 0

        color_resolution = color_resolution or resolution
        depth_resolution = depth_resolution or resolution
        color_fps = color_fps if color_fps is not None else fps
        depth_fps = depth_fps if depth_fps is not None else fps
        self._user_specified_stream = any(
            value is not None
            for value in (resolution, fps, color_resolution, depth_resolution, color_fps, depth_fps)
        )

        device = self._find_device()
        self.device_name = device.get_info(rs.camera_info.name)
        self.serial_number = device.get_info(rs.camera_info.serial_number)

        if not self._user_specified_stream and self.connected_device_count > 1:
            fallback = self._select_fallback_config(device)
            if fallback is not None:
                self.color_config, self.depth_config = fallback
                print(
                    f"RealSenseCamera 检测到 {self.connected_device_count} 台相机，"
                    "默认使用多相机安全配置: "
                    f"RGB={self.color_config.width}x{self.color_config.height}@{self.color_config.fps}, "
                    f"depth={self.depth_config.width}x{self.depth_config.height}@{self.depth_config.fps}"
                )
            else:
                self._select_default_stream_configs(device, color_resolution, color_fps, depth_resolution, depth_fps)
        else:
            self._select_default_stream_configs(device, color_resolution, color_fps, depth_resolution, depth_fps)

        try:
            self._start_pipeline()
            self._start_reader()
            self._wait_for_initial_frame()
        except Exception as exc:
            self.close()
            if self._try_start_with_fallback(device, exc):
                return
            raise

    def get_rgb_frame(self, timeout: Optional[float] = None) -> CameraFrame:
        """返回最新 RGB 帧，image 为 HxWx3 uint8 RGB 数组。"""
        return self._get_latest("_latest_color", timeout)

    def get_depth_frame(self, timeout: Optional[float] = None) -> CameraFrame:
        """返回最新深度帧，image 为 uint16 z16 原始深度数组。"""
        return self._get_latest("_latest_depth", timeout)

    def get_depth_frame_meters(self, timeout: Optional[float] = None) -> CameraFrame:
        """返回最新深度帧，image 转换为 float32 米单位。"""
        frame = self.get_depth_frame(timeout)
        if self.depth_scale is None:
            raise CameraError("深度尺度不可用，无法转换为米")

        return CameraFrame(
            image=frame.image.astype(np.float32) * self.depth_scale,
            timestamp_ms=frame.timestamp_ms,
            frame_number=frame.frame_number,
            received_time=frame.received_time,
            timestamp_domain=frame.timestamp_domain,
        )

    def get_frames(self, timeout: Optional[float] = None) -> Tuple[CameraFrame, CameraFrame]:
        """返回最新 RGB 与 depth 帧。"""
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cond:
            self._raise_if_dead()
            while self._latest_color is None or self._latest_depth is None:
                self._raise_if_dead()
                wait_time = self._remaining_time(deadline)
                if wait_time is not None and wait_time <= 0:
                    raise TimeoutError("等待 RGB-D 帧超时")
                self._cond.wait(wait_time)

            self._raise_if_dead()
            return self._latest_color, self._latest_depth

    def close(self) -> None:
        """停止后台线程并关闭 RealSense pipeline。"""
        self._stop_event.set()
        with self._cond:
            self._cond.notify_all()

        thread = self._thread
        if thread is not None and thread.is_alive() and threading.current_thread() is not thread:
            thread.join(timeout=2.0)

        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except RuntimeError:
                pass
            self._pipeline = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _start_pipeline(self) -> None:
        config = rs.config()
        config.enable_device(self.serial_number)
        config.enable_stream(
            rs.stream.color,
            self.color_config.width,
            self.color_config.height,
            self.color_config.format,
            self.color_config.fps,
        )
        config.enable_stream(
            rs.stream.depth,
            self.depth_config.width,
            self.depth_config.height,
            self.depth_config.format,
            self.depth_config.fps,
        )

        self._pipeline = rs.pipeline()
        try:
            self._profile = self._pipeline.start(config)
        except RuntimeError as exc:
            raise CameraError(
                "RealSense pipeline 启动失败: "
                f"RGB={self.color_config.width}x{self.color_config.height}@{self.color_config.fps}, "
                f"depth={self.depth_config.width}x{self.depth_config.height}@{self.depth_config.fps}; {exc}"
            ) from exc

        depth_sensor = self._profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def _try_start_with_fallback(self, device, original_error: BaseException) -> bool:
        if self._user_specified_stream:
            return False

        fallback = self._select_fallback_config(device)
        if fallback is None:
            return False

        color_config, depth_config = fallback
        if color_config == self.color_config and depth_config == self.depth_config:
            return False

        print(
            "RealSenseCamera 默认最高规格启动失败，改用多相机安全配置重试: "
            f"RGB={color_config.width}x{color_config.height}@{color_config.fps}, "
            f"depth={depth_config.width}x{depth_config.height}@{depth_config.fps}; "
            f"原错误: {original_error}"
        )

        self.color_config = color_config
        self.depth_config = depth_config
        self._reset_runtime_state()

        try:
            self._start_pipeline()
            self._start_reader()
            self._wait_for_initial_frame()
            return True
        except Exception:
            self.close()
            return False

    def _select_fallback_config(self, device):
        color_profiles = self._video_profiles(device, rs.stream.color, rs.format.rgb8)
        depth_profiles = self._video_profiles(device, rs.stream.depth, rs.format.z16)

        color_by_mode = {
            (profile.width, profile.height, profile.fps): profile
            for profile in color_profiles
        }
        depth_by_mode = {
            (profile.width, profile.height, profile.fps): profile
            for profile in depth_profiles
        }

        for mode in self.MULTI_CAMERA_FALLBACK_MODES:
            color_config = color_by_mode.get(mode)
            depth_config = depth_by_mode.get(mode)
            if color_config is not None and depth_config is not None:
                return color_config, depth_config

        return None

    def _select_default_stream_configs(
            self,
            device,
            color_resolution: Optional[Resolution],
            color_fps: Optional[int],
            depth_resolution: Optional[Resolution],
            depth_fps: Optional[int],
    ) -> None:
        self.color_config = self._select_stream_config(
            device=device,
            stream=rs.stream.color,
            stream_name="RGB",
            fmt=rs.format.rgb8,
            resolution=color_resolution,
            fps=color_fps,
        )
        self.depth_config = self._select_stream_config(
            device=device,
            stream=rs.stream.depth,
            stream_name="depth",
            fmt=rs.format.z16,
            resolution=depth_resolution,
            fps=depth_fps,
        )

    def _reset_runtime_state(self) -> None:
        self._pipeline = None
        self._profile = None
        self._stop_event = threading.Event()
        self._thread = None
        with self._cond:
            self._latest_color = None
            self._latest_depth = None
            self._last_error = None

    def _start_reader(self) -> None:
        self._thread = threading.Thread(
            target=self._reader_loop,
            name=f"RealSenseCamera-{self.model}-{self.serial_number}",
            daemon=True,
        )
        self._thread.start()

    def _reader_loop(self) -> None:
        assert self._pipeline is not None

        while not self._stop_event.is_set():
            try:
                frames = self._pipeline.wait_for_frames(self.frame_timeout_ms)
                if self._align is not None:
                    frames = self._align.process(frames)

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                now = time.time()
                color = self._make_frame(color_frame, now)
                depth = self._make_frame(depth_frame, now)

                with self._cond:
                    self._latest_color = color
                    self._latest_depth = depth
                    self._last_error = None
                    self._cond.notify_all()
            except Exception as exc:
                if self._stop_event.is_set():
                    break
                with self._cond:
                    self._last_error = exc
                    self._cond.notify_all()
                time.sleep(0.02)

    @staticmethod
    def _make_frame(frame, received_time: float) -> CameraFrame:
        return CameraFrame(
            image=np.asanyarray(frame.get_data()).copy(),
            timestamp_ms=frame.get_timestamp(),
            frame_number=frame.get_frame_number(),
            received_time=received_time,
            timestamp_domain=str(frame.get_frame_timestamp_domain()),
        )

    def _wait_for_initial_frame(self) -> None:
        timeout = self.frame_timeout_ms / 1000.0
        try:
            self.get_frames(timeout=timeout)
        except Exception as exc:
            raise CameraError(f"RealSense 相机已连接，但初始化后未能获取 RGB-D 帧: {exc}") from exc

    def _get_latest(self, attr_name: str, timeout: Optional[float]) -> CameraFrame:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._cond:
            self._raise_if_dead()
            frame = getattr(self, attr_name)
            while frame is None:
                self._raise_if_dead()
                wait_time = self._remaining_time(deadline)
                if wait_time is not None and wait_time <= 0:
                    raise TimeoutError("等待相机帧超时")
                self._cond.wait(wait_time)
                frame = getattr(self, attr_name)
            self._raise_if_dead()
            return frame

    def _raise_if_dead(self) -> None:
        if self._last_error is not None:
            raise CameraError(f"RealSense 后台取帧失败: {self._last_error}") from self._last_error
        if self._thread is not None and not self._thread.is_alive():
            raise CameraError("RealSense 后台取帧线程已停止")

    @staticmethod
    def _remaining_time(deadline: Optional[float]) -> Optional[float]:
        if deadline is None:
            return None
        return max(0.0, deadline - time.monotonic())

    def _find_device(self):
        context = rs.context()
        devices = list(context.devices)
        self.connected_device_count = len(devices)
        if not devices:
            raise CameraError(f"未检测到 RealSense 硬件，创建 {self.model} 相机失败")

        candidates = []
        for device in devices:
            name = device.get_info(rs.camera_info.name)
            if self.model.lower() in name.lower().replace(" ", ""):
                candidates.append(device)

        if not candidates:
            connected = ", ".join(device.get_info(rs.camera_info.name) for device in devices)
            raise CameraError(f"未检测到型号为 {self.model} 的相机，当前连接设备: {connected}")

        return candidates[0]

    @classmethod
    def _normalize_model(cls, model: str) -> str:
        normalized = model.strip().upper().replace("-", "")
        if normalized not in cls.SUPPORTED_MODELS:
            supported = ", ".join(cls.SUPPORTED_MODELS)
            raise ValueError(f"不支持的 RealSense 型号: {model!r}，支持: {supported}")
        return "D435i" if normalized == "D435I" else normalized

    def _select_stream_config(
            self,
            device,
            stream,
            stream_name: str,
            fmt,
            resolution: Optional[Resolution],
            fps: Optional[int],
    ) -> StreamConfig:
        profiles = self._video_profiles(device, stream, fmt)
        if resolution is not None:
            width, height = resolution
            profiles = [profile for profile in profiles if profile.width == width and profile.height == height]
        if fps is not None:
            profiles = [profile for profile in profiles if profile.fps == fps]

        if not profiles:
            supported = self._supported_modes(device, stream, fmt)
            raise CameraError(
                f"{self.model} 不支持 {stream_name} 流配置 "
                f"resolution={resolution}, fps={fps}。可用配置: {supported}"
            )

        return max(profiles, key=lambda profile: (profile.width * profile.height, profile.width, profile.height, profile.fps))

    @staticmethod
    def _video_profiles(device, stream, fmt):
        profiles = []
        seen = set()
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() != stream or profile.format() != fmt:
                    continue
                if not profile.is_video_stream_profile():
                    continue

                video_profile = profile.as_video_stream_profile()
                key = (video_profile.width(), video_profile.height(), profile.fps(), profile.format())
                if key in seen:
                    continue
                seen.add(key)
                profiles.append(
                    StreamConfig(
                        width=video_profile.width(),
                        height=video_profile.height(),
                        fps=profile.fps(),
                        format=profile.format(),
                    )
                )
        return profiles

    def _supported_modes(self, device, stream, fmt) -> str:
        profiles = sorted(
            self._video_profiles(device, stream, fmt),
            key=lambda item: (item.width * item.height, item.fps, item.width, item.height),
            reverse=True,
        )
        return ", ".join(f"{item.width}x{item.height}@{item.fps}" for item in profiles)


RealSenseCamera = Camera
