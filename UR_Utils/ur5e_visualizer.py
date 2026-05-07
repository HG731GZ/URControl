from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin

from UR_Utils.ur5e_gl_renderer import RobotGLWidget
from UR_Utils.ur5e_kinematics import UR5eKinematics


class UR5eDualVisualizer:
    """UR5e 实际/虚拟双机械臂可视化封装。"""

    def __init__(
        self,
        mjcf_path: str,
        tool_offset: Optional[np.ndarray] = np.array([0, 0.1, 0]),
        tool_rotation: Optional[np.ndarray] = None,
        zmq_url: Optional[str] = None,
        lazy_virtual: bool = True,
        camera_azimuth_deg: float = 45.0,
        camera_elevation_deg: float = 30.0,
        camera_distance: float = 2.0,
        camera_target: Sequence[float] = (0.0, 0.0, 0.3),
        camera_fov_deg: float = 45.0,
        show_camera_info: bool = True,
    ):
        """
        参数
        ----------
        mjcf_path:
            UR5e MJCF XML 文件路径。
        tool_offset:
            TCP 相对 wrist_3_link 的平移偏置，单位为米。
        tool_rotation:
            TCP 相对 wrist_3_link 的 XYZ 欧拉角偏置，单位为弧度。
        zmq_url:
            兼容旧 Meshcat 接口的保留参数，原生 OpenGL 渲染不再使用。
        lazy_virtual:
            兼容旧 Meshcat 接口的保留参数，原生 OpenGL 渲染不再使用。
        camera_azimuth_deg:
            初始方位角，单位为度。左键横向拖动会调整这个参数。
        camera_elevation_deg:
            初始俯仰角，单位为度。左键纵向拖动会调整这个参数。
        camera_distance:
            初始相机距离目标点的距离，单位为米。滚轮会调整这个参数。
        camera_target:
            初始观察目标点 [x, y, z]，单位为米。中键拖动会调整这个参数。
        camera_fov_deg:
            初始透视视场角，单位为度。
        show_camera_info:
            是否在可视化窗口左上角显示相机参数。
        """
        _ = zmq_url, lazy_virtual
        self.mjcf_path = str(Path(mjcf_path).resolve())

        if tool_offset is None:
            self.tool_offset = np.zeros(3)
        else:
            self.tool_offset = np.asarray(tool_offset, dtype=np.float64)

        if tool_rotation is None:
            self.tool_rotation = np.eye(3)
        else:
            r = np.asarray(tool_rotation, dtype=np.float64)
            self.tool_rotation = pin.rpy.rpyToMatrix(r[0], r[1], r[2])

        # wrist_3_link 的 Y/Z 轴和 UR 工具坐标系约定不同，这里补偿 -90 度 X 轴旋转。
        r_x_neg90 = np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
            dtype=np.float64,
        )

        self.kinematics = UR5eKinematics(
            mjcf_path=self.mjcf_path,
            end_frame_name="wrist_3_link",
            tcp_offset=self.tool_offset,
            tcp_rotation=self.tool_rotation @ r_x_neg90,
            load_geometry=True,
        )

        self.model = self.kinematics.model
        self.visual_model_actual = self.kinematics.visual_model
        self.visual_model_virtual = self.visual_model_actual.clone()

        self.data_actual = self.model.createData()
        self.data_virtual = self.model.createData()
        self.actual_geo_data = self.visual_model_actual.createData()
        self.virtual_geo_data = self.visual_model_virtual.createData()

        self._gl_widget = RobotGLWidget(
            self.mjcf_path,
            visual_model=self.visual_model_actual,
            camera_azimuth_deg=camera_azimuth_deg,
            camera_elevation_deg=camera_elevation_deg,
            camera_distance=camera_distance,
            camera_target=camera_target,
            camera_fov_deg=camera_fov_deg,
            show_camera_info=show_camera_info,
        )

        # 先显示零位姿态，UR 尚未连接时可视化窗口也有稳定初始画面。
        q_zero = np.zeros(6, dtype=np.float64)
        self.update_actual(q_zero)
        self.update_virtual(q_zero)
        print("UR5e OpenGL 双机械臂可视化已初始化。")

    @property
    def widget(self) -> RobotGLWidget:
        """返回可直接加入 Qt 布局的 OpenGL 控件。"""
        return self._gl_widget

    def update_actual(self, q: np.ndarray) -> None:
        """
        更新实际机械臂关节角。

        参数
        ----------
        q:
            [q1..q6, gripper]，这里只使用前 6 个关节角，单位为弧度。
        """
        q6 = np.asarray(q[:6], dtype=np.float64)
        transforms, ee_tf = self._compute_geometry_transforms(
            q6,
            self.data_actual,
            self.visual_model_actual,
            self.actual_geo_data,
        )
        self._gl_widget.update_actual_transforms(transforms, ee_tf)

    def update_virtual(self, q: np.ndarray) -> None:
        """
        更新虚拟机械臂关节角。

        参数
        ----------
        q:
            [q1..q6, gripper]，这里只使用前 6 个关节角，单位为弧度。
        """
        q6 = np.asarray(q[:6], dtype=np.float64)
        transforms, ee_tf = self._compute_geometry_transforms(
            q6,
            self.data_virtual,
            self.visual_model_virtual,
            self.virtual_geo_data,
        )
        self._gl_widget.update_virtual_transforms(transforms, ee_tf)

    def _compute_geometry_transforms(
        self,
        q6: np.ndarray,
        data: pin.Data,
        visual_model: pin.GeometryModel,
        visual_data: pin.GeometryData,
    ):
        self.kinematics.update_forward_kinematics(q6, data, update_frames=True)
        pin.updateGeometryPlacements(
            self.model,
            data,
            visual_model,
            visual_data,
        )
        transforms = [placement.homogeneous.copy() for placement in visual_data.oMg]
        ee_tf = data.oMf[self.kinematics.frame_id].homogeneous.copy()
        return transforms, ee_tf

    def close(self) -> None:
        """释放 OpenGL 控件资源。"""
        if self._gl_widget is not None:
            self._gl_widget.close()
            self._gl_widget = None
