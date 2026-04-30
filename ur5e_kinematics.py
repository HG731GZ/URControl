from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple, Union

import numpy as np
import pinocchio as pin

from ur5e_model_cache import build_model_from_mjcf, build_models_from_mjcf
from ur_pose_math import matrix_to_rotvec, rotvec_to_matrix


ArrayLike = Union[np.ndarray, list, tuple]


class UR5eKinematics:
    """
    基于本地 MJCF 模型和 Pinocchio 的 UR5e 运动学类。

    返回的 6 维 UR 位姿遵循 Universal Robots 的 p[x, y, z, rx, ry, rz]
    约定：位置单位为米，姿态为旋转向量，单位为弧度。

    雅可比矩阵行顺序使用 Pinocchio 的空间速度向量顺序：
        [vx, vy, vz, wx, wy, wz]
    """

    DEFAULT_MJCF_PATH = (
        Path(__file__).resolve().parent / "universal_robots_ur5e" / "ur5e.xml"
    )

    def __init__(
        self,
        mjcf_path: Optional[Union[str, Path]] = None,
        end_frame_name: str = "attachment_site",
        tcp_offset: Optional[ArrayLike] = None,
        tcp_rotation: Optional[ArrayLike] = None,
        correct_ur_base: bool = True,
        load_geometry: bool = False,
    ) -> None:
        """
        参数
        ----------
        mjcf_path:
            MJCF 文件路径。默认使用 universal_robots_ur5e/ur5e.xml。
        end_frame_name:
            作为法兰/末端的 Pinocchio 坐标系。MJCF 中的 attachment_site
            对应零 TCP 偏置下的 UR 法兰。
        tcp_offset:
            可选 TCP 平移 [x, y, z]，在末端坐标系下表示，单位为米。
        tcp_rotation:
            可选 TCP 姿态，相对于末端坐标系。支持 3 维旋转向量或 3x3 旋转矩阵。
        correct_ur_base:
            是否应用与 ur5e_visualizer.py 相同的基坐标系修正。
        load_geometry:
            是否同时加载 visual/collision geometry。纯运动学计算保持 False，
            可视化器需要 mesh 时才设为 True。
        """
        self.mjcf_path = (
            Path(mjcf_path) if mjcf_path is not None else self.DEFAULT_MJCF_PATH
        )
        self.end_frame_name = end_frame_name

        self.constraint_models = None
        self.collision_model = None
        self.visual_model = None
        if load_geometry:
            (
                self.model,
                self.constraint_models,
                self.collision_model,
                self.visual_model,
            ) = build_models_from_mjcf(self.mjcf_path)
        else:
            self.model = build_model_from_mjcf(self.mjcf_path)

        if correct_ur_base:
            r_z_180 = np.array(
                [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            self.model.jointPlacements[1] = (
                pin.SE3(r_z_180, np.zeros(3)) * self.model.jointPlacements[1]
            )

        if not self.model.existFrame(end_frame_name):
            raise ValueError(f"在 {self.mjcf_path} 中找不到坐标系 '{end_frame_name}'")

        self._base_end_frame_id = self.model.getFrameId(end_frame_name)
        self.tcp_offset = self._as_vector3(tcp_offset, default=np.zeros(3))
        self.tcp_rotation = self._as_rotation_matrix(tcp_rotation)
        self.frame_id = self._build_tcp_frame_if_needed()
        self.data = self.model.createData()

    def forward_kinematics(
        self,
        q: ArrayLike,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        计算正运动学。

        参数
        ----------
        q:
            6 个 UR5e 关节角，单位为弧度。

        返回
        -------
        T:
            末端坐标系相对于基坐标系的 4x4 齐次变换矩阵。
        R:
            3x3 旋转矩阵。
        P:
            3 维平移向量，单位为米。
        ur_pose:
            6 维 UR 位姿 [x, y, z, rx, ry, rz]，其中 rx/ry/rz 为旋转向量。
        """
        q = self._as_q(q)
        pin.framesForwardKinematics(self.model, self.data, q)
        placement = self.data.oMf[self.frame_id]

        T = placement.homogeneous.copy()
        R = placement.rotation.copy()
        P = placement.translation.copy()
        ur_pose = np.concatenate((P, matrix_to_rotvec(R)))
        return T, R, P, ur_pose

    def update_forward_kinematics(
        self,
        q: ArrayLike,
        data: Optional[pin.Data] = None,
        update_frames: bool = False,
    ) -> pin.Data:
        """
        为自行管理 Data 对象的调用方更新 Pinocchio 正运动学。

        可视化器通常需要为多个显示状态维护不同的 Data 对象。这个包装函数保证
        这些调用方使用本类中已经修正过坐标系的同一个模型。
        """
        q = self._as_q(q)
        data = self.data if data is None else data
        if update_frames:
            pin.framesForwardKinematics(self.model, data, q)
        else:
            pin.forwardKinematics(self.model, data, q)
        return data

    def jacobian_base(self, q: ArrayLike) -> np.ndarray:
        """
        计算机器人基坐标系下的 6x6 几何雅可比矩阵。

        返回的线速度部分是末端坐标系原点在基坐标系下表达的速度。
        这里使用 Pinocchio 的 LOCAL_WORLD_ALIGNED，符合基坐标系机器人控制中
        常用的几何雅可比约定。
        """
        return self._jacobian(q, pin.LOCAL_WORLD_ALIGNED)

    def jacobian_end(self, q: ArrayLike) -> np.ndarray:
        """计算在末端坐标系下表达的 6x6 几何雅可比矩阵。"""
        return self._jacobian(q, pin.LOCAL)

    def jacobian(self, q: ArrayLike, reference: str = "base") -> np.ndarray:
        """
        计算末端雅可比矩阵。

        参数
        ----------
        q:
            6 个 UR5e 关节角，单位为弧度。
        reference:
            "base" 表示基坐标系雅可比，"end" 表示末端坐标系雅可比。
        """
        if reference == "base":
            return self.jacobian_base(q)
        if reference in ("end", "tool", "local"):
            return self.jacobian_end(q)
        raise ValueError("reference 必须为 'base' 或 'end'")

    def inverse_kinematics(
        self,
        target: ArrayLike,
        q_ref: ArrayLike,
        max_iterations: int = 200,
        tolerance: float = 1e-6,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
        damping: float = 1e-3,
        step_size: float = 1.0,
        max_step: float = 0.2,
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
        pose_position_unit: str = "m",
        return_info: bool = False,
    ) -> Union[np.ndarray, Tuple[np.ndarray, dict]]:
        """
        使用阻尼最小二乘迭代法求逆运动学。

        阻尼项可以降低奇异位形附近雅可比病态导致的关节增量放大，但它不是
        完整的奇异规避策略；目标不可达或初值不合适时仍可能不收敛。

        参数
        ----------
        target:
            目标位姿，可以是 4x4 齐次变换矩阵 T，也可以是 UR 风格 6 维位姿
            [x, y, z, rx, ry, rz]。
        q_ref:
            6 个参考关节角，单位为弧度。迭代从这个参考关节角开始。
        max_iterations:
            最大迭代次数。
        tolerance:
            当未单独指定位置/姿态容差时，两者共同使用的默认容差。
        position_tolerance:
            位置收敛容差，单位为米。
        orientation_tolerance:
            姿态收敛容差，单位为弧度。
        damping:
            最小二乘求解的阻尼系数。
        step_size:
            每次关节更新量的倍率。
        max_step:
            单次关节更新量的最大欧氏范数，单位为弧度。
        position_weight:
            位置误差行的权重。
        orientation_weight:
            姿态误差行的权重。
        pose_position_unit:
            当 target 为 6 维位姿时，target[0:3] 的位置单位："m"、"mm" 或
            "auto"。UR 官方位姿使用米。
        return_info:
            为 True 时返回 (q, info)，否则只返回 q。

        返回
        -------
        q:
            求解得到的 6 个关节角，单位为弧度。
        info:
            可选的收敛诊断信息字典。
        """
        target_T = self._as_target_transform(target, pose_position_unit)
        target_R = target_T[:3, :3]
        target_P = target_T[:3, 3]

        q = self._as_q(q_ref).copy()
        position_tolerance = tolerance if position_tolerance is None else position_tolerance
        orientation_tolerance = (
            tolerance if orientation_tolerance is None else orientation_tolerance
        )

        row_weights = np.array(
            [position_weight] * 3 + [orientation_weight] * 3,
            dtype=np.float64,
        )
        damping2 = float(damping) * float(damping)
        success = False
        iterations = 0
        position_error_norm = np.inf
        orientation_error_norm = np.inf

        for iterations in range(1, max_iterations + 1):
            _, R, P, _ = self.forward_kinematics(q)
            position_error = target_P - P
            orientation_error = matrix_to_rotvec(target_R @ R.T)
            position_error_norm = float(np.linalg.norm(position_error))
            orientation_error_norm = float(np.linalg.norm(orientation_error))

            if (
                position_error_norm <= position_tolerance
                and orientation_error_norm <= orientation_tolerance
            ):
                success = True
                break

            error = np.concatenate((position_error, orientation_error))
            jacobian = self.jacobian_base(q)
            weighted_error = row_weights * error
            weighted_jacobian = row_weights[:, None] * jacobian

            normal_matrix = (
                weighted_jacobian @ weighted_jacobian.T
                + damping2 * np.eye(6, dtype=np.float64)
            )
            try:
                dq = weighted_jacobian.T @ np.linalg.solve(
                    normal_matrix,
                    weighted_error,
                )
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(weighted_jacobian) @ weighted_error

            step_norm = float(np.linalg.norm(dq))
            if step_norm > max_step:
                dq *= max_step / step_norm

            q = pin.integrate(self.model, q, step_size * dq)

        info = {
            "success": success,
            "iterations": iterations,
            "position_error": position_error_norm,
            "orientation_error": orientation_error_norm,
        }
        if return_info:
            return q, info
        return q

    def ik(self, *args, **kwargs) -> Union[np.ndarray, Tuple[np.ndarray, dict]]:
        """inverse_kinematics() 的短别名。"""
        return self.inverse_kinematics(*args, **kwargs)

    def _jacobian(self, q: ArrayLike, reference_frame: pin.ReferenceFrame) -> np.ndarray:
        q = self._as_q(q)
        return pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.frame_id,
            reference_frame,
        ).copy()

    def _build_tcp_frame_if_needed(self) -> int:
        tcp_placement = pin.SE3(self.tcp_rotation, self.tcp_offset)
        if np.allclose(tcp_placement.homogeneous, np.eye(4)):
            return self._base_end_frame_id

        base_frame = self.model.frames[self._base_end_frame_id]
        tcp_frame_id = self.model.addFrame(
            pin.Frame(
                f"{self.end_frame_name}_tcp",
                base_frame.parentJoint,
                self._base_end_frame_id,
                base_frame.placement * tcp_placement,
                pin.OP_FRAME,
            ),
            False,
        )
        return tcp_frame_id

    @staticmethod
    def _as_target_transform(target: ArrayLike, pose_position_unit: str) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64)
        if target.shape == (4, 4):
            T = target.copy()
            if not np.allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0])):
                raise ValueError("目标变换矩阵的最后一行必须为 [0, 0, 0, 1]")
            return T

        target = target.reshape(-1)
        if target.size != 6:
            raise ValueError("target 必须是 4x4 变换矩阵或 6 维 UR 位姿")

        unit = pose_position_unit.lower()
        position = target[:3].copy()
        if unit in ("m", "meter", "meters"):
            pass
        elif unit in ("mm", "millimeter", "millimeters"):
            position *= 1e-3
        elif unit == "auto":
            if float(np.linalg.norm(position)) > 10.0:
                position *= 1e-3
        else:
            raise ValueError("pose_position_unit 必须为 'm'、'mm' 或 'auto'")

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rotvec_to_matrix(target[3:6])
        T[:3, 3] = position
        return T

    @staticmethod
    def _as_q(q: ArrayLike) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.size != 6:
            raise ValueError(f"期望输入 6 个关节角，实际为 {q.size} 个")
        return q

    @staticmethod
    def _as_vector3(value: Optional[ArrayLike], default: np.ndarray) -> np.ndarray:
        if value is None:
            return default.astype(np.float64)
        value = np.asarray(value, dtype=np.float64).reshape(-1)
        if value.size != 3:
            raise ValueError(f"期望输入 3 维向量，实际形状为 {value.shape}")
        return value

    @staticmethod
    def _as_rotation_matrix(value: Optional[ArrayLike]) -> np.ndarray:
        if value is None:
            return np.eye(3, dtype=np.float64)

        value = np.asarray(value, dtype=np.float64)
        if value.shape == (3,):
            return rotvec_to_matrix(value)
        if value.shape == (3, 3):
            return value
        raise ValueError("tcp_rotation 必须为 3 维向量或 3x3 矩阵")
