from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple, Union

import numpy as np
import pinocchio as pin

from ur_pose_math import matrix_to_rotvec, rotvec_to_matrix


ArrayLike = Union[np.ndarray, list, tuple]


class UR5eKinematics:
    """
    UR5e kinematics based on the local MJCF model and Pinocchio.

    The returned 6D UR pose follows Universal Robots' p[x, y, z, rx, ry, rz]
    convention: position in meters, orientation as a rotation vector in radians.

    Jacobian rows use Pinocchio's motion-vector order:
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
    ) -> None:
        """
        Parameters
        ----------
        mjcf_path:
            MJCF file path. Defaults to universal_robots_ur5e/ur5e.xml.
        end_frame_name:
            Pinocchio frame used as the flange/end frame. The MJCF's
            attachment_site matches the UR flange with zero TCP offset.
        tcp_offset:
            Optional TCP translation [x, y, z] in end-frame coordinates, meters.
        tcp_rotation:
            Optional TCP rotation relative to the end frame. Accepts either a
            3-vector rotation vector or a 3x3 rotation matrix.
        correct_ur_base:
            Apply the same base-frame correction used by ur5e_visualizer.py.
        """
        self.mjcf_path = (
            Path(mjcf_path) if mjcf_path is not None else self.DEFAULT_MJCF_PATH
        )
        self.end_frame_name = end_frame_name

        result = pin.shortcuts.buildModelsFromMJCF(str(self.mjcf_path))
        self.model = result[0]

        if correct_ur_base:
            r_z_180 = np.array(
                [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            self.model.jointPlacements[1] = (
                pin.SE3(r_z_180, np.zeros(3)) * self.model.jointPlacements[1]
            )

        if not self.model.existFrame(end_frame_name):
            raise ValueError(f"Frame '{end_frame_name}' not found in {self.mjcf_path}")

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
        Compute forward kinematics.

        Parameters
        ----------
        q:
            Six UR5e joint angles in radians.

        Returns
        -------
        T:
            4x4 homogeneous transform of the end frame in the base frame.
        R:
            3x3 rotation matrix.
        P:
            3D translation vector in meters.
        ur_pose:
            6D UR pose [x, y, z, rx, ry, rz], with rx/ry/rz as a rotation vector.
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
        Update Pinocchio forward kinematics for callers that manage their own Data.

        Visualizers commonly need separate Data objects for multiple displayed
        robot states. This wrapper keeps those callers on the same corrected
        model used by this class.
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
        Compute the 6x6 geometric Jacobian in the robot base frame.

        The returned linear velocity is the end-frame origin velocity expressed
        in the base frame. This uses Pinocchio LOCAL_WORLD_ALIGNED, which is the
        usual geometric Jacobian convention for base-frame robot control.
        """
        return self._jacobian(q, pin.LOCAL_WORLD_ALIGNED)

    def jacobian_end(self, q: ArrayLike) -> np.ndarray:
        """Compute the 6x6 geometric Jacobian expressed in the end frame."""
        return self._jacobian(q, pin.LOCAL)

    def jacobian(self, q: ArrayLike, reference: str = "base") -> np.ndarray:
        """
        Compute the end-frame Jacobian.

        Parameters
        ----------
        q:
            Six UR5e joint angles in radians.
        reference:
            "base" for base-frame Jacobian, "end" for end-frame Jacobian.
        """
        if reference == "base":
            return self.jacobian_base(q)
        if reference in ("end", "tool", "local"):
            return self.jacobian_end(q)
        raise ValueError("reference must be 'base' or 'end'")

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
        Solve inverse kinematics with damped least squares iteration.

        Parameters
        ----------
        target:
            Either a 4x4 homogeneous transform T, or a UR-style 6D pose
            [x, y, z, rx, ry, rz].
        q_ref:
            Six joint angles in radians. Iteration starts from this reference.
        max_iterations:
            Maximum number of solver iterations.
        tolerance:
            Default tolerance used for both position and orientation if their
            dedicated tolerances are not provided.
        position_tolerance:
            Position convergence tolerance in meters.
        orientation_tolerance:
            Orientation convergence tolerance in radians.
        damping:
            Damping coefficient for the least-squares solve.
        step_size:
            Multiplier applied to each joint update.
        max_step:
            Maximum Euclidean norm of one joint update, in radians.
        position_weight:
            Weight applied to position error rows.
        orientation_weight:
            Weight applied to orientation error rows.
        pose_position_unit:
            Unit for target[0:3] when target is a 6D pose: "m", "mm", or
            "auto". UR official pose uses meters.
        return_info:
            If True, return (q, info). Otherwise return q only.

        Returns
        -------
        q:
            Solved six joint angles in radians.
        info:
            Optional dict with convergence diagnostics.
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
        """Short alias for inverse_kinematics()."""
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
                raise ValueError("Target transform last row must be [0, 0, 0, 1]")
            return T

        target = target.reshape(-1)
        if target.size != 6:
            raise ValueError("target must be a 4x4 transform or a 6D UR pose")

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
            raise ValueError("pose_position_unit must be 'm', 'mm', or 'auto'")

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rotvec_to_matrix(target[3:6])
        T[:3, 3] = position
        return T

    @staticmethod
    def _as_q(q: ArrayLike) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.size != 6:
            raise ValueError(f"Expected 6 joint angles, got {q.size}")
        return q

    @staticmethod
    def _as_vector3(value: Optional[ArrayLike], default: np.ndarray) -> np.ndarray:
        if value is None:
            return default.astype(np.float64)
        value = np.asarray(value, dtype=np.float64).reshape(-1)
        if value.size != 3:
            raise ValueError(f"Expected a 3-vector, got shape {value.shape}")
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
        raise ValueError("tcp_rotation must be a 3-vector or a 3x3 matrix")
