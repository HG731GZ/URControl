from __future__ import annotations

import numpy as np


def pose_trans(pose_a: np.ndarray, pose_b: np.ndarray) -> np.ndarray:
    Ta = pose_to_transform(pose_a)
    Tb = pose_to_transform(pose_b)
    return transform_to_pose(Ta @ Tb)


def pose_to_transform(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = rotvec_to_matrix(pose[3:6])
    T[0:3, 3] = pose[0:3]
    return T


def transform_to_pose(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=float)
    return np.concatenate([T[0:3, 3], matrix_to_rotvec(T[0:3, 0:3])])


def twist_tool_to_base(pose_now: np.ndarray, twist_tool: np.ndarray) -> np.ndarray:
    pose_now = np.asarray(pose_now, dtype=float)
    twist_tool = np.asarray(twist_tool, dtype=float)

    R = rotvec_to_matrix(pose_now[3:6])
    v_base = R @ twist_tool[0:3]
    w_base = R @ twist_tool[3:6]
    return np.concatenate([v_base, w_base])


def is_pose_reached(
    actual_pose: np.ndarray,
    target_pose: np.ndarray,
    position_tolerance: float = 5e-4,
    rotation_tolerance: float = 1e-3,
) -> bool:
    actual_pose = np.asarray(actual_pose, dtype=float)
    target_pose = np.asarray(target_pose, dtype=float)

    position_error = float(np.linalg.norm(actual_pose[0:3] - target_pose[0:3]))
    R_actual = rotvec_to_matrix(actual_pose[3:6])
    R_target = rotvec_to_matrix(target_pose[3:6])
    rotation_error = float(np.linalg.norm(matrix_to_rotvec(R_target.T @ R_actual)))

    return position_error < position_tolerance and rotation_error < rotation_tolerance


def rotvec_to_matrix(r: np.ndarray) -> np.ndarray:
    r = np.asarray(r, dtype=float)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)

    k = r / theta
    kx, ky, kz = k
    K = np.array([
        [0.0, -kz, ky],
        [kz, 0.0, -kx],
        [-ky, kx, 0.0],
    ], dtype=float)

    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    cos_theta = (float(np.trace(R)) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))

    if theta < 1e-12:
        return np.zeros(3, dtype=float)

    if abs(np.pi - theta) < 1e-5:
        axis = np.empty(3, dtype=float)
        axis[0] = np.sqrt(max((R[0, 0] + 1.0) * 0.5, 0.0))
        axis[1] = np.sqrt(max((R[1, 1] + 1.0) * 0.5, 0.0))
        axis[2] = np.sqrt(max((R[2, 2] + 1.0) * 0.5, 0.0))

        if R[2, 1] - R[1, 2] < 0:
            axis[0] = -axis[0]
        if R[0, 2] - R[2, 0] < 0:
            axis[1] = -axis[1]
        if R[1, 0] - R[0, 1] < 0:
            axis[2] = -axis[2]

        n = np.linalg.norm(axis)
        if n < 1e-12:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            axis = axis / n
        return axis * theta

    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=float) / (2.0 * np.sin(theta))

    return axis * theta
