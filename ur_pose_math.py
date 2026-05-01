from __future__ import annotations

from typing import Optional

import numpy as np


def apply_pose_delta(pose_now: np.ndarray, delta_pose: np.ndarray, frame: str) -> np.ndarray:
    pose_now = np.asarray(pose_now, dtype=float)
    delta_pose = np.asarray(delta_pose, dtype=float)

    if frame == "tool":
        # 使用齐次变换矩阵在末端坐标系下叠加增量：
        # T_base_target = T_base_tcp @ T_tcp_delta。
        return pose_trans(pose_now, delta_pose)

    if frame == "base_add":
        return pose_now + delta_pose

    raise ValueError(f"Unsupported frame: {frame}")


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


def twist_to_base(pose_now: np.ndarray, twist: np.ndarray, frame: str) -> np.ndarray:
    twist = np.asarray(twist, dtype=float)

    if frame == "base_add":
        return twist.copy()

    if frame == "tool":
        return twist_tool_to_base(pose_now, twist)

    raise ValueError(f"Unsupported frame: {frame}")


def rate_limit_tcp_pose(
    pose_cmd: np.ndarray,
    pose_target: np.ndarray,
    dx_max: Optional[float],
    dq_max: Optional[float],
    dt: float,
) -> np.ndarray:
    pose_cmd = np.asarray(pose_cmd, dtype=float)
    pose_target = np.asarray(pose_target, dtype=float)

    if dx_max is None and dq_max is None:
        return pose_target.copy()

    pose_next = pose_cmd.copy()

    # 平动限速：限制的是三维位移向量的模长，对应 ||v_xyz|| <= dx_max，
    # 而不是分别对 x/y/z 三个分量裁剪。这样斜向运动不会因为逐轴限幅而改变方向。
    if dx_max is None:
        pose_next[:3] = pose_target[:3]
    else:
        dp = pose_target[:3] - pose_cmd[:3]
        pose_next[:3] = pose_cmd[:3] + limit_vector_norm(dp, dx_max * dt)

    # 姿态限速：旋转向量不能长期当普通欧氏向量相减。
    # 这里先计算从当前命令姿态到目标姿态的相对旋转：
    #     R_rel = R_cmd.T @ R_target
    # 再把这个相对旋转转换成旋转向量，并限制其模长，
    # 对应 ||omega|| <= dq_max。最后把受限的小旋转左乘回当前命令姿态。
    if dq_max is None:
        pose_next[3:] = pose_target[3:]
    else:
        r_cmd = rotvec_to_matrix(pose_cmd[3:])
        r_target = rotvec_to_matrix(pose_target[3:])
        r_rel = r_cmd.T @ r_target
        rel_rotvec = matrix_to_rotvec(r_rel)
        rel_step = limit_vector_norm(rel_rotvec, dq_max * dt)
        r_next = r_cmd @ rotvec_to_matrix(rel_step)
        pose_next[3:] = matrix_to_rotvec(r_next)

    return pose_next


def limit_vector_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    vec = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or norm < 1e-12:
        return vec.copy()
    return vec * (max_norm / norm)


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
        # 小角度时 arccos(trace) 容易因为浮点舍入直接变成 0。
        # 使用 log(R) 的一阶近似 vee((R - R.T) / 2)，能保留非常小的姿态误差。
        return 0.5 * np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ], dtype=float)

    if abs(np.pi - theta) < 1e-5:
        # 接近 180 度时标准公式中的 sin(theta) 接近 0。
        # 取特征值 1 对应的实特征向量作为旋转轴，避免除以极小数。
        eigenvalues, eigenvectors = np.linalg.eig(R)
        idx = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, idx])
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-12:
            return np.zeros(3, dtype=float)
        axis = axis / axis_norm
        return axis * theta

    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=float) / (2.0 * np.sin(theta))

    return axis * theta
