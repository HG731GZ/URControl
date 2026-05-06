from __future__ import annotations

from typing import Optional

import numpy as np


def apply_pose_delta(pose_now: np.ndarray, delta_pose: np.ndarray, frame: str) -> np.ndarray:
    """在指定参考系下，将增量位姿叠加到当前位姿上，得到目标位姿。

    Args:
        pose_now: 当前 TCP 位姿，shape (6,)，格式 [x, y, z, rx, ry, rz]（旋转向量）。
        delta_pose: 增量位姿，shape (6,)，格式同 pose_now。
        frame: 参考系，``"tool"`` 表示在工具系（末端）下作用增量（右乘齐次变换），
               ``"base_add"`` 表示在基座标系下直接向量相加。

    Returns:
        np.ndarray: 叠加后的目标位姿(基坐标系)，shape (6,)。
    """
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
    """通过齐次变换矩阵将两个位姿相乘（右乘），等价于在 pose_a 末端叠加 pose_b。

    Args:
        pose_a: 位姿 A，shape (6,)，格式 [x, y, z, rx, ry, rz]。
        pose_b: 位姿 B，shape (6,)，格式同上。

    Returns:
        np.ndarray: 合成位姿 T_a @ T_b，shape (6,)。
    """
    Ta = pose_to_transform(pose_a)
    Tb = pose_to_transform(pose_b)
    return transform_to_pose(Ta @ Tb)


def pose_to_transform(pose: np.ndarray) -> np.ndarray:
    """将 UR 位姿向量转换为 4x4 齐次变换矩阵。

    Args:
        pose: 位姿向量，shape (6,)，格式 [x, y, z, rx, ry, rz]（旋转向量）。

    Returns:
        np.ndarray: 4x4 齐次变换矩阵，旋转部分由 rotvec_to_matrix 计算。
    """
    pose = np.asarray(pose, dtype=float)
    T = np.eye(4, dtype=float)
    T[0:3, 0:3] = rotvec_to_matrix(pose[3:6])
    T[0:3, 3] = pose[0:3]
    return T


def transform_to_pose(T: np.ndarray) -> np.ndarray:
    """将 4x4 齐次变换矩阵转换为 UR 位姿向量。

    Args:
        T: 4x4 齐次变换矩阵。

    Returns:
        np.ndarray: 位姿向量，shape (6,)，格式 [x, y, z, rx, ry, rz]，
                    平移部分取自 T[:3, 3]，旋转部分由 matrix_to_rotvec 计算。
    """
    T = np.asarray(T, dtype=float)
    return np.concatenate([T[0:3, 3], matrix_to_rotvec(T[0:3, 0:3])])


def twist_tool_to_base(pose_now: np.ndarray, twist_tool: np.ndarray) -> np.ndarray:
    """将工具系（末端）下描述的 twist（线速度 + 角速度）旋转到基座标系。

    Args:
        pose_now: 当前 TCP 位姿，shape (6,)，用于提取末端姿态旋转矩阵。
        twist_tool: 工具系下的 twist，shape (6,)，格式 [vx, vy, vz, wx, wy, wz]。

    Returns:
        np.ndarray: 基座标系下的 twist，shape (6,)。
    """
    pose_now = np.asarray(pose_now, dtype=float)
    twist_tool = np.asarray(twist_tool, dtype=float)

    R = rotvec_to_matrix(pose_now[3:6])
    v_base = R @ twist_tool[0:3]
    w_base = R @ twist_tool[3:6]
    return np.concatenate([v_base, w_base])


def twist_to_base(pose_now: np.ndarray, twist: np.ndarray, frame: str) -> np.ndarray:
    """根据参考系将 twist 转换到基座标系。

    Args:
        pose_now: 当前 TCP 位姿，shape (6,)。
        twist: 输入 twist，shape (6,)，格式 [vx, vy, vz, wx, wy, wz]。
        frame: 参考系，``"tool"`` 时通过 twist_tool_to_base 旋转到基系，
               ``"base_add"`` 时直接返回原值（已在基系）。

    Returns:
        np.ndarray: 基座标系下的 twist，shape (6,)。
    """
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
    """对 TCP 位姿命令进行速率限制，防止单步运动过大。

    平动部分限制三维位移向量模长（保持方向），姿态部分通过旋转向量模长限制角速度。
    任一限幅参数为 ``None`` 时跳过对应部分的限幅。

    Args:
        pose_cmd: 当前已发出的位姿命令，shape (6,)。
        pose_target: 期望的目标位姿，shape (6,)。
        dx_max: 最大线速度 (m/s)，或 ``None`` 不限平动。
        dq_max: 最大角速度 (rad/s)，或 ``None`` 不限姿态。
        dt: 控制周期 (s)。

    Returns:
        np.ndarray: 限幅后的位姿命令，shape (6,)。
    """
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
    """限制向量的模长，保持方向不变。

    Args:
        vec: 输入向量，任意 shape。
        max_norm: 允许的最大模长。

    Returns:
        np.ndarray: 若模长 <= max_norm 则返回原向量副本，否则按比例缩放到 max_norm。
    """
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
    """判断当前位姿是否已到达目标位姿（平动与姿态分别检查）。

    Args:
        actual_pose: 实际 TCP 位姿，shape (6,)。
        target_pose: 目标 TCP 位姿，shape (6,)。
        position_tolerance: 位置容差 (m)，默认 0.5 mm。
        rotation_tolerance: 姿态容差 (rad)，默认 1e-3 rad（以旋转向量模长度量）。

    Returns:
        bool: 位置误差和姿态误差均在容差内时返回 ``True``。
    """
    actual_pose = np.asarray(actual_pose, dtype=float)
    target_pose = np.asarray(target_pose, dtype=float)

    position_error = float(np.linalg.norm(actual_pose[0:3] - target_pose[0:3]))
    R_actual = rotvec_to_matrix(actual_pose[3:6])
    R_target = rotvec_to_matrix(target_pose[3:6])
    rotation_error = float(np.linalg.norm(matrix_to_rotvec(R_target.T @ R_actual)))

    return position_error < position_tolerance and rotation_error < rotation_tolerance


def rotvec_to_matrix(r: np.ndarray) -> np.ndarray:
    """将旋转向量（轴角表示）转换为 3x3 旋转矩阵（Rodrigues 公式）。

    Args:
        r: 旋转向量，shape (3,)，方向为转轴，模长为转角 (rad)。

    Returns:
        np.ndarray: 3x3 旋转矩阵。模长接近 0 时返回单位阵。
    """
    r = np.asarray(r, dtype=float)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)

    k = r / theta
    kx, ky, kz = k
    K = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=float,
    )

    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    """将 3x3 旋转矩阵转换为旋转向量（轴角表示）。

    小角度（< 1e-12 rad）时使用一阶近似避免数值不稳定，
    接近 180° 时通过特征值分解获取旋转轴。

    Args:
        R: 3x3 旋转矩阵。

    Returns:
        np.ndarray: 旋转向量，shape (3,)，方向为转轴，模长为转角 (rad)。
    """
    R = np.asarray(R, dtype=float)
    cos_theta = (float(np.trace(R)) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))

    if theta < 1e-12:
        # 小角度时 arccos(trace) 容易因为浮点舍入直接变成 0。
        # 使用 log(R) 的一阶近似 vee((R - R.T) / 2)，能保留非常小的姿态误差。
        return 0.5 * np.array(
            [
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1],
            ],
            dtype=float,
        )

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

    axis = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) / (2.0 * np.sin(theta))

    return axis * theta
