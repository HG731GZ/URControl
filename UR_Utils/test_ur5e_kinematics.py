from __future__ import annotations

import numpy as np

from ur5e_kinematics import UR5eKinematics


def assert_close(name: str, actual: np.ndarray, expected: np.ndarray, atol: float) -> None:
    error = float(np.max(np.abs(actual - expected)))
    print(f"{name}: max_abs_error={error:.3e}")
    if error > atol:
        raise AssertionError(f"{name} failed: {error:.3e} > {atol:.3e}")


def check_forward_kinematics(kin: UR5eKinematics) -> None:
    q = np.zeros(6)
    q = np.array([-82.3, -85.47, -91.74, -90.12, 89.72, 7.70]) / 180 * np.pi
    T, R, P, ur_pose = kin.forward_kinematics(q)

    expected_T = np.array(
        [
            [1.0, 0.0, 0.0, -0.817],
            [0.0, 0.0, -1.0, -0.234],
            [0.0, 1.0, 0.0, 0.063],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    expected_pose = np.array(
        [-0.817, -0.234, 0.063, np.pi / 2.0, 0.0, 0.0],
        dtype=float,
    )

    print("\nForward kinematics at q = zeros:")
    print("T =")
    print(np.array2string(T, precision=6, suppress_small=True))
    print("UR pose [x, y, z, rx, ry, rz] =")
    print(np.array2string(ur_pose, precision=6, suppress_small=True))

    # assert_close("T", T, expected_T, atol=1e-12)
    # assert_close("R", R, expected_T[:3, :3], atol=1e-12)
    # assert_close("P", P, expected_T[:3, 3], atol=1e-12)
    # assert_close("UR pose", ur_pose, expected_pose, atol=1e-12)


def check_jacobian_shapes(kin: UR5eKinematics) -> None:
    q = np.array([0.2, -1.0, 0.7, -0.4, 0.5, 0.3])
    j_base = kin.jacobian_base(q)
    j_end = kin.jacobian_end(q)

    print("\nJacobian shapes:")
    print(f"J_base: {j_base.shape}")
    print(f"J_end : {j_end.shape}")

    if j_base.shape != (6, 6):
        raise AssertionError(f"Unexpected J_base shape: {j_base.shape}")
    if j_end.shape != (6, 6):
        raise AssertionError(f"Unexpected J_end shape: {j_end.shape}")


def check_base_jacobian_finite_difference(kin: UR5eKinematics) -> None:
    q = np.array([0.2, -1.0, 0.7, -0.4, 0.5, 0.3])
    eps = 1e-7

    _, _, p0, _ = kin.forward_kinematics(q)
    j_base = kin.jacobian_base(q)
    j_pos_fd = np.zeros((3, 6), dtype=float)

    for i in range(6):
        dq = np.zeros(6, dtype=float)
        dq[i] = eps
        _, _, p1, _ = kin.forward_kinematics(q + dq)
        j_pos_fd[:, i] = (p1 - p0) / eps

    print("\nBase-frame position Jacobian finite difference:")
    print("analytic =")
    print(np.array2string(j_base[:3, :], precision=6, suppress_small=True))
    print("finite difference =")
    print(np.array2string(j_pos_fd, precision=6, suppress_small=True))

    assert_close("J_base position finite difference", j_base[:3, :], j_pos_fd, atol=1e-6)


def check_local_world_jacobian_consistency(kin: UR5eKinematics) -> None:
    q = np.array([0.2, -1.0, 0.7, -0.4, 0.5, 0.3])
    _, R, _, _ = kin.forward_kinematics(q)

    j_base = kin.jacobian_base(q)
    j_end = kin.jacobian_end(q)
    j_end_to_base = np.vstack((R @ j_end[:3, :], R @ j_end[3:, :]))

    print("\nEnd-frame Jacobian transformed to base orientation:")
    print(np.array2string(j_end_to_base, precision=6, suppress_small=True))

    assert_close("J_end transformed to base", j_end_to_base, j_base, atol=1e-12)


def check_tcp_offset(kin: UR5eKinematics) -> None:
    kin_tcp = UR5eKinematics(tcp_offset=[0.0, 0.0, 0.1])
    q = np.zeros(6)
    _, R, P, _ = kin.forward_kinematics(q)
    _, _, p_tcp, _ = kin_tcp.forward_kinematics(q)
    expected_p_tcp = P + R @ np.array([0.0, 0.0, 0.1])

    print("\nTCP offset check:")
    print("P without TCP =", np.array2string(P, precision=6, suppress_small=True))
    print("P with TCP    =", np.array2string(p_tcp, precision=6, suppress_small=True))

    assert_close("TCP offset position", p_tcp, expected_p_tcp, atol=1e-12)


def main() -> None:
    kin = UR5eKinematics()

    check_forward_kinematics(kin)
    # check_jacobian_shapes(kin)
    # check_base_jacobian_finite_difference(kin)
    # check_local_world_jacobian_consistency(kin)
    # check_tcp_offset(kin)

    # print("\nAll UR5e kinematics checks passed.")


if __name__ == "__main__":
    main()
