import time
import numpy as np

from URRTDEController import URRTDEController

ROBOT_IP = "127.0.0.1"  # URSim 常见 IP，按你的实际情况修改

home_q = [
    0.0,
    -np.pi / 2,
    -np.pi / 2,
    0.0,
    np.pi / 2,
    0.0,
]

with URRTDEController(
    ROBOT_IP,
    frequency=500.0,
    default_dq_max=0.5,
    lookahead_time=0.08,
    gain=300,
    use_safety_check=False,
) as ctrl:

    # # 1. 模式 1：闭环跟踪关节角
    # ctrl.track_joint(home_q, dq_max=0.5)
    # time.sleep(3.0)
    #
    # for i in range(6):
    #     home_q[i]=home_q[i]+0.1
    # ctrl.track_joint(home_q, dq_max=0.5)
    # time.sleep(3.0)
    #
    # # 2. 模式 2：关节增量控制
    # # 基于当前实际 q，目标为 q_now + delta_q
    # ctrl.move_joint_delta(
    #     delta_q=[0.05, 0, 0, 0, 0, 0],
    #     dq_max=0.3,
    # )
    # time.sleep(1.0)

    # 3. 模式 3：末端增量控制
    # 默认 frame="tool"，表示沿当前 TCP 坐标系移动
    # target_pose = ctrl.move_tcp_delta(
    #     delta_pose=[-0.2, 0, 0, 0, 0, 0],
    #     dq_max=0.3,
    #     frame="tool",
    # )
    # print("target tcp pose:", np.round(target_pose, 4))
    #
    # for i in range(30):
    #     time.sleep(0.1)
    #     print(f"tcp {i:02d}:", ctrl.debug_status_string())
    ctrl.set_speed_slider(1)
    target_pose = ctrl.move_tcp_delta(
        delta_pose=[-0.2, 0, 0, 0, 0, 0],
        frame="base_add",
    )
    print("target tcp pose:", np.round(target_pose, 4))
    time.sleep(10)
    print("target tcp pose:", np.round(target_pose, 4))


    # 4. 关闭闭环跟踪
    ctrl.stop()
    time.sleep(0.5)
