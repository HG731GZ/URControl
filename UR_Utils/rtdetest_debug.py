import time
import numpy as np

from URRTDEController import URRTDEController

ROBOT_IP = "127.0.0.1"  # 按你的 URSim/真机 IP 修改

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
    servo_speed=0.5,
    servo_acceleration=0.5,
    # URSim 调试阶段可以先关掉安全检查，确认能动以后再打开。
    use_safety_check=True,
) as ctrl:
    print("after start:", ctrl.debug_status_string())

    # 先用关节跟踪测试，最容易判断 servoJ 有没有真正生效。
    print("actual q before:", np.round(ctrl.get_actual_q(), 4))
    ctrl.track_joint(home_q, dq_max=0.5)
    print(f"FirstCommand:", ctrl.debug_status_string())
    time.sleep(1)

    for i in range(30):
        time.sleep(0.1)
        print(f"track {i:02d}:", ctrl.debug_status_string())
    time.sleep(1)
    # 再测试 TCP 增量。
    target_pose = ctrl.move_tcp_delta(
        delta_pose=[-0.2, 0, 0, 0, 0, 0],
        dq_max=0.3,
        frame="tool",
    )
    print("target tcp pose:", np.round(target_pose, 4))

    for i in range(30):
        time.sleep(0.1)
        print(f"tcp {i:02d}:", ctrl.debug_status_string())

    ctrl.stop()
    time.sleep(0.5)
    print("after stop:", ctrl.debug_status_string())
