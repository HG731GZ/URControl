"""主从 UDP 端点与反馈频率；会话由通信客户端自动发现和生成。"""

import os

FEEDBACK_HZ = 200
# 主从通信走交换机有线网卡 enp4s0f2np2；机器人网段独立配置。
LOCAL_IP = os.environ.get("UR_UDP_LOCAL_IP", "192.168.1.14")
LOCAL_PORT = int(os.environ.get("UR_UDP_LOCAL_PORT", "5005"))
REMOTE_IP = os.environ.get("UR_UDP_REMOTE_IP", "192.168.1.5")
REMOTE_PORT = int(os.environ.get("UR_UDP_REMOTE_PORT", "6005"))

