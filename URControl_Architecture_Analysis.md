# URControl 项目架构与功能分析

更新日期：2026-10-08。本文描述修改后的实现；修改背景和验收要求见 `URControl_代码修改方案.md`。

## 文件结构

```text
URControl/
├── main.py                         # MainWindow、连接与控制协调、显示和采集
├── main_window.ui                  # Qt Designer 界面源文件
├── ui_main_window.py               # pyuic5 生成的界面代码
├── DataCollector.py                # 分组采集、异步写盘、视频和元数据
├── RealSenseCamera.py              # RealSense 后台取帧与来源时间
├── GripperController.py            # 夹爪 Modbus RTU 控制与反馈
├── NetWorkSet.py                    # 本机地址查询和 IPv4 校验
└── UR_Utils/
    ├── URTcpClient.py              # Dashboard、URScript、Realtime 的 TCP 基类
    ├── URDashboardClient.py        # 电源、抱闸和程序管理
    ├── URScriptClient.py           # 单次定位和脚本停止
    ├── URRealtimeClient.py         # 后台读取机器人状态
    ├── URRealtimeUtils.py          # 实时报文解析和 URRealtimeState
    ├── URRTDEController.py         # RTDE 跟踪、速度控制和控制器生命周期
    ├── URRTDETorqueClient.py       # 只读力矩订阅，同步读取或后台缓存
    ├── URUdpClient.py              # 主端指令接收和反馈发送
    ├── ur_pose_math.py             # 位姿变换
    ├── ur5e_kinematics.py           # 运动学模型
    ├── ur5e_model_cache.py         # 模型缓存
    ├── ur5e_gl_renderer.py         # 原生 OpenGL 控件
    ├── ur5e_visualizer.py          # 实际与目标机械臂显示
    └── universal_robots_ur5e/      # UR5e 模型资源
```

## 通信与连接

| 通道 | 模块 | 用途 |
|---|---|---|
| TCP 29999 | `URDashboardClient` | 上电、下电、松抱闸、程序命令 |
| TCP 30002 | `URScriptClient` | `movej` 单次定位、`stopj` 停止 |
| TCP 30013 | `URRealtimeClient` | 后台读取关节、末端位姿、力和运行状态 |
| RTDE | `URRTDEController` | 使用 `ur_rtde` 的控制、接收和 IO 接口 |
| RTDE 30004 | `URRTDETorqueClient` | 使用官方 `rtde` 客户端订阅电流换算力矩 |
| TCP 54321 | `GripperController` | 通过机器人串口转发控制夹爪 |
| UDP 5005 / 6005 | `URUDPClient` | 接收主端指令 / 发送机器人和夹爪反馈 |

RTDE 控制器不继承 `URTcpClient`。机器人 IP、UDP 地址和端口等配置集中在 `main.py` 顶部。

`robot_ip` 表示待连接目标，`connected_ip` 表示实际连接地址。输入框显示待连接目标，顶部“实际连接”标签显示已连接地址，状态栏和悬浮提示同时显示两者；编辑输入框后需要点击 IP 按钮应用。切换 IP 或 URSim 时先停止控制、释放旧连接，再更新目标；用户点击连接后建立新连接。重复连接同样先释放旧客户端和线程。Dashboard、URScript、Realtime、RTDE 和夹爪共用目标地址；本机 URSim 跳过不存在的夹爪转发服务。

机器人连接与 RTDE 就绪分别管理。状态回调读取 Realtime 缓存，在机器人进入 RUNNING 且安全模式为正常或减速时创建 RTDE；因此先连接、再上电和松抱闸也能就绪。初始化失败会显示错误，需要显式重连。运行状态丢失时停止控制并释放 RTDE。按钮由 `update_buttons()` 集中根据连接、运行状态、RTDE 就绪和 UDP 指令有效性启用。

## 控制流程

`control_source` 只取 `idle`、`jog`、`udp`。单次 URScript 定位由 `_script_motion_active` 标记，以便停止按钮能终止未完成的定位。

`stop_control()` 的顺序为：

1. 停止点动和 UDP 控制定时器。
2. 清空控制来源、点动速度和模式，暂停夹爪目标下发。
3. 停止 RTDE 控制并清除目标；存在单次定位时发送 `stopj`。
4. 恢复目标关节角输入。

两个停止按钮、点动释放、控制来源切换、回零、关节定位、地址切换和断开均复用这个入口。单次定位前使用 `stop_script=True` 释放机器人端 RTDE 脚本。GUI 创建 RTDE 时设置 `auto_start_on_command=False`，只有用户明确启动点动或跟随才调用 `start()`；底层控制器其他调用方仍保持原有默认行为。

点动使用 `speedJ` 或 `speedL`，分别支持关节、工具坐标系、基坐标系。速度常量明确区分 rad/s、m/s 和角速度 rad/s。原有 `time_s=1` 参数保持不变。

UDP 跟随只接受 `UDPControlMode.JOINT_TRACK`。OFF、不支持的模式、接收错误或超过 `UDP_COMMAND_TIMEOUT_S=0.5` 秒未收到新帧都会停止跟随，不再提交机械臂和夹爪目标。收到后续数据不会自动恢复运动。单次 UDP 同步也只接受未过期的关节跟踪帧。

RTDE 内部由当前目标选择行为：关节目标交给后台 `servoJ` 并按 `dq_max` 限速，末端目标交给 `servoL`；`speedJ`、`speedL` 则直接发送速度指令并使用已有速度看门狗。`stop()` 清空目标，`close()`/`shutdown()` 释放连接和线程。

## 显示与线程

界面使用 `widget_RobotView` 嵌入原生 OpenGL 可视化控件。修改布局时编辑 `main_window.ui`，再用项目环境的 `pyuic5` 生成 `ui_main_window.py`；生成文件的说明注释统一使用中文。

| 定时器 | 周期 | 职责 |
|---|---|---|
| `status_timer` | 100 ms | 读取状态缓存、RTDE 就绪和按钮状态 |
| `robot_view_timer` | 10 ms | 显示关节、末端、UDP、夹爪和力矩缓存 |
| `jog_timer` | 2 ms，仅点动期间 | 提交速度命令 |
| `udp_control_timer` | 10 ms，仅跟随期间 | 校验 UDP 新鲜度、提交关节和夹爪目标 |
| `camera_timer` | 50 ms | 显示后台取得的相机帧 |
| `collection_timer` | 50 ms，仅采集期间 | 独立读取各数据源并推送采集快照 |

Realtime、UDP、相机和夹爪分别使用现有后台线程。勾选力矩显示且机器人可运行时建立只读力矩订阅，由 `URRTDETorqueClient` 后台线程读取；刷新界面只读 `get_latest()` 缓存。取消勾选、读取失败或断开时结束订阅。`read()` 仍保留同步接口供原有采集代码使用，不与后台线程同时调用。

周期显示不再同步查询 Dashboard 或等待力矩网络数据。连接建立、RTDE 初始化和用户发出的运动命令仍可能占用 GUI 线程，本次未引入通用异步任务框架。

## 数据采集

`DATA_COLLECT_FREQ=20.0` 同时决定采样周期和注册视频的 `video_fps`。目录由 `DATA_DIRECTORY` 配置，会话名称使用 `SESSION_NAME_FORMAT` 加创建时间生成。采集器采用 `write_mode="realtime"`、`async_write=True`，图像尽快进入后台写盘队列，不积累整段原始图像。

采集开始和结束使用 `collector.episode_active` 判断。`collect_sample()` 自行读取 UDP、Realtime、夹爪和相机，独立于运动控制和显示缓存。同一周期的数值数据共享一个采样时间戳；UDP 和 Realtime 没有新帧时不重复记录。UDP 过期帧不再入库，停止跟随后仍能采集新到达的主端数据，包括 OFF 状态下的新帧。

视频索引里的时间戳使用 `CameraFrame.received_time`，即相机后台接收该帧的主机时间，不使用采集回调取帧的时间，也不将设备时钟误当作主机时间。`DataCollector.push_image(..., timestamp=...)` 接受该时间，原调用方省略时保持使用当前时间。

主要输出包括每组数值 CSV、每路视频、视频帧与采样步号的时间索引，以及 `metadata.json`。异步队列仍要求磁盘持续写入速度跟上采样；实际相机分辨率、图像内容和设备性能需要实物联调。

## 退出收尾

`closeEvent()` 先停止六个定时器，再通过统一断开流程停止运动并释放机器人和夹爪。随后停止 UDP 和相机，调用 `collector.close()` 等待待写数据、结束视频并保存元数据，最后关闭可视化。仅调用 `end_episode()` 不代表写盘完成。

## 验证范围

本次按修改方案进行语法检查和临时 URSim 联调，不新增回归测试文件、框架或用例，不计算文件哈希。相机和夹爪实物通信、真实画面编码吞吐仍需设备可用时验证。软件停止按钮是控制指令停止，不是硬件急停；原下电按钮已改为“UR下电”，与实际 Dashboard `power_off()` 行为一致。

### 2026-10-08 联调结果

- 25 个 Python 文件语法解析通过，差异空白检查通过；未新增测试文件。
- 主窗口可显示，36 个点动按钮完成信号绑定，原生 OpenGL 上下文有效、20 个网格正常加载。
- 本机 `127.0.0.1` URSim 的连接、重复连接和 IP 切换通过；旧 Realtime 线程退出、旧 RTDE 控制器关闭，各机器人客户端地址一致。
- 在 POWER_OFF 时连接，随后上电至 IDLE、松抱闸至 RUNNING，RTDE 自动就绪，运动按钮随状态启用。
- 临时将点动速度降低后验证关节、工具和基坐标系点动，释放后实际关节速度为零；UDP 与点动切换时仅一个定时器运行。
- 两类连续控制停止后不再提交目标；UDP OFF、断流停止并清空关节目标，新帧不会自动恢复跟随。完成后恢复初始关节位置，最大误差约 `2.2e-8 rad`，模拟器恢复 RUNNING。
- 力矩后台缓存正常更新，取消显示后读取线程退出。
- 未启动跟随时主端数据也可入库；采集中关闭窗口后写盘完成、元数据存在、无残留 Python 后台线程。
- 使用两路临时合成 RGB 图像（1280×720、480×640）按 20 Hz 写入 3 秒：两段视频各 60 帧、声明 20 FPS、时长 3 秒且可解码，来源时间戳保持一致。观测队列峰值为 1，采样结束时待写任务为 0，整段原始图像缓存为 0，关闭收尾约 0.002 秒。这是当前机器的短时合成图像结果，不代表实物相机长期采集验收。
