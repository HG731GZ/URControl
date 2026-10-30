# URControl

基于 PyQt5 的 UR 机械臂控制、状态显示和数据采集程序，支持 URSim、RealSense 相机及夹爪。主界面统一使用 classic，保留独立导纳调参与监视窗口。

六轴固定参考导纳提供无需主端的独立窗口。当前进度、参数和适用范围见 [后续规划](URControl_导纳后续规划_20261028_讨论稿.md)与[实机联调记录](URControl_导纳实机联调_20261029.md)；本文件说明安装、使用和维护方法。

## 配置环境

环境版本以当前 Linux 开发环境为基准：Python 3.10、NumPy 2.2.6、Pinocchio 4.0.0。完整依赖见 [environment.yml](environment.yml)，其中固定了主要直接依赖的版本；底层依赖仍由 Conda 和 pip 解析。

先安装 Miniconda 或 Anaconda，以及 Git。安装过程需要访问 conda-forge、PyPI 和 GitHub。在项目根目录的 Conda 终端中运行；Windows 可使用 Anaconda Prompt：

```bash
python setup_environment.py
conda activate urcontrol
python main.py
```

[setup_environment.py](setup_environment.py) 创建或更新名为 `urcontrol` 的环境，然后在该环境中检查依赖导入及 UR5e 模型加载。已有同名环境中的相关包会按清单调整版本。要单独创建一个环境，可指定名称：

```bash
python setup_environment.py --name urcontrol-new
conda activate urcontrol-new
python main.py
```

脚本安装及检查时禁用用户级包目录，并在环境中设置 `PYTHONNOUSERSITE=1`。这样不会因为本机用户目录恰好装过某个包，就遗漏目标环境的安装。更新已激活的环境后，重新执行 `conda activate urcontrol` 以应用环境变量；使用自定义名称时相应替换。

安装失败时脚本会停止并保留原始错误信息；解决网络或安装错误后可重新运行。脚本按自身位置定位环境清单，启动 `main.py` 时请仍切换到项目根目录。

只检查当前 Python 环境，不安装包：

```bash
conda activate urcontrol
python setup_environment.py --check-only
```

检查会输出实际使用的 Python 和各模块路径，并验证 `rtde`、`rtde_control` 等模块和 Pinocchio 的 MJCF 模型加载。检查过程不连接机器人、不打开相机、不启动窗口。PyCharm 中也应选择同一个 Conda 环境的解释器，并在运行配置中设置 `PYTHONNOUSERSITE=1`。

这份环境清单面向 Linux/Windows 的 64 位桌面环境；本机验证平台为 Linux，Windows 尚未完成整套环境的实际安装验收。图形显示需要系统 OpenGL 支持，RealSense 实物使用还需要对应驱动及设备访问权限。即使只使用 URSim，当前启动导入仍需要相机和夹爪的 Python 依赖。

## `git pull` 后提示缺少 `rtde`

`git pull` 只更新代码，不会同步 Python 环境。项目使用了两套不同的 RTDE 客户端，两者都需要安装：

| 安装来源 / 包名 | Python 导入名称 | 本项目用途 |
|---|---|---|
| [ur-rtde](https://pypi.org/project/ur-rtde/1.6.3/) | `rtde_control`、`rtde_receive`、`rtde_io` | 机器人运动控制、状态接收及 IO |
| [Universal Robots 官方 RTDE 客户端](https://github.com/UniversalRobots/RTDE_Python_Client_Library)（发行包名 `UrRtde`） | `rtde`、`rtde.rtde` | 关节力矩订阅及导纳 RTDE 配方序列化 |

代码整理后，官方 `rtde` 客户端从按需导入改为程序启动时导入。因此，只安装了 `ur_rtde` / `ur-rtde` 的电脑可能在启动时出现：

```text
ModuleNotFoundError: No module named 'rtde'
```

如果原环境其余依赖都已安装，只需在**运行项目的同一个环境**中补装官方客户端：

```bash
conda activate urcontrol
python -m pip install "git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git@28db033e2cde98f782431452eb810fb76ebad17c"
python -c "from rtde.rtde import RTDE, RTDEException; import rtde_control, rtde_receive, rtde_io; print('两套 RTDE 客户端均可导入')"
```

这里固定本机安装元数据记录的官方源码提交，发行包版本为 `UrRtde 2.8.4`；导纳传输适配器明确要求此版本，不自动尝试其他版本。安装方式见[官方说明](https://github.com/UniversalRobots/RTDE_Python_Client_Library#using-rtde-library)。使用 `python -m pip` 可以明确把包装入该 Python 对应的环境。

若仍报缺包，先确认终端和 IDE 的解释器路径一致：

```bash
python -c "import sys; print(sys.executable)"
python -m pip --version
```

## 依赖名称说明

- Pinocchio 使用 Conda 的 `pinocchio` 包，确保具备项目使用的 MJCF 模型加载能力；[官方安装说明](https://pypi.org/project/pin/4.0.0/)中的 pip 包名为 `pin`。
- OpenCV 使用 `opencv-python-headless`，提供图像处理和视频编码；窗口由 PyQt5 提供。独立环境中不需要再安装 `opencv-python` 或 `opencv-contrib-python`。
- RealSense 使用 `pyrealsense2`；串口与夹爪使用 `pyserial`、`minimalmodbus`。

## 运行

连接真机前，在 `main.py` 顶部配置机器人地址、UDP 地址及端口。使用本机 URSim 时勾选“URSim”，再点击“连接UR”。上电并松抱闸后，RTDE 就绪时启用运动控制按钮。

数据默认保存到 `data/`，采样频率与视频帧率均由 `DATA_COLLECT_FREQ` 配置。关闭窗口时程序等待写盘完成并保存元数据。

夹爪是可选设备。末端未安装夹爪时可直接连接 UR 并使用定点柔顺，无需启动主端。夹爪串口转发不可用时跳过夹爪连接；转发存在但夹爪不响应时，后台仅做低频只读探测，同一故障不重复刷屏。界面显示“未连接 / 无反馈”，不记录默认值或过期夹爪数据；取得有效反馈后恢复正常显示。暂停夹爪时只暂停目标下发，仍读取反馈。

主端离线时 UDP 反馈在后台退避重试，网络异常不退出程序或阻塞界面。机械臂反馈不依赖夹爪，夹爪不可用时三个反馈字段为 `NaN`，详见 [UDP 协议](UR_Utils/UDP_Protocol.md)。无末端负载测试仍使用 UR 端当前的负载与 TCP 配置，程序不根据夹爪通信状态自动改写这些配置。

## classic 界面与屏幕适配

主程序直接加载经典界面，保留原有控件位置和配色。窗口根据完整内容、状态栏和系统标题栏适配鼠标所在显示器；自动模式优先完整显示，小屏幕或大字号时通过滚动条查看。右上角“界面大小”支持自动、90%、110%、125%，按显示器记忆；拖动到另一显示器后重新适配，已最大化的窗口保持最大化。

系统 DPI 缩放由 Qt 处理，界面适配使用逻辑像素。相机标签保持原始宽高比，机械臂视图由业务代码加入预留容器。

勾选“力矩”后的显示来源由 [main.py](main.py) 的 `SHOW_RAW_JOINT_TORQUES` 决定，修改后重启生效：

- `False`（当前值）：显示 UR 内部补偿关节力矩，尚未减去导纳启动偏置，也未经过滤波和死区。
- `True`：显示 `actual_current_as_torque` 的 raw 力矩。
- 取消勾选：显示 TCP 广义力。

补偿显示在普通控制时由后台读取；导纳运行时复用 `joint_torque` 载荷页的 `input_raw_nm`。没有有效样本时留空。该开关只选择显示源，不改变导纳输入或控制参数。

### 修改与预览界面

| 界面 | Designer 源文件 | 自动生成文件 |
| --- | --- | --- |
| 经典主界面 | [classic_window.ui](classic_window.ui) | [ui_classic_window.py](ui_classic_window.py) |
| 导纳调参与监视 | [admittance_window.ui](admittance_window.ui) | [ui_admittance_window.py](ui_admittance_window.py) |

先在 Qt Designer 编辑 `.ui`，再生成 Python 文件，最后修改业务绑定；不要手写生成文件中的控件、布局或静态文案。

```bash
conda activate urcontrol
python generate_ui.py
python preview_ui.py
python preview_ui.py --admittance
```

预览使用离线演示数据，不连接机器人、不打开相机、不创建采集或调试日志。`--scenario fixed-joint`、`preset-joint`、`teleop-joint` 等选项可查看相应演示；未开放的末端场景仅供离线显示。`window_display.py` 负责经典主界面的屏幕适配，`ui_widgets.py` 提供保持画面比例的相机标签。

## 导纳调参与监视

### 从主窗口使用六关节导纳（空法兰）

在项目目录运行：

```bash
conda activate urcontrol
python main.py
```

连接实机后，点击主窗口的“启动导纳”打开导纳界面。默认场景为定点柔顺、关节空间，无需主端在线或主从同步。该按钮始终可以打开窗口，实际启动按钮仅在机器人就绪、无其他运动且参数已应用时启用；重复点击只显示同一个窗口，不重置运行和参数。

默认已应用“六轴日常使用：固定参考，持续运行”。保持机械臂静止、无接触，在导纳界面点击“启动导纳”；显示运行中后即可轻推，松手后回到本轮参考位置附近。结束时点击“停止导纳”或主窗口全局停止，并等待停止确认。主程序中的导纳窗口关闭只隐藏，运行继续；断开、下电、切换 IP 和关闭主程序均在停止确认之后继续。

每次启动重新捕获实际关节角并采集载荷偏置，不需主从同步。当前部署核对零负载、零 TCP 偏移、NORMAL、静止反馈和 100% 速度滑条，不自动改写机器人配置或复位保护停止。六轴死区为 `[10,10,10,2,2,2]` N·m，命令速度均为 `0.03 rad/s`；J1 命令行程为相对锚点 ±0.40 rad，其余轴为 ±0.10 rad。请跟随让位速度轻推，明显变硬时撤力；命令限幅不是接触力限制。

日常入口默认关闭高频调试日志，可在窗口开启。联调日志按实际收到的 500 Hz 原始反馈记录，磁盘增长较快；停止确认不依赖磁盘记录。每次新建窗口加载部署默认，同一窗口隐藏再打开保留参数；修改后仍需应用。要跨应用重启复用自定义值，可导出并导入预设。

### 可选独立入口

`python admittance_app.py` 只加载导纳窗口与控制进程。打开后自动只读检查实机，首次地址为 `192.168.3.15`，可在窗口修改并检查，检查成功后记住地址。可用 `--robot-ip` 指定地址、`--log` 开启本次日志。此入口不依赖主程序、相机、夹爪或 UDP。关闭独立窗口会先停止，确认程序退出和六轴静止、完成日志收尾后退出；启动预检期间关闭会取消尚未上传的启动。

### 选择候选与应用参数

导纳窗口默认加载 `fixed_joint_deployment_config()`：六轴、固定参考、持续运行至停止，打开窗口本身不启动运动。原始 `snapshot()` 仍为固定保持，人工限时联调候选继续保留。切换候选或修改参数后，点击应用参数再启动：

| 场景 | 已开放的启用集合 |
| --- | --- |
| 定点柔顺 + 关节空间 | J1～J6 单轴、J1+J3、完整六轴 |
| 遥操作导纳 + 关节空间 | J1、J3、J1+J3 |
| 末端空间 | 保留配置和离线展示，启动未开放 |

两种关节场景都属于关节导纳，区别是参考位置固定还是来自主端。表中的 J1/J3 限制只适用于主端参考的导纳分支；普通 UDP 关节跟随继续使用六轴目标。

固定参考由机器人静止后捕获的本轮实际关节角确定；该模式无需主端，也不随 UDP 目标运动。禁用轴保持启动锚点。`DIAGNOSTIC_AXIS` 只选择逐周期统计与趋势观测轴，不决定哪些轴参与运动。六轴候选默认观测 J6。

界面编辑的 13 项包括关节或末端启用、M/D/K、目标载荷、连续死区及末端柔顺方向。编辑和导入只修改草稿；“应用参数”将草稿与候选的完整配置合并，校验后用于下一轮。存在未应用草稿时不能启动，运行及停止未确认期间参数锁定。“恢复默认”恢复当前候选；JSON 预设不包含运行候选，导入前须先选候选。

滤波、命令范围与速度、实际运动守卫、伺服周期、通信期限等在 [UR_Utils/admittance_config.py](UR_Utils/admittance_config.py) 中配置，修改后重启。界面、JSON 和采集均使用 SI 单位；目标显示可切换 °/rad、mm/m。

固定参考导纳要求启用轴 M/D/K 为正、目标载荷为零。启动前核对实际 TCP、负载、工作区域并保持静止无接触；程序不根据夹爪在线状态自动改写安装配置。停止执行减速停止，不先回锚点；下一次启动重新捕获锚点。

### 监视与导出

显示分别区分主端最新接收、主机已提交、机器人已采用，以及模型生成/提交和执行器采用的片段。`continuous.execution` 的命令具有执行身份；“采用/反馈差（异步）”不能当作同周期跟踪误差。目标 TCP 是本地模型估算，实际 TCP 来自机器人反馈。

当前遥测布局为 `telemetry_schema_version=7`，包含独立冻结的 `anchor`、`load`、`execution`、`timing`、`statistics`、`reference` 六页。运行时间整数改用毫秒，避免原微秒编码约 35.8 分钟溢出；伺服、模型及通信间隔仍使用微秒，计时浮点量仍用秒。各页保留独立身份和时刻，不能拼接成同步样本。逐周期统计只覆盖本轮观测轴，其余载荷页是抽样数据。历史布局 3～6 的日志保持原解释。

数值显示目标 20 Hz，曲线重绘目标 10 Hz，最多缓存 1200 个样本；支持暂停、清空、时间窗口和光标读数。隐藏、换轮或过期产生断点，显示操作不改变控制。

“导出草稿预设”使用版本 3，保存 13 项界面参数、来源和空间；旧版 v1/v2 不自动转换。“导出本轮生效记录”另存完整参数、修订号、运行身份、TCP/负载、锚点、模型与执行快照及停止信息，不能作为草稿预设导入。运行参数不热更新。

### 人工联调入口

以下命令供明确安排的现场联调使用，每次只执行一轮。`--environment` 是记录标签，实际连接目标由 `--robot-ip` 指定。实机联调前仍需现场确认空法兰、空间和接触条件。

```bash
# 只读状态，不启动运动。
python admittance_manual.py inspect --robot-ip 192.168.3.15 --output /tmp/urcontrol_robot_state.json

# 指定关节的双向无接触往返。
python admittance_manual.py joint-motion --robot-ip 192.168.3.15 --environment real_robot --label J2双向无接触 --empty-flange --no-contact --axis 2

# 六轴固定参考无接触保持，最长 60 秒。
python admittance_manual.py joint --robot-ip 192.168.3.15 --environment real_robot --label 六轴无接触 --empty-flange --no-contact --axes 1 2 3 4 5 6 --duration 20

# 打开六轴手推窗口，由现场点击“开始导纳”。
python admittance_contact_manual.py --robot-ip 192.168.3.15 --environment real_robot --label 六轴手推 --empty-flange --force-cue-n 25 --axes 1 2 3 4 5 6 --protocol timed_push --contact-point 空法兰末端 --contact-direction 轻推并改变法兰姿态
```

`timed_push` 流程为无接触 5 秒、手推提示 20 秒、撤力宽限 5 秒、随后观察，总计 60 秒。25 N 是末端力增量观测提示，不是接触力限制；提示时刻也不是精确物理接触时刻。现场接触位置、方向及反馈需如实记录。

其他入口包括 J1 首阶段 `stage-a`、J3 正向余弦轨迹 `j3-motion`，以及 `joint --axes 1 3 --reference-source teleop`。终端遥操作用 `target` 加六轴相对启动角度、`pause` 暂停新参考、`stop` 请求停止；终端目标不代表真实主端设备。当前候选参数与各项验证范围见 [实机联调记录](URControl_导纳实机联调_20261029.md)。

## 数据与导纳日志

`data/` 保存主界面 20 Hz 采集和视频。`logs/admittance/` 保存逐轮导纳调试日志，无需开启主界面采集；两个日常入口默认关闭，可显式开启。主程序中隐藏监视窗口不停止已开启的日志；独立入口关闭时先停止并完成日志收尾。软件不自动清理历史目录。

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 完整生效配置、脚本文本、日志/遥测版本、环境、字段说明、开关历史、结束摘要 |
| `records.jsonl` | 实际收发、机器人状态、输入、人工事件及停止过程的逐条记录 |
| `manual_final_status.json` 等 | 人工入口的停止确认；具体文件以该轮内容为准 |
| `descriptive_report.json`、`response_summary.json` 等 | 离线分析、响应指标及曲线对应摘要 |
| `operator_confirmation.json` | 现场反馈与参数取舍 |

日志格式为 `log_schema_version=3`，诊断订阅目标 500 Hz。主机、机器人及遥测页各保留自己的身份和时间，写盘丢弃、桥接缺口、主动停记与不完整收尾分别记录。`actual_TCP_force`、电流、补偿关节力矩和死区后 `drive` 含义不同；分析须按该轮元数据解释，不能用当前参数改写历史结论。

只读分析完整保留的轮次，不连接设备：

```bash
python analyze_admittance_log.py logs/admittance/具体轮次目录
python analyze_admittance_log.py --stage-a logs/admittance/J1第一轮目录 logs/admittance/J1第二轮目录 logs/admittance/J1第三轮目录
```

此前清理保留了关键参数依据、最终手推、遥操作机制与六轴故障定位数据；部分中间轮次仅保留元数据和摘要，不能重新做逐帧分析。本次部署新增的四轮完整原始日志及结果见 [部署证据汇总](logs/admittance/20261030_deployment/completion_summary.json)。`data/` 与导纳日志均不纳入 Git，移交原始证据时需单独携带对应目录。

## 代码与维护入口

| 位置 | 职责 |
| --- | --- |
| `main.py`、`classic_window.ui` | 主界面、普通控制、设备与采集协调 |
| `admittance_window.py`、`admittance_parameters.py`、`admittance_monitor.py`、`admittance_plot.py` | 导纳窗口、参数、快照解释与曲线 |
| `UR_Utils/admittance_config.py`、`UR_Utils/scripts/admittance_control.script` | 运行候选、约束和机器人模型/执行/遥测 |
| `UR_Utils/URAdmittanceController.py`、`admittance_lifecycle.py` | 导纳生命周期、控制交接与停止确认 |
| `UR_Utils/admittance_process.py`、`UR_Utils/admittance_transport.py`、`UR_Utils/admittance_telemetry.py` | 独立通信进程、RTDE 收发和遥测解码 |
| `admittance_logger.py`、`analyze_admittance_log.py` | 日志记录与离线分析 |
| `admittance_manual.py`、`admittance_contact_manual.py`、`preview_ui.py` | 单轮人工联调、接触提示及离线预览 |

维护时使用静态语法检查、UI 生成、配置与脚本离线渲染、人工预览，以及明确安排的人工联调。代码注释和文档字符串使用中文。设备运动约束、停止确认和输入契约在各自负责的边界处理。
