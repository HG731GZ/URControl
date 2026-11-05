# URControl

基于 PyQt5 的 UR 机械臂控制、状态显示和数据采集程序，支持 URSim、RealSense 相机及夹爪。主界面统一使用 classic，保留独立导纳调参与监视窗口。

六轴固定参考导纳提供无需主端的独立窗口。当前实现、参数、阶段记录和已知限制统一见 [项目现状](URControl_项目现状.md)；本文件说明安装、使用和维护方法。**遥操作导纳目前未做过任何测试，仅保留代码实现，没有测试或验收结论。**

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

连接期间按钮显示“连接中…”，收到首帧实时状态后显示“断开UR”；点击断开会先确认当前控制停止。实时状态由独立只读进程接收，避免 `ur-rtde` 初始化占用 Python 执行锁时阻塞收包。断流或接收失败会在消息区显示原因，按钮变为“重连UR”，由用户显式重连；程序不自动重连。

数据默认保存到 `data/`，采样频率与视频帧率均由 `DATA_COLLECT_FREQ` 配置。关闭窗口时程序等待写盘完成并保存元数据。

夹爪是可选设备。末端未安装夹爪时可直接连接 UR 并使用定点柔顺，无需启动主端。夹爪串口转发不可用时跳过夹爪连接；转发存在但夹爪不响应时，后台仅做低频只读探测，同一故障不重复刷屏。界面显示“未连接 / 无反馈”，不记录默认值或过期夹爪数据；取得有效反馈后恢复正常显示。暂停夹爪时只暂停目标下发，仍读取反馈。

主端离线时 UDP 反馈在后台退避重试，网络异常不退出程序或阻塞界面。机械臂反馈不依赖夹爪，夹爪不可用时三个反馈字段为 `NaN`，详见 [UDP 协议](URControl_项目现状.md#udp-protocol)。无末端负载测试仍使用 UR 端当前的负载与 TCP 配置，程序不根据夹爪通信状态自动改写这些配置。

## classic 界面与屏幕适配

主程序直接加载经典界面，保留原有控件位置和配色。窗口根据完整内容、状态栏和系统标题栏适配鼠标所在显示器；自动模式优先完整显示，小屏幕或大字号时通过滚动条查看。右上角“界面大小”支持自动、90%、110%、125%，按显示器记忆；拖动到另一显示器后重新适配，已最大化的窗口保持最大化。

系统 DPI 缩放由 Qt 处理，界面适配使用逻辑像素。相机标签保持原始宽高比，机械臂视图由业务代码加入预留容器。

勾选“力矩”后的显示来源由 [main.py](main.py) 的 `SHOW_RAW_JOINT_TORQUES` 决定，修改后重启生效：

- `False`（当前值）：显示 UR 内部补偿关节力矩，尚未减去导纳启动偏置，也未经过滤波和死区。
- `True`：显示 `actual_current_as_torque` 的 raw 力矩。
- 取消勾选：显示 TCP 广义力。

补偿显示在普通控制时由后台读取；导纳运行时复用 `joint_torque` 载荷页的 `input_raw_nm`。没有有效样本时留空。该开关只选择显示源，不改变导纳输入或控制参数。

主面板连接机械臂后，点击“开始遥测”即可独立记录 500 Hz 标准 RTDE 状态，静止、普通运动和导纳运行期间均可使用，再次点击停止并保存。按钮旁显示当前六轴中最大的偏差、碰撞比例；悬停可查看目录及记录数量，结束路径也写入消息区。记录保存在 `logs/telemetry/`，无需开启主界面采集。

遥测同时记录六轴 raw 力矩和 UR 补偿后力矩，单位均为 N·m，轴顺序为 J1～J6：

| 数据 | `records.jsonl` 中的位置 | 来源 |
| --- | --- | --- |
| raw 力矩 | `payload.raw.actual_current_as_torque` | 标准 RTDE 电流换算力矩，保留重力、摩擦等分量，与本包标准状态共用时间戳 |
| 补偿后力矩 | `payload.compensated_joint_torque.sample.values_nm` | UR `get_joint_torques()`，未减导纳启动偏置、未滤波、未过死区 |

补偿力矩与“力矩”显示共用后台读取，记录不依赖显示开关。普通控制时通过当前 `getJointTorques` 通道读取，保存主机调用起止时刻及关联 RTDE 时间；导纳运行时复用载荷页，保存 `run_id`、`model_sample_seq`、`page_seq` 和相对脚本起点的 `sample_time_s`。`status=available` 表示样本有效；程序未运行、控制交接或缺少有效载荷页时，记录 `status=unavailable`、`sample=null` 和原因。读取错误明确报告并结束本次遥测。补偿缓存可重复出现在相邻 RTDE 帧中，更新频率以来源身份为准，不视为与 raw 同步的 500 Hz 测量。补偿定义见 [UR 官方说明](https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-scriptmanual/all_scripts/get_joint_torques.htm)。

勾选主面板“导纳诊断日志”则开启本轮导纳的原始反馈及内部模型、片段、伺服计时记录，可在启动前选择或运行中切换，与导纳窗口日志开关同步，文件保存在 `logs/admittance/`。两个记录入口均不启动运动；单独的标准遥测不包含导纳内部寄存器页。

当前配方包含 `joint_position_deviation_ratios` 和 `collision_detection_ratios`，要求控制器支持这两个字段（本项目实机为 PolyScope 5.26.3，URSim 也须使用支持字段的版本）。两项均保存 J1～J6 的原始比例，定义见 [UR 官方 RTDE 指南](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html)。字段不可用时明确报错，不自动删减配方。

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

| 场景 | 代码允许的启用集合 |
| --- | --- |
| 定点柔顺 + 关节空间 | J1～J6 单轴、J1+J3、完整六轴 |
| 遥操作导纳 + 关节空间 | J1、J3、J1+J3；目前未做过任何测试 |
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

其他固定参考入口包括 J1 首阶段 `stage-a`、J3 正向余弦轨迹 `j3-motion`。代码还保留 `joint --axes 1 3 --reference-source teleop` 及终端 `target`、`pause`、`stop` 指令，但遥操作导纳目前未做过任何测试，不能将入口存在理解为已经验证。当前参数与实际验证范围见 [项目现状](URControl_项目现状.md#current-status)。

## 数据与导纳日志

`data/` 保存主界面 20 Hz 采集和视频。`logs/admittance/` 保存逐轮导纳调试日志，无需开启主界面采集；两个日常导纳入口默认关闭日志，可显式开启。主程序中隐藏监视窗口不停止已开启的日志；独立入口关闭时先停止并完成日志收尾。`logs/telemetry/` 保存主面板独立遥测，同样使用 `metadata.json` 和 `records.jsonl`；`kind=robot_telemetry` 的 `payload.raw` 为标准状态，`payload.compensated_joint_torque` 为带来源身份的补偿力矩缓存。结束摘要含采样缺口、各轴比例最大值、控制器实时线程耗时、两种力矩的各轴绝对峰值，以及附带有效/缺测补偿缓存的 RTDE 帧数；这些帧数不是独立补偿采样次数。断开机器人或关闭主程序时会完成遥测收尾。软件不自动清理历史目录。

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 完整生效配置、脚本文本、日志/遥测版本、环境、字段说明、开关历史、结束摘要 |
| `records.jsonl` | 实际收发、机器人状态、输入、人工事件及停止过程的逐条记录 |
| `manual_final_status.json` 等 | 人工入口的停止确认；具体文件以该轮内容为准 |
| `descriptive_report.json`、`response_summary.json` 等 | 离线分析、响应指标及曲线对应摘要 |
| `operator_confirmation.json` | 现场反馈与参数取舍 |

日志格式为 `log_schema_version=3`，诊断订阅目标 500 Hz。主机、机器人及遥测页各保留自己的身份和时间，写盘丢弃、桥接缺口、主动停记与不完整收尾分别记录。`actual_TCP_force`、电流、补偿关节力矩和死区后 `drive` 含义不同；分析须按该轮元数据解释，不能用当前参数改写历史结论。

新增比例和 raw 力矩不改变导纳寄存器页版本 7；完整输出配方为 73 字段、1048 B，其中标准字段 25 项。导纳分析报告的“全阶段UR控制器诊断”汇总两种比例及 `actual_execution_time`；后者单位为 ms，是控制器实时线程耗时，不能当作导纳模型计算时间。“关节力矩_Nm”分别汇总 raw 力矩和运行载荷页的 UR 补偿力矩；后者仍位于 `payload.control.load.input_raw_nm`，仅 `load_source=joint_torque` 时适用。旧日志没有新增字段时显示缺测。

只读分析完整保留的轮次，不连接设备：

```bash
python analyze_admittance_log.py logs/admittance/20261030_175645_275707_standalone_fixed_1793354205274867198
```

2026-11-01 精简旧版日志后，旧版仅保留 2026-10-30 17:50 的六轴 C159A0 故障轮次和 17:56 的六轴手推接受轮次，各自保留完整原始日志、元数据、停止状态和分析记录。具体目录见 [日志留存说明](URControl_项目现状.md#retained-logs)，解释与适用范围见 [部署记录](URControl_项目现状.md#deployment-evidence)。其余旧版轮次、独立汇总及预览附件已删除，历史结论统一保存在项目现状文档，不能再逐帧复核。2026-11-02 新增的 5.26.3 无接触、手推及主面板遥测联调另见 [升级验证](URControl_项目现状.md#upgrade-5263)。`data/`、导纳和独立遥测日志均不纳入 Git，移交原始证据时需单独携带对应目录。

## 代码与维护入口

| 位置 | 职责 |
| --- | --- |
| `main.py`、`classic_window.ui` | 主界面、普通控制、设备与采集协调 |
| `robot_telemetry.py` | 主面板独立只读 RTDE 记录、状态与收尾摘要 |
| `admittance_window.py`、`admittance_parameters.py`、`admittance_monitor.py`、`admittance_plot.py` | 导纳窗口、参数、快照解释与曲线 |
| `UR_Utils/admittance_config.py`、`UR_Utils/scripts/admittance_control.script` | 运行候选、约束和机器人模型/执行/遥测 |
| `UR_Utils/URAdmittanceController.py`、`admittance_lifecycle.py` | 导纳生命周期、控制交接与停止确认 |
| `UR_Utils/admittance_process.py`、`UR_Utils/admittance_transport.py`、`UR_Utils/admittance_telemetry.py` | 独立通信进程、RTDE 收发和遥测解码 |
| `admittance_logger.py`、`analyze_admittance_log.py` | 日志记录与离线分析 |
| `admittance_manual.py`、`admittance_contact_manual.py`、`preview_ui.py` | 单轮人工联调、接触提示及离线预览 |

维护时使用静态语法检查、UI 生成、配置与脚本离线渲染、人工预览，以及明确安排的人工联调。代码注释和文档字符串使用中文。设备运动约束、停止确认和输入契约在各自负责的边界处理。

## 六轴关节阻抗（direct_torque）

主窗口新增“关节阻抗”，打开非模态窗口；关闭子窗口只隐藏，主窗口全局停止仍有效。六轴始终执行 `K*(q_ref-q)+D*(qd_ref-qd)`，机器人端固定 500 Hz，摩擦补偿两组数组均固定为 **0**，不额外叠加重力。当前 K 为 `[2400, 3200, 2600, 1000, 800, 1000]`，D 为 `[150, 240, 180, 60, 40, 40]`，力矩变化率为 `[300, 300, 200, 200, 200, 200]` N·m/s。2026-11-04 完成前三轴追加联调，并按后续要求调整守卫，默认值和 [唯一六轴预设](presets/impedance_real_robot.json) 当前为：

| 关节 | 参考速度上限 rad/s | 实际速度停止阈值 rad/s | 往返目标速度预算 rad/s | 力矩上限 N·m | 参考整形加速度 rad/s² |
| --- | ---: | ---: | ---: | ---: | ---: |
| J1 | 0.823529 | 1.2 | 0.7 | 24 | 30 |
| J2 | 0.705882 | 1.2 | 0.6 | 24 | 30 |
| J3 | 0.705882 | 1.2 | 0.6 | 18 | 30 |
| J4～J6 | 1.0 | 1.2 | 0.85 | 6 | 60 |

统一 `stopj` 减速度为 8 rad/s²；六轴轨迹/阶跃相对启动点包络均为 ±0.22 rad。此前 ±0.2 rad 联调在当时的守卫参数下使用原始中心各轴 ±0.3 rad 净空；放宽守卫后需重新计算停止余量，不能沿用该净空结论。30/60 rad/s² 是参考整形上限，已完成往返轨迹源的峰值加速度约为 J1 3.95、J2/J3 2.91、J4～J6 5.68 rad/s²。往返生成器保留 0.85 速度预算；实际速度停止阈值会触发停止，不是将实际速度连续钳在该值内的限速器。

当前循环间隔守卫为 **超过 10 ms 停止**，正常控制仍为 **2 ms / 500 Hz**。已删除持续力矩饱和及生效参考与实际位置偏差的停机条件，适用于轨迹、阶跃、遥操作。位置误差完整参与 K/D 力矩计算并用于显示和统计，力矩仍受幅值/变化率限制，饱和状态继续记录。以上守卫变更仅完成离线核对，没有新增实机或接触验证，见[守卫调整记录](URControl_项目现状.md#impedance-guards-20261104)。重启程序后使用当前默认或唯一预设；旧导出预设含已删除的 `SATURATION_TIMEOUT_S` 或 `MAX_ERROR_RAD`，不再符合当前参数契约。

最终完整参数组合下，J1/J2/J3 的 ±0.2 rad 单轴复核实测峰速为 0.795/0.708/0.764 rad/s，运动段生成目标 RMS 为 0.598°/0.462°/0.561°，均未达到 0.2° 努力目标；双向小阶跃最终误差为 0.027°～0.107°，均进入并保持 ±0.2° 稳定带。J1 0.85 rad/s 和 J2 0.7 rad/s 的失败候选未采用。详见 [前三轴追加联调](URControl_项目现状.md#impedance-proximal-20261104)。

J4/J5/J6 保留早前 ±0.2 rad 往返实测峰速 0.901/0.911/0.967 rad/s、运动段生成目标 RMS 0.467°/0.497°/0.450° 的参数，见 [首次分档提速记录](URControl_项目现状.md#impedance-speed-20261104)。上述峰速包含追赶波动，不代表恒速能力或已找到机械臂的绝对最高速度。仅覆盖当前 1.12 kg 工具、当前姿态附近的逐轴短时运动；后三轴未在新前三轴保持参数下重跑，也未完成新力矩上限下的完整双向阶跃。六轴同时高速及长时间遥操作尚未验证。旧进程需要重启，或在单关节窗口“恢复默认草稿”后“应用参数”；不会自动覆盖已有草稿。

**URSim 的 direct_torque 不驱动模拟关节运动**，这是 [UR 官方示例说明](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_client_library/doc/examples/direct_torque_control.html)中的已知限制。因此 URSim 日志用于检查脚本、时序、目标源和停止，不代表真实跟踪性能。实机性能以各轴原始实测日志为准。

操作顺序：

1. 在示教器正确设置当前工具的负载，连接并准备好机器人，停止普通运动。阻抗直接使用机器人当前负载设置，不与预设质量匹配；实际负载仍写入日志。当前预设要求零 TCP、NORMAL、静止、速度滑条 100%；程序不会改写安装参数。
2. 打开“关节阻抗”加载 `trajectories/impedance_j1.csv`，该窗口仅负责文件轨迹与监视。单关节调参使用“单关节阻抗调参”非模态窗口；内部和 CSV 使用 rad。
3. 在单关节窗口逐轴修改 K/D、指令力矩/变化率、参考速度/加速度后点击“应用参数”；此操作可离线完成且不启动运动。其他守卫与安装参数可通过完整 JSON 预设编辑；导入后也要应用。仅保留最终六轴预设 `presets/impedance_real_robot.json`，与主 UI 默认一致；旧阶段与 URSim preset 已删除。
4. 设置“轨迹播放采样率（行/秒）”，预览时长及曲线，再点击“开始阻抗”。修改参数、文件或频率必须先停止。相对轨迹只绑定一次机器人确认的锚点；绝对轨迹先显式“移动到轨迹起点”。
5. 文件结束后，参考到达末点且实际低速保持后自动停止，或点击停止。这表示计划执行结束，是否到位须看实际误差。停止确认要求新鲜反馈、程序退出、连续静止；失败则保持控制权，禁止其他运动。
6. 主 UI 单独点击“遥操作阻抗”。主端为新鲜 `JOINT_TRACK`、机器人可接管、参数已应用且执行权空闲时，可以不预先同步直接开启；参考从实际位置连续追赶最新目标。“同步到主端”是独立的一次定位，使用点击时冻结的目标，主端后续变化不改终点。同步期间断流、模式改变或反馈失效会停止。停止确认前不释放执行权，恢复来包不自动重启。文件模式不驱动夹爪。

每轮自动记录到 `logs/impedance/<run>/`，包括完整参数、版本、安装配置、轨迹副本、渲染脚本、源/采纳身份、100 Hz 同周期冻结样本、500 Hz 实际收到的标准遥测、停止事件和摘要。遥操作窗口按 20 Hz 显示最新主端和实际角度，隐藏时暂停其绘图；界面源曲线是异步显示，离线分析才按 `adopted_reference_seq` 对齐源目标。`tau_cmd` 不含 UR 内部补偿，不等于电机总力矩。

```bash
conda activate urcontrol
# 离线预览；不连接设备
python preview_ui.py --impedance
# 生成、执行和分析；执行入口默认只连接 127.0.0.1
python generate_impedance_trajectory.py /tmp/j1.csv --axis 1 --amplitude .05 --segment 2
python run_impedance_trajectory.py /tmp/j1.csv --play-hz 100
python analyze_impedance_log.py logs/impedance/<run>
# 标准文件采样率为 100；50/100/200 行每秒分别为 0.5/1/2 倍速
```

早期 URSim 验证结论保留在 [项目现状](URControl_项目现状.md)。普通运动、导纳、阻抗共用执行权检查；共享传输保持原导纳导入兼容，暂停叠加发送异常由独立 Dashboard/只读 RTDE 监督收尾。停止后不自动上传旧位置控制脚本。

### 单轴调参测试

在阻抗窗口点击“单轴调参测试”。默认参数已从主入口直接带入；“恢复默认草稿”载入同一组默认值并保留当前所选关节。早期六轴调参使用空法兰；当前分档参数的新增证据使用 1.12 kg 工具，适用范围见上表和分档提速记录。

选择“单次阶跃（J1～J6）”时，建议偏置 2°，先保持 1 秒，再直接将位置参考切换到 `当前位置＋偏置`，目标速度为零。阶跃绕过平滑参考整形，保留实际位置、实际速度、力矩幅值/变化率、心跳和停止确认；观察默认 5 秒后收尾，不自动返回起点。六轴单次阶跃幅度统一最多 0.1 rad，正负方向均适用（约 5.72958°；当前角度输入保留三位小数，可输入 5.729°，5.730° 已略超上限），同时须满足源跳变和位置范围约束。参考速度和参考加速度在阶跃模式下不参与参考生成，但可以独立编辑并应用，供下一轮遥操作使用。

选择“平滑往返”时，可选择 J1～J6，设置最大速度、带符号偏置以及 K/D 等参数。其余轴保持生成时的角度；最大速度输入同时遵守参考上限和实际速度停止阈值的 0.85 预算。

点击“生成”读取当前六轴角度并预览。往返模式依次经过 `当前位置 → 当前位置＋偏置 → 当前位置－偏置 → 当前位置`。三段五次插值在每个端点速度和加速度为零；自动按最大速度及加速度预算确定时长，速度较低或行程较小时可能达不到速度上限。轨迹采样固定 100 Hz，时长最多 600 秒。

点击“执行”应用当前调参配置，并通过既有阻抗后端执行已生成的绝对目标。起点超出容差须重新生成；改变模式、目标轴或任何参数也须重新生成。运行中参数锁定，可点击“停止”。生成目标必须满足已配置的运动包络，不再根据偏置自动扩大包络。主阻抗窗口的 J1～J6 仅选择观测轴。参考速度/加速度由共享配置提供，以 °/s、°/s² 编辑，内部为 rad/s、rad/s²；往返生成器在参考上限的 0.85 预算内生成，不再反向改写参考速度。

运行时显示生效参考和实际角度；结束后从完整日志报告同周期参考误差、按采纳身份对齐的生成目标误差和最终目标误差，单位为度。平滑跟踪以运动阶段的生成目标 RMS 为性能目标：J1～J3 为 0.2°，J4～J6 放宽到 0.5°；去掉起点和末点保持，生效参考 RMS 及峰值误差单列，不以静止保持样本稀释运动误差。阶跃另列 10%→90% 上升时间、超调、稳定时间和末 1 秒平均误差；最终位置误差及稳定带均采用 ±0.2°（0.00349066 rad），稳定时间要求此后保持在带内且至少观察 1 秒。没有进入时明确显示未稳定进入，时间指标分辨率约 10 ms。数据不完整或提前结束不作达标判定。详细报告区可以滚动查看。这些角度只参与性能评价，超出不会触发停机；单次阶跃的独立幅度限制继续保留。

界面报告为 `tuning_report.json`；`python analyze_impedance_tuning.py logs/impedance/<run>` 输出 `response_report.json` 和 `response.png`。阶跃目标保存为 `step.json`，平滑轨迹保存为 `trajectory.csv`。提前停止、缺口和日志丢弃明确标注；绘图抽样不改变完整采样统计。不计算哈希、不运行回归测试。

`python preview_ui.py --impedance-tuning` 可离线预览新窗口。现场单轮入口 `impedance_manual.py` 复用相同后端，并用同一份 `--workspace-state` 约束整次调参的原始工作区；参数说明可通过 `python impedance_manual.py --help` 查看。提速联调可显式指定 `--reference-speed-limit`、`--actual-speed-limit` 和 `--stop-acceleration`；入口按实际速度停止阈值扣除机器人端三周期守卫反应距离、理论制动距离与锚点余量，不再固定预留 0.03 rad。`--workspace-radius` 须取现场确认的净空，默认 0.2 rad，支持到 0.3 rad。`--stop-at-speed` 用于一次现场主动停止，保存触发快照和独立停止后反馈；理论距离不能替代实际制动验证。`--mode move --target-offset 0` 是显式安排的单段低速平滑回中心，不会在故障后自动重启。


### 遥操作阻抗独立窗口

主 UI 的“遥操作阻抗”可在未连接机器人时打开。主端目标和半透明模型直接取同一份 UDP 快照的 `command.q_arm`，实体模型取新鲜实际反馈，后端 `q_ref` 留在原监视与日志中。六轴角度显示为度、保留四位小数；无效数值显示“—”，旧姿态标为“已过期”，首次无数据的模型隐藏。两个窗口复用模型、具有独立 OpenGL 上下文和相机。

完整六轴参数由主窗口的 `ImpedanceIntegration` 持有，单关节窗口是唯一编辑入口，切换关节保留其他轴草稿。应用、预设导入/导出、恢复默认和放弃修改均在该窗口完成。导入及恢复默认仅改变草稿；未应用草稿阻止遥操作启动，运行及停止未确认期间禁止编辑。每轮冻结配置与应用版本，写入同一阻抗日志元数据；重开窗口不重置参数，应用重启不隐式持久化草稿。

遥操作取消 0.003 rad 首目标对齐门槛，以及相对启动锚点的调参包络；轨迹和阶跃保留原检查。遥操作仍保留 ±2π 绝对关节范围、参考速度/加速度、实际速度、力矩幅值/变化率、源跳变、通信期限及停止确认。参考与实际位置可以持续分离，不因位置偏差或持续饱和单独停机；参考整形仍从实际位置开始连续追赶。首次主端目标建立输入基线，后续新帧按 `MAX_SOURCE_STEP_RAD` 检查。未对齐启动不调用 `moveJ`，也不把主端目标直接赋给 `q_ref`。

UDP 有界排空后只发布本批最新有效位置；持续积压或线程停顿会产生终止事件，旧包排空后不补走。模式退出及输入错误拥有单调事件身份，不能被随后的位置包撤销。适配器在后台启动交接期间接入，只在 IPC 接受后推进提交身份；IPC 保持单格覆盖。主端指令为 133 字节，同一帧携带六轴关节角和速度。从端反馈为 85 字节。源身份和时刻均由从端接收时产生。

遥操作阻抗的 v3 预设默认使用「限速位置跟随」。单关节调参窗口可选择六轴共用的参考模式，切换关节可编辑该轴的速度上限 V、加速度预算 A 和跟随时间常数 T。机器人每 2 ms 向主端位置限速推进，并用 `max(T, 2V/A)` 平滑参考。J2/J3 初值为 0.35 rad/s、6 rad/s²、0.12 s。其余轴沿用原速度与加速度预算，T 初值为 0.05 s。载入 v2 预设会选中「同帧位置＋速度」，使用原带宽、速度和加速度参数。导出草稿时保存为 v3。

从 v3 起，遥操作日志在 `reference` 中记录主端原始速度和送给模型的速度，在运行期冻结样本中记录 `qd_ref`。限速位置跟随的模型输入速度为零。UDP 接收和适配器提交写入 `source_timing.jsonl`，IPC 读取、RTDE 发送及采纳写入 `records.jsonl`。输出整数寄存器 47 报告最新采纳的提交序号，采纳时刻取首次观察到该序号的 500 Hz 机器人反馈时刻。主机侧时刻使用本机单调钟。运行 `python analyze_impedance_log.py <本轮目录> --output <分析目录>` 可按源位置变化与保持分别查看新源间隔、最长间隔的两个源序号、重复次数、源年龄和最大位置步长。

遥操作阻抗日志 v4 以 500 Hz 订阅从端 RTDE 的 `actual_TCP_force`。每个收到的 `standard` 帧把 Fx、Fy、Fz（N）和 Mx、My、Mz（N·m）与同帧的 `timestamp`、`actual_q`、`actual_qd` 写入 `records.jsonl`。三轴力与三轴力矩均保留控制器原值。

离线分析生成的 `tracking.pdf` 和 `tracking.png` 在关节跟踪曲线下方分别绘制 TCP 力与力矩，以首个跟踪快照的 RTDE 观测时刻为时间零点。力曲线使用该时间范围内的原始标准帧，反馈缺帧处断线。`tracking.csv` 按冻结关节快照逐行导出，用 `sample.robot_timestamp_s` 精确匹配 `standard.timestamp`，缺帧留空。`robot_timestamp_s` 是首次读到该快照的反馈帧时刻，`sample_time_s` 是脚本冻结关节量时相对本轮起点的时刻。旧版日志继续输出原有关节图表。

断流按最近一次有效 UDP 收包的本机单调时间计时，超过 **250 ms** 无新有效目标则停止。持续收到完全相同的关节角度也会推进接收身份并刷新计时，允许主端保持不动；仅反复读取缓存或将同一来源身份转发给机器人不会刷新来源年龄。当前协议没有主端时间戳或发送序号，不能据此区分相同内容来自新采样还是发送端重复旧内容。

关闭遥操作窗口只隐藏，同步和遥操作继续；隐藏运行时主入口显示“遥操作阻抗运行中（窗口已隐藏）”。窗口停止仅作用于自己的同步或遥操作；主 UI 停止保持全局含义，断开、换 IP、下电、退出均等待停止确认。没有新增负载检查或设置流程。

```bash
# 离线预览，不连接设备；可选状态见 --help
python preview_ui.py --teleop-impedance --teleop-state ready
python preview_ui.py --teleop-impedance --teleop-state stale
python preview_ui.py --impedance-tuning
```

现场单轮入口 `teleop_impedance_manual.py` 复用正式窗口、共享配置、集成层、UDP 及控制进程，不初始化相机或夹爪。必须提供本次已确认的工作区状态文件、机器人地址、场景和独立结果路径；它会真实运动，仅用于已安排现场监护的联调。每轮通过本地及控制器安全查询检查端点，保留默认控制参数；结果保存实际范围、停止状态和日志目录。详见项目现状中的本轮验收记录。
