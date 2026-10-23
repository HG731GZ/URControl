# URControl

基于 PyQt5 的 UR 机械臂控制、状态显示和数据采集程序，支持 URSim、RealSense 相机及夹爪。

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
| [Universal Robots 官方 RTDE 客户端](https://github.com/UniversalRobots/RTDE_Python_Client_Library)（发行包名 `UrRtde`） | `rtde`、`rtde.rtde` | 订阅关节力矩 |

代码整理后，官方 `rtde` 客户端从按需导入改为程序启动时导入。因此，只安装了 `ur_rtde` / `ur-rtde` 的电脑可能在启动时出现：

```text
ModuleNotFoundError: No module named 'rtde'
```

如果原环境其余依赖都已安装，只需在**运行项目的同一个环境**中补装官方客户端：

```bash
conda activate urcontrol
python -m pip install "git+https://github.com/UniversalRobots/RTDE_Python_Client_Library.git@v2.7.12"
python -c "from rtde.rtde import RTDE, RTDEException; import rtde_control, rtde_receive, rtde_io; print('两套 RTDE 客户端均可导入')"
```

这里固定官方版本 `v2.7.12`，与当前已验证环境一致，安装方式见[官方说明](https://github.com/UniversalRobots/RTDE_Python_Client_Library#using-rtde-library)。使用 `python -m pip` 可以明确把包装入该 Python 对应的环境。

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

夹爪是可选设备。末端未安装夹爪时可直接连接 UR 并使用主动柔顺，无需启动主端。夹爪串口转发不可用时跳过夹爪连接；转发存在但夹爪不响应时，后台仅做低频只读探测，同一故障不重复刷屏。界面显示“未连接 / 无反馈”，不记录默认值或过期夹爪数据；取得有效反馈后恢复正常显示。暂停夹爪时只暂停目标下发，仍读取反馈。

主端离线时 UDP 反馈在后台退避重试，网络异常不退出程序或阻塞界面。机械臂反馈不依赖夹爪，夹爪不可用时三个反馈字段为 `NaN`，详见 [UDP 协议](UR_Utils/UDP_Protocol.md)。无末端负载测试仍使用 UR 端当前的负载与 TCP 配置，程序不根据夹爪通信状态自动改写这些配置。

## 界面与多显示器

新版界面采用布局管理器和可拖动分隔条。左侧切换关节/TCP 控制，以及 UDP、力/力矩、夹爪和运行日志；右侧显示机械臂和两路相机。日志标签会提示未读消息数。底部的限速、采集、停止和下电按钮始终位于固定操作栏。窗口较小时，左侧控制区可以滚动。停止、下电、采集和主要控制按钮使用更大的尺寸和字号；禁用控件使用灰底、灰字和虚线边框。XYZ 点动按钮保留原来的红、绿、蓝颜色，旋转轴使用对应深色；禁用时仅保留轴颜色标记。

窗口初次打开时使用普通窗口，新版以 `.ui` 中的 1360×800 为基准，再适配鼠标所在屏幕，通常占屏幕宽度约七成；不会自动最大化或全屏。字号按屏幕的可用逻辑尺寸适配：4K、200% 缩放时更紧凑，2K、100% 时使用更大的字号。拖到另一台显示器后会重新适配，并保留输入内容和当前标签页；已经手动最大化的窗口保持最大化。右上角“界面大小”可在自动大小基础上选择 90%、110%、125%，选择按显示器和界面模式分别记忆。

“界面 → 经典界面（改版前）”可立即回到原来的布局，“界面 → 新版界面”可切回，程序会记住最后的选择。切换复用同一批业务控件，保留连接、输入值、采集状态和机器人视图，不发送运动命令。经典界面保留原有控件位置和配色，内容随字号等比例缩放；打开、切换屏幕或调整界面大小时，窗口会根据完整内容（包括底部相机）、菜单栏和系统标题栏自动调整尺寸。自动模式优先完整显示，并保留桌面边距；屏幕较小或选择大字导致内容放不下时，通过滚动条查看。新版仍使用较小的默认窗口。

系统 DPI 缩放由 Qt 处理，字号适配使用逻辑尺寸。逻辑像素与物理像素的关系见 [Qt High DPI 说明](https://doc.qt.io/qt-6.8/highdpi.html)。实际跨屏行为取决于桌面环境向 Qt 提供的屏幕信息。

### 修改界面

`main_window.ui` 是新版布局、控件、配色和基准字号的源文件，`classic_window.ui` 保存经典界面。用 Qt Designer 编辑对应的 `.ui`，再在项目根目录执行：

```bash
conda activate urcontrol
python generate_ui.py
python preview_ui.py
```

`preview_ui.py` 支持两种主界面的预览与切换，`python preview_ui.py --admittance` 打开独立导纳窗口的离线演示，不连接机器人、不打开相机，也不创建采集文件。确认后用 `python main.py` 正常运行。独立导纳窗口的静态结构位于 `admittance_window.ui`。`generate_ui.py` 使用当前 Python 环境的 PyQt5 生成器统一生成三份 UI，并转换头部说明为中文；不要直接编辑自动生成的 `ui_main_window.py`、`ui_classic_window.py` 或 `ui_admittance_window.py`。

相机标签在 Designer 中提升为 `ui_widgets.AspectRatioLabel`，运行时保持横屏/竖屏画面的原始比例。`window_display.py` 负责屏幕适配、字号偏好、日志提示和力矩字段标签，`window_layouts.py` 负责复用业务控件并切换布局；机械臂视图在运行时加入 `.ui` 中预留的 `verticalLayout_RobotViewport`。

导纳控制的设计、文件改动和人工联调步骤见 [URControl 导纳控制规划](URControl_导纳控制规划.md)。

### 导纳调参与监视

新版和经典主界面的“导纳调参与监视…”打开同一个独立非模态窗口，首次打开不启动运动。窗口可独立移动、最大化和放在另一块屏幕。关闭该窗口只隐藏监视并暂停显示采样，机器人控制、主端提交和原有采集继续运行；重新打开保留草稿和曲线缓存，隐藏期间用断点表示。切换主界面布局、暂停或清空曲线也不改变运动。

窗口显式选择“遥操作导纳 / 主动柔顺”和“关节 / 末端”空间。启动后固定场景和空间，停止确认后才能切换。遥操作要求新鲜的主端关节跟踪数据，先保持主端静止，点击“同步主从”，等待同步完成且误差满足准备参数中的容差，再启动。主动柔顺无需主端在线，即使 UDP 持续到达，也只做主端监视，不跟随其目标、不驱动夹爪；断流不会自动切换场景或恢复运动。

参数页只保留常规导纳调节的 13 项配置，并随关节 / 末端空间显示对应参数：

- 关节：六轴启用、虚拟惯量 M、阻尼 D、恢复刚度 K、目标力矩、连续死区。
- 末端：柔顺方向、六轴启用、虚拟质量 / 惯量 M、阻尼 D、恢复刚度 K、目标载荷、连续死区。

恢复刚度 K 仅在遥操作场景使用。载荷滤波、偏移与速度限幅、伺服参数、通信频率、超时、启动条件及寄存器等其余配置仍在 [UR_Utils/admittance_config.py](UR_Utils/admittance_config.py) 中调整，修改后重启程序生效。界面默认值也来自该文件；“恢复默认”仅重置这 13 项的草稿，不改写 Python 文件。

编辑和导入仅修改界面草稿；“应用参数”把 13 项草稿与当前程序加载的 Python 配置合并，使用现有校验入口检查完整 51 项配置，更新下一轮准备值并递增修订号。预设不能覆盖界面之外的配置。存在未应用草稿时不能启动。启动时传入完整准备值副本，后端覆盖控制器的关节范围后返回本轮生效值；差异页仅展示关键参数的草稿、准备值和生效值。启动、偏置采样、运行及停止未确认期间参数锁定。

参数编辑、JSON 文件和采集保留 SI 单位。目标显示可以切换 ° / rad、mm / m；姿态为旋转向量 rx、ry、rz，角度保持连续圈数。`tool_start` 表示启动时固定的工具方向，柔顺中心仍在当前 TCP。主动柔顺使用质量—阻尼模型，本轮恢复刚度为 0。

建议操作顺序：

1. 连接机器人并确认就绪，在独立窗口选择场景与空间。
2. 修改并应用参数；遥操作执行同步主从。
3. 核对当前 TCP、负载及工作区域，保持机器人静止、无外部接触，再启动导纳。
4. 对照主端已采用、名义参考、导纳最终目标和实际反馈，观察分开显示的误差、驱动力及导纳状态。
5. 点击“停止导纳”或主界面全局停止，等待停止确认后再调整下一轮参数或导出生效记录。

主端“最新接收”“主机已提交”“机器人已采用”分开展示；接收计数是本机有效帧计数，并非主端硬件序号。机器人回显采用序号经原后端匹配后才关联已采用目标。核心最终目标来自导纳专用快照的 `q_cmd`，不是主界面实时协议的 `q_target`。运行误差使用同一次状态快照；停止后的目标标记为上一轮，不与空闲实时反馈重新计算误差。

目标 TCP 通过本地 UR5e 模型及本轮 TCP 偏置计算，明确标为“模型估算”；实际 TCP 来自机器人反馈。未取得本轮 TCP 配置时显示等待状态。模型估算与实际 TCP 的差包含模型和校准差异，不能视为精确控制误差。机器人未上报的候选目标、内部偏置、滤波量和遥操作偏移速度，以及主端未提供的编码器映射和增益，会明确标记缺失，不填零。

数值显示目标 20 Hz、曲线重绘目标 10 Hz，最多缓存 1200 个显示样本，按各自时间戳和计数去重；支持 5 / 15 / 30 / 60 秒窗口、暂停、清空和光标读数。数据过期、隐藏和换轮均产生断点。配置频率与计数估算速率分开显示，主机心跳发送成功不表示机器人已收到，也不将计数进度差称为网络延迟或真实丢包率。

“导出草稿预设”保存 UTF-8 JSON，格式版本 `schema_version` 为 2，完整保存 13 个界面参数及场景、空间和名称。导入严格检查版本、完整字段和现有参数约束，仅载入草稿；应用后才用于下一轮。旧版 v1 的 46 项预设不兼容，导入时会明确拒绝；请重新设置界面关键参数并导出 v2 预设，其余参数在 Python 文件中配置，不自动迁移旧值。“导出本轮生效记录”继续使用独立运行记录格式（版本 1），保存全部 51 项实际参数、修订号、开始结束时间、场景空间、TCP/负载和退出信息，其中原有 5 项只读参数仍单独列出。运行记录不能当作调参预设导入。这些操作不覆盖 Python 默认配置文件，也不触发运动。

启停连接交接和停止等待在专用 Qt 工作线程执行，界面继续刷新；所有界面状态、定时器和采集元数据仍由主线程管理。停止请求贯穿排队、交接及偏置采样。停止超时或交接失败后保持控制占用，允许再次显式停止，不自动恢复其他运动。下电、断开、切换 IP 和退出主程序在同一停止确认之后继续。

机器人端导纳公式、脚本与寄存器约定保持原实现：专用 RTDE 使用上半区输入整数 24–28、输入浮点 24–29，以及输出整数 24–28、输出浮点 24–47，不增加运动输出通道。默认参数仍只是低速联调起点。

采集继续由原 `DataCollector` 独立以 20 Hz 执行。除已有导纳组外，新增主机提交目标、提交来源序号、UDP 最新接收监视及主机收发计数；主端监视组仅记录接收事实，主动柔顺不生成“已采用主端”目标。每轮元数据以实际生效参数为准，即使在取得 TCP 配置之前失败，也保存已经接受的参数。显示窗口隐藏不影响采集。

离线预览：

```bash
conda activate urcontrol
python generate_ui.py
python preview_ui.py --admittance
```

可用 `--scenario teleop-cartesian`、`--scenario free-joint` 或 `--scenario free-cartesian` 查看其余场景，默认是 `teleop-joint`。本机离线预览时需让 Pinocchio 使用 Conda 环境的 C++ 运行库；若出现 `CXXABI_1.3.15` 缺失，在已激活 `urcontrol` 的终端运行：

```bash
LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" python preview_ui.py --admittance
```

预览明确标记“演示数据”，提供四种场景供检查布局；不导入 `main.py`，不初始化机器人、相机、夹爪或采集器。实施验证使用静态检查及离线界面预览，按规划不新增或运行回归测试、不计算哈希、不连接 URSim 或真机。真实启动、偏置采样、停止确认、断流处理、模型配置、接触稳定性及多屏交互仍需按 [导纳调参界面代码规划](URControl_导纳调参界面代码规划.md) 的人工验收表联调。
