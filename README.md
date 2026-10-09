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

## 界面与多显示器

新版界面采用布局管理器和可拖动分隔条。左侧切换关节/TCP 控制，以及 UDP、力/力矩、夹爪和运行日志；右侧显示机械臂和两路相机。日志标签会提示未读消息数。底部的限速、采集、停止和下电按钮始终位于固定操作栏。窗口较小时，左侧控制区可以滚动。停止、下电、采集和主要控制按钮使用更大的尺寸和字号；禁用控件使用灰底、灰字和虚线边框。XYZ 点动按钮保留原来的红、绿、蓝颜色，旋转轴使用对应深色；禁用时仅保留轴颜色标记。

窗口初次打开时使用普通窗口，新版以 `.ui` 中的 1360×800 为基准，再适配鼠标所在屏幕，通常占屏幕宽度约七成；不会自动最大化或全屏。字号按屏幕的可用逻辑尺寸适配：4K、200% 缩放时更紧凑，2K、100% 时使用更大的字号。拖到另一台显示器后会重新适配，并保留输入内容和当前标签页；已经手动最大化的窗口保持最大化。右上角“界面大小”可在自动大小基础上选择 90%、110%、125%，选择按显示器和界面模式分别记忆。

“界面 → 经典界面（改版前）”可立即回到原来的布局，“界面 → 新版界面”可切回，程序会记住最后的选择。切换复用同一批业务控件，保留连接、输入值、采集状态和机器人视图，不发送运动命令。经典界面保留原有控件位置和配色，内容随字号等比例缩放，窗口较小时通过滚动条查看。切换模式时使用对应界面的默认窗口大小。

系统 DPI 缩放由 Qt 处理，字号适配使用逻辑尺寸。逻辑像素与物理像素的关系见 [Qt High DPI 说明](https://doc.qt.io/qt-6.8/highdpi.html)。实际跨屏行为取决于桌面环境向 Qt 提供的屏幕信息。

### 修改界面

`main_window.ui` 是新版布局、控件、配色和基准字号的源文件，`classic_window.ui` 保存经典界面。用 Qt Designer 编辑对应的 `.ui`，再在项目根目录执行：

```bash
conda activate urcontrol
pyuic5 main_window.ui -o ui_main_window.py
pyuic5 classic_window.ui -o ui_classic_window.py
python preview_ui.py
```

`preview_ui.py` 支持两种界面的预览与切换，不连接机器人、不打开相机，也不创建采集文件。确认后用 `python main.py` 正常运行。不要直接编辑自动生成的 `ui_main_window.py` 和 `ui_classic_window.py`。

相机标签在 Designer 中提升为 `ui_widgets.AspectRatioLabel`，运行时保持横屏/竖屏画面的原始比例。`window_display.py` 负责屏幕适配、字号偏好、日志提示和力矩字段标签，`window_layouts.py` 负责复用业务控件并切换布局；机械臂视图在运行时加入 `.ui` 中预留的 `verticalLayout_RobotViewport`。

离线显示检查（无需设备）：

```bash
python -m unittest discover -s tests -v
```
