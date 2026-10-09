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
