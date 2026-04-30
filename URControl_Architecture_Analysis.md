# URControl 项目架构与功能分析

## 一、项目概述

URControl 是一个基于 Python 的 **Universal Robots (UR) 机械臂远程控制与遥操作上位机软件**，使用 PyQt5 构建图形界面，通过 TCP/IP 协议族与 UR 机械臂控制器进行多通道通信，实现机器人状态监控、关节/末端空间实时伺服控制、Dashboard 指令下发以及 URScript 脚本发送等功能。

**目标平台**: UR e-Series (UR5e 等)，同时兼容 URSim 仿真环境。

---

## 二、项目文件结构

```
URControl/
├── main.py                          # 程序入口，PyQt5 主窗口逻辑
├── ui_main_window.py                # Qt Designer 自动生成的 UI 布局代码
├── main_window.ui                   # Qt Designer UI 布局源文件
├── URRTDEController.py              # RTDE 实时伺服控制核心（闭环跟踪）
├── URRealtimeClient.py              # Realtime 数据流客户端（后台持续读取）
├── URRealtimeUtils.py               # Realtime 报文解析器与状态数据结构
├── URTcpClient.py                   # TCP 通信基类
├── URScriptClient.py                # URScript 脚本发送客户端
├── URDashboardClient.py             # Dashboard Server 命令客户端
├── NetWorkSet.py                    # 网络工具函数（IP 获取/校验）
├── rtdetest.py                      # RTDE 控制器功能测试脚本
├── rtdetest_debug.py                # RTDE 控制器调试脚本
└── URRTDEController（复件）.py       # 备份文件（与主文件内容相同）
```

---

## 三、系统架构

### 3.1 整体分层架构

```
┌─────────────────────────────────────────────────────────┐
│                    GUI Layer (PyQt5)                     │
│  main.py  ──  UI_MainWindow                             │
│  ui_main_window.py  ──  Ui_MainWindow (auto-generated)  │
│  main_window.ui  ──  Qt Designer source                 │
└──────────┬──────────┬──────────┬──────────┬─────────────┘
           │          │          │          │
    ┌──────▼──┐ ┌─────▼────┐ ┌──▼──────┐ ┌─▼──────────┐
    │Dashboard│ │URScript  │ │Realtime │ │   RTDE      │
    │ Client  │ │ Client   │ │ Client  │ │ Controller  │
    │(29999)  │ │(30002)   │ │(30013)  │ │  (RTDE)    │
    └────┬────┘ └────┬─────┘ └────┬────┘ └──────┬──────┘
         │           │            │              │
    ┌────▼───────────▼────────────▼──────────────▼──────┐
    │              URTcpClient (Base Class)              │
    │          Low-level TCP communication               │
    └────────────────────────┬───────────────────────────┘
                             │
                    ┌────────▼────────┐
                    │  UR Robot Arm   │
                    │  (real/sim)     │
                    └─────────────────┘
```

### 3.2 通信通道说明

UR 机械臂提供多个 TCP 端口用于不同目的：

| 端口 | 协议 | 对应模块 | 功能 |
|------|------|----------|------|
| 29999 | Dashboard Server | `URDashboardClient` | 上电/下电/松抱闸/加载程序/状态查询 |
| 30002 | Secondary Interface | `URScriptClient` | 发送 URScript 脚本指令 (movej/movel) |
| 30013 | Realtime Read-Only | `URRealtimeClient` | 持续读取 500Hz 实时状态数据 |
| RTDE | Real-Time Data Exchange | `URRTDEController` | 实时伺服控制闭环 (servoJ) |

---

## 四、各模块详细分析

### 4.1 `URTcpClient.py` — TCP 通信基类

**设计定位**: 提供统一的 TCP 客户端底层能力，所有 UR 通信客户端均继承此类。

**功能**:
- 连接管理: `connect()`, `close()`, `reconnect()`
- 数据发送: `send_bytes()`, `send_text()`
- 数据接收: `recv_once()`, `recv_until()`, `recv_text()`
- 请求-响应模式: `request_bytes()`, `request_text()`
- 线程安全: 使用 `threading.RLock()` 保护 socket 操作
- 上下文管理器: 支持 `with` 语法

**设计亮点**:
- 自动检测连接断开并在异常时关闭 socket
- 支持自定义结束符 (terminator)，适配不同 UR 协议

### 4.2 `URDashboardClient.py` — Dashboard 命令客户端

**继承**: `URTcpClient`，端口默认 29999

**功能**:
- 电源控制: `power_on()`, `power_off()`, `brake_release()`
- 程序控制: `load_program()`, `play()`, `pause()`, `stop()`
- 状态查询: `robot_mode()`, `program_state()`, `safety_status()`, `running()`
- 安全相关: `unlock_protective_stop()`, `close_popup()`, `shutdown()`, `quit()`
- 连接时自动读取欢迎信息

### 4.3 `URScriptClient.py` — URScript 发送客户端

**继承**: `URTcpClient`，端口默认 30002

**功能**:
- `movej(q, a, v, t, r)` — 关节空间点到点运动
- `movel(pose, a, v, t, r)` — 笛卡尔空间直线运动
- `speedj(qd, a, t)` — 关节速度控制
- `speedl(xd, a, t)` — 末端速度控制
- `stopj(a)` / `stopl(a)` — 停止运动
- `send_script(script)` — 发送自定义 URScript

### 4.4 `URRealtimeUtils.py` — 实时数据解析

**数据结构**:
- `URRealtimeState` — dataclass，存储解析后的机械臂状态
  - `q_actual`: 实际关节角 (6维)
  - `qd_actual`: 实际关节速度 (6维)
  - `tcp_pose`: 实际 TCP 位姿 [x,y,z,rx,ry,rz]
  - `tcp_force`: 末端广义力 (6维)
  - `tcp_speed`: 末端速度 (6维)
  - `robot_mode`, `safety_mode`, `safety_status`, `program_state`: 运行状态
  - `digital_inputs` / `digital_outputs`: 数字 IO 读取
  - `motor_temperatures`: 电机温度
  - 以及其他 30+ 个状态字段

- `URRealtimeParser` — 报文解析器
  - 基于 UR 官方 Realtime 5.9/5.10 数据表格
  - 报文格式: `int32 message_size` + `double data[...]`
  - 支持严格/非严格两种解析模式
  - 字段按 1-based Gnuplot column 编号索引

### 4.5 `URRealtimeClient.py` — 实时数据流客户端

**继承**: `URTcpClient`，端口默认 30013

**设计架构**: 后台线程持续读取 + 主线程异步消费

**核心机制**:
- 连接后启动后台 daemon 线程 `_reader_loop()`
- 线程循环调用 `_read_packet()` → `parser.parse()` → 更新 `_latest_state`
- 使用 `threading.Condition` 实现线程间状态同步
- 提供阻塞等待接口: `wait_first_state()`, `wait_next_state()`
- 数据读取使用 `_recv_exact()` 确保完整帧接收

**读取流程**:
```
_recv_exact(4) → 读包头 message_size (大端 int32)
_recv_exact(message_size-4) → 读 payload
→ parser.parse() → URRealtimeState
→ 更新 _latest_state，通知等待者
```

### 4.6 `URRTDEController.py` — RTDE 实时伺服控制器

**依赖**: `ur_rtde` 库 (RTDEControlInterface + RTDEReceiveInterface)

这是项目中**最核心、最复杂的模块**，实现了闭环伺服控制。总代码约 1200 行。

**控制方式**:
控制器不再暴露独立的控制模式枚举。后台控制循环根据当前 active target 自动选择行为：
有 `_target_q` 时走 `servoJ` 关节跟踪，有 `_target_tcp_pose` 时走 `servoL` TCP 跟踪，二者都为空时保持空闲。

**对外接口**:
- `start(force_reupload)` — 启动/恢复控制线程（设计为可重复调用且幂等）
- `stop(stop_script)` — 停止控制线程，可选关闭 RTDE script
- `shutdown()` — 最终关闭，断开 RTDE 连接
- `track_joint(q, dq_max)` — 跟踪关节角
- `move_joint_delta(delta_q, dq_max)` — 关节增量
- `move_tcp_delta(delta_pose, dq_max, frame, reference)` — 末端增量
- `get_status()` / `debug_status_string()` — 状态查询
- `get_actual_q()` / `get_actual_tcp_pose()` / `get_target_tcp_pose()` — 位姿读取

**关键设计决策**:
1. **TCP 目标在控制线程执行**: TCP 增量目标由 `_control_loop()` 线程下发，而非外部 API 线程，避免多线程同时调用 RTDEControlInterface
2. **TCP 增量基准策略**: 默认 `reference="target"`，连续增量以上一条目标TCP位姿为基准，避免实际跟踪误差累积造成的姿态漂移
3. **Python 侧限速**: `_rate_limit_q()` 在关节空间对每周期步长进行 clip，与 UR 控制器底层 `servoJ` 的速度/加速度参数形成双层保护
4. **安全恢复**: `start()` 被设计为可重复调用，即使 RTDE control script 被外部 URScript 顶掉，也能通过 `reuploadScript() + start()` 恢复

**控制循环流程**:
```
_control_loop():
  while _running:
    snapshot = _snapshot_command()
    
    if mode == OFF:
        servoStop() + sleep(dt)  → 不空转 initPeriod
    
    elif JOINT_TRACK / JOINT_DELTA:
        active_target_q = target_q (或 q_actual + delta_q)
        安全检查 (硬关节限位 ± 可选 RTDE 安全)
    
    elif TCP_DELTA:
        IK求解 (getInverseKinematics) 在控制线程中执行
        安全检查
    
    q_cmd = _rate_limit_q(q_cmd, target_q, dq_max, dt)  # Python 侧限速
    servoJ(q_cmd, speed, acc, dt, lookahead_time, gain)  # 底层伺服
    waitPeriod()  → 保持固定频率
```

**姿态变换**:
- `_pose_trans_local()` — URScript `poseTrans()` 的纯 Python 等价实现
- `_rotvec_to_matrix()` / `_matrix_to_rotvec()` — 旋转向量 ↔ 旋转矩阵转换 (Rodrigues 公式)
- 支持 base_add 和 tool 两种 frame 的增量位姿计算

### 4.7 `NetWorkSet.py` — 网络工具

**功能**:
- `is_valid_ipv4(ip)` — IPv4 地址格式校验
- `is_bad_local_ip(ip)` — 过滤回环/APIPA/测试网段等不适用地址
- `get_local_ip_by_target(target_ip)` — 通过 UDP connect 获取本机对应目标网段的 IP
- `get_local_ip_from_ipconfig()` — Windows 下 ipconfig 解析
- `get_local_ip_from_ifconfig()` — Linux/macOS 下 ifconfig 解析
- `get_local_ip(target_ip)` — 统一入口，自动适配平台

### 4.8 `main.py` + `ui_main_window.py` + `main_window.ui` — GUI 层

**UI 布局**:
- 左上区: 连接控制 (IP 输入、连接/上电/松抱闸/急停按钮、消息日志)
- 右上区: 三列实时数据显示 (关节角 / TCP位姿 / 末端广义力)
- 中部: 笛卡尔空间控制按钮组 (X/Y/Z/RX/RY/RZ +/- 方向)
- 下部: URScript MoveJ 测试按钮
- 状态栏: 机器人连接和运行状态文字

**实时刷新机制**:
- `timer_URStatus` (100ms): 刷新 Dashboard 连接状态
- `timer_URStatus_RT` (10ms): 刷新关节角/TCP/力传感器实时数值
- `timer_URTCPControl` (2ms): 遥操作按钮按下时，持续发送增量伺服命令

**遥操作交互**: 按钮按下 → 启动 RTDE → 启动 2ms 定时器持续发送增量 → 按钮释放 → 停止定时器和 RTDE

---

## 五、数据流示意图

```
                    ┌──────────────────────────┐
                    │      UI_MainWindow       │
                    │                          │
                    │  ┌───────────────────┐   │
                    │  │ timer_URStatus_RT │   │
                    │  │      (10ms)       │   │
                    │  └────────┬──────────┘   │
                    │           │get_latest_state()
                    │  ┌────────▼──────────┐   │
                    │  │  URRealtimeClient │   │
                    │  │   (30013, 后台)    │───┼──► UR 实时数据流
                    │  └───────────────────┘   │
                    │                          │
                    │  ┌───────────────────┐   │
                    │  │ timer_URTCPControl│   │
                    │  │      (2ms)        │   │
                    │  └────────┬──────────┘   │
                    │           │move_*_delta()
                    │  ┌────────▼──────────┐   │
                    │  │ URRTDEController  │   │
                    │  │  (RTDE, servoJ)   │───┼──► UR 伺服控制
                    │  └───────────────────┘   │
                    │                          │
                    │  ┌───────────────────┐   │
                    │  │  URDashboardClient│   │
                    │  │  (29999, 命令)     │───┼──► UR 状态机控制
                    │  └───────────────────┘   │
                    │                          │
                    │  ┌───────────────────┐   │
                    │  │   URScriptClient  │   │
                    │  │  (30002, 脚本)     │───┼──► UR 脚本执行
                    │  └───────────────────┘   │
                    └──────────────────────────┘
```

---

## 六、代码质量评估与优化建议

### 6.1 严重问题

#### 6.1.1 GUI 按钮事件绑定大量重复代码 (`main.py`)
`button_events_def()` 方法中有 **24 个按钮 × 2 种事件 (pressed/released) = 48 行重复的信号连接**，影响可维护性。

**建议**: 使用循环批量绑定，例如：

```python
for i in range(1, 7):
    getattr(self, f"pushButton_JUp{i}").pressed.connect(
        lambda idx=i: self.on_JUp_Button_Pressed(idx))
    getattr(self, f"pushButton_JUp{i}").released.connect(
        self.on_RTControl_Button_Released)
    # ... 同理 JDown, TUp, TDown
```
可将 `button_events_def()` 从约55行缩减到约15行。

#### 6.1.2 定时器信号重复连接 (`main.py`)
`on_JUp_Button_Pressed` / `on_JDown_Button_Pressed` / `on_TUp_Button_Pressed` / `on_TDown_Button_Pressed` 中，**每次按下按钮都重新做一次 `timer.timeout.connect(lambda: ...)`**。如果快速重复按下按钮，会累积多个连接到同一信号，导致控制命令叠加发送。

**建议**: 在 `__init__` 中一次性连接信号，使用变量切换 dq/mode 的值；或在每次连接前先调用 `timer.timeout.disconnect()` 清除旧连接。

#### 6.1.3 缺少窗口关闭时的资源清理 (`main.py`)
窗口被关闭时没有调用 `URRTDEController.shutdown()` / `URRealtimeClient.close()` / `URDashboardClient.close()`，导致 RTDE 连接和 TCP socket 可能未被正确释放。

**建议**: 重写 `closeEvent()` 方法，在其中清理所有连接资源。

#### 6.1.4 备份文件重复 (`URRTDEController（复件）.py`)
该文件与 `URRTDEController.py` 内容完全相同（文件名含中文字符），容易造成修改时两边不一致。

**建议**: 删除备份文件，使用 Git 进行版本管理。

### 6.2 中等问题

#### 6.2.1 缺少 `requirements.txt`
项目依赖 `pyqt5`, `numpy`, `ur_rtde` 等第三方库，但没有依赖声明文件。

**建议**: 创建 `requirements.txt` 或 `pyproject.toml`。

#### 6.2.2 硬编码 IP 地址 (`main.py` 第34行)
```python
self.URIP = '127.0.0.1'
```
**建议**: 通过配置文件或启动参数指定默认 IP。

#### 6.2.3 控制速度量纲命名误导 (`main.py`)
```python
self.UR_J_Control_Speed = 0.1 / 500   # 实际是每2ms的增量，不是速度
self.UR_TCP_Control_Speed = 0.01 / 500
```
变量名暗示是"速度"，但除数是 500 (频率)，实际表达的是**每个控制周期的步长**。

**建议**: 重命名为 `UR_J_Control_Step` / `UR_TCP_Control_Step`，或将 timer 间隔和步长做语义一致的关联。

#### 6.2.4 `main.py` 中 RTDE 控制器在机器人非 RUNNING 模式时不初始化
第 70 行仅在 `robot_mode() == 'Robotmode: RUNNING'` 时创建 `URRTDEController`，但在 `on_URBrakeRelease_Button` 中又补充创建。这意味着用户先松抱闸再连接的流程中 RTDE 控制器不会自动创建，存在逻辑不一致。

**建议**: 统一在连接时创建 RTDE 控制器（或独立提供创建按钮），模式检查和启动分离。

#### 6.2.5 `URDashboardClient.connect()` 异常处理过于宽松
`connect()` 方法中使用裸 `except:` 捕获所有异常并静默返回 `None`，会掩盖如网络不通等重要错误。

**建议**: 至少记录异常信息到日志，或将异常传播给调用者。

#### 6.2.6 旋转向量 → 矩阵转换中 π 附近判断可能不稳定
`URRTDEController._matrix_to_rotvec()` 的 π 角度附近处理使用多个逐元素符号判断来决定旋转轴方向，当 θ ≈ 180° 时可能存在数值不稳定。

**建议**: 参考标准做法使用 `logm()` 或对称方法提取旋转轴，或使用 `scipy.spatial.transform.Rotation`。

### 6.3 轻微问题

#### 6.3.1 缺少类型标注
`main.py` 中的方法大多缺少类型标注 (type hints)，降低了可读性和 IDE 支持。

#### 6.3.2 字符串控制类型比较 (`main.py` 第168行)
```python
if mode == 'tcp_tool':
```
UI 层仍使用字符串区分按钮触发的控制类型，建议后续改成局部常量或专用 UI 枚举，避免拼写错误。

#### 6.3.3 `URRealtimeClient._recv_exact()` 中锁内检查
每次 recv 循环都检查 `_stop_reader_event.is_set()` 和 `_sock`，在高速数据流 (500Hz) 下会引入不必要的开销。考虑到 _reader_loop 是唯一调用者，可以将停止检查放在外层循环。

#### 6.3.4 缺少日志系统
整个项目使用 `print()` 进行调试输出，没有统一的日志框架。

**建议**: 引入 `logging` 模块，支持不同级别的日志输出。

#### 6.3.5 `rtdetest.py` 和 `rtdetest_debug.py` 高度相似
两个测试文件功能基本一致，后者增加了安全检查和调试打印。可以合并为一个带 `--debug` 参数的测试脚本。

#### 6.3.6 窗口标题未设置
`main_window.ui` 中窗口标题为 "MainWindow"，没有改为有意义的中文标题。

---

## 七、总结

### 优势
- **模块化设计清晰**: 按 UR 通信通道分工，每个模块职责单一
- **RTDE 控制器设计精良**: 线程安全、模式清晰、安全恢复机制完善、TCP 增量漂移抑制策略合理
- **实时数据解析完整**: 覆盖 UR 官方 Realtime 5.9/5.10 表格大部分字段
- **可扩展性好**: 基类 `URTcpClient` 提供良好继承体系

### 主要改进方向
1. **减少 GUI 层代码重复**: 按钮绑定可批量生成
2. **修复资源泄漏**: 窗口关闭时清理连接
3. **修复定时器信号累积**: 确保信号连接幂等
4. **补充工程化设施**: requirements.txt、logging、配置文件
5. **清理冗余文件**: 删除备份文件，使用版本控制
