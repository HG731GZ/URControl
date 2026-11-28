# UDP v2 测时、从端事件与联调

本项目为 UR 从端，主端为 DMArm。两端必须同时升级；不接受旧 133 B 命令，不发送旧 230 B 反馈。本轮仅修改通信、事件身份和日志，阻抗增益、位置/速度/力矩限制、机器人驱动周期、控制律不变。主端发布 5 ms；从端沿用独立的本地 200 Hz 反馈节拍，不因收到命令立即回复，不等待第一帧主端包。

## 1. 启动、端点与退出

正常模式 `kind=0` 直接 `python main.py`，会话由程序生成和发现，不读取会话环境变量，也不要求输入 ID。常规运行不创建 UDP 日志目录、不写逐包或逐样本 UDP 事件文件；只有 `udp_diagnostic.py` 开启 UDP 后台记录。

当前端点为本机 `enp4s0f2np2 / 192.168.1.14:5005` ↔ DMArm `192.168.1.5:6005`，由 `udp_settings.py` 配置。可用四个 `UR_UDP_LOCAL_IP/LOCAL_PORT/REMOTE_IP/REMOTE_PORT` 环境变量覆盖。发送和接收共用已绑定端口。机器人地址仍单独配置。

启动后即收发 UDP，不依赖首个主端包或机器人连接。运动须按原界面显式开启。退出先停止控制并确认，再关闭窗口。

在 PyCharm 中打开 `udp_diagnostic.py`，直接点击运行，无需填写运行参数。文件顶部两项用于日常配置：

```python
TEST_MODE = 'network'
TEST_DURATION_S = 120
```

默认纯网络 kind=1、运行两分钟自动收尾；主端也须选 kind=1。要观察正常通信，把 `TEST_MODE` 改为 `'normal'`。端点仍在 `udp_settings.py` 统一设置，日志固定写入项目下 `logs/udp/`，不随 PyCharm 工作目录改变。运行前关闭占用 5005 的 main.py。命令行参数保留为可选覆盖项，IDE 的运行参数留空即可使用顶部配置。

仅测网络、完全不连接驱动（可选命令行方式）：

```bash
python udp_diagnostic.py --duration 120
```

正常业务通信的无驱动观察入口：

```bash
python udp_diagnostic.py --mode normal --duration 120
```

前者 kind=1，原业务载荷全零；后者 kind=0，mode/teleop_state=OFF，所有缺测浮点量为 NaN，采样及采用时间为零。两种入口都不导入机器人控制器、Qt、相机或夹爪，不会执行收到的目标。120 秒自动退出，或 Ctrl+C/SIGTERM 退出；不要与 main.py 同时占用 5005。

### 自动会话和断流处理

- 每次本端创建 UDP 发布 socket，自动生成非零随机 uint64 `slave_session`，本端 `fb_seq` 从 1 递增。
- 收到正确端点、模式、长度、版本、CRC 的首个有效包，自动发现 `master_session`；主端先启动时可以从任意首见序号开始。
- 来包 session 不同于当前且未退役：识别为主端重启，把旧 session 退役，清除接收序号高水位、回显、目标及采用反馈快照，接纳新 session 的序号。已退役 session 的迟到包只记 `retired_session`，不能反向接管。
- 同 session 的重复/乱序不更新目标、回显或新鲜度；不会因为 seq=1 自动清空状态。重启发布方必须换自己的 session。
- 超过 250 ms 没有有效新包，报告 timeout，清除可用目标和回显。相同 session 恢复发送时继续检查原高水位；不同 session 则按新会话处理。
- 接收断流或网络错误、首次发送错误会请求一次本端发布换代：下一周期自动换 `slave_session`、`fb_seq=1`，连续同一故障不逐帧换 ID。周期发送继续发送最新反馈，不重放失败旧帧。后续新包/发送成功记录恢复事件。
- 仅发现对端 session 改变不主动换本端 session，避免双方互相触发无限重启。两端各管理自己的发送序号，接收对端新 session 时只重置对端接收序号。
- 会话变化/断流触发既有运动停止路径；通信自动恢复不等于自动重启运动，普通跟随、阻抗和导纳仍需按既有停止确认与显式启动流程运行。

退役集合在从端客户端实例内保留，进程全新启动时自动发现来包，不依赖人工允许列表。随机 session 不是可比较的时间戳：对从未观察过的 ID，原协议没有握手信息可证明其真实先后；不能据此宣称可识别任意历史重放。主端应采用同样的自动发现、退役、按 session 分组计数规则处理 `slave_session`。

## 2. 线协议

每个 UDP 数据报一帧。大端、紧密排列、无填充：

| 方向 | 原业务载荷 | 扩展 | 扩展偏移 | CRC 偏移 | 帧尾偏移 | 整帧 |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 主→从 | `>6dB3d6d`，121 B | `>BB3Q`，26 B | 125 | 151 | 155 | 159 B |
| 从→主 | `>6dB3d6d6dB6d`，218 B | `>BB13Q`，106 B | 222 | 328 | 332 | 336 B |

帧头 `55 AA 55 AA`；帧尾 `0D 0A 0D 0A`。CRC 为 `zlib.crc32(packet[4:-8]) & 0xffffffff`，四字节大端，覆盖业务载荷及完整扩展。

主→从扩展：`version, kind, master_session, cmd_seq, t1_us`。

从→主扩展：`version, kind, slave_session, fb_seq, echo_master_session, echo_cmd_seq, echo_t1_us, t2_us, t3_us, applied_master_session, applied_cmd_seq, t_apply_us, t_qdq_us, t_tau_us, t_ddq_us`。

`version=2`；`kind=0` 正常，`kind=1` 诊断。各发送方向在本端发送会话内从 1 递增。会话、序号和所有 `*_us` 用 Python 整数/网络 uint64；不会经浮点数或 CSV 数值列保存。已有控制器 31 位寄存器序号仍使用本地来源/提交序号，另带完整 UDP 会话和命令序号跨进程传递，不把 64 位值塞进浮点寄存器。

诊断业务载荷的 **218/121 个原始字节全部为零**；反馈采用字段及三类采样时间全部为零。尚未收到本轮有效新包时四个回显字段为零。每次取用回显时，四个字段来自同一条校验通过、匹配模式/端点、当前会话序号递增的命令；重复/乱序包只记录，不推进回显或运动年龄。

### 原载荷字段保持

| 偏移 | 主→从 | 从→主 |
| ---: | --- | --- |
| 4～51 | 六轴目标角，rad | 六轴实际关节角，rad |
| 52 | mode | mode=OFF |
| 53～76 | 三个夹爪值 | 夹爪开度、原始电流、原始位置 |
| 77～124 | 六轴目标速度，rad/s | 六轴实际速度，rad/s |
| 125～172 | 扩展从 125 开始 | 六轴力矩，Nm |
| 173 | — | teleop_state：0 未开启、1 普通跟随、2 阻抗遥操作 |
| 174～221 | — | 六轴实际加速度，rad/s² |

正常模式缺测位置/速度、夹爪、力矩或加速度沿用 NaN 表示；对应缺测的采样时间为 0。阻抗力矩字段仍是限幅后的命令力矩 `tau_cmd`；普通控制和关节力矩导纳仍为补偿实测力矩。阻抗实测力矩保存在既有阻抗控制日志中，不能把反馈命令力矩误当实测力矩。加速度仍复用既有阻抗交替寄存器页；非阻抗没有该数据源，时间为 0，不新建计算或传感器来源。

## 3. 事件边界及代码位置

所有新增时间为 `time.monotonic_ns() // 1000`，单位整数微秒。主从 PC 不要求同钟；同一从端的主进程和 spawn 子进程共享本机单调时间轴。原有浮点秒字段只服务已有控制期限及旧诊断，不能代替新增的整数事件时间。

| 字段/事件 | 实际取值函数 | 可观测边界 |
| --- | --- | --- |
| t2_us | `UR_Utils/URUdpClient.py::_read_datagram` | `recvfrom` 返回后第一时间取；校验通过后才可配对回显。不是网卡硬件或内核时间 |
| 收到命令 | `::_accept_received` | 更新接收身份、回显与分类计数；仅测试脚本逐包保存 session/seq/t1/t2 及处理分类 |
| 发布目标 | `::_receive_batches` | 批次排空后发布最新业务目标；保留既有停顿/积压保护 |
| 普通目标取用 | `URRTDEController::_snapshot_command`、`::_control_loop` | 目标与 UDP 身份一起取快照 |
| 普通 t_apply_us | `URRTDEController::_control_loop` → `udp_events.applied_event` | `rtde_c.servoJ` 返回成功后。`q_applied_rad` 为当次实际提交的限速后六轴 q_cmd，仅保留最近采用快照供反馈 |
| 阻抗 t_apply_us | `URImpedanceController::_execute` 内 `_send(packet)` 返回后 → `applied_event` | 控制子进程已成功提交 RTDE 输入；快照保留当次六轴 q_rad。不是机器人闭环内部的 q_ref 或采纳时刻 |
| 导纳 t_apply_us | `URAdmittanceController::_send_inputs` → `applied_event` | `send_prepared` 完成后，快照保留实际六轴 wire_q。不是机器人内部生成的柔顺伺服目标 |
| t_qdq_us | `URRealtimeClient::_reader_loop` | 既有 Realtime 只读状态（默认 30013）解析后、更新 q/dq 缓存处；值和时间同锁发布；常规运行不另写 UDP 样本日志 |
| 普通 t_tau_us | `URRTDEController::get_compensated_torque_snapshot` | `getJointTorques()` 返回后；后台补偿读取器保留此时刻，不以反馈读取时刻重打 |
| 导纳 t_tau_us | `URAdmittanceController::_consume_state` 中载荷页解码处 | 首次观察新的载荷页身份；同页复用时刻 |
| 阻抗 t_tau_us / t_ddq_us | `URImpedanceController` 的 RTDE 接收处理循环 | 首次解码新命令力矩周期/实测力矩页/加速度页时的 PC 时刻，分别随样本保留 |
| t3_us | `URUDPClient::_send_packet` | 反馈快照和序号已定、临近 `sendto`；之后只打包扩展和 CRC。不是提交发送队列的时刻 |

各类样本本来异步，包内不强行伪装成同一采样时刻。一个反馈帧将选中的值/原时间、最近采用事件及四元回显冻结后发送；重复发缓存时沿用原时间。最近采用事件保持至下一次成功采用，即使其间只保持同一目标。接收、主进程队列、IPC 或控制提交前被新目标覆盖的命令都不更新采用快照。

本轮 `t_apply_us` 明确是 PC 提交成功边界。已有阻抗 `records.jsonl/adoption` 仍记录机器人寄存器回报首次可见时刻，但 500 Hz 回报只能看到最新采纳身份，不能承诺重建机器人固件内所有采纳过程。当前常规运行只保留最近一次 PC 采用快照用于反馈，不保存完整 UDP 采用轨迹；需要后续响应辨识时须另行安排数据采集。接口提交成功不代表机械关节已经到达目标。

## 4. 后台日志与离线核对

只有 `udp_diagnostic.py`（network 或无驱动 normal 模式）创建 `logs/udp/<启动时间与进程号>/`，保存端点、模式、时钟边界和 `events_<pid>_<创建时刻>.jsonl`。它显式开启工程 `RecordWriter` 后台写盘，退出后保存 `.summary.json`；检查 `log_complete`、`log_dropped`、`log_error`。不再使用 `UR_UDP_LOG_DIR` 环境变量，不向控制子进程传播写盘配置。

`main.py`、实时状态进程及控制进程不启动 UDP 写盘器；普通跟随不写 `servo_target`，源缓存不另写 `sample_qdq/sample_tau/sample_ddq`。遥操作阻抗的 `source_timing.jsonl` 只保留控制适配器提交记录，不再保存 `udp_receive/udp_publish` 或内核收包时间。既有阻抗、导纳、独立遥测及用户采集日志按各自原流程运行。

主要事件：

- `udp_receive`：所有通过结构/CRC/业务数值校验的包。`disposition` 为 new、duplicate、out_of_order、retired_session、wrong_kind、wrong_peer 或 backlog_discard；另保留 duplicate/out_of_order 标志。
- `udp_rejected`：长度、帧边界、版本、CRC、零载荷契约或数值校验失败及实际接收时刻。
- `udp_publish`：实际发布给控制消费者的本地来源身份与 UDP 会话/命令序号。
- `udp_send`：扩展整数数组严格按协议顺序，包含 fb_seq、t3、采用及采样时间；成功时附原载荷十六进制，失败记录 error。序号在发送尝试前分配，失败序号不重用，不自动重发旧反馈。

```bash
python udp_log_summary.py logs/udp/本次记录目录
# 可选：从主端日志取得各发送会话的最终序号，核对尾部缺失。
python udp_log_summary.py logs/udp/本次记录目录 --master-final 会话ID:最终序号
```

摘要列出各分类计数、缺失区间、发送失败和最终 fb_seq。测试脚本不执行目标，命令没有采用记录是预期行为；分析工具仍可读取历史正常运行日志中的采用事件。缺失从 1 统计，乱序后补到的包不再算缺失。没有 `--master-final` 时只能核对到已见最大序号，尾部丢包不可从从端单独推断。主端日志需要以每个 slave_session 的 fb_seq 对照从端 udp_send，才能识别反馈方向最终丢失；从端 sendto 成功不代表 DMArm 已接收。

排序使用各事件的 `*_us` 字段，不依赖文件行号或后台写入时间。UDP 字段不要导入 Excel/JavaScript Number 后再回写；uint64 必须按整数或十进制字符串保真。

## 5. 约两分钟静止对测

1. 停止已有控制并确认机械臂静止，退出正常入口。纯网络实验无需连接/使能驱动。
2. 在两端选择 kind=1，设置两端实际网卡地址与互指端口。记录交换机型号/连线方案，保持主端 5 ms 和从端 200 Hz。
3. 从端在 PyCharm 直接运行 `udp_diagnostic.py`（顶部 network / 120 秒），或使用上面的命令；主端先启动或从端先启动均可，从端尚无对应命令时，四个回显字段应全零。主端绑定自己的 6005 端口周期发 159 B，收 336 B。
4. 主端在每次 recv 返回后取 t4_us，逐帧记录两个会话及两个序号、echo_t1_us/t2_us/t3_us/t4_us。计算 `(t4-echo_t1)`、`(t3-t2)` 和两者之差；没有对应回显的零值帧不能进入时延统计。回显重复不等于反馈重复；主端按 fb_seq 核对反馈、按 echo_cmd_seq 标记同一命令的多次回显。
5. 120 秒结束并收尾后，交换双方最终发送序号；跑离线摘要，核对重复/乱序、命令和反馈最终缺失、日志完整性，再统计时延的中位数、P95/P99/最大值。不要把启动空白帧或发送失败当作成功样本。
6. 换交换机后重新启动两端发布，再做相同条件的 120 秒诊断。应用层双向传输耗时仍包含 PC socket、线程调度及协议处理；它排除了从端等待反馈周期的驻留，不等于交换机硬件转发延迟。没有独立时钟同步依据，不计算绝对单向时延。
7. 正常模式静止核对时重新启动两端发布和 main.py，保持运动跟随关闭；确认 q/dq 等时间随各源更新，尚无采用时三项采用字段为零。后续响应辨识必须另行安排既有安全流程下的运动联调，本轮未连接机器人执行。

## 6. 本轮离线验证

示例诊断命令扩展 `(2,1,1,1,1000000)`，159 B，业务全零，CRC **0939AAE6**。

示例诊断反馈扩展 `(2,1,2,1,1,1,1000000,2000000,2000500,0,0,0,0,0,0)`，336 B，业务全零，CRC **EE67C7B7**。

以 t4=1001500 µs 手工核对：命令到回显 1500 µs，从端驻留 500 µs，扣除后 1000 µs。

2026-11-28 实际联调：从端无驱动 normal 入口接收 21,226 条有效主端命令，自动发现初始会话并识别两次主端重开，两次新会话首帧均为 seq=1。发送 24,001 帧，socket 发送错误为 0，后台日志完整且无丢弃。主端确认收到反馈但判为无效；原因未提供。重建的正常反馈为 336 B，头尾/CRC 正确，浮点量为 NaN、teleop_state=OFF。此结果只证明从端接收/自动切会话和反馈提交，不能据此宣称主端业务校验通过或取得往返时延测量。

原始记录在 `logs/udp/20261128_201738_1795868258200392019_2858785/`，解码和联调摘要在 `results/udp_auto_session_live/`。用户要求后已结束此次联调，未连接机器人驱动或执行运动，未运行回归测试。

### 主端判帧要点

线协议有效性先校验长度、头尾、CRC、version、kind、非零发送 session/seq；之后自动切换接收会话、分类序号、记录测时。`echo_master_session` 为零表示尚未收到命令；与当前主端会话不符的回显不参与本轮时延，不能因此否认从端已经重启。

normal 无驱动反馈中的 NaN 表示缺测，不是 CRC 或字节布局错误。主端可拒绝将该业务样本用于控制/力反馈，但通信统计和会话识别应与业务数值有效性分开。测量缺失时采样时间为 0，teleop_state=OFF。若只需核对网络与时延，两端都选 kind=1，载荷必须是全零原始字节，避免依赖机器人样本。禁止把 NaN 替换为伪造的实测零角度/零力矩。

## 7. 修改文件索引

| 文件 | 内容 |
| --- | --- |
| `UR_Utils/URUdpClient.py` | v2 编解码、模式/端点/序号准入、t2/t3、回显快照及逐包日志 |
| `UR_Utils/udp_events.py` | 整数时钟、采用快照、测试入口显式开启的后台日志 |
| `udp_settings.py`、`udp_diagnostic.py`、`udp_log_summary.py` | 端点/模式配置、无驱动诊断入口、离线序号核对 |
| `main.py` | 全生命周期独立反馈、正常模式收发、提交序号及采样时间汇集 |
| `UR_Utils/URRealtimeClient.py`、`URRealtimeMonitor.py` | q/dq 源更新时刻及跨进程状态快照 |
| `UR_Utils/URRTDEController.py` | 普通 servoJ 采用、普通补偿力矩时间及反馈快照 |
| `UR_Utils/URImpedanceController.py`、`impedance_process.py` | 阻抗接口采用、力矩/加速度源时间、跨进程反馈快照 |
| `UR_Utils/impedance_reference.py`、`impedance_sources.py`、`teleop_impedance_window.py` | 不可变目标身份从接收到控制子进程的传递；仅业务数据，无 UI 结构改动 |
| `UR_Utils/URAdmittanceController.py`、`admittance_process.py` | 导纳接口采用、载荷页时间、身份跨进程传递 |
| `teleop_impedance_manual.py` | 既有人工设备联调发布器升级 v2，未执行该入口 |
| `.gitignore`、`README.md`、`URControl_项目现状.md`、本文 | 运行日志忽略规则和现行文档 |
