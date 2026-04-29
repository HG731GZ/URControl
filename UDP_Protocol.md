# URControl UDP 通讯协议说明

## 概述

本协议用于 URControl 项目通过 UDP 收发机械臂控制指令。数据包为固定长度，包含帧头/帧尾定界、CRC 校验和结构化数据载荷。

## 物理层

| 项目 | 参数 |
|------|------|
| 传输协议 | UDP |
| 字节序 | 大端 (Network Byte Order) |
| 对齐 | 无填充 |
| 帧长度 | **85 字节**（固定） |
| 默认端口 | 5005 |

## 帧结构

```
 0               4                              77          81   85
┌───────────────┬───────────────────────────────┬───────────┬───────────┐
│   帧头 (4B)    │         数据段 (73B)           │ CRC32 (4B) │ 帧尾 (4B)  │
│ 55 AA 55 AA   │  6×float64 + uint8 + 3×float64 │           │ 0D 0A 0D 0A│
└───────────────┴───────────────────────────────┴───────────┴───────────┘
```

| offset | 长度 | 类型 | 字段 | 说明 |
|--------|------|------|------|------|
| 0 | 4 | `uint8[4]` | **帧头** | 固定 `0x55 0xAA 0x55 0xAA` |
| 4 | 8 | `float64` | q[0] | 机械臂广义坐标 1 |
| 12 | 8 | `float64` | q[1] | 机械臂广义坐标 2 |
| 20 | 8 | `float64` | q[2] | 机械臂广义坐标 3 |
| 28 | 8 | `float64` | q[3] | 机械臂广义坐标 4 |
| 36 | 8 | `float64` | q[4] | 机械臂广义坐标 5 |
| 44 | 8 | `float64` | q[5] | 机械臂广义坐标 6 |
| 52 | 1 | `uint8` | **mode** | 控制模式 |
| 53 | 8 | `float64` | grip[0] | 夹钳广义坐标 1 |
| 61 | 8 | `float64` | grip[1] | 夹钳广义坐标 2 |
| 69 | 8 | `float64` | grip[2] | 夹钳广义坐标 3 |
| 77 | 4 | `uint32` | **CRC32** | 数据段校验 (offset 4~76) |
| 81 | 4 | `uint8[4]` | **帧尾** | 固定 `0x0D 0x0A 0x0D 0x0A` |

## 控制模式

| 值 | 名称 | 含义 | q[0..5] 语义 |
|----|------|------|-------------|
| 0 | `OFF` | 关闭 | — |
| 1 | `JOINT_TRACK` | 关节角闭环跟踪 | 目标关节角 (rad) |
| 2 | `JOINT_DELTA` | 关节角增量控制 | 关节角增量 (rad) |
| 3 | `TCP_DELTA` | 末端位姿增量控制 | 末端增量 [dx,dy,dz, drx,dry,drz] (m, rad) |
| 4 | `TCP_VEL` | 末端速度控制 | 末端速度 [vx,vy,vz, wx,wy,wz] (m/s, rad/s) |

## CRC 校验

| 项目 | 参数 |
|------|------|
| 算法 | CRC32 / IEEE 802.3 |
| 多项式 | `0xEDB88320` (reversed) |
| 校验范围 | offset 4 ~ 76（数据段 73 字节） |
| 字节序 | 大端 |
| Python 实现 | `binascii.crc32(data) & 0xFFFFFFFF` |

## 数据包示例

```
帧头:  55 AA 55 AA
数据:  3F F0 00 00 00 00 00 00  40 0B 14 7A E1 47 AE 14  ...  (73 bytes)
CRC:   1A 2B 3C 4D
帧尾:  0D 0A 0D 0A
```

## 校验流程

接收端按以下顺序逐项校验，任一失败则丢弃该帧并记录错误：

1. **长度检查** — 收到的字节数必须等于 85
2. **帧头检查** — offset 0~3 必须等于 `0x55 0xAA 0x55 0xAA`
3. **CRC 校验** — 对 offset 4~76 计算 CRC32，必须与 offset 77~80 一致
4. **帧尾检查** — offset 81~84 必须等于 `0x0D 0x0A 0x0D 0x0A`

## 跨语言实现参考

**C 结构体 (packed):**

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[4];       // {0x55, 0xAA, 0x55, 0xAA}
    double   q[6];            // 机械臂广义坐标
    uint8_t  mode;            // 控制模式
    double   grip[3];         // 夹钳广义坐标
    uint32_t crc;             // CRC32 of q, mode, grip
    uint8_t  footer[4];       // {0x0D, 0x0A, 0x0D, 0x0A}
} udp_frame_t;
#pragma pack(pop)
```

**Python 打包:**

```python
import struct, binascii

HEADER = b'\x55\xAA\x55\xAA'
FOOTER = b'\x0D\x0A\x0D\x0A'

data = struct.pack('!6dB3d', *q, mode, *grip)
crc  = struct.pack('!I', binascii.crc32(data) & 0xFFFFFFFF)
frame = HEADER + data + crc + FOOTER  # 85 bytes
```

**Python 解包:**

```python
import struct, binascii

header  = frame[0:4]
data    = frame[4:77]
crc_rx  = struct.unpack('!I', frame[77:81])[0]
footer  = frame[81:85]

assert header  == b'\x55\xAA\x55\xAA'
assert footer  == b'\x0D\x0A\x0D\x0A'
assert crc_rx  == (binascii.crc32(data) & 0xFFFFFFFF)

q     = list(struct.unpack('!6d', data[0:48]))
mode  = data[48]
grip  = list(struct.unpack('!3d', data[49:73]))
```

## 夹钳坐标说明

夹钳广义坐标 `grip[0..2]` 共 3 个 float64，具体语义取决于所使用的夹钳型号和控制方式，预留由夹钳控制模块解释。
