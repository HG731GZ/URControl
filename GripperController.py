"""
CTAG2F90D 夹钳控制器 SDK
基于 Modbus RTU (RS-485)，支持物理串口与 TCP 串口两种连接方式。

控制周期：1 条批量写 (0x10, 0x0102~0x0108) +
           1 条读状态 (0x03, 0x0401) +
           1 条读反馈 (0x03, 0x0418~0x041B) = 共 3 条 Modbus 指令

依赖: pip install minimalmodbus pyserial
"""

import time
import threading
import logging
import minimalmodbus
import serial

logger = logging.getLogger(__name__)

# ========================
# 寄存器地址
# ========================
# ----- 写入 (功能码 0x10) -----
REG_TARGET_POS_HI = 0x0102  # 目标位置 高16位
REG_TARGET_POS_LO = 0x0103  # 目标位置 低16位
REG_TARGET_SPEED = 0x0104  # 目标速度
REG_TARGET_FORCE = 0x0105  # 目标力/力矩
REG_TARGET_ACCEL = 0x0106  # 目标加速度
REG_TARGET_DECEL = 0x0107  # 目标减速度
REG_MOTION_TRIGGER = 0x0108  # 运动触发

# ----- 读取 (功能码 0x03) -----
REG_STATUS = 0x0401  # 电机运行状态 (bit位)
REG_REAL_POS_HI = 0x0418  # 当前位置 高16位
REG_REAL_POS_LO = 0x0419  # 当前位置 低16位
REG_REAL_SPEED = 0x041A  # 当前速度
REG_REAL_CURRENT = 0x041B  # 当前电流/力矩

# ----- 状态位 -----
BIT_POS_REACHED = 0x01  # bit 0: 位置到达
BIT_SPEED_REACHED = 0x02  # bit 1: 速度到达
BIT_TORQUE_REACHED = 0x04  # bit 2: 力矩到达


class GripperFeedback:
    """单次控制周期的反馈数据"""

    __slots__ = ("status", "position", "speed", "current", "pos_reached", "speed_reached", "torque_reached")

    def __init__(self, status=0, position=0, speed=0, current=0):
        self.status = status
        self.position = position
        self.speed = speed
        self.current = current
        self.pos_reached = bool(status & BIT_POS_REACHED)
        self.speed_reached = bool(status & BIT_SPEED_REACHED)
        self.torque_reached = bool(status & BIT_TORQUE_REACHED)

    def __repr__(self):
        return f"GripperFeedback(pos={self.position}, speed={self.speed}, " f"current={self.current}, arrived={self.pos_reached}, " f"torque_ok={self.torque_reached})"


class GripperController:
    """CTAG2F90D 夹钳控制器

    用法:
        # 物理串口
        g = Gripper(port='/dev/ttyUSB0', slave_id=1, connection_type='serial')

        # TCP 串口 (如 USR-TCP232、Elfin EW11 等串口服务器)
        g = Gripper(port='192.168.1.100:8899', slave_id=1, connection_type='tcp')

        g.on_feedback(lambda status, pos, speed, cur: print(f"{pos=} {cur=}"))
        g.start()

        g.move(9000, speed=50, force=25)   # 暂停→改参→恢复
        time.sleep(3)
        g.move(0, speed=50, force=25)

        g.stop()
        g.close()
    """

    def __init__(self, port, slave_id=1, connection_type="serial", baudrate=115200, timeout=0.5, debug=False):
        """
        参数:
            port: 串口路径 (connection_type='serial') 或
                  'host:port' (connection_type='tcp')
            slave_id: Modbus 从机地址
            connection_type: 'serial' 或 'tcp'
            baudrate: 串口波特率 (TCP 模式下被忽略)
            timeout: 收发超时 (秒)
            debug: True 时每个控制周期打印通讯详情
        """
        self.debug = debug
        self.slave_id = slave_id
        self.connection_type = connection_type
        self.baudrate = baudrate
        self.timeout = timeout

        if connection_type == "tcp":
            if ":" not in port:
                raise ValueError("TCP 模式下 port 需为 'host:port' 格式")
            host, tcp_port = port.rsplit(":", 1)
            self._url = f"socket://{host}:{tcp_port}"
        else:
            self._url = port

        # ---- 线程同步 ----
        self._lock = threading.Lock()
        self._pause = threading.Event()
        self._pause.set()  # 初始非暂停
        self._stop = threading.Event()
        self._thread = None
        self._interval = 0.05

        # ---- 目标参数 ----
        self._target_position = 0
        self._target_speed = 0
        self._target_force = 0
        self._target_accel = 60
        self._target_decel = 60
        self._dirty = False  # 参数变更标记，下一周期触发运动

        # ---- 最新反馈 ----
        self.feedback = GripperFeedback()

        # ---- 回调 ----
        self._on_feedback = None

        # ---- 连接 ----
        self.instrument = None
        self._connect()

    # ========================
    # 连接
    # ========================
    def _connect(self):
        """建立 Modbus RTU 连接"""
        ser = serial.serial_for_url(
            self._url,
            baudrate=self.baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=self.timeout,
        )
        self.instrument = minimalmodbus.Instrument(ser, self.slave_id)
        self.instrument.mode = minimalmodbus.MODE_RTU
        logger.info("已连接: %s (slave=%d)", self._url, self.slave_id)

    # ========================
    # 反馈回调
    # ========================
    def on_feedback(self, callback):
        """设置反馈回调 callback(status, position, speed, current)"""
        self._on_feedback = callback

    # ========================
    # 参数设置 (线程安全)
    # ========================
    def set_target_position(self, pos):
        with self._lock:
            self._target_position = pos & 0xFFFFFFFF
            self._dirty = True

    def set_target_speed(self, speed):
        with self._lock:
            self._target_speed = speed & 0xFFFF
            self._dirty = True

    def set_target_force(self, force):
        with self._lock:
            self._target_force = force & 0xFFFF
            self._dirty = True

    def set_target_accel(self, accel):
        with self._lock:
            self._target_accel = accel & 0xFFFF
            self._dirty = True

    def set_target_decel(self, decel):
        with self._lock:
            self._target_decel = decel & 0xFFFF
            self._dirty = True

    def set_motion_params(self, position=None, speed=None, force=None, accel=None, decel=None):
        """批量修改运动参数 (不立即生效，等待 resume 或下一周期)"""
        with self._lock:
            if position is not None:
                self._target_position = position & 0xFFFFFFFF
            if speed is not None:
                self._target_speed = speed & 0xFFFF
            if force is not None:
                self._target_force = force & 0xFFFF
            if accel is not None:
                self._target_accel = accel & 0xFFFF
            if decel is not None:
                self._target_decel = decel & 0xFFFF
            self._dirty = True

    # ========================
    # 线程控制
    # ========================
    def start(self, interval=0.05):
        """启动控制线程 (每 interval 秒执行一次完整控制周期)"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._interval = interval
        self._stop.clear()
        self._pause.set()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        logger.info("控制线程已启动 (间隔 %.3fs)", interval)

    def stop(self):
        """停止控制线程"""
        self._stop.set()
        self._pause.set()  # 解除暂停，允许退出
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        logger.info("控制线程已停止")

    def pause(self):
        """暂停控制循环 (参数仍可通过 set_* 修改)"""
        self._pause.clear()
        logger.info("控制线程已暂停")

    def resume(self):
        """恢复控制循环"""
        self._pause.set()
        logger.info("控制线程已恢复")

    @property
    def is_paused(self):
        return not self._pause.is_set()

    @property
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def move(self, position, speed=None, force=None, accel=None, decel=None, block=False):
        """
        便捷方法: 暂停 → 修改参数 → 恢复

        block=True 时阻塞等待位置到达
        """
        self.pause()
        self.set_motion_params(position, speed, force, accel, decel)
        self.resume()
        if block:
            while not self.feedback.pos_reached:
                time.sleep(0.01)

    # ========================
    # 控制循环 (3 条 Modbus 指令/周期)
    # ========================
    def _control_loop(self):
        while not self._stop.is_set():
            self._pause.wait()
            if self._stop.is_set():
                break

            try:
                # ---- 快照当前目标参数 ----
                with self._lock:
                    dirty = self._dirty
                    self._dirty = False
                    trigger = 1 if dirty else 0
                    pos = self._target_position
                    speed = self._target_speed
                    force = self._target_force
                    accel = self._target_accel
                    decel = self._target_decel

                # ---- 1. 批量写: 目标参数 + 触发 (0x0102 → 0x0108) ----
                pos_hi = (pos >> 16) & 0xFFFF
                pos_lo = pos & 0xFFFF
                self.instrument.write_registers(
                    REG_TARGET_POS_HI,
                    [pos_hi, pos_lo, speed, force, accel, decel, trigger],
                )

                # ---- 2. 读运行状态 (0x0401) ----
                status = self.instrument.read_register(REG_STATUS)

                # ---- 3. 读反馈 (0x0418 → 0x041B) ----
                regs = self.instrument.read_registers(REG_REAL_POS_HI, 4)
                real_pos = (regs[0] << 16) | regs[1]
                if real_pos & 0x80000000:
                    real_pos -= 0x100000000
                real_speed = regs[2]
                real_current = regs[3]

                # ---- 更新反馈 ----
                fb = GripperFeedback(status, real_pos, real_speed, real_current)
                self.feedback = fb

                if self.debug:
                    self._debug_print(pos, speed, force, accel, decel, trigger,
                                      status, regs, real_pos, real_speed, real_current)

                if self._on_feedback:
                    try:
                        self._on_feedback(status, real_pos, real_speed, real_current)
                    except Exception:
                        pass

            except Exception as e:
                logger.error("控制周期异常: %s", e)

            time.sleep(self._interval)

    def _debug_print(self, pos, speed, force, accel, decel, trigger,
                     status, regs, real_pos, real_speed, real_current):
        """打印单次控制周期的通讯详情"""
        pos_hi = (pos >> 16) & 0xFFFF
        pos_lo = pos & 0xFFFF
        print(f"\n{'='*60}")
        print(f"[写] 功能码 0x10  起始地址 0x0102 → 7 个寄存器")
        print(f"  目标位置     0x0102-0x0103  {pos_hi:04X} {pos_lo:04X}  → {pos} (0x{pos:08X})")
        print(f"  目标速度     0x0104          {speed:04X}            → {speed}")
        print(f"  目标力/力矩  0x0105          {force:04X}            → {force}")
        print(f"  目标加速度   0x0106          {accel:04X}            → {accel}")
        print(f"  目标减速度   0x0107          {decel:04X}            → {decel}")
        print(f"  运动触发     0x0108          {trigger:04X}            → {'触发' if trigger else '空闲'}")

        # 解析状态位
        bits = []
        if status & 0x01:
            bits.append("bit0:位置到达")
        if status & 0x02:
            bits.append("bit1:速度到达")
        if status & 0x04:
            bits.append("bit2:力矩到达")
        # 其他可能的状态位
        for b in range(3, 16):
            if status & (1 << b):
                bits.append(f"bit{b}")

        print(f"[读] 功能码 0x03  起始地址 0x0401 → 1 个寄存器")
        print(f"  运行状态     0x0401          {status:04X}            → {status}  位: {' | '.join(bits) if bits else '无'}")

        print(f"[读] 功能码 0x03  起始地址 0x0418 → 4 个寄存器")
        print(f"  当前位置 HI  0x0418          {regs[0]:04X}")
        print(f"  当前位置 LO  0x0419          {regs[1]:04X}            → {real_pos}")
        print(f"  当前速度     0x041A          {regs[2]:04X}            → {real_speed}")
        print(f"  当前电流     0x041B          {regs[3]:04X}            → {real_current}")
        print(f"{'='*60}")

    # ========================
    # 手动操作
    # ========================
    def read_status(self):
        """手动读取运行状态字 (0x0401)"""
        return self.instrument.read_register(REG_STATUS)

    def read_feedback(self):
        """手动读取反馈: 位置、速度、电流 (0x0418~0x041B)"""
        regs = self.instrument.read_registers(REG_REAL_POS_HI, 4)
        pos = (regs[0] << 16) | regs[1]
        if pos & 0x80000000:
            pos -= 0x100000000
        return {"status": self.read_status(), "position": pos, "speed": regs[2], "current": regs[3]}

    def close(self):
        """关闭线程并断开连接"""
        self.stop()
        if self.instrument is not None and self.instrument.serial is not None:
            try:
                self.instrument.serial.close()
            except Exception:
                pass
            self.instrument = None
        logger.info("连接已关闭")


# ========================
# 示例
# ========================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    # --- 物理串口 ---
    # gripper = Gripper(port='/dev/ttyUSB0', slave_id=1, connection_type='serial')

    # --- TCP 串口服务器 ---
    gripper = GripperController(port="192.168.3.15:54321", slave_id=1, connection_type="tcp")

    def on_data(status, pos, speed, current):
        arrived = "✓" if (status & BIT_POS_REACHED) else "✗"
        print(f"  pos={pos:6d}  speed={speed:3d}  cur={current:3d}  arrived={arrived}")

    gripper.on_feedback(on_data)
    gripper.start(interval=0.1)

    try:
        for i in range(5):
            print(f"\n--- 第 {i+1}/5 次 ---")
            gripper.move(9000, speed=20, force=25)
            time.sleep(3)
            gripper.move(0, speed=20, force=25)
            time.sleep(3)

        # 最终张开
        gripper.move(0, speed=20, force=25)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        gripper.close()
