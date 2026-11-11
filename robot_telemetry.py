"""机器人遥测：标准状态与已有控制通道的补偿力矩缓存分别记录，不上传脚本。"""

from pathlib import Path
import threading
import time

from admittance_logger import AdmittanceLogger
from UR_Utils.admittance_telemetry import ROBOT_FIELDS, robot_sample
from UR_Utils.admittance_transport import AdmittanceRTDE


TELEMETRY_FREQUENCY_HZ = 500
RATIO_FIELDS = ("joint_position_deviation_ratios", "collision_detection_ratios")


class RobotTelemetryRecorder:
    """每次点击开始创建一个实例；停止后保留状态和文件路径供界面展示。"""

    def __init__(self, robot_ip, *, compensated_torque_provider, base_dir=None):
        self.robot_ip = robot_ip
        self._compensated_torque_provider = compensated_torque_provider
        self._logger = AdmittanceLogger(
            base_dir=Path(__file__).resolve().parent / "logs" / "telemetry"
            if base_dir is None else base_dir)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._receive_loop, name="RobotTelemetry", daemon=True)
        self._status = {"phase": "starting", "error": None, "sample_count": 0, "latest": None,
                        "compensated_joint_torque": None}

    def start(self):
        """开启后台连接；主界面无需等待网络协商或磁盘操作。"""
        self._logger.start({
            "run_id": f"telemetry_{time.time_ns()}",
            "recording_kind": "robot_telemetry",
            "robot_ip": self.robot_ip,
            "frequency_hz": TELEMETRY_FREQUENCY_HZ,
            "robot_fields": {
                name: {"rtde_type": field_type, "unit": unit, "description": description}
                for name, field_type, unit, description in ROBOT_FIELDS
            },
            "joint_torque_schema": {
                "version": 1, "unit": "N*m", "axis_order": ["J1", "J2", "J3", "J4", "J5", "J6"],
                "raw": "payload.raw.actual_current_as_torque，实际电流换算力矩，保留重力、摩擦等分量",
                "compensated": "payload.compensated_joint_torque.sample.values_nm，UR get_joint_torques() 补偿力矩，未减导纳偏置、未滤波、未过死区",
                "availability": "status=available 时 sample 有效；unavailable 时 sample=null 并说明原因；读取错误结束本轮并报告",
                "ordinary_time": "source=rtde_control_getJointTorques：保留调用的主机起止时间；robot_timestamp_s 为返回时关联的 RTDE 时间，不是精确力矩采样时刻",
                "admittance_time": "source=admittance_load_page：sample_time_s 为脚本相对采样时间，run_id/model_sample_seq/page_seq 标识原载荷页；重复身份不是新测量",
                "impedance_time": "source=impedance_get_joint_torques：run_id/sample_seq 标识力矩测量，robot_timestamp_s 为首次观测该样本的 RTDE 帧时间",
                "sampling": "标准状态目标 500 Hz；补偿力矩复用独立后台缓存，保留来源时间，不保证同频或与 raw 同步",
            },
            "scope": "标准 RTDE 字段与当前控制通道的补偿力矩缓存；导纳内部模型、片段与伺服计时由导纳诊断日志记录",
        })
        self._thread.start()

    def request_stop(self):
        """仅停止本次记录，不发送机器人停止或其他控制命令。"""
        self._stop.set()
        with self._lock:
            if self._status["phase"] != "finished":
                self._status["phase"] = "stopping"

    def close(self):
        """断开设备或关闭应用时等待接收结束及日志排空。"""
        self.request_stop()
        self._thread.join()

    def get_status(self):
        """返回已发布快照；原始样本发布后不再修改。"""
        with self._lock:
            status = dict(self._status)
        status["log"] = self._logger.get_status()
        return status

    def _receive_loop(self):
        """独立采样，明确记录接收错误与观测缺口，不自动重连。"""
        connection = None
        summary = {
            "sample_count": 0, "first_timestamp_s": None, "last_timestamp_s": None,
            "max_timestamp_interval_s": None, "timestamp_gap_count": 0,
            "estimated_missing_frames": 0, "nonpositive_timestamp_interval_count": 0,
            "max_actual_execution_time_ms": None,
            "max_ratios": {name: None for name in RATIO_FIELDS},
            "max_abs_raw_joint_torque_nm": None, "max_abs_compensated_joint_torque_nm": None,
            "compensated_available_frame_count": 0, "compensated_unavailable_frame_count": 0,
            "compensated_source_frame_counts": {},
            "compensated_count_scope": "按附带有效补偿缓存的标准 RTDE 帧计数，不是独立补偿采样数量",
            "gap_scope": "只统计已接收 RTDE 样本的时间缺口，不表示伺服帧缺失",
            "error": None,
        }
        try:
            connection = AdmittanceRTDE(self.robot_ip, buffer_limit_bytes=65536)
            connection.connect()
            version = connection.get_controller_version()
            names = [field[0] for field in ROBOT_FIELDS]
            types = [field[1] for field in ROBOT_FIELDS]
            connection.send_output_setup(names, types, frequency=TELEMETRY_FREQUENCY_HZ)
            self._logger.update_context({"controller_version": list(version)})
            if not connection.send_start():
                raise RuntimeError("控制器拒绝开始遥测同步")
            with self._lock:
                self._status["phase"] = "stopping" if self._stop.is_set() else "recording"
            while not self._stop.is_set():
                state = connection.receive()
                if state is None:
                    raise TimeoutError("遥测一秒内未收到机器人状态")
                raw = robot_sample(state)
                compensated = self._compensated_torque_provider()
                if compensated["status"] == "error":
                    raise RuntimeError(f"补偿力矩读取失败：{compensated['reason']}")
                accepted = self._logger.record("robot_telemetry", {
                    "host_monotonic_s": time.monotonic(), "raw": raw,
                    "compensated_joint_torque": compensated,
                })
                if not accepted:
                    log_error = self._logger.get_status()["error"]
                    if log_error is not None:
                        raise RuntimeError(f"遥测写盘失败：{log_error}")
                previous = summary["last_timestamp_s"]
                if previous is None:
                    summary["first_timestamp_s"] = raw["timestamp"]
                    summary["max_actual_execution_time_ms"] = raw["actual_execution_time"]
                    summary["max_ratios"] = {name: raw[name] for name in RATIO_FIELDS}
                    summary["max_abs_raw_joint_torque_nm"] = [abs(value) for value in raw["actual_current_as_torque"]]
                else:
                    interval = raw["timestamp"] - previous
                    maximum = summary["max_timestamp_interval_s"]
                    summary["max_timestamp_interval_s"] = interval if maximum is None else max(maximum, interval)
                    summary["nonpositive_timestamp_interval_count"] += int(interval <= 0)
                    if interval > 1.5 / TELEMETRY_FREQUENCY_HZ:
                        summary["timestamp_gap_count"] += 1
                        summary["estimated_missing_frames"] += round(interval * TELEMETRY_FREQUENCY_HZ) - 1
                    summary["max_actual_execution_time_ms"] = max(
                        summary["max_actual_execution_time_ms"], raw["actual_execution_time"])
                    summary["max_ratios"] = {
                        name: [max(old, current) for old, current in zip(summary["max_ratios"][name], raw[name])]
                        for name in RATIO_FIELDS
                    }
                    summary["max_abs_raw_joint_torque_nm"] = [
                        max(old, abs(value)) for old, value in
                        zip(summary["max_abs_raw_joint_torque_nm"], raw["actual_current_as_torque"])
                    ]
                if compensated["status"] == "available":
                    summary["compensated_available_frame_count"] += 1
                    sample = compensated["sample"]
                    counts = summary["compensated_source_frame_counts"]
                    counts[sample["source"]] = counts.get(sample["source"], 0) + 1
                    torques = [abs(value) for value in sample["values_nm"]]
                    maximum = summary["max_abs_compensated_joint_torque_nm"]
                    summary["max_abs_compensated_joint_torque_nm"] = (torques if maximum is None else
                        [max(old, current) for old, current in zip(maximum, torques)])
                else:
                    summary["compensated_unavailable_frame_count"] += 1
                summary["last_timestamp_s"] = raw["timestamp"]
                summary["sample_count"] += 1
                with self._lock:
                    self._status.update(sample_count=summary["sample_count"], latest=raw,
                                        compensated_joint_torque=compensated)
        except Exception as exc:
            # 后台异常必须进入界面和结束摘要，不能让线程静默退出。
            summary["error"] = f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._status["error"] = summary["error"]
        finally:
            if connection is not None:
                connection.disconnect()
            self._logger.finish(summary)
            with self._lock:
                self._status["phase"] = "finished"
