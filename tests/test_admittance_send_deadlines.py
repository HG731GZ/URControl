"""离线验证发送观测与退出期限；不连接机器人或模拟器。"""

from contextlib import ExitStack
import importlib
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from UR_Utils import admittance_config, admittance_transport
from analyze_admittance_log import HostBudgetSummary


controller_module = importlib.import_module("UR_Utils.URAdmittanceController")


class Clock:
    now = 100.0

    def sample(self):
        return self.now, self.now / 100


class Socket:
    def __init__(self, clock, writes):
        self.clock, self.writes, self.buffers = clock, iter(writes), []

    def send(self, data):
        self.buffers.append(bytes(data))
        count, duration = next(self.writes)
        self.clock.now += duration
        return count


class Log:
    def __init__(self):
        self.records = []

    def record(self, kind, payload):
        self.records.append({"kind": kind, "payload": payload, "seq": len(self.records) + 1})

    def update_context(self, context):
        pass


class SendDeadlineTests(unittest.TestCase):
    def run_send(self, config, writes, prepare_s=0.0, error=None):
        clock, log = Clock(), Log()
        controller = controller_module.URAdmittanceController(
            "127.0.0.1", "fixed", "joint", config=config, log_sink=log)
        connection = admittance_transport.AdmittanceRTDE.__new__(admittance_transport.AdmittanceRTDE)
        connection._sock = Socket(clock, writes)

        def prepare(packet):
            clock.now += prepare_s
            return bytes(range(84))

        connection.prepare_inputs = prepare
        controller._connection = connection
        with ExitStack() as stack:
            stack.enter_context(patch.object(controller_module, "clock_sample", clock.sample))
            stack.enter_context(patch.object(admittance_transport, "clock_sample", clock.sample))
            stack.enter_context(patch.object(admittance_transport.select, "select",
                                            lambda r, w, x, timeout: ([], w, [])))
            if error is None:
                self.assertFalse(controller._send_inputs(SimpleNamespace(), heartbeat=True))
            else:
                with self.assertRaises(error):
                    controller._send_inputs(SimpleNamespace(), heartbeat=True)
        return controller, connection, log

    def test_late_complete_packet_is_observed_for_default_and_candidate(self):
        for config in (admittance_config.snapshot(), admittance_config.j1_no_contact_config()):
            with self.subTest(mode=config["DIAGNOSTIC_MODE"]):
                c, conn, log = self.run_send(config, [(84, .003)])
                self.assertEqual(c.get_status()["send_warning_count"], 1)
                self.assertEqual(c.get_status()["heartbeat_send_count"], 1)
                self.assertEqual(conn.last_send_timing["sent_bytes"], 84)
                record = log.records[-1]
                self.assertEqual(record["kind"], "rtde_input")
                self.assertTrue(record["payload"]["send_warning_exceeded"])
                budget = HostBudgetSummary()
                budget.add(record)
                self.assertEqual(len(budget.send_warnings), 1)
                self.assertEqual(budget.send_failures, [])

    def test_default_exits_after_ten_milliseconds(self):
        c, conn, log = self.run_send(admittance_config.snapshot(), [(84, .011)], error=TimeoutError)
        self.assertEqual(conn.last_send_timing["sent_bytes"], 84)
        self.assertEqual(c.get_status()["heartbeat_send_count"], 0)
        self.assertEqual(log.records[-1]["kind"], "rtde_input_send_failed")

    def test_preparation_consumes_same_deadline(self):
        c, conn, log = self.run_send(admittance_config.j1_no_contact_config(),
                                     [(84, .006)], prepare_s=.005, error=TimeoutError)
        self.assertEqual(c.get_status()["heartbeat_send_count"], 0)
        self.assertTrue(log.records[-1]["payload"]["send_warning_exceeded"])

    def test_partial_write_does_not_restart_deadline_or_resend_prefix(self):
        _, conn, _ = self.run_send(admittance_config.j1_no_contact_config(),
                                   [(20, .006), (64, .006)], error=TimeoutError)
        self.assertEqual(conn._sock.buffers, [bytes(range(84)), bytes(range(20, 84))])
        self.assertEqual(conn.last_send_timing["send_calls"], 2)

    def test_zero_write_remains_failure(self):
        _, _, log = self.run_send(admittance_config.j1_no_contact_config(), [(0, .001)],
                                  error=ConnectionError)
        self.assertEqual(log.records[-1]["kind"], "rtde_input_send_failed")
        self.assertFalse(log.records[-1]["payload"]["send_warning_exceeded"])

    def test_small_packet_and_missing_old_observation_are_distinguished(self):
        c, _, log = self.run_send(admittance_config.j1_no_contact_config(), [(84, .001)])
        self.assertEqual(c.get_status()["send_warning_count"], 0)
        record = log.records[-1]
        record["payload"].pop("send_warning_exceeded")
        record["payload"].pop("send_warning_threshold_s")
        budget = HostBudgetSummary()
        budget.add(record)
        self.assertEqual(budget.send_warning_observed, 0)
        self.assertEqual(budget.send_warning_missing, 1)

    def test_invalid_deadline_order_is_rejected(self):
        for warning, timeout in ((.011, .010), (.002, .020), (.002, float("nan"))):
            with self.subTest(warning=warning, timeout=timeout):
                config = admittance_config.j1_no_contact_config()
                config.update(ADMITTANCE_RTDE_SEND_WARNING_S=warning,
                              ADMITTANCE_RTDE_SEND_TIMEOUT_S=timeout)
                with self.assertRaises(ValueError):
                    admittance_config.validate_run_config("fixed", "joint", config=config)


if __name__ == "__main__":
    unittest.main()
