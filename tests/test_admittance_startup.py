"""导纳上传与 Primary 异常解包的离线回归，不连接机器人。"""

from pathlib import Path
import struct
import unittest
from unittest.mock import patch

from UR_Utils import admittance_config
from UR_Utils.URScriptClient import URScriptClient
from UR_Utils.URTcpClient import URTcpError


def runtime_error(line=226, column=10, message=b"compile_error_name_not_found:abs:"):
    body = struct.pack(">QbBII", 123456, -2, 10, line, column) + message
    return struct.pack(">IB", 5 + len(body), 20) + body


class ScriptDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.client = URScriptClient("127.0.0.1", port=30001)

    def test_fragmented_error_preserves_line_and_reason(self):
        packet = runtime_error()
        for fragment in (packet[:2], packet[2:17], packet[17:-3]):
            self.client._script_response_buffer.extend(fragment)
            self.assertIsNone(self.client._consume_script_messages())
        self.client._script_response_buffer.extend(packet[-3:])
        self.assertEqual(
            self.client._consume_script_messages(),
            "URScript 错误（第 226 行，第 10 列）：compile_error_name_not_found:abs:",
        )
        self.assertFalse(self.client._script_response_buffer)

    def test_coalesced_state_and_errors_keep_first_cause(self):
        state = struct.pack(">IB", 8, 16) + b"abc"
        self.client._script_response_buffer.extend(
            state + runtime_error() + runtime_error(300, 2, b"later_error") + state[:3]
        )
        self.assertIn("name_not_found:abs", self.client._consume_script_messages())
        self.assertEqual(self.client._script_response_buffer, state[:3])
        self.client._script_response_buffer.extend(state[3:])
        self.assertIsNone(self.client._consume_script_messages())

    def test_text_message_is_not_a_script_error(self):
        body = struct.pack(">QbB", 1, -2, 0) + b"hello"
        self.client._script_response_buffer.extend(struct.pack(">IB", 5 + len(body), 20) + body)
        self.assertIsNone(self.client._consume_script_messages())

    def test_invalid_length_fails_without_waiting_for_more_bytes(self):
        for size in (0, 4, 1024 * 1024 + 1):
            with self.subTest(size=size):
                self.client._script_response_buffer = bytearray(struct.pack(">IB", size, 20))
                with self.assertRaises(URTcpError):
                    self.client._consume_script_messages()

    def test_poll_does_not_read_when_socket_is_not_ready(self):
        with patch.object(self.client, "_ensure_connected", return_value=object()), \
                patch("UR_Utils.URScriptClient.select.select", return_value=([], [], [])), \
                patch.object(self.client, "recv_once") as recv:
            self.assertIsNone(self.client.poll_script_error())
            recv.assert_not_called()

    def test_poll_reads_fragmented_packets_without_waiting(self):
        packet = runtime_error()
        with patch.object(self.client, "_ensure_connected", return_value=object()), \
                patch("UR_Utils.URScriptClient.select.select", side_effect=[
                    ([1], [], []), ([1], [], []), ([], [], [])]), \
                patch.object(self.client, "recv_once", side_effect=[packet[:12], packet[12:]]):
            self.assertIn("name_not_found:abs", self.client.poll_script_error())


class ScriptRenderingTests(unittest.TestCase):
    def test_four_modes_are_ascii_and_preserve_template_line_numbers(self):
        template = Path(admittance_config.__file__).with_name("scripts").joinpath(
            "admittance_control.script").read_text(encoding="utf-8")
        for source in ("teleop", "free"):
            for space in ("joint", "cartesian"):
                with self.subTest(source=source, space=space):
                    script = admittance_config.render_script(source, space)
                    script.encode("ascii")
                    self.assertEqual(len(script.splitlines()), len(template.splitlines()))
                    self.assertNotIn("${", script)
                    self.assertTrue(script.endswith("end\n"))


if __name__ == "__main__":
    unittest.main()
