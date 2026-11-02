"""读取单轴调参的本轮日志，统计完整采样误差，不使用界面刷新缓存。"""

import json
import math
from pathlib import Path

from PyQt5 import QtCore


def error_metrics(errors):
    """输入角度误差，输出以度为单位的采样统计。"""
    if not errors:
        return None
    return dict(
        sample_count=len(errors),
        rmse_deg=math.sqrt(sum(e * e for e in errors) / len(errors)),
        max_abs_deg=max(map(abs, errors)),
        last_error_deg=errors[-1],
    )


class TuningReportWorker(QtCore.QThread):
    """日志落盘结束后后台读取，避免阻塞控制界面的事件循环。"""

    def __init__(self, directory, run_id, axis, final_angle, parent=None):
        super().__init__(parent)
        self.directory = Path(directory)
        self.run_id, self.axis, self.final_angle = run_id, axis, final_angle
        self.result = None
        self.error = None

    def run(self):
        try:
            self.result = self._read()
        except Exception as exc:
            self.error = str(exc)

    def _read(self):
        summary = json.loads((self.directory / "summary.json").read_text())
        metadata = json.loads((self.directory / "metadata.json").read_text())
        if summary.get("run_id") != self.run_id:
            raise ValueError("日志轮次与本次调参不一致")
        references, samples, seen = {}, [], set()
        with (self.directory / "records.jsonl").open() as stream:
            for line in stream:
                row = json.loads(line)
                payload = row["payload"]
                if row["kind"] == "reference":
                    references[payload["reference_seq"]] = payload["q_source"][
                        self.axis
                    ]
                elif row["kind"] == "sample":
                    key = (payload["run_id"], payload["cycle_seq"])
                    if (
                        key[0] == self.run_id
                        and key not in seen
                        and payload["phase"] in ("RUNNING", "ENDING")
                    ):
                        seen.add(key)
                        samples.append(payload)
        if not samples:
            raise ValueError("本轮没有运行阶段的有效采样，无法计算跟踪误差")
        start = samples[0]["sample_time_s"]
        rows, errors, source_errors = [], [], []
        gaps = 0
        previous = None
        for sample in samples:
            seq = sample["cycle_seq"]
            if previous is not None and seq - previous > 5:
                gaps += max(0, (seq - previous) // 5 - 1)
            previous = seq
            reference = math.degrees(sample["q_ref"][self.axis])
            actual = math.degrees(sample["q_actual"][self.axis])
            source = references.get(sample["adopted_reference_seq"])
            source = math.degrees(source) if source is not None else None
            errors.append(reference - actual)
            if source is not None:
                source_errors.append(source - actual)
            rows.append((sample["sample_time_s"] - start, reference, actual, source))
        complete = bool(
            summary.get("completed_trajectory")
            and summary.get("stopped_confirmed")
            and not summary.get("last_error")
            and not summary.get("stop_error")
        )
        report = dict(
            record_type="single_axis_impedance_tuning",
            run_id=self.run_id,
            axis=self.axis + 1,
            completed=complete,
            reference_error=error_metrics(errors),
            generated_target_error=error_metrics(source_errors),
            return_to_start_error_deg=math.degrees(self.final_angle) - rows[-1][2],
            snapshot_gaps=gaps,
            log_complete=summary.get("log_complete", False),
            log_dropped=summary.get("log_dropped", 0),
            log_error=summary.get("log_error"),
            last_error=summary.get("last_error") or summary.get("stop_error"),
            environment=metadata.get("environment"),
            source_alignment_missing=len(samples) - len(source_errors),
        )
        path = self.directory / "tuning_report.json"
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        return dict(report=report, rows=rows, path=str(path))
