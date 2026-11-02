"""复用项目的只读 Qt 趋势控件，不新增绘图库依赖。"""

from collections import deque
from PyQt5.QtWidgets import QTabWidget
from admittance_plot import AdmittanceTrendWidget


class ImpedanceMonitor(QTabWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.samples = deque(maxlen=3000)
        self.axis = 0
        self.last = None
        self.position = AdmittanceTrendWidget()
        self.error = AdmittanceTrendWidget()
        self.torque = AdmittanceTrendWidget()
        for widget, name in (
            (self.position, "角度"),
            (self.error, "跟踪误差"),
            (self.torque, "指令力矩"),
        ):
            self.addTab(widget, name)

    def update_status(self, status):
        sample = status.get("sample")
        if not sample:
            return
        key = (status["run_id"], sample["cycle_seq"])
        if key != self.last:
            if self.last and key[0] != self.last[0]:
                self.samples.clear()
            self.samples.append((sample, status.get("q_source")))
            self.last = key
        i = self.axis

        def series(name, color, style, value):
            return dict(
                name=name,
                color=color,
                style=style,
                points=[
                    dict(
                        time=s["sample_time_s"],
                        value=value(s, q),
                        valid=True,
                        run_id=key[0],
                        segment=0,
                    )
                    for s, q in self.samples
                ],
            )

        self.position.set_data(
            dict(
                title=f"J{i+1} 关节角",
                unit="rad",
                now=sample["sample_time_s"],
                series=[
                    series(
                        "源目标（异步）",
                        "#888888",
                        "dot",
                        lambda s, q: q[i] if q else float("nan"),
                    ),
                    series("生效参考", "#2474b5", "solid", lambda s, q: s["q_ref"][i]),
                    series("实际", "#df7c25", "dash", lambda s, q: s["q_actual"][i]),
                ],
            )
        )
        self.error.set_data(
            dict(
                title=f"J{i+1} 同周期参考−实际",
                unit="rad",
                now=sample["sample_time_s"],
                series=[
                    series(
                        "误差",
                        "#2474b5",
                        "solid",
                        lambda s, q: s["q_ref"][i] - s["q_actual"][i],
                    )
                ],
            )
        )
        self.torque.set_data(
            dict(
                title=f"J{i+1} 指令力矩",
                unit="N·m",
                now=sample["sample_time_s"],
                series=[
                    series("tau_cmd", "#df7c25", "solid", lambda s, q: s["tau_cmd"][i])
                ],
            )
        )
