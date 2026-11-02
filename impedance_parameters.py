"""草稿、准备值与本轮冻结值；预设与运行记录类型严格分离。"""

from copy import deepcopy
import json
from pathlib import Path
from UR_Utils.impedance_config import snapshot, validate


class ImpedanceParameters:
    def __init__(self):
        self.draft = snapshot()
        self.prepared = snapshot()
        self.effective = None
        self.revision = 1

    @property
    def dirty(self):
        return self.draft != self.prepared

    def apply(self):
        self.prepared = validate(self.draft)
        self.revision += 1
        return deepcopy(self.prepared)

    def load(self, path):
        data = json.loads(Path(path).read_text())
        if (
            data.get("record_type") != "joint_impedance_preset"
            or data.get("schema_version") != 1
        ):
            raise ValueError(
                "仅接受 joint_impedance_preset v1，不接受导纳预设或运行记录"
            )
        self.draft = validate(data["parameters"])

    def save(self, path):
        Path(path).write_text(
            json.dumps(
                dict(
                    record_type="joint_impedance_preset",
                    schema_version=1,
                    parameters=validate(self.draft),
                ),
                ensure_ascii=False,
                indent=2,
            )
        )
