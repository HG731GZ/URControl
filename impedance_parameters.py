"""草稿、准备值与本轮冻结值；预设与运行记录类型严格分离。"""

from copy import deepcopy
from PyQt5 import QtCore
import json
from pathlib import Path
from UR_Utils.impedance_config import read_preset, snapshot, validate


class ImpedanceParameters(QtCore.QObject):
    changed = QtCore.pyqtSignal()

    def __init__(self, is_locked=lambda: False):
        super().__init__()
        self._is_locked = is_locked
        self.effective_revision = None
        self.draft = snapshot()
        self.prepared = deepcopy(self.draft)
        self.effective = None
        self.revision = 1

    @property
    def locked(self):
        return self._is_locked()

    @property
    def dirty(self):
        return self.draft != self.prepared

    def apply(self):
        self.require_editable()
        self.prepared = validate(self.draft)
        self.revision += 1
        self.changed.emit()
        return deepcopy(self.prepared)

    def require_editable(self):
        if self.locked:
            raise ValueError("运动、交接或停止确认期间不能修改参数")

    def freeze(self):
        if self.dirty:
            raise ValueError("参数有未应用修改，请在单关节窗口处理")
        self.effective = deepcopy(self.prepared)
        self.effective_revision = self.revision
        return deepcopy(self.effective)

    def load(self, path):
        self.require_editable()
        data = json.loads(Path(path).read_text())
        self.draft = read_preset(data)
        self.changed.emit()

    def save(self, path):
        Path(path).write_text(
            json.dumps(
                dict(
                    record_type="joint_impedance_preset",
                    schema_version=3,
                    parameters=validate(self.draft),
                ),
                ensure_ascii=False,
                indent=2,
            )
        )
