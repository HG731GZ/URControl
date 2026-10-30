"""导纳字段说明、三份参数及严格的预设文件读写；内部始终使用 SI 单位。"""

from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path

from UR_Utils import admittance_config


@dataclass(frozen=True)
class ParameterField:
    """描述一个已在 Designer 文件中声明的配置编辑器。"""

    key: str
    title: str
    unit: str
    group: str
    widget: str
    scope: str = "all"
    editable: bool = True

    def participates(self, reference_source, space):
        return (self.scope == "all"
                or self.scope == reference_source
                or self.scope == space
                or self.scope == reference_source + "_" + space)


# 界面只维护常用导纳参数；完整运行配置仍由 Python 配置和这些参数共同组成。
FIELDS = (
    ParameterField("JOINT_AXES", "柔顺轴启用", "J1–J6", "joint", "JointAxis", "joint"),
    ParameterField("JOINT_MASS", "虚拟惯量 M", "kg·m²", "joint", "JointMass", "joint"),
    ParameterField("JOINT_DAMPING", "阻尼 D", "N·m·s/rad", "joint", "JointDamping", "joint"),
    ParameterField("JOINT_STIFFNESS", "恢复刚度 K", "N·m/rad", "joint", "JointStiffness", "joint"),
    ParameterField("JOINT_TARGET_TORQUE_NM", "目标力矩", "N·m", "joint", "JointTargetTorque", "joint"),
    ParameterField("JOINT_DEADBAND_NM", "连续死区", "N·m", "joint", "JointDeadband", "joint"),
    ParameterField("CART_FRAME", "柔顺方向", "固定方向，中心为当前 TCP", "cart", "CartFrame", "cartesian"),
    ParameterField("CART_AXES", "柔顺轴启用", "x、y、z、rx、ry、rz", "cart", "CartAxis", "cartesian"),
    ParameterField("CART_MASS", "虚拟质量 / 惯量 M", "前三轴 kg；后三轴 kg·m²", "cart", "CartMass", "cartesian"),
    ParameterField("CART_DAMPING", "阻尼 D", "前三轴 N·s/m；后三轴 N·m·s/rad", "cart", "CartDamping", "cartesian"),
    ParameterField("CART_STIFFNESS", "恢复刚度 K", "前三轴 N/m；后三轴 N·m/rad", "cart", "CartStiffness", "cartesian"),
    ParameterField("CART_TARGET_WRENCH", "目标载荷", "前三轴 N；后三轴 N·m", "cart", "CartTargetWrench", "cartesian"),
    ParameterField("CART_DEADBAND", "连续死区", "前三轴 N；后三轴 N·m", "cart", "CartDeadband", "cartesian"),
)
FIELD_MAP = {field.key: field for field in FIELDS}
EDITABLE_KEYS = frozenset(field.key for field in FIELDS)
# 运行记录单列控制器范围及寄存器约定，使用新的固定参考语义。
READONLY_KEYS = frozenset({
    "JOINT_POSITION_LIMIT_RAD", "ADMITTANCE_INPUT_INT_BASE", "ADMITTANCE_INPUT_FLOAT_BASE",
    "ADMITTANCE_OUTPUT_INT_BASE", "ADMITTANCE_OUTPUT_FLOAT_BASE",
})


def _merge_program_configuration(parameters, program_configuration):
    """界面参数覆盖当前显式候选，保留候选的运行模式及逐轴命令约束。"""
    values = deepcopy(program_configuration)
    values.update({key: deepcopy(parameters[key]) for key in EDITABLE_KEYS})
    return values


def _unique_object(pairs):
    """重复键会掩盖用户配置，文件边界明确拒绝。"""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"JSON 包含重复字段：{key}")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"JSON 不允许非有限数值：{value}")


class AdmittanceParameters:
    """分别持有草稿、下轮准备值和后端确认的本轮生效值。"""

    def __init__(self):
        self.program_configuration = admittance_config.snapshot()
        self.draft_parameters = deepcopy(self.program_configuration)
        self.prepared_parameters = admittance_config.validate_config(self.draft_parameters)
        self.effective_parameters = None
        self.revision = 0
        self.effective_revision = None

    @property
    def has_unapplied_changes(self):
        return self.draft_parameters != self.prepared_parameters

    def set_draft(self, values):
        """兼容完整配置或完整界面参数输入，不接受非界面值对 Python 配置的覆盖。"""
        self.draft_parameters = _merge_program_configuration(values, self.program_configuration)

    def select_candidate(self, values):
        """由候选选择入口载入完整配置，保留应用前后的差异。"""
        self.program_configuration = admittance_config.validate_config(values)
        self.draft_parameters = deepcopy(self.program_configuration)

    def restore_defaults(self):
        """恢复界面参数为 Python 中的默认值，其余配置仍由 Python 决定。"""
        defaults = self.program_configuration
        for key in EDITABLE_KEYS:
            self.draft_parameters[key] = deepcopy(defaults[key])

    def apply(self):
        self.prepared_parameters = admittance_config.validate_config(
            _merge_program_configuration(self.draft_parameters, self.program_configuration))
        self.draft_parameters = deepcopy(self.prepared_parameters)
        self.revision += 1
        return self.prepared_copy()

    def prepared_copy(self):
        return deepcopy(self.prepared_parameters)

    def set_effective(self, parameters, revision=None):
        self.effective_parameters = admittance_config.validate_config(parameters)
        self.effective_revision = self.revision if revision is None else revision

    def differences(self):
        """逐字段列出三份值，不计算指纹。"""
        return [{"field": field, "draft": deepcopy(self.draft_parameters[field.key]),
                 "prepared": deepcopy(self.prepared_parameters[field.key]),
                 "effective": (None if self.effective_parameters is None else
                               deepcopy(self.effective_parameters[field.key])),
                 "changed": self.draft_parameters[field.key] != self.prepared_parameters[field.key]}
                for field in FIELDS]

    def import_preset(self, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"),
                          object_pairs_hook=_unique_object, parse_constant=_reject_constant)
        if not isinstance(data, dict) or set(data) != {
                "schema_version", "name", "reference_source", "space", "parameters"}:
            raise ValueError("预设必须完整包含 schema_version、name、reference_source、space、parameters；运行记录不可作为预设导入")
        if type(data["schema_version"]) is not int or data["schema_version"] != 3:
            raise ValueError("仅支持 schema_version 为 3 的预设；旧版 v1/v2 的模式语义及参数字段已变化，请参照原值重新建立新预设")
        if not isinstance(data["name"], str) or not data["name"].strip():
            raise ValueError("预设名称必须为非空文字")
        if data["reference_source"] not in ("teleop", "fixed") or data["space"] not in ("joint", "cartesian"):
            raise ValueError("预设场景或空间无效")
        if not isinstance(data["parameters"], dict) or set(data["parameters"]) != EDITABLE_KEYS:
            raise ValueError("调参预设必须包含全部 13 个界面参数，不能包含 Python 配置项或未知字段")
        merged = _merge_program_configuration(data["parameters"], self.program_configuration)
        self.draft_parameters = admittance_config.validate_config(merged)
        return data["reference_source"], data["space"], data["name"]

    def export_preset(self, path, name, reference_source, space):
        values = admittance_config.validate_config(
            _merge_program_configuration(self.draft_parameters, self.program_configuration))
        data = {"schema_version": 3, "name": name,
                "reference_source": reference_source, "space": space,
                "parameters": {field.key: values[field.key] for field in FIELDS if field.editable}}
        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")

    def export_effective(self, path, metadata):
        if self.effective_parameters is None:
            raise ValueError("尚无后端确认的生效参数")
        data = {"record_type": "admittance_run_record", "schema_version": 3,
                "purpose": "本轮完整生效参数与只读元数据；不可作为调参预设导入",
                "reference_behavior": {"teleop": "moving_reference", "fixed": "fixed_anchor"}[metadata["reference_source"]],
                "model": self.effective_parameters["DIAGNOSTIC_MODE"] if self.effective_parameters["DIAGNOSTIC_MODE"] != "admittance" else "mass_damper_spring",
                "backend": "urscript",
                "source_codes": {1: "teleop", 2: "free", 3: "fixed"},
                "units": admittance_config.units(metadata["reference_source"], metadata["space"],
                                                config=self.effective_parameters),
                "parameters_revision": self.effective_revision,
                "parameters": deepcopy(self.effective_parameters),
                "readonly_parameters": {key: deepcopy(self.effective_parameters[key]) for key in READONLY_KEYS},
                "run_metadata": deepcopy(metadata)}
        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
