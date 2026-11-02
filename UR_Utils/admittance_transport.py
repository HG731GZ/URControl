"""兼容原导纳导入；传输实现由两个控制后端共享。"""
from UR_Utils.rtde_script_transport import (
    ScriptRTDE as AdmittanceRTDE, ScriptClient as AdmittanceScriptClient,
    DashboardStop as AdmittanceDashboardStop, clock_sample, elapsed_sample,
    clock_interval, RTDE_LIBRARY_VERSION, PRIMARY_MAX_MESSAGE_BYTES,
)
