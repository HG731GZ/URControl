"""从经典主界面与导纳窗口的 Designer 源文件生成界面，只转换生成器说明注释。"""

from pathlib import Path
import subprocess
import sys


def generate():
    """使用当前 Python 环境中的 PyQt5 生成器，不计算内容签名。"""
    directory = Path(__file__).resolve().parent
    for name in ("classic_window", "admittance_window"):
        source = directory / (name + ".ui")
        destination = directory / ("ui_" + name + ".py")
        subprocess.run([sys.executable, "-m", "PyQt5.uic.pyuic", str(source),
                        "-o", str(destination)], check=True)
        text = destination.read_text(encoding="utf-8")
        begin = text.index("from PyQt5 import")
        header = ("# -*- coding: utf-8 -*-\n\n"
                  f"# 本文件由 PyQt5 界面生成器根据 {name}.ui 自动生成。\n"
                  "# 修改控件结构和静态文字请编辑对应 .ui，再运行 generate_ui.py。\n"
                  "# 重新生成将覆盖本文件，业务逻辑请放在独立模块。\n\n")
        destination.write_text(header + text[begin:], encoding="utf-8")
        print(f"已生成 {destination.name}")


if __name__ == "__main__":
    generate()
