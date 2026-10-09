"""创建或更新 Conda 环境，并检查项目依赖；不连接机器人或打开相机。"""

import argparse
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys


PROJECT_DIR = Path(__file__).resolve().parent


def check_environment():
    """检查当前解释器的依赖导入和 MJCF 模型加载。"""
    print(f'Python：{sys.executable}', flush=True)
    print(f'版本：{sys.version.split()[0]}', flush=True)
    for module in (
        'numpy', 'PyQt5.QtCore', 'PyQt5.QtWidgets', 'cv2',
        'serial', 'minimalmodbus', 'pyrealsense2',
        'pinocchio', 'OpenGL.GL', 'trimesh',
        'rtde.rtde', 'rtde_control', 'rtde_receive', 'rtde_io',
    ):
        loaded = importlib.import_module(module)
        print(f'导入成功：{module}（{loaded.__file__}）', flush=True)

    from UR_Utils.ur5e_model_cache import build_models_from_mjcf

    model_path = PROJECT_DIR / 'UR_Utils' / 'universal_robots_ur5e' / 'ur5e.xml'
    model, _, _, visual_model = build_models_from_mjcf(model_path)
    print(f'模型加载成功：{model.nq} 个配置变量，{visual_model.ngeoms} 个可视化几何体')
    print('环境检查完成。图形显示及硬件通信需启动程序后确认。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default='urcontrol', help='创建或更新的 Conda 环境名称')
    parser.add_argument('--check-only', action='store_true',
                        help='只检查当前 Python 环境，不安装依赖')
    args = parser.parse_args()
    if args.check_only:
        check_environment()
        return

    conda = os.environ.get('CONDA_EXE') or shutil.which('conda')
    if conda is None:
        parser.error('未找到 Conda，请安装 Miniconda/Anaconda，并在 Conda 终端运行此脚本。')
    if shutil.which('git') is None:
        parser.error('未找到 Git，官方 RTDE 客户端需要从 GitHub 安装，请先安装 Git。')

    print(f'正在创建或更新环境：{args.name}', flush=True)
    # 安装和检查均只使用目标环境，避免把用户目录的包当作已安装依赖。
    install_env = dict(os.environ, PYTHONNOUSERSITE='1')
    subprocess.run(
        [conda, 'env', 'update', '--name', args.name,
         '--file', str(PROJECT_DIR / 'environment.yml')],
        cwd=PROJECT_DIR, env=install_env, check=True,
    )
    subprocess.run(
        [conda, 'run', '--no-capture-output', '--name', args.name,
         'python', str(Path(__file__).resolve()), '--check-only'],
        cwd=PROJECT_DIR, env=install_env, check=True,
    )
    print(f'\n配置完成。启动命令：\nconda activate {args.name}\npython main.py')


if __name__ == '__main__':
    main()
