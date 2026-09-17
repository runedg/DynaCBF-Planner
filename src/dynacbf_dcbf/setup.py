import os
from glob import glob

from setuptools import find_packages, setup

# ---------------------------------------------------------------------------
# dynacbf_dcbf —— MPC-D-CBF 局部规划层（ament_python）
#
# 说明（隔离要求）：
#   * data_files 只指向本包目录内的实体文件（resource / package.xml /
#     launch / config），**不指向任何外部目录**，也不包含任何符号链接。
#   * 三个可执行入口：
#       local_planner_node   —— MPC-D-CBF 本体（逐字复制自参考实现，含其类名/节点名）
#       controller_node      —— local_plan/cmd_move → cmd_vel 转发（逐字复制自参考实现）
#       bspline_path_sampler —— 名义 B 样条 → 弧长采样的 nav_msgs/Path（本项目新写的融合缝）
# ---------------------------------------------------------------------------
package_name = 'dynacbf_dcbf'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DynaCBF-Planner maintainers',
    maintainer_email='dynacbf@example.com',
    description='MPC-D-CBF local planner (faithful port) and B-spline arc-length path sampler',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 与参考实现 setup.py 的同名入口保持一致（模块路径换成本包）
            'controller_node = dynacbf_dcbf.controller:main',
            'local_planner_node = dynacbf_dcbf.local_planner:main',
            # 本项目新写的融合缝节点
            'bspline_path_sampler = dynacbf_dcbf.bspline_path_sampler:main',
        ],
    },
)
