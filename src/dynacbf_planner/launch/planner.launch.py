#!/usr/bin/env python3
# ============================================================================
# planner.launch.py —— dynacbf_planner 最小启动文件（本包新增，非上游迁移文件）
#
# 只启动规划链路节点 dynacbf_planner_node，并载入 config/planner.yaml。
# 上游的 run.launch.py / simulator.launch.py 会一并拉起控制器与仿真器，因此未随
# 本包迁移；rviz.launch.py 依赖上游 RViz 配置，也未迁移。
# 需要全链路（Gazebo 场地 + 感知 + 跟踪 + 本规划器 + MPC-D-CBF 控制器）请用
# dynacbf_bringup 的 dynacbf.launch.py —— 那是唯一的总装入口。
#
# 参数：
#   node_name   规划节点名（默认 dynacbf_planner_node，参数化）
#   config_file 参数文件路径（默认 share/dynacbf_planner/config/planner.yaml）
#
# 注意：ROS 2 按节点名匹配参数文件中的顶层键，若覆盖 node_name，需要同步修改
# config_file 中 `dynacbf_planner_node:` 这一顶层键名，否则参数不会生效。
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("dynacbf_planner"), "config", "planner.yaml"
    )

    node_name = LaunchConfiguration("node_name")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="dynacbf_planner_node",
                description="规划节点名（默认 dynacbf_planner_node）",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="规划器参数文件，默认 share/dynacbf_planner/config/planner.yaml",
            ),
            Node(
                package="dynacbf_planner",
                executable="dynacbf_planner_node",
                name=node_name,
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
