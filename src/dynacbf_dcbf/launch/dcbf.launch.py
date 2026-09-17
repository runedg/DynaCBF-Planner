# =============================================================================
# dcbf.launch.py —— 启动 MPC-D-CBF 局部规划层（三个节点）
# =============================================================================
# 节点构成（前两个是参考实现的逐字复制，第三个是本项目新写的融合缝）：
#   local_planner_node    —— MPC-D-CBF 本体（订阅 curr_state / obs_predict_pub /
#                            global_path；发布 local_plan / local_path /
#                            pub_path_vis / cmd_move）
#   controller_node       —— local_plan + cmd_move → cmd_vel 转发；同时用 TF
#                            ('world' → 'base_link') 查位姿并发布 curr_state
#   bspline_path_sampler  —— 融合缝：planning/bspline（B 样条）按弧长几何采样后
#                            发布到 global_path，供 local_planner_node 直接消费
#
# 话题全部使用参考实现的原始名字（相对名/绝对名均未改），因此**没有任何 remap**。
# 唯一的运行前提是 TF 'world' → 'base_link' 存在（由仿真/里程计侧提供）。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('dynacbf_dcbf')
    config_file_path = os.path.join(share_dir, 'config', 'dcbf_params.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='Path to the ROS 2 parameters file to use.'
    )

    sampler_declare = DeclareLaunchArgument(
        'start_sampler',
        default_value='true',
        description='是否同时启动融合缝节点 bspline_path_sampler '
                    '(planning/bspline -> global_path)；'
                    '若已有其它节点发布 global_path，可置 false。'
    )

    local_planner_node = Node(
        package='dynacbf_dcbf',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    controller_node = Node(
        package='dynacbf_dcbf',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    bspline_path_sampler_node = Node(
        package='dynacbf_dcbf',
        executable='bspline_path_sampler',
        name='bspline_path_sampler',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(LaunchConfiguration('start_sampler')),
    )

    return LaunchDescription([
        params_declare,
        sampler_declare,
        local_planner_node,
        controller_node,
        bspline_path_sampler_node,
    ])
