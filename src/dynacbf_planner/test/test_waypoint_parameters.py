"""Verify that the flat ROS 2 waypoint array is accepted at startup."""

# 测试意图：在 planner.yaml 之外再叠加加载 keypoints.example.yaml（预设航点参数），
# 验证扁平化航点数组 fsm.waypoints 与 fsm.navi_mode 能被 dynacbf_planner_node 正常接受。
#
# 迁移来源：SCAN-Planner 上游的 test_waypoint_parameters.py
# 移植修正：上游的 `class TestWaypointParameters:` 没有继承 unittest.TestCase，pytest 收集
# 到 0 个测试用例，测试恒为“通过”（假绿）。此处补上继承与 unittest 导入。

import os
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # 待测节点：同时加载默认规划参数与示例航点参数（后者覆盖 fsm.* 键）
    planner = launch_ros.actions.Node(
        package="dynacbf_planner",
        executable="dynacbf_planner_node",
        name="dynacbf_planner_node",
        parameters=[
            os.path.join(root, "config", "planner.yaml"),
            os.path.join(root, "config", "keypoints.example.yaml"),
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestWaypointParameters(unittest.TestCase):
    # 核心断言：带航点参数启动时节点须在 15 秒内完成启动（解析失败会直接导致启动崩溃）
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
