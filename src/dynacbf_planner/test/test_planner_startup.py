"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

# 测试意图：迁移后配置的冒烟测试——仅以 planner.yaml 为参数启动 dynacbf_planner_node，
# 验证节点进程能正常创建并成功读取该参数文件。参数非法或缺键时节点按设计以非零码退出
# （见 grid_map / optimization / manager 参数的有限性与范围校验），因此本测试能真实捕获
# 配置回归，而不是空跑。
#
# 迁移来源：SCAN-Planner 上游的 test_planner_startup.py
# 移植修正：上游的 `class TestPlannerStartup:` 没有继承 unittest.TestCase，pytest 收集到
# 0 个测试用例，测试恒为“通过”（假绿）。此处补上继承与 unittest 导入。

import os
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    # 定位被测节点使用的参数文件（本文件上级目录的 config/planner.yaml）
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "planner.yaml")
    )
    # 待测节点：启动 dynacbf_planner_node 并加载上述参数文件，输出到屏幕
    planner = launch_ros.actions.Node(
        package="dynacbf_planner",
        executable="dynacbf_planner_node",
        name="dynacbf_planner_node",
        parameters=[config],
        output="screen",
    )
    # 启动描述：运行被测节点后由 launch_testing 接管，进入 ReadyToTest 阶段
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


class TestPlannerStartup(unittest.TestCase):
    # 核心断言：被测节点须在 15 秒内完成启动（进程未崩溃即视为通过）
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
