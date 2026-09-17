// ============================================================================
// dynacbf_planner_node.cpp —— dynacbf_planner 规划链路节点主程序（ROS 2 入口）
//
// 迁移来源：SCAN-Planner 上游规划链路的节点入口（本文件为移植版，
// 保留原有职责与异常处理语义）。
//
// 职责：创建 ROS 2 规划节点（默认名 dynacbf_planner_node），实例化并初始化规划
// 状态机 SCANReplanFSM（其 init 内部完成参数装载、规划模块装配、定时器/订阅/
// 发布建立），然后用单线程执行器驱动所有回调（10ms FSM 主循环、50ms 安全
// 检查、各话题订阅回调，以及外部重规划请求话题 planning/replan_request）。
// 节点关闭时随之析构。
//
// 异常处理：初始化阶段任何配置/启动错误（如 fsm.navi_mode 非法、预设航点
// 缺失等，见 SCANReplanFSM::init 抛出的 runtime_error）都会被捕获并记录为
// FATAL 日志，随后正常关闭 ROS 并以退出码 1 结束 —— 保证配置错误不会让
// 节点带着半初始化状态继续空转。
// ============================================================================

#include <cstdlib>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <dynacbf_planner/planner/scan_replan_fsm.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 节点名参数化（默认 dynacbf_planner_node）。ROS 2 参数必须在节点创建之后才能
  // 声明，因此节点名提供两条创建前的覆盖途径，优先级从低到高：
  //   1) 环境变量 DYNACBF_PLANNER_NODE_NAME；
  //   2) ROS 2 标准节点名重映射，例如
  //      ros2 run dynacbf_planner dynacbf_planner_node --ros-args -r __node:=other
  //      launch 文件中 Node(name="other") 同样通过该重映射生效，并覆盖环境变量。
  std::string node_name = "dynacbf_planner_node";
  if (const char *env_node_name = std::getenv("DYNACBF_PLANNER_NODE_NAME");
      env_node_name != nullptr && env_node_name[0] != '\0')
  {
    node_name = env_node_name;
  }

  // 创建规划节点（参数 / 话题 / 定时器均挂在该节点上）
  auto node = std::make_shared<rclcpp::Node>(node_name);

  try
  {
    // 创建并初始化规划状态机（FSM）：参数校验失败等异常在此抛出
    dynacbf::SCANReplanFSM planner;
    planner.init(node.get());
    // 单线程执行器：依次执行 10ms FSM / 50ms 安全检查定时器及各订阅回调
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // 阻塞运行直到节点被关闭
  }
  catch (const std::exception &error)
  {
    // 初始化失败（配置错误/模块创建失败）：致命日志 + 正常关闭 + 非零退出码
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize dynacbf_planner: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
