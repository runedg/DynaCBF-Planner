cmake_minimum_required(VERSION 3.8)
project(dynacbf_bringup)

# ---------------------------------------------------------------------------
# dynacbf_bringup 是纯编排包：没有 C++ 代码，不产生任何可执行文件或库。
# 本文件只负责把 launch 安装到 share/dynacbf_bringup 下。
# 本包**不携带任何参数文件**：每个节点的参数都由拥有该节点的包提供，保持单一来源。
#
# RViz 配置不放在本包：统一用 dynacbf_simulator/rviz/rviz_config.rviz，
# 由 launch 的 rviz_config 参数指向，避免两份配置各自漂移。
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
