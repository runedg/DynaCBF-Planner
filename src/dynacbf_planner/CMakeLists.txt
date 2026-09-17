cmake_minimum_required(VERSION 3.8)
project(dynacbf_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dynacbf_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# 注意：本包不引入 osqp / osqp_vendor。上游 MPC 控制器（已由新控制器取代）才用 OSQP。
set(dynacbf_planner_dependencies
  ament_index_cpp
  cv_bridge
  dynacbf_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# ---------------------------------------------------------------------------
# 规划核心静态库：四个迁移模块（plan_env / path_searching / bspline_opt / traj_utils）
# ---------------------------------------------------------------------------
add_library(dynacbf_planner_core STATIC
  src/plan_env/grid_map.cpp
  src/plan_env/raycast.cpp
  src/path_searching/dyn_a_star.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/bspline_opt/uniform_bspline.cpp
  src/traj_utils/planning_visualization.cpp
  src/traj_utils/polynomial_traj.cpp
)
# 移植修正：上游 plan_env/CMakeLists.txt 的 target_include_directories 未用 $<BUILD_INTERFACE:>
# 包裹，会把构建机的绝对系统路径烘进 export 结果；此处统一使用 BUILD_INTERFACE / INSTALL_INTERFACE。
target_include_directories(dynacbf_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynacbf_planner_core ${dynacbf_planner_dependencies})
# 注意：ament_target_dependencies 使用 target_link_libraries 的 plain signature，
# 因此这里不能再混用 PUBLIC/PRIVATE 关键字签名，否则 CMake 会直接报错。
target_link_libraries(dynacbf_planner_core Eigen3::Eigen)

# ---------------------------------------------------------------------------
# 规划节点：入口 + 重规划状态机 + 规划管理器
# ---------------------------------------------------------------------------
add_executable(dynacbf_planner_node
  src/planner/dynacbf_planner_node.cpp
  src/planner/scan_replan_fsm.cpp
  src/planner/planner_manager.cpp
)
target_include_directories(dynacbf_planner_node PRIVATE include)
ament_target_dependencies(dynacbf_planner_node ${dynacbf_planner_dependencies})
target_link_libraries(dynacbf_planner_node dynacbf_planner_core)

install(TARGETS dynacbf_planner_core dynacbf_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uniform_bspline test/test_uniform_bspline.cpp)
  if(TARGET test_uniform_bspline)
    target_link_libraries(test_uniform_bspline dynacbf_planner_core)
  endif()

  # 移植修正：上游两个 launch test 的测试类未继承 unittest.TestCase，pytest 收集到 0 个用例，
  # 恒为“通过”（假绿）；本包迁移时已在测试文件里补上继承。此处用 QUIET 查找 launch_testing，
  # 避免测试依赖缺失时直接阻断包的构建。
  find_package(launch_testing_ament_cmake QUIET)
  if(launch_testing_ament_cmake_FOUND)
    add_launch_test(test/test_planner_startup.py TIMEOUT 30)
    add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
  else()
    message(STATUS "launch_testing_ament_cmake not found: skipping planner launch tests")
  endif()
endif()

ament_package()
