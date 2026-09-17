cmake_minimum_required(VERSION 3.8)
project(dynacbf_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  "msg/DetectedEllipse.msg"
  "msg/DetectionArray.msg"
  "msg/DynamicObstacle.msg"
  "msg/DynamicObstacleArray.msg"
  "msg/PredictedObstacle.msg"
  "msg/PredictedObstacleArray.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
