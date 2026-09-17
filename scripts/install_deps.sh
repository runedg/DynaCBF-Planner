#!/usr/bin/env bash
# ============================================================================
# DynaCBF-Planner 所需的系统依赖安装脚本
#
# 为什么需要 sudo：本脚本通过 apt 安装系统包，需要 root 权限。
# 建议先通读本文件确认内容，再由你本人执行。
#
# 用法（在项目根目录）：
#     sudo bash scripts/install_deps.sh
#
# 脚本只做三件事：apt update → 安装缺失的 ROS 2 包与系统库 → 打印校验结果。
# 不修改任何配置文件，不添加软件源（ROS 2 Humble 源本机已配置好）。
# ============================================================================
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 sudo 运行： sudo bash scripts/install_deps.sh" >&2
  exit 1
fi

echo "==> apt update"
apt update

echo "==> 安装 ROS 2 包（grid_map 套件 / turtlebot3 套件 / pcl_ros / rviz_visual_tools）"
apt install -y \
  ros-humble-pcl-ros \
  ros-humble-rviz-visual-tools \
  ros-humble-grid-map \
  ros-humble-grid-map-octomap \
  ros-humble-grid-map-core \
  ros-humble-grid-map-ros \
  ros-humble-grid-map-cv \
  ros-humble-grid-map-filters \
  ros-humble-grid-map-loader \
  ros-humble-grid-map-msgs \
  ros-humble-grid-map-pcl \
  ros-humble-grid-map-rviz-plugin \
  ros-humble-grid-map-sdf \
  ros-humble-grid-map-visualization \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description

echo "==> 安装系统库（CGAL；pcl / octomap / eigen 本机已有，重复安装无害）"
apt install -y \
  libcgal-dev \
  libcgal-qt5-dev \
  libpcl-dev \
  liboctomap-dev \
  libeigen3-dev

echo
echo "===================== 校验 ====================="
ok=0; miss=0
for p in grid_map_core grid_map_ros grid_map_octomap grid_map_pcl \
         turtlebot3_gazebo turtlebot3_description \
         rviz_visual_tools pcl_ros; do
  if [ -d "/opt/ros/humble/share/$p" ]; then
    printf "  %-24s OK\n" "$p"; ok=$((ok+1))
  else
    printf "  %-24s 缺失\n" "$p"; miss=$((miss+1))
  fi
done
for l in libcgal-dev liboctomap-dev libpcl-dev; do
  if dpkg -s "$l" >/dev/null 2>&1; then
    printf "  %-24s OK\n" "$l"; ok=$((ok+1))
  else
    printf "  %-24s 缺失\n" "$l"; miss=$((miss+1))
  fi
done

echo
echo "通过 $ok 项，缺失 $miss 项"
if [ "$miss" -eq 0 ]; then
  echo "全部就绪。接下来（无需 sudo）："
  echo "  cd ~/ROBOT/DynaCBF-Planner"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  export TURTLEBOT3_MODEL=burger"
  echo "  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
  echo "  source install/setup.bash"
  echo "  ros2 launch dynacbf_bringup dynacbf.launch.py"
  echo
  echo "注意：本项目约定**不使用** --symlink-install；详细的用法见 README.md。"
else
  echo "仍有缺失，请把上面的输出发给我。" >&2
  exit 2
fi
