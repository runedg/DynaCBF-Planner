# 单个被跟踪动态障碍物的状态估计
#
# 坐标系约定：所有几何量均在 header.frame_id 指定的同一坐标系下表达，
# 通常为世界系（odom/map）。速度是**世界系**速度，不是机体系速度。

std_msgs/Header header

# 跨帧稳定的唯一 ID，由跟踪层分配并保持
uint32 id

# 轨迹生命周期状态
uint8 STATE_TENTATIVE = 0
uint8 STATE_CONFIRMED = 1
uint8 STATE_LOST = 2
uint8 STATE_DELETED = 3
uint8 state

# 估计中心位置
geometry_msgs/Point center

# 估计平面速度（世界系）
geometry_msgs/Vector3 velocity

# 最小包围椭圆：半长轴、半短轴（米）与朝向（弧度）
# orientation 定义为世界系 x 轴逆时针旋转到椭圆**长轴**方向的夹角
float64 semi_major
float64 semi_minor
float64 orientation

# [px, py, vx, vy] 的 4x4 协方差矩阵，行主序展开
float64[16] covariance

# 跟踪置信度，范围 [0, 1]
float64 confidence

# 该目标最近一次被观测到的时间戳，用于陈旧判定
builtin_interfaces/Time last_seen
