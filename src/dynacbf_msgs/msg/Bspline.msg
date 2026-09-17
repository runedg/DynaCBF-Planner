# 名义 B 样条轨迹（与参考项目 scan_planner_msgs/Bspline 字段保持一致，便于对照）
#
# 注意：上游从不填充 yaw_pts / yaw_dt，朝向由几何切线隐式给出。
# 本项目保留这些字段以维持接口兼容，但控制器不得依赖它们。

int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt
