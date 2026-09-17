# 单个障碍物在 MPC 预测时域上的预测
#
# 关键约定：
#   * 所有数组长度必须相等，且等于预测步数 N。
#   * stamp[k] 必须与 MPC 第 k 步的时间戳**逐一对齐**，不得使用单一时间戳代替。
#   * center[k] 是该步的预测中心；semi_major/minor[k] 是**已膨胀**的半轴
#     （含机器人尺寸、安全裕量与预测不确定性膨胀），可直接用于 D-CBF。
#   * raw 字段保存膨胀前的 MBE 半轴，用于可审计性与单元测试。

std_msgs/Header header

uint32 id

# 预测步数
uint32 horizon

# 逐步时间戳（长度 horizon）
builtin_interfaces/Time[] stamp

# 逐步预测中心（长度 horizon）
geometry_msgs/Point[] center

# 逐步预测朝向（长度 horizon，弧度）
float64[] orientation

# 逐步膨胀后半轴（长度 horizon，米）
float64[] semi_major
float64[] semi_minor

# 逐步膨胀前原始 MBE 半轴（长度 horizon，米）
float64[] raw_semi_major
float64[] raw_semi_minor

# 膨胀所用的不确定性置信系数（半轴按 k_sigma * sigma 膨胀）
float64 inflation_sigma

# 逐步预测协方差，每步 4x4 行主序展开，总长 16*horizon
# 顺序为 [px, py, vx, vy]
float64[] covariance
