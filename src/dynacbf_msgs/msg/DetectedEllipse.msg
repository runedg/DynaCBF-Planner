# 单帧椭圆检测结果（感知层输出，供跟踪层关联使用）
#
# 由 dynacbf_perception 从点云聚类 + 最小包围椭圆拟合得到，
# 尚未跨帧关联，因此没有稳定 ID、也没有速度。

std_msgs/Header header

# 椭圆中心（世界系）
geometry_msgs/Point center

# 最小包围椭圆参数：半长轴、半短轴（米）与朝向（弧度，世界系 x 轴到长轴）
float64 semi_major
float64 semi_minor
float64 orientation

# 该聚类包含的点数，用于置信度评估
uint32 point_count

# 拟合质量指标（残差，语义由感知包定义并在其文档中说明）
float64 residual

# 检测置信度 [0, 1]
float64 confidence
