# 规划器数据展示消息（与参考项目 scan_planner_msgs/DataDisp 字段保持一致）
#
# 说明：上游 plan_manage 中该消息的 a..e 字段从未被赋值（恒为 0），
# 本项目保留定义以维持接口兼容，但不作为任何逻辑依据。

std_msgs/Header header
float64 a
float64 b
float64 c
float64 d
float64 e
