# 预测障碍物数组，控制器与可视化共用
std_msgs/Header header

# 预测起始时刻与步长，便于消费方校验时间对齐
builtin_interfaces/Time start_time
float64 dt
uint32 horizon

PredictedObstacle[] obstacles
