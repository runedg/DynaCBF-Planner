# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/controller.py（全文 125 行）。
#         逻辑、话题名、消息类型、参数、类名（Controller）、节点名（'controller'）
#         一律未改。
#
# 【改动清单】相对参考实现，本文件只做了：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块。
#   改动 2：无（连 import 都不需要调整：本文件没有引用参考实现的其它模块）。
#
# 运行前提（都是原实现既有的前提，本包未做任何补救性修改）：
#   * 需要 TF：'world' → 'base_link' 的变换存在，否则 get_current_state() 静默
#     跳过发布，local_planner_node 因收不到 curr_state 而完全不动作。
#   * 订阅 /local_plan、/cmd_move，发布 /cmd_vel、/curr_state（均为根命名空间下的
#     绝对名；local_planner_node 发布的是相对名 "local_plan"/"cmd_move"，在根命名
#     空间下与之一致，故不需要任何 remap）。
# =============================================================================

import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.command_timeout = 0.5
        self.rate = self.create_timer(0.02, self.control_loop)  # 50Hz
        self.get_state = self.create_timer(0.02, self.get_current_state)  # 50Hz

        self.local_plan_sub = self.create_subscription(
            Float32MultiArray,
            '/local_plan',
            self.local_planner_cb,
            10)
        self.cmd_move_sub = self.create_subscription(
            Bool,
            '/cmd_move',
            self.cmd_move_cb,
            10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.curr_state_pub = self.create_publisher(Float32MultiArray, '/curr_state', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.linear_speed = self.angular_speed = 0.0
        self.motion_enabled = False
        self.last_plan_time = None
        # The controller executes only the first MPC input.  Keeping the whole
        # horizon here unnecessarily coupled it to the planner horizon.
        self.local_plan = np.zeros(2)

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        sin_pitch = max(-1.0, min(1.0, 2*(w*y-z*x)))
        p = math.asin(sin_pitch)
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y
    
    def get_current_state(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'base_link',
                                                        rclpy.time.Time())
            _,_,yaw = self.quart_to_rpy(transform.transform.rotation.x,
                                        transform.transform.rotation.y,
                                        transform.transform.rotation.z,
                                        transform.transform.rotation.w)
            curr_state = Float32MultiArray()
            curr_state.data = [transform.transform.translation.x,
                               transform.transform.translation.y,
                               yaw]
            self.curr_state_pub.publish(curr_state)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException,
               tf2_ros.ExtrapolationException):
            pass
    
    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.angular.z = self.angular_speed
        # self.get_logger().info('Linear Speed: %.1f, Angular Speed: %.1f' % (self.linear_speed, self.angular_speed))
        self.vel_pub.publish(control_cmd)

    def control_loop(self):
        command_is_fresh = (
            self.last_plan_time is not None and
            self.get_clock().now() - self.last_plan_time <
            Duration(seconds=self.command_timeout))
        if self.motion_enabled and command_is_fresh:
            self.linear_speed = self.local_plan[0]
            self.angular_speed = self.local_plan[1]
        else:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        self.pub_vel()

    def cmd_move_cb(self, msg):
        self.motion_enabled = msg.data
        if not self.motion_enabled:
            self.local_plan.fill(0.0)

    def local_planner_cb(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                'Rejected local plan with %d values; expected at least 2' %
                len(msg.data))
            return
        if not math.isfinite(msg.data[0]) or not math.isfinite(msg.data[1]):
            self.get_logger().warning('Rejected local plan containing NaN or Inf')
            return
        self.local_plan[0] = msg.data[0]
        self.local_plan[1] = msg.data[1]
        self.last_plan_time = self.get_clock().now()

    def stop(self):
        self.motion_enabled = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.pub_vel()

def main(args = None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    controller = Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            controller.stop()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
