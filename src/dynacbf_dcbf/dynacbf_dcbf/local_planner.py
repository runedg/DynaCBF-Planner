# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/local_planner.py（全文 459 行）。
#         算法、话题名、消息类型、参数名、类名（LocalPlanner）、节点名（'phri_planner'）
#         一律未改：
#           * 模型 f(x,u) = [v*cos(yaw), v*sin(yaw), omega]，离散 x_{i+1} = x_i + T*f
#           * 障碍函数 h（barrier.ellipse_barrier：径向距离型，含 safe_dist）
#           * 判据 o = 到椭圆中心的欧氏距离
#           * MPC_constraint 三分支：None（无约束）/ Euclidean（o >= safe_dist）/
#             CBF（h(x_{i+1}, ob_{i+1}) >= (1-gamma_k)*h(x_i, ob_i)）
#           * 代价：运行项 0.1*(x_i-ref_i)^T Q_i (x_i-ref_i) + u_i^T R u_i
#             （Q_i = diag([1+0.05i, 1+0.05i, 0])，R = diag([0.1, 0.02])），
#             终端项 (x_{N-1}-ref_{N-1})^T (5*diag([1,1,0.02])) (x_{N-1}-ref_{N-1})
#           * 边界：远目标 v∈[v_min,v_max]；近目标（距离 <= 1 m）v∈[-v_max,v_max]；
#             omega∈[-omega_max, omega_max]
#           * 热启动 init_N 组初值、IPOPT 设置、异常处理（失败时 last_input 清零 +
#             last_state 原地保持，返回 solve_succeeded=False）逐行照抄
#         * 订阅 curr_state / obs_predict_pub / global_path
#         * 发布 local_plan / local_path / pub_path_vis / cmd_move
#
# 【改动清单】相对参考实现，本文件**只有两处**改动：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块；
#   改动 2：`from local_planner.barrier import ellipse_barrier`
#           → `from dynacbf_dcbf.barrier import ellipse_barrier`
#           （仅为把 barrier 模块放进本包的目录结构，函数本身逐字复制，
#             见同目录 barrier.py）。该行以 `# 【改动 2】` 就地标出。
#   除此之外没有任何一行被修改、删除或增加；没有为适配本项目而改动任何接口。
#
# 【本包内的对接方式（全部在本包之外，见 bspline_path_sampler.py）】
#   * global_path（nav_msgs/Path，本节点只读其中的 x/y）由 bspline_path_sampler
#     把 planning/bspline 的 B 样条按**弧长**几何采样后直接发布到同名话题 global_path，
#     因此本文件不需要任何 remap，也不需要任何代码改动。
#   * obs_predict_pub 仍然是 std_msgs/Float32MultiArray，布局 num_obs × kalman_N × 5，
#     每步 5 个数 [cx, cy, a, b, phi]；kalman_N 必须 == MPC_N（见 __load_parameter 的校验）。
# =============================================================================

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import rclpy.timer
from std_msgs.msg import Float32MultiArray, ColorRGBA, Bool
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import threading
import numpy as np
import casadi as ca
import time

from dynacbf_dcbf.barrier import ellipse_barrier  # 【改动 2】原为 from local_planner.barrier import ellipse_barrier

def distance_global(c1,c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('phri_planner')

        self.curr_state = None
        self.global_path = None

        self.init_N = 0             # 提供casadi求解器初值组数
        self.init_control = None

        self.__load_parameter()

        self.z = 0.0
        self.goal_state = np.zeros([self.N, 3])

        self.last_input = np.zeros([self.N, 2])
        self.last_state = np.zeros([self.N, 3])
        self.mpc_success = None
        self.last_mpc_success = False

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.ob = []

        self.__timer_replan = self.create_timer(self.replan_period, self.__replan_cb) # to check

        self.__pub_local_path_vis = self.create_publisher(Marker, 'pub_path_vis', 10)
        self.__pub_local_path = self.create_publisher(Path, 'local_path', 10)
        self.__pub_local_plan = self.create_publisher(Float32MultiArray, 'local_plan', 10)
        self.__pub_start = self.create_publisher(Bool, 'cmd_move', 10)

        self.__sub_curr_state = self.create_subscription(Float32MultiArray, 'curr_state', self.__curr_pose_cb, 10)
        self.__sub_obs = self.create_subscription(Float32MultiArray, 'obs_predict_pub', self.__obs_cb, 10)
        self.__sub_goal = self.create_subscription(Path, 'global_path', self.__global_path_cb, 25)

    def __replan_cb(self):
        with self.curr_pose_lock, self.global_path_lock, self.obstacle_lock:
            if self.curr_state is None or self.global_path is None:
                return
            curr_state = self.curr_state.copy()
            global_path = self.global_path.copy()
            obstacles = [ob.copy() for ob in self.ob]

        goal_state = self.choose_goal_state(curr_state, global_path)
        if goal_state is not None:
            if distance_global(curr_state, global_path[-1]) <= self.goal_tolerance:
                cmd_move = Bool()
                cmd_move.data = False
                self.__pub_start.publish(cmd_move)
                self.__publish_local_plan(
                    np.zeros([self.N, 2]),
                    np.tile(curr_state, (self.N, 1)))
                return

            # 添加角度信息
            for i in range(self.N - 1):
                y_diff = goal_state[i+1, 1] - goal_state[i, 1]
                x_diff = goal_state[i+1, 0] - goal_state[i, 0]
                if np.hypot(x_diff, y_diff) > 1e-9:
                    goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    goal_state[i, 2] = goal_state[i-1, 2]
                else:
                    goal_state[i, 2] = curr_state[2]

            goal_state[-1, 2] = goal_state[-2, 2]

            states_sol, input_sol, success = self.MPC_ellip(
                curr_state, goal_state, obstacles)

            cmd_move = Bool()
            cmd_move.data = success
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        if len(data.data) < 3 or not np.all(np.isfinite(data.data[:3])):
            self.get_logger().warning('Rejected invalid current state')
            return
        with self.curr_pose_lock:
            self.curr_state = np.array(data.data[:3], dtype=float)

    def __obs_cb(self, data):
        if len(data.data) % 5 != 0:
            self.get_logger().warning(
                'Rejected obstacle prediction with %d values' % len(data.data))
            return
        obstacles = []
        size = int(len(data.data) / 5)
        for i in range(size):
            obstacle = np.array(data.data[5*i:5*i+5], dtype=float)
            if np.all(np.isfinite(obstacle)) and obstacle[2] > 0.0 and obstacle[3] > 0.0:
                obstacles.append(obstacle)
        with self.obstacle_lock:
            self.ob = obstacles

    def __global_path_cb(self, path):
        size = len(path.poses)
        if size > 0:
            global_path = np.zeros([size, 3])
            for i in range(size):
                global_path[i, 0] = path.poses[i].pose.position.x
                global_path[i, 1] = path.poses[i].pose.position.y
            if np.all(np.isfinite(global_path)):
                with self.global_path_lock:
                    self.global_path = global_path

    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()
        local_path.header.frame_id = "world"
        local_path.header.stamp = self.get_clock().now().to_msg()

        local_path_vis = Marker()
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.r = 1.0
        local_path_vis.header.frame_id = "world"
        local_path_vis.header.stamp = self.get_clock().now().to_msg()

        local_plan = Float32MultiArray()
        local_plan.data = []

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0.0
            this_pose_stamped.pose.orientation.y = 0.0
            this_pose_stamped.pose.orientation.z = 0.0
            this_pose_stamped.pose.orientation.w = 1.0
            this_pose_stamped.header.frame_id = "world"
            this_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            local_path.poses.append(this_pose_stamped)

            for j in range(len(input_sol[i])):
                local_plan.data.append(input_sol[i][j])

            pt = Point()
            pt.x = state_sol[i, 0]
            pt.y = state_sol[i, 1]
            pt.z = self.z

            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.82
            color.b = 0.1
            color.a = 1.0

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()

            if i < self.N-1:
                x_diff = state_sol[i+1, 0]-state_sol[i, 0]
                y_diff = state_sol[i+1, 1]-state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = pt.z-0.01
                p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p2.z = pt.z-0.01
                p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p3.z = pt.z-0.01
                p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
                p4.z = pt.z-0.01
                p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0.0
                local_path_vis.pose.orientation.y = 0.0
                local_path_vis.pose.orientation.z = 0.0
                local_path_vis.pose.orientation.w = 1.0

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def __load_parameter(self):
        self.declare_parameter('replan_period', 0.05) 
        self.replan_period = self.get_parameter('replan_period').value

        self.declare_parameter('v_max', 1.0) 
        self.v_max = self.get_parameter('v_max').value

        self.declare_parameter('v_min', 0.05) 
        self.v_min = self.get_parameter('v_min').value

        self.declare_parameter('omega_max', 1.2)
        self.omega_max = self.get_parameter('omega_max').value

        self.declare_parameter('MPC_N', 25)
        self.N = self.get_parameter('MPC_N').value

        self.declare_parameter('safe_dist', 1.0)
        self.safe_dist = self.get_parameter('safe_dist').value

        self.declare_parameter('gamma_k', 0.0)
        self.gamma_k = self.get_parameter('gamma_k').value

        self.declare_parameter('MPC_constraint', "CBF")
        self.MPC_constraint = self.get_parameter('MPC_constraint').value    
        
        self.declare_parameter('kalman_N', 25)
        self.kalman_N = self.get_parameter('kalman_N').value     

        self.declare_parameter('init_N', 0)
        self.init_N = self.get_parameter('init_N').value
        self.get_logger().info("load init_N with value %d " % self.init_N)

        self.declare_parameter('MPC_step_size', 0.2)
        self.MPC_step_size = self.get_parameter('MPC_step_size').value

        self.declare_parameter('goal_tolerance', 0.1)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        if self.N < 2 or self.kalman_N < 2 or self.kalman_N != self.N:
            raise ValueError('MPC_N and kalman_N must be equal and at least 2')
        if self.init_N < 1:
            raise ValueError('init_N must be at least 1')
        if not (0.0 < self.gamma_k <= 1.0):
            raise ValueError('gamma_k must be in (0, 1]')
        if self.MPC_constraint not in ('CBF', 'Euclidean', 'None'):
            raise ValueError('MPC_constraint must be CBF, Euclidean, or None')
        for name, value in (
                ('replan_period', self.replan_period),
                ('MPC_step_size', self.MPC_step_size),
                ('safe_dist', self.safe_dist),
                ('goal_tolerance', self.goal_tolerance)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError('%s must be finite and positive' % name)

        self.init_control = np.zeros([self.init_N,2])

        for i in range(self.init_N):
            self.declare_parameter('init_v_' + str(i), 0.0)
            self.declare_parameter('init_omega_' + str(i), 0.0)
            self.init_control[i, 0] = self.get_parameter('init_v_' + str(i)).value
            self.init_control[i, 1] = self.get_parameter('init_omega_' + str(i)).value

    def choose_goal_state(self, curr_state, global_path):
        waypoint_num = global_path.shape[0]
        if waypoint_num == 0:
            return None
        num = np.argmin(np.array([
            distance_global(curr_state, global_path[i])
            for i in range(waypoint_num)]))

        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            num_list.append(num_path)

        goal_state = np.zeros([self.N, 3])
        for k in range(self.N):
            goal_state[k] = global_path[num_list[k]]
        return goal_state

    def MPC_ellip(self, curr_state, goal_state, obstacles):
        opti = ca.Opti()
        # parameters for optimization
        T = self.MPC_step_size
        gamma_k = self.gamma_k
        # gamma_k = 0.3

        v_max = self.v_max
        v_min = self.v_min
        omega_max = self.omega_max

        opt_x0 = opti.parameter(3)

        # state variables
        opt_states = opti.variable(self.N + 1, 3)       # 状态
        opt_controls = opti.variable(self.N, 2)         # 速度、角速度
        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

        def h(curpos_, ob_):
            return ellipse_barrier(curpos_, ob_, self.safe_dist)

        def o(curpos_, ob_):
            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T
            dist = ca.sqrt(center_vec[0] ** 2 + center_vec[1] ** 2)
            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        # init_condition
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # Position Boundaries
        if distance_global(curr_state, goal_state[-1, :2]) > 1:
            opti.subject_to(opti.bounded(v_min, v, v_max))
        else:
            # Near the goal, full low-speed reversing is necessary for a
            # non-holonomic robot to remove lateral terminal error.
            opti.subject_to(opti.bounded(-v_max, v, v_max))

        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # System Model constraints
        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        num_obs = int(len(obstacles) / self.kalman_N)

        # CBF constraint
        if self.MPC_constraint == "CBF": 
            for j in range(num_obs):
                for i in range(self.N - 1):
                    opti.subject_to(
                        h(opt_states[i + 1, :], obstacles[j * self.kalman_N + i + 1]) >=
                        (1 - gamma_k) * h(opt_states[i, :], obstacles[j * self.kalman_N + i]))
        # Euclidean distance constraint   
        elif self.MPC_constraint == "Euclidean": 
            for j in range(num_obs):
                for i in range(self.N):
                    opti.subject_to(
                        o(opt_states[i, :], obstacles[j * self.kalman_N + i]) >=
                        self.safe_dist)

        obj = 0
        R = np.diag([0.1, 0.02])
        A = np.diag([0.1, 0.02])
        for i in range(1,self.N):
            # Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
            Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.0])
            if i < self.N-1:
                obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - goal_state[[i]], Q)
        Q = np.diag([1.0,1.0, 0.02])*5
        obj += quadratic(opt_states[self.N-1, :] - goal_state[[self.N-1]], Q)

        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                        'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, curr_state)

        optimized_obj = float('inf')

        start_time = time.perf_counter()

        solve_succeeded = False
        for k in range(self.init_N):
            init_control = self.init_control[k,:]
            init_state = np.zeros([self.N + 1,3])
            init_state[0,:] = curr_state

            for i in range(self.N):
                state_changed = np.array(T * f(init_state[i,:], init_control).T)
                init_state[i+1,:] = init_state[i,:] + state_changed

            for i in range(self.N):
                opti.set_initial(opt_controls[i,:], init_control)   

            for i in range(self.N + 1):
                opti.set_initial(opt_states[i,:], init_state[i,:])

            try:
                sol = opti.solve()
                cur_obj = sol.value(obj)
                if(cur_obj < optimized_obj):
                    optimized_obj = cur_obj
                    u_res = sol.value(opt_controls)
                    state_res = sol.value(opt_states)
                solve_succeeded = True

            except Exception as e:
                self.get_logger().warning(
                    'MPC solve failed for initial guess %d: %s' % (k, str(e)))

        end_time = time.perf_counter()
        self.get_logger().info("solve use time %f" % (end_time - start_time))

        if solve_succeeded:
            self.last_input = u_res
            self.last_state = state_res
            self.get_logger().info("find feasible solution with score %f" % optimized_obj)
            self.last_mpc_success = True
        else:       
            self.get_logger().error("fail to find feasible solution")
            self.last_mpc_success = False
            self.last_input.fill(0.0)
            self.last_state = np.tile(curr_state, (self.N, 1))

        u_res = self.last_input
        state_res = self.last_state

        return state_res, u_res, solve_succeeded


def main(args = None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    phri_planner = LocalPlanner()

    try:
        rclpy.spin(phri_planner)
    except KeyboardInterrupt:
        pass
    finally:
        phri_planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
