#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# bspline_path_sampler.py —— 融合缝：名义 B 样条 → 按弧长几何采样的 nav_msgs/Path
# =============================================================================
# 【本文件是全项目唯一由我们新写的对接逻辑】
#   本包的另外三个文件（local_planner.py / controller.py / barrier.py）都是参考实现的
#   **逐字复制**，逻辑零改动；所有为适配本项目而写的东西都在本文件里。
#
# 【为什么必须按弧长采样（本融合的关键）】
#   规划层发布的名义轨迹 planning/bspline（dynacbf_msgs/Bspline）是一条**带时间的**
#   均匀 B 样条：控制点/曲线参数与时间轴绑定（有效参数域 [knots[order], knots[m-order]]，
#   对应 [0, duration]）。而 DCBF-MPC（local_planner.py 的 __global_path_cb）要的是
#   **几何路径**：它只读 nav_msgs/Path 里每个路点的 x/y，然后用
#   "最近点 + 向前 N 个点"（choose_goal_state）当成参考序列。
#
#   如果按时间采样（第 k 个参考点 = 曲线在 exec_time + k*dt 处的位置），一旦机器人为了
#   满足 D-CBF 硬约束而减速/让行，时间轴上的参考点仍按原速度往前跑，代价函数就持续把
#   机器人往"它追不上的参考"上拽 —— 参考与 CBF 约束直接对抗，典型后果是 IPOPT 求解
#   不可行（然后 MPC_ellip 走失败分支停车），或者被迫牺牲安全性。
#
#   按弧长采样后：先找机器人在曲线上的最近点对应的弧长 s_now，再从 s_now 起沿曲线向前
#   按固定弧长步长 arc_step 取 path_points 个点。于是参考窗永远"贴着机器人当前进度"
#   向前推固定距离：
#     * 机器人减速/停住 → 参考窗不前进 → 不产生任何附加代价，CBF 与参考不再矛盾；
#     * 机器人前进 → 参考窗跟着前进（resample_on_odom 打开时随 body_pose 刷新）。
#
# 【发布话题】global_path（与参考实现 local_planner.py 第 55 行订阅的话题名**完全同名**，
#   因此 local_planner_node 不需要任何 remap；本文件也只发布这一个话题，不改动它）。
#
# 【算法来源】B 样条求值与弧长重参数化复刻自本项目早期 C++ 控制器中的 BsplineEvaluator
#   （原文件 dynacbf_controller/src/dcbf_math.cpp，该包在融合重构时已整体删除），保留其
#   de Boor 的节点区间选择规则、弧长表、弧长↔参数互查、最近参数
#   扫描；节点向量约定来自 dynacbf_planner/src/bspline_opt/uniform_bspline.cpp:16-43
#   （knots 长度 = 控制点数 + order + 1，有效域 [knots[order], knots[控制点数]]）。
#   本文件是独立实现：只 import 标准库 / rclpy / ROS 2 消息包 / numpy / dynacbf_msgs，
#   不 import 也不依赖任何外部仓库或其它包。
# =============================================================================

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from dynacbf_msgs.msg import Bspline

# 弧长表采样数：与早期 C++ 实现（已删除）中
# `buildArcLengthTable(int samples = 400)` 的默认值一致。
ARC_TABLE_SAMPLES = 400


class BsplineCurve:
    """二维 B 样条的 de Boor 求值 + 弧长重参数化。

    [算法原文] 复刻早期 C++ 实现的 BsplineEvaluator（源文件已删除）：

        n_ = 控制点数 - 1
        m_ = n_ + p + 1                （p = order）
        节点向量长度 = m_ + 1 = 控制点数 + order + 1
        有效参数域   = [knots[order], knots[m_ - order]]
        求值时把 t 平移成 u = t + knots[order]（positionAt 的约定）

    只保留二维（x, y）：DCBF-MPC 是世界系平面上的 unicycle 模型，
    消息 pos_pts 的 z 分量被忽略（参考实现同样只读 Path 的 x/y）。
    """

    # 与 C++ 版一致的节点分母保护阈值（dcbf_math.cpp:198、331）
    MIN_KNOT_DENOM = 1e-12

    def __init__(self, order, knots, points):
        self.order = int(order)
        self.knots = [float(k) for k in knots]
        self.points = [(float(p[0]), float(p[1])) for p in points]
        self.valid = False
        self.m = 0
        self.duration = 0.0
        self.total_length = 0.0
        self.arc_t = []
        self.arc_s = []

        cols = len(self.points)
        # dcbf_math.cpp:152-157
        if self.order < 2 or cols <= self.order:
            return
        self.m = cols + self.order
        if len(self.knots) != self.m + 1:
            return
        # dcbf_math.cpp:169-176：节点必须严格非降且有限，否则 de Boor 会除零
        for i in range(1, len(self.knots)):
            if (not (self.knots[i] >= self.knots[i - 1])) or \
                    (not math.isfinite(self.knots[i])):
                return
        t_begin = self.knots[self.order]
        t_end = self.knots[self.m - self.order]
        if not (t_end > t_begin):
            return
        self.duration = t_end - t_begin
        self.valid = True
        self.build_arc_length_table()

    # ---------------- 求值 ----------------
    @staticmethod
    def _de_boor(pts, order, knots, m, u):
        """[算法原文] dcbf_math.cpp:296-338：含 u 的夹取与节点区间选择。"""
        cols = len(pts)

        # 与 C++ 一致：u 夹到 [u_p, u_{m-p}]
        ub = min(max(u, knots[order]), knots[m - order])

        # 确定 ub 所在节点区间：k 从 order 开始，直到 u_{k+1} >= ub
        k = order
        while k + 1 <= m and knots[k + 1] < ub:
            k += 1
        if k + 1 > m:
            k = m - 1

        base = k - order
        if base < 0:
            return pts[0]
        if base + order >= cols:
            return pts[-1]

        d = [pts[base + i] for i in range(order + 1)]
        for r in range(1, order + 1):
            for i in range(order, r - 1, -1):
                denom = knots[i + 1 + k - r] - knots[i + k - order]
                if abs(denom) > BsplineCurve.MIN_KNOT_DENOM:
                    alpha = (ub - knots[i + k - order]) / denom
                else:
                    alpha = 0.0
                d[i] = ((1.0 - alpha) * d[i - 1][0] + alpha * d[i][0],
                        (1.0 - alpha) * d[i - 1][1] + alpha * d[i][1])
        return d[order]

    def position_at(self, t):
        """[算法原文] dcbf_math.cpp:340-345：positionAt(t)。"""
        if not self.valid:
            return (0.0, 0.0)
        u = t + self.knots[self.order]
        return self._de_boor(self.points, self.order, self.knots, self.m, u)

    # ---------------- 弧长重参数化 ----------------
    def build_arc_length_table(self, samples=ARC_TABLE_SAMPLES):
        """[算法原文] dcbf_math.cpp:230-254：按等参数间隔累积弦长建表。"""
        self.arc_t = []
        self.arc_s = []
        self.total_length = 0.0
        if (not self.valid) or self.duration <= 0.0 or samples < 2:
            return

        self.arc_t.append(0.0)
        self.arc_s.append(0.0)
        acc = 0.0
        prev = self.position_at(0.0)
        for i in range(1, samples + 1):
            t = self.duration * float(i) / float(samples)
            cur = self.position_at(t)
            acc += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            self.arc_t.append(t)
            self.arc_s.append(acc)
            prev = cur
        self.total_length = acc

    def arc_length_at(self, t):
        """[算法原文] dcbf_math.cpp:256-267：arcLengthAt(t)（分段线性插值）。"""
        if len(self.arc_t) < 2:
            return 0.0
        tc = min(max(t, 0.0), self.duration)
        i = 1
        while i < len(self.arc_t) and self.arc_t[i] < tc:
            i += 1
        if i >= len(self.arc_t):
            return self.total_length
        t0, t1 = self.arc_t[i - 1], self.arc_t[i]
        s0, s1 = self.arc_s[i - 1], self.arc_s[i]
        if t1 - t0 < 1e-12:
            return s1
        return s0 + (s1 - s0) * (tc - t0) / (t1 - t0)

    def parameter_at_arc_length(self, s):
        """[算法原文] dcbf_math.cpp:269-281：parameterAtArcLength(s)。

        s < 0 夹到 0，s > 总长 夹到 duration —— 因此"采样窗越过曲线末端"时，
        末端之后的采样点会全部落在曲线终点（等效于参考点饱和，与
        local_planner.py choose_goal_state 里 `min(waypoint_num - 1, ...)` 的行为一致；
        机器人跑到曲线终点后，MPC 的到达判定随之触发，cmd_move=False 停车）。
        """
        if len(self.arc_s) < 2:
            return 0.0
        if s <= 0.0:
            return 0.0
        if s >= self.total_length:
            return self.duration
        i = 1
        while i < len(self.arc_s) and self.arc_s[i] < s:
            i += 1
        if i >= len(self.arc_s):
            return self.duration
        s0, s1 = self.arc_s[i - 1], self.arc_s[i]
        t0, t1 = self.arc_t[i - 1], self.arc_t[i]
        if s1 - s0 < 1e-12:
            return t1
        return t0 + (t1 - t0) * (s - s0) / (s1 - s0)

    def nearest_parameter(self, p):
        """[算法原文] dcbf_math.cpp:283-294：在弧长表采样点上取最近参数。

        分辨率 = 总长 / ARC_TABLE_SAMPLES，与 C++ 版一致（不做事后细化）。
        """
        if not self.arc_t:
            return 0.0
        best = float('inf')
        best_t = 0.0
        for t in self.arc_t:
            q = self.position_at(t)
            d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
            if d < best:
                best = d
                best_t = t
        return best_t


class BsplinePathSampler(Node):
    """订阅 planning/bspline，按弧长几何采样后发布 global_path（nav_msgs/Path）。"""

    def __init__(self):
        super().__init__('bspline_path_sampler')

        # ---------------- 参数（本包新增；参考实现里没有这个节点） ----------------
        # 输入：规划层的名义 B 样条
        self.declare_parameter('bspline_topic', 'planning/bspline')
        self.bspline_topic = self.get_parameter('bspline_topic').value

        # 输出：参考实现 local_planner.py 订阅的 global_path（同名，无需 remap）
        self.declare_parameter('out_topic', 'global_path')
        self.out_topic = self.get_parameter('out_topic').value

        # B 样条 / Path 所在坐标系（参考实现里 local_planner.py 把 local_path 的
        # frame_id 硬编码为 "world"，controller.py 也是查 'world'→'base_link'，
        # 故这里默认 "world"，与它们保持一致）
        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').value

        # 每次采样输出的路点数；必须 >= MPC_N（默认 25），否则 MPC 的参考窗被截短
        self.declare_parameter('path_points', 120)
        self.path_points = self.get_parameter('path_points').value

        # 弧长步长 [m]。标定依据：参考实现的 global_path_publisher.cpp:28 发布的
        # 参考路径间距是 0.1 m，而 MPC_step_size = 0.25 s，即"隐含参考速度"约
        # 0.4 m/s（v_max 0.5）。若用 0.05 m，MPC 的参考窗会走得比机器人慢
        # （每步只有 0.05 m，而 0.25 s 内能走 0.125 m），跟踪代价会把车速拖到
        # 约 0.2 m/s。要复现参考实现的速度，请把本值设为 0.1（见 config 注释）。
        self.declare_parameter('arc_step', 0.05)
        self.arc_step = self.get_parameter('arc_step').value

        # 取"机器人在曲线上的最近点"必须知道机器人在哪：与参考实现同源订阅位姿。
        # 本项目用 body_pose（nav_msgs/Odometry，与 tracking/controller 同一来源）。
        self.declare_parameter('odom_topic', 'body_pose')
        self.odom_topic = self.get_parameter('odom_topic').value

        # 是否在 body_pose 更新时刷新采样窗（进度前进 >= arc_step 才重发）。
        # 目的：规划层长时间不重规划时，采样窗仍能跟着机器人前进，不会被"用光"。
        self.declare_parameter('resample_on_odom', True)
        self.resample_on_odom = self.get_parameter('resample_on_odom').value

        # ---------------- 启动期校验（非法值抛异常，让节点启动失败） ----------------
        for name, value in (('bspline_topic', self.bspline_topic),
                            ('out_topic', self.out_topic),
                            ('odom_topic', self.odom_topic),
                            ('frame_id', self.frame_id)):
            if not isinstance(value, str) or not value:
                raise ValueError('%s must be a non-empty string' % name)
        if isinstance(self.path_points, bool) or not isinstance(self.path_points, int):
            raise ValueError('path_points must be an integer')
        if self.path_points < 2:
            raise ValueError('path_points must be at least 2')
        if not np.isfinite(self.arc_step) or self.arc_step <= 0.0:
            raise ValueError('arc_step must be finite and positive')
        if not isinstance(self.resample_on_odom, bool):
            raise ValueError('resample_on_odom must be a boolean')

        # ---------------- 状态 ----------------
        self.curve = None
        self.robot_xy = None
        self.anchor_s = None
        self.__warned_no_pose = False
        self.__warned_bad_bspline = False
        self.__logged_first_path = False

        # ---------------- I/O ----------------
        # 规划层发布 planning/bspline 用默认 QoS(depth=10)（reliable/volatile），
        # 这里用 QoS(10) 订阅与之匹配；body_pose 用 SensorDataQoS（与项目一致）。
        self.__sub_bspline = self.create_subscription(
            Bspline, self.bspline_topic, self.__bspline_cb, 10)
        self.__sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.__odom_cb, qos_profile_sensor_data)
        self.__pub_path = self.create_publisher(Path, self.out_topic, 10)

        self.get_logger().info(
            'bspline_path_sampler started: in=%s odom=%s out=%s frame_id=%s '
            'path_points=%d arc_step=%.3f (window=%.2f m) resample_on_odom=%s'
            % (self.bspline_topic, self.odom_topic, self.out_topic, self.frame_id,
               self.path_points, self.arc_step,
               self.path_points * self.arc_step, self.resample_on_odom))

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def __bspline_cb(self, msg):
        """每次收到新 B 样条 → 重建曲线 → 按弧长重新采样并发布。"""
        points = [(p.x, p.y) for p in msg.pos_pts]
        curve = BsplineCurve(msg.order, msg.knots, points)
        if not curve.valid:
            # 非法消息只告警（一次性）并**保留上一条有效轨迹**，不清空、不发布。
            if not self.__warned_bad_bspline:
                self.__warned_bad_bspline = True
                self.get_logger().warning(
                    'Rejected invalid B-spline (order=%d, pos_pts=%d, knots=%d); '
                    'expected order>=2, pos_pts>order and len(knots)==pos_pts+order+1. '
                    'Keeping the previous valid trajectory.'
                    % (msg.order, len(points), len(msg.knots)))
            return
        self.curve = curve
        self.__resample_and_publish()

    def __odom_cb(self, msg):
        """缓存机体位置；(可选) 当在曲线上的进度前进 >= arc_step 时刷新采样窗。"""
        position = msg.pose.pose.position
        xy = (float(position.x), float(position.y))
        if not (math.isfinite(xy[0]) and math.isfinite(xy[1])):
            self.get_logger().warning('Rejected invalid odometry position')
            return
        self.robot_xy = xy

        if not self.resample_on_odom or self.curve is None:
            return
        s_now = self.curve.arc_length_at(
            self.curve.nearest_parameter(self.robot_xy))
        if self.anchor_s is None or abs(s_now - self.anchor_s) >= self.arc_step:
            self.__resample_and_publish()

    # ------------------------------------------------------------------
    # 采样与发布
    # ------------------------------------------------------------------
    def __resample_and_publish(self):
        curve = self.curve
        if curve is None or not curve.valid:
            return

        # ---- 当前进度 s_now = 机器人在曲线上的最近点对应的弧长 ----
        if self.robot_xy is not None:
            s_now = curve.arc_length_at(curve.nearest_parameter(self.robot_xy))
        else:
            if not self.__warned_no_pose:
                self.__warned_no_pose = True
                self.get_logger().warning(
                    'No %s received yet; sampling the nominal B-spline from s=0 '
                    'until a robot pose arrives' % self.odom_topic)
            s_now = 0.0
        self.anchor_s = s_now

        # ---- 从 s_now 起按固定弧长步长取 path_points 个点 ----
        samples = []
        for k in range(self.path_points):
            s = s_now + k * self.arc_step
            t = curve.parameter_at_arc_length(s)
            samples.append(curve.position_at(t))

        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        for k in range(len(samples)):
            x, y = samples[k]
            # 朝向取几何切线（纯信息性：DCBF-MPC 只读 x/y，见 local_planner.py
            # 的 __global_path_cb）。切线退化时沿用上一段切线。
            if k + 1 < len(samples):
                dx = samples[k + 1][0] - x
                dy = samples[k + 1][1] - y
            elif k > 0:
                dx = x - samples[k - 1][0]
                dy = y - samples[k - 1][1]
            else:
                dx = dy = 0.0
            if math.hypot(dx, dy) > 1e-9:
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = stamp
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(0.5 * yaw)
            pose_stamped.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose_stamped)

        self.__pub_path.publish(path)

        if not self.__logged_first_path:
            self.__logged_first_path = True
            self.get_logger().info(
                'first path published to %s: %d points, curve length %.2f m, s_now %.2f m'
                % (self.out_topic, len(path.poses), curve.total_length, s_now))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    sampler = BsplinePathSampler()

    try:
        rclpy.spin(sampler)
    except KeyboardInterrupt:
        pass
    finally:
        sampler.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
