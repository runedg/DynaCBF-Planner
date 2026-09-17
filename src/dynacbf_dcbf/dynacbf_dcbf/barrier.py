# =============================================================================
# 【来源】本文件**逐字复制**自参考实现（另一套已在本机验证跑通的 ROS 2 复现）的
#         local_planner/local_planner/barrier.py（全文 34 行，含文件头的模块 docstring）。
#
# 【改动清单】相对参考实现，本文件只做了：
#   改动 1：在本文件顶部增加这段来源/改动说明注释块。
#   改动 2：无。import（casadi / numpy）与全部函数体一字未动。
#
# 说明：原参考实现的 local_planner/local_planner/local_planner.py 依赖本模块的
#       ellipse_barrier（D-CBF 的 h 函数），因此必须一并复制进来；
#       local_planner.py 中那一行 import 是本包内唯一被调整的 import 语句。
# =============================================================================

"""Geometry helpers for the dynamic elliptical control barrier function."""

import casadi as ca
import numpy as np


def ellipse_barrier(position, obstacle, safe_distance):
    """Return radial clearance from a predicted ellipse minus safety margin.

    ``obstacle`` contains ``[cx, cy, semimajor, semiminor, yaw]``.  The
    expression follows the ray from the ellipse centre to the robot, computes
    its intersection with the ellipse, and uses that boundary distance in the
    D-CBF.  It accepts either numeric positions or CasADi expressions.
    """
    c = float(np.cos(obstacle[4]))
    s = float(np.sin(obstacle[4]))
    dx = position[0] - float(obstacle[0])
    dy = position[1] - float(obstacle[1])
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    semimajor = max(float(obstacle[2]), 1e-3)
    semiminor = max(float(obstacle[3]), 1e-3)
    center_distance = ca.sqrt(dx ** 2 + dy ** 2 + 1e-12)
    radial_denominator = ca.sqrt(
        (semiminor * local_x) ** 2 +
        (semimajor * local_y) ** 2 + 1e-12)
    ellipse_radius = (
        semimajor * semiminor * center_distance / radial_denominator)
    return center_distance - ellipse_radius - safe_distance


def ellipse_barrier_value(position, obstacle, safe_distance):
    """Numeric convenience wrapper used by tests and diagnostics."""
    return float(ellipse_barrier(position, obstacle, safe_distance))
