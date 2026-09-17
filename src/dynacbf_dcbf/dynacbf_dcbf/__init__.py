# -*- coding: utf-8 -*-
"""dynacbf_dcbf —— MPC-D-CBF 局部规划层（沿用参考实现的节点名与话题名）。

包内文件与来源：

* :mod:`dynacbf_dcbf.local_planner`      —— **逐字复制**自参考实现
  local_planner/local_planner/local_planner.py（唯一改动：顶部来源注释 +
  barrier 的 import 前缀，见文件顶部【改动清单】）。
* :mod:`dynacbf_dcbf.controller`         —— **逐字复制**自参考实现
  local_planner/local_planner/controller.py（唯一改动：顶部来源注释）。
* :mod:`dynacbf_dcbf.barrier`            —— **逐字复制**自参考实现
  local_planner/local_planner/barrier.py（被 local_planner.py 依赖，故一并复制；
  唯一改动：顶部来源注释）。
* :mod:`dynacbf_dcbf.bspline_path_sampler` —— **本包唯一新写的对接逻辑**（融合缝）：
  把 planning/bspline 的名义 B 样条按弧长几何采样成 nav_msgs/Path 发布到
  global_path，供逐字复制的 local_planner_node 直接消费，无需任何 remap。

本模块刻意不做任何 import（包括延迟 import），以保证 `import dynacbf_dcbf`
不触发 casadi / rclpy 的加载。
"""

__all__ = []
