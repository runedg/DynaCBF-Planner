#!/usr/bin/env python3
# ============================================================================
# dynacbf.launch.py —— DynaCBF-Planner 融合闭环总装（唯一入口）
# ============================================================================
# 【本文件职责】只做跨包编排：把「Gazebo 场地 → 感知 → 跟踪 → 规划 →
# MPC-D-CBF 控制 → 底盘」这条链路按名字拉起来，并统一参数来源。
# 不含任何算法逻辑，也不改其它包的参数文件。
#
# 【编排原则】
#   1) 不涉及"给节点传 parameters"的，尽量 include 复用子包既有 launch；
#   2) **凡是要传 parameters=[...] 的节点，一律在本文件直接声明，不用 include**。
#      原因是 ROS 2 launch 的一个真实缺陷组合（详见下方第 8 步注释）：
#      IncludeLaunchDescription 不做作用域隔离，而 gazebo_ros/gzserver.launch.py
#      会抢先声明 params_file 且默认值为**空串**，任何被 include 进来、依赖
#      LaunchConfiguration('params_file') 的子 launch 都会拿到空串 → 参数静默失效。
#
# 【启动动作构成】
#    1. Gazebo 场地          include dynacbf_simulator/launch/start.launch.py
#                            （传 world + gui；它内部再 include gazebo_ros/gazebo.launch.py）
#    2. static TF            world → odom（单位变换，与参考实现 sim.launch.py 同款）
#    3. local_map            直接声明。dynacbf_perception 没有 launch 文件。
#                            velodyne_points → local_pcd / ellipse_vis / for_obs_track
#    4. move_test            include dynacbf_simulator/launch/move_test.launch.py
#    5. obs_kf               直接声明。dynacbf_tracking 没有 launch 文件。
#                            /for_obs_track → /obs_predict_pub
#    6. pseudo_odom          直接声明。dynacbf_simulator 没有单给它的 launch。
#                            /gazebo/model_states → /base_pose_ground_truth
#    7. dynacbf_planner_node 直接声明。需要 3 条 remap 与 fsm.navi_mode 覆盖，
#                            planner.launch.py（最小启动）承载不了。
#                            规划层（栅格 → A* → B 样条）→ planning/bspline
#    8. MPC-D-CBF 三节点     直接声明（理由见第 8 步注释）：
#                            local_planner_node + controller_node +
#                            bspline_path_sampler，共用该包的 dcbf_params.yaml。
#                            （该包自带的 dcbf.launch.py 保留作单独调试用，本文件不 include）
#    9. 本体                 include turtlebot3_gazebo 的
#                            robot_state_publisher.launch.py + spawn_turtlebot3.launch.py
#   10. rviz2（可选）
#
# 【节点名说明 —— 看起来与源码不一致，但这是有意为之】
#   local_map / obs_kf / pseudo_odom 三个名字与源码内部 Node("...") 不同：
#     源码里分别是 local_map_pub / obs_param_node / localization_node。
#   本文件沿用参考实现 sim.launch.py 的写法统一改名，原因：这三个节点的参数文件
#   顶层键都是 `/**`（通配，不限节点名）或压根没有参数，改名不会让参数失效；
#   而统一成"可执行文件名"更便于 ros2 node list 排障。
#   唯一必须精确匹配的是规划器：planner.yaml 的顶层键是 `dynacbf_planner_node:`，
#   因此 name 必须写 dynacbf_planner_node（见 dynacbf_planner/launch/planner.launch.py
#   顶部关于这一点的警告）。
#
# 【闭环链 —— 也是排障时的验证顺序】
#   /velodyne_points → /local_pcd → /ellipse_vis → /for_obs_track
#     → /obs_predict_pub → /planning/bspline → /global_path
#     → /local_plan → /cmd_vel（非零）→ Gazebo 底盘
#
# 【与参考实现的差异 —— 只有两处，且都是"替换"不是"改逻辑"】
#   * 参考的 scene/launch/sim.launch.py 会起 linear_path_publisher 往 /global_path
#     发一条直线路径；本项目没有该包，改由 dynacbf_planner 产出 B 样条，再由
#     bspline_path_sampler 按弧长采样成 /global_path 来取代它。
#     —— 这也是本文件**不能**直接 include simulator/launch/sim.launch.py 的原因：
#        那份文件里仍留着对 linear_path_publisher 的引用。
#   * 参考把 local_planner / controller 单独放在 local_planner_launch.py 里；
#     本文件把它们连同融合缝一并纳入，使整个闭环只需一条命令启动。
#   其余节点、话题名、参数来源均与参考实现一致。
#
# 【三条硬性前置（缺一条就"看起来没反应"）】
#   1. /base_pose_ground_truth 必须由 pseudo_odom 发布（依赖 Gazebo 已起）。
#   2. controller_node 需要 TF world → base_link 存在：
#      world→odom 由本文件发；odom→base_footprint 由 Gazebo diff_drive 插件发
#      （turtlebot3_burger_velodyne/model.sdf 里已设 <publish_odom_tf>true</publish_odom_tf>）；
#      base_footprint→base_link 由 robot_state_publisher 发。
#   3. /obs_predict_pub 必须每条障碍恰好 kalman_N=25 步、步间隔 MPC_step_size=0.25 s。
#      本链路天然满足：发布方 obs_kf.cpp 里 `N(25)` 与 `T = 0.25` 都是硬编码常量，
#      与 dcbf_params.yaml 的 kalman_N=25 / MPC_step_size=0.25 逐字对齐（无参数可漂移）。
#
# 【图形界面】Gazebo GUI 与 RViz 默认都关闭 —— WSL 下由后台进程启动的 GUI 会因
#   X11 连接断开而死。需要看画面时，请在你自己的终端里显式传 gui:=true / rviz:=true。
# ============================================================================
import os

# spawn_turtlebot3.launch.py 会读 TURTLEBOT3_MODEL 并把模型名拼成
# turtlebot3_<MODEL>_velodyne —— 即实际生成的是**带 VLP-16 的机型**，它发布的
# /velodyne_points 正是感知链路的输入（普通 turtlebot3_burger 没有这个传感器）。
# 这里 setdefault 只补默认值、不覆盖已有环境变量，确保不预先 export 也能一条命令跑通。
os.environ.setdefault("TURTLEBOT3_MODEL", "burger")

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory("dynacbf_simulator")
    perception_share = get_package_share_directory("dynacbf_perception")
    planner_share = get_package_share_directory("dynacbf_planner")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")

    # ---------------- 参数文件：全部取自各包自己的 share/config ----------------
    # 注意：本包（dynacbf_bringup）自己**不带任何参数文件**。每个节点的参数都由
    # 拥有该节点的包提供，保持单一来源，避免两份 yaml 各说各话。
    perception_yaml = os.path.join(perception_share, "config", "config.yaml")
    simulator_yaml = os.path.join(sim_share, "config", "config.yaml")
    planner_yaml = os.path.join(planner_share, "config", "planner.yaml")
    keypoints_default = os.path.join(planner_share, "config", "keypoints.demo.yaml")

    # RViz 配置用 dynacbf_simulator 自带的那份（与参考实现 sim.launch.py 用的是同一份）。
    rviz_config = os.path.join(sim_share, "rviz", "rviz_config.rviz")

    # ---------------- launch 参数 ----------------
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    navi_mode = LaunchConfiguration("navi_mode")
    keypoints_file = LaunchConfiguration("keypoints_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_args = [
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="是否启动 Gazebo 图形客户端。默认 false（WSL 下应留 false；"
            "需要看画面时请在你自己的终端里传 true）。",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="是否启动 RViz2（加载 dynacbf_simulator/rviz/rviz_config.rviz）。"
            "默认 true：本项目的常规用法是在 RViz 里用工具栏的 2D Goal Pose 给目标"
            "（对应 navi_mode=1），所以默认把 RViz 一起拉起来。"
            "无头跑请传 rviz:=false。",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(sim_share, "worlds", "world1.world"),
            description="Gazebo 世界文件。默认 world1.world，与参考实现 sim.launch.py 一致。",
        ),
        DeclareLaunchArgument(
            "navi_mode",
            default_value="1",
            description="规划器导航模式，作为 fsm.navi_mode 注入："
            "1=等待 move_base_simple/goal（**默认**；在 RViz 里用工具栏的 2D Goal Pose "
            "点一个目标即可，且首个目标会被里程计门控）；"
            "2=按预设航点序列自动开跑（收到第一帧位姿后自动启动，适合无头跑）；"
            "3=reference_path。",
        ),
        DeclareLaunchArgument(
            "keypoints_file",
            default_value=keypoints_default,
            description="navi_mode:=2 使用的航点参数 YAML（顶层键 dynacbf_planner_node）。"
            "默认用 dynacbf_planner/config/keypoints.demo.yaml。",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="只传给 robot_state_publisher 的仿真时间开关（Gazebo 会发 /clock）。"
            "算法链路各节点沿用其自身 config 的设定 —— 参考实现同样如此，本参数不覆盖它们。",
        ),
    ]

    # ---------------- 1. Gazebo 场地 ----------------
    # 复用 dynacbf_simulator 自己的 start.launch.py（逐字复制自参考实现的 scene 包），
    # 它内部再 include gazebo_ros/gazebo.launch.py，把 world 交给 Gazebo。
    # 注意其自带默认 gui:=true，这里一律用本文件的 gui 覆盖（默认 false）。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "start.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items(),
    )

    # ---------------- 2. world → odom 静态 TF ----------------
    # 与参考实现 sim.launch.py 同款。完整链路：
    # world → odom（本节点）→ base_footprint（Gazebo diff_drive）→ base_link（robot_state_publisher）。
    world_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_odom_broadcaster",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "world", "--child-frame-id", "odom",
        ],
    )

    # ---------------- 3. 感知：local_map ----------------
    # 订阅 velodyne_points（用 TF 转到 world），发布 gridmap / local_pcd（world 系点云）
    # / ellipse_vis / for_obs_track。该包没有 launch 文件，只能在此声明节点。
    local_map = Node(
        package="dynacbf_perception",
        executable="local_map",
        name="local_map",
        output="screen",
        parameters=[perception_yaml],
    )

    # ---------------- 4. 移动障碍物：move_test ----------------
    # 行为：订阅 cmd_move（由 local_planner_node 发布），障碍物**不会**在启动时立即
    # 运动，而是等规划器给出第一条有效指令后才开始 —— 这从构造成立了"初始状态安全"
    # 这一 MPC-D-CBF 安全保证的前提，不需要靠固定延迟硬凑。
    #
    # ⚠️ 这里**必须直接声明节点，不能 include move_test.launch.py**。
    # 原因（已在 ROS 2 Humble 源码里确认，不是猜测）：
    #   * launch.actions.IncludeLaunchDescription 不做作用域隔离 —— 其 execute() 结尾是
    #     `return [*set_launch_configuration_actions, launch_description]`，参数直接写进
    #     共享的 launch_configurations；
    #   * DeclareLaunchArgument.execute() 在发现同名键已存在时**静默跳过**默认值
    #     （declare_launch_argument.py:205 `if self.name not in context.launch_configurations`）。
    #   move_test.launch.py 与 dcbf.launch.py 各自 DeclareLaunchArgument('params_file')，
    #   同处一个作用域时后者必被前者压制 → dcbf 三节点收不到 --params-file
    #   → local_planner_node 的 init_N 回落代码默认值 0 → 启动即抛
    #   ValueError('init_N must be at least 1') 退出（首次实测即栽在这里）。
    #   参考实现天然避开此问题：它把两层拆成两条独立命令、两个独立 launch 进程。
    #   本包要"一条命令起全链路"，故在此直接声明，并把参数文件路径写成明确的绝对路径。
    move_test = Node(
        package="dynacbf_simulator",
        executable="move_test",
        name="move_test",
        output="screen",
        parameters=[simulator_yaml],
    )

    # ---------------- 5. 跟踪：obs_kf ----------------
    # 订阅 /for_obs_track，发布 /obs_predict_pub（预测椭圆序列）与 /obs_predict_vis_pub。
    # 该包没有 launch 文件，且该节点无任何声明参数（N=25 / T=0.25 硬编码在
    # obs_kf.cpp 构造函数里），因此这里不需要传 parameters。
    obs_kf = Node(
        package="dynacbf_tracking",
        executable="obs_kf",
        name="obs_kf",
        output="screen",
    )

    # ---------------- 6. 位姿源：pseudo_odom ----------------
    # /gazebo/model_states → /base_pose_ground_truth（nav_msgs/Odometry，world→base_link）。
    # 它是本链路唯一的位姿源头，被规划器、融合缝、以及 local_map 的 TF 共同依赖。
    # 无声明参数，故不传 parameters。
    pseudo_odom = Node(
        package="dynacbf_simulator",
        executable="pseudo_odom",
        name="pseudo_odom",
        output="screen",
    )

    # ---------------- 7. 规划层：dynacbf_planner_node ----------------
    # 三条 remap 的理由（不是为新命名，而是对接既有话题）：
    #   cloud       ← local_pcd                ：规划器的 GridMap 要**世界系**点云做射线投射，
    #                                            而 local_map 的 local_pcd 正是
    #                                            header.frame_id="world" 的全局系点云
    #                                            （planner.yaml 亦设 cloud_is_world: true）。
    #                                            不能接原始 velodyne_points —— 那是传感器系。
    #   sensor_pose ← /base_pose_ground_truth  ：GridMap 用传感器位姿定位滑窗地图中心；
    #                                            因 need_extrinsic=false，只需机体位姿即可。
    #   body_pose   ← /base_pose_ground_truth  ：FSM 的里程计输入。
    # name 必须与 planner.yaml 的顶层键 `dynacbf_planner_node:` 完全一致，否则参数静默失效。
    planner_node = Node(
        package="dynacbf_planner",
        executable="dynacbf_planner_node",
        name="dynacbf_planner_node",
        output="screen",
        remappings=[
            ("cloud", "local_pcd"),
            ("sensor_pose", "/base_pose_ground_truth"),
            ("body_pose", "/base_pose_ground_truth"),
        ],
        parameters=[
            planner_yaml,
            keypoints_file,
            # 放在最后，保证 launch 参数 navi_mode 永远覆盖
            # keypoints.demo.yaml 里写死的 fsm.navi_mode。
            {"fsm.navi_mode": ParameterValue(navi_mode, value_type=int)},
        ],
    )

    # ---------------- 8. MPC-D-CBF 三节点 ----------------
    # ⚠️ 这三个节点必须**直接声明**，不能用 IncludeLaunchDescription 引入
    # dynacbf_dcbf/launch/dcbf.launch.py。原因（已逐条在源码中确认，非推测）：
    #   * gazebo_ros/gzserver.launch.py（被第 1 步的 Gazebo 间接 include）声明了
    #     `'params_file', default_value=''` —— 注意默认值是**空字符串**
    #     （该文件第 173 行）；
    #   * DeclareLaunchArgument.execute() 发现同名键已存在时**静默跳过**默认值
    #     （launch/actions/declare_launch_argument.py:205）；
    #   * IncludeLaunchDescription 不做作用域隔离 —— 其 execute() 结尾为
    #     `return [*set_launch_configuration_actions, launch_description]`，
    #     参数直接写进共享的 launch_configurations，没有 GroupAction(scoped=True)。
    #   三者叠加的后果：dcbf.launch.py 里 params_file 的默认值被 gzserver 的空串压制
    #   → 三个节点收不到 --params-file → launch_ros 报
    #     "Parameter file path is not a file: ."（空串被 abspath 成 "."）
    #   → local_planner_node 的 init_N 回落代码默认值 0 → 启动即抛
    #     ValueError('init_N must be at least 1') 并退出（首次实测即栽在这里）。
    #
    #   对照实验可证问题只出在"被 bringup include"这条路径上：
    #     单独执行 `ros2 launch dynacbf_dcbf dcbf.launch.py` 时，
    #     三个节点都带上了 --params-file 且 local_planner_node 正确打印
    #     "load init_N with value 2"。
    #   故 dcbf.launch.py 本身完好，保留作为"单独调试 MPC-D-CBF 层"的入口。
    dcbf_yaml = os.path.join(
        get_package_share_directory("dynacbf_dcbf"), "config", "dcbf_params.yaml"
    )
    # 参数一律来自 dynacbf_dcbf/config/dcbf_params.yaml —— 该文件顶层键是 `/**`
    # （通配，对三个节点同时生效），前段是参考实现 config.yaml 的逐字复制，
    # 末尾是本项目新写的融合缝参数；三节点共用一份且互不干扰
    # （ROS 2 对"节点未声明"的键是忽略，不报错）。
    local_planner_node = Node(
        package="dynacbf_dcbf",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[dcbf_yaml],
    )
    controller_node = Node(
        package="dynacbf_dcbf",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[dcbf_yaml],
    )
    bspline_path_sampler = Node(
        package="dynacbf_dcbf",
        executable="bspline_path_sampler",
        name="bspline_path_sampler",
        output="screen",
        parameters=[dcbf_yaml],
    )

    # ---------------- 9. 机器人本体 ----------------
    tb3_launch_dir = os.path.join(tb3_share, "launch")
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "spawn_turtlebot3.launch.py")
        ),
        # 出生点与 keypoints.demo.yaml 的起点一致（原点附近），
        # 避免首个航点方向与出生朝向冲突。
        launch_arguments={"x_pose": "0.0", "y_pose": "0.0"}.items(),
    )

    # ---------------- 10. RViz（可选） ----------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    # ---------------- 编排 ----------------
    return LaunchDescription(
        declare_args
        + [
            LogInfo(
                msg="[dynacbf_bringup] 启动融合闭环：Gazebo → 感知 → 跟踪 → 规划 → MPC-D-CBF → 底盘。"
                " 无头模式（gui/rviz 默认 false）；需要画面请在自己的终端传 gui:=true rviz:=true。"
            ),
            gazebo,
            world_odom_tf,
            local_map,
            move_test,
            obs_kf,
            pseudo_odom,
            planner_node,
            local_planner_node,
            controller_node,
            bspline_path_sampler,
            robot_state_publisher,
            spawn_turtlebot3,
            rviz_node,
        ]
    )
