#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

#include <cstdlib>
#include <string>

// ============================================================================
// 【本项目对上游的唯一改动 —— 仅此两处，其余与上游逐字一致】
//
// 上游在 ModelStatesCallback() 里硬编码 `msg->name[i] != "jackal"`，这是从
// Jackal 机器人项目遗留下来的字面量。本项目仿真的是 turtlebot3，Gazebo 里的
// 实体名由 spawn_turtlebot3.launch.py 的 `-entity` 参数决定，其取值就是环境变量
// TURTLEBOT3_MODEL（该 launch 第 26 / 52 行），即 "burger"。
// 二者永不相等 → has_robot_state_ 恒为 false → OdomPubCallback() 每次直接 return
// → /base_pose_ground_truth **一帧都不会发布** → 规划器持续报
//   "[GridMap] no sensor_pose received"、FSM 报 "no odom."，整条闭环停摆。
//
// 修法：改为读同一个环境变量 TURTLEBOT3_MODEL（缺省 "burger"），与 spawn 侧
// 保持单一来源，避免模型名在两处各自硬编码、再次漂移。
// ============================================================================

double Unwrap(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle <= -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double getYaw(const tf2::Quaternion& q) {
    double sinr_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosr_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    return atan2(sinr_cosp, cosr_cosp);
}

class PseudoOdom : public rclcpp::Node
{
public:
    PseudoOdom() : Node("localization_node")
    {
        // 机器人实体名与 spawn_turtlebot3.launch.py 的 -entity 同源（见文件头说明）。
        const char* model_env = std::getenv("TURTLEBOT3_MODEL");
        robot_model_name_ = (model_env != nullptr && model_env[0] != '\0')
                                ? std::string(model_env)
                                : std::string("burger");

        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 5, std::bind(&PseudoOdom::ModelStatesCallback, this, std::placeholders::_1));
        odom_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PseudoOdom::OdomPubCallback, this));
        pseudo_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("base_pose_ground_truth", 5);
    }

private:
    void ModelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] != robot_model_name_)
                continue;
            if (!has_robot_state_)
            {
                last_update_stamp_ = this->now();
                robot_pose_ = msg->pose[i];
                has_robot_state_ = true;
                return;
            }

            // Update time stamp
            double dt = (this->now() - last_update_stamp_).seconds();
            last_update_stamp_ = this->now();
            // Calculate velocity by differential
            double dx = msg->pose[i].position.x - robot_pose_.position.x;
            double dy = msg->pose[i].position.y - robot_pose_.position.y;

            double last_yaw = getYaw(tf2::Quaternion(
                robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z, robot_pose_.orientation.w));
            double yaw = getYaw(tf2::Quaternion(
                msg->pose[i].orientation.x, msg->pose[i].orientation.y,
                msg->pose[i].orientation.z, msg->pose[i].orientation.w));
            double delta_yaw = Unwrap(yaw - last_yaw);

            Eigen::Vector2d map_vel = {dx / (dt + 1e-8), dy / (dt + 1e-8)}; // add small value to avoid singularity
            Eigen::Matrix2d rot_mat;
            rot_mat << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
            Eigen::Vector2d body_vel = rot_mat * map_vel;

            double yaw_rate = delta_yaw / (dt + 1e-8);
            // Update pose and velocity
            robot_pose_ = msg->pose[i];
            robot_twist_.linear.x = body_vel(0);
            robot_twist_.linear.y = body_vel(1);
            robot_twist_.angular.z = yaw_rate;
        }
    }
    void OdomPubCallback()
    {
        if (!has_robot_state_)
            return;
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = last_update_stamp_;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose = robot_pose_;
        odom_msg.twist.twist = robot_twist_;
        pseudo_odom_pub_->publish(odom_msg);
    }

    std::string robot_model_name_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pseudo_odom_pub_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Twist robot_twist_;
    rclcpp::Time last_update_stamp_;
    bool has_robot_state_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PseudoOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
