#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/convert.h>
#include <tf2/transform_storage.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <eigen3/Eigen/Core>

// #include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "ellipse.hpp"
#include "DBSCAN.hpp"
#include "KM.hpp"

using namespace grid_map;

GridMap map_({"elevation", "local_lidar", "gradient_map"});

class LocalMapPubNode : public rclcpp::Node
{
public:
    LocalMapPubNode() : Node("local_map_pub")
    {
        InitParameters();
        InitializeTFListener();
        InitializeSubPub();
        
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 5));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LocalMapPubNode::timer_callback, this));
    }

private:
    void InitParameters()
    {
        this->declare_parameter<float>("localmap_x_size", 10.0);
        this->declare_parameter<float>("localmap_y_size", 10.0);
        this->declare_parameter<float>("max_heigh", 2.0);
        this->declare_parameter<float>("resolution", 0.1);

        this->declare_parameter<float>("obs_height", 0.4);
        this->declare_parameter<float>("step_height", 0.5);
        // 【本项目新增】local_pcd 的 z 下限（滤除地面回波）。
        // 原版这里没有该参数，pass_z 的下限被硬编码成 -1，于是地面点被一并灌进
        // local_pcd；而下游 GridMap 的 raycastProcess 对「5 m 内且在地图内」的点
        // **无条件标 hit**（grid_map.cpp:675 `setCacheOccupancy(pt_w, 1)`），
        // 结果地面被写成 occupied，再经 `obstacles_inflation_z_down: 0.4` 向上覆盖到
        // 机体碰撞高度 → 处处都是障碍 → A* 永远搜不到路 → 规划器不出轨迹。
        // VLP-16 是 360° 3D 雷达、地面回波极多，必须在感知层就滤掉。
        // 取 0.1 的依据：此值**必须低于雷达自身的安装高度**，否则连打向障碍物的点
        // 都会被一起滤掉。实测 base_scan 安装高度为 0.172 m
        // （turtlebot3_burger.urdf 的 joint base_scan_joint: origin xyz="-0.032 0 0.172"），
        // 而 VLP-16 接近水平扫描、打在障碍物上的点 z≈0.17 —— 先前误取 0.2 时实测
        // obs=0、椭圆数=0，障碍物被整个滤除，D-CBF 因此拿不到任何障碍输入。
        // 0.1 可滤掉地面回波（world 系 z≈0，5 m 外甚至为负），同时完整保留障碍物点。
        this->declare_parameter<float>("min_pass_z", 0.1);
        this->declare_parameter<float>("DBSCAN_R", 5.0);
        this->declare_parameter<int>("DBSCAN_N", 5);

        this->declare_parameter<int>("block_size", 5);
        this->declare_parameter<int>("block_num", 5);

        this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("base_scan", "base_scan");

        // ----------------------------------------------------------------
        this->get_parameter<float>("localmap_x_size", localmap_x_size);
        this->get_parameter<float>("localmap_y_size", localmap_y_size);
        this->get_parameter<float>("max_heigh", max_heigh);
        
        this->get_parameter<float>("resolution", resolution);

        this->get_parameter<float>("obs_height", obs_height);
        this->get_parameter<float>("step_height", step_height);
        this->get_parameter<float>("min_pass_z", min_pass_z);
        this->get_parameter<float>("DBSCAN_R", DBSCAN_R);
        this->get_parameter<int>("DBSCAN_N", DBSCAN_N);

        this->get_parameter<int>("block_size", block_size);
        this->get_parameter<int>("block_num", block_num);
        
        this->get_parameter<std::string>("lidar_topic", lidar_topic);
        this->get_parameter<std::string>("base_link", base_link);
        this->get_parameter<std::string>("base_scan", base_scan);

        _inv_resolution = 1 / resolution;
        map_index_len = localmap_x_size * _inv_resolution;

        MapParam.localmap_x_size = localmap_x_size;
        MapParam.localmap_y_size = localmap_y_size;
        MapParam.resolution = resolution;

        map_.setFrameId("world");
        map_.setGeometry(Length(localmap_x_size, localmap_y_size), resolution);

        lidar_pcd_matrix = Eigen::MatrixXf(map_index_len, map_index_len);
        map_interpolate = Eigen::MatrixXf(map_index_len, map_index_len);
        gradient_map = Eigen::MatrixXf(map_index_len, map_index_len);

        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);
        pass_z.setFilterFieldName("z");
        // 【本项目修正】这里必须先按参考实现保持 -1（不过滤地面）。
        // 原因：pcd_transform() 的输出 velodyne_cloud_global 有**两个**下游 ——
        //   1) lidar2gridmap() → 高度图 → gradient_map_processing() → 障碍检测
        //      这里必须拿到完整点云：梯度法（Sobel）本来就能区分"连续平地"与
        //      "高度突变"，预先滤地面反而会把障碍物下部的点一起削掉、
        //      让椭圆拟合点数不足（实测 obs 从多个掉到 0~1）。
        //   2) local_pcd() 发布给 GridMap → 这里**需要**滤掉地面。
        // 因此把 min_pass_z 的过滤挪到"发布 local_pcd"那一步（见 timer_callback），
        // 而不是在这里做 —— 一个过滤不能同时满足两个用途。
        pass_z.setFilterLimits(-1, max_heigh);

        sor.setLeafSize(0.05f, 0.05f, 0.05f);

        received_lidar_data = false;

        RCLCPP_INFO(this->get_logger(), "max heigh is set to %f" , max_heigh);
    }

    // 初始化subscribe和publish
    void InitializeSubPub()
    {
        velodyne_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 1, std::bind(&LocalMapPubNode::PointCloudCallback, this, std::placeholders::_1));

        // obs_pc_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("obs_pc", 1);

        gridmap_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("gridmap", 1);
        local_pcd_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_pcd", 1);
        ellipse_vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("ellipse_vis", 1);
        for_obs_track_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("for_obs_track", 1);
        RCLCPP_INFO(this->get_logger(), "init subscribe/publish complete");
    }

    // 初始化tf获取
    void InitializeTFListener()
    {
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        RCLCPP_INFO(this->get_logger(), "init tf complete");
    }

    bool updateTF()
    {
        try
        {
            geometry_msgs::msg::TransformStamped base_link_transformStamped = tf_buffer->lookupTransform("world", base_link, rclcpp::Time(0));
            base_link_transform = base_link_transformStamped;

            geometry_msgs::msg::TransformStamped lidar_link_transformStamped = tf_buffer->lookupTransform("world", base_scan, rclcpp::Time(0));
            lidar_link_transform = lidar_link_transformStamped;
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return false;
        }
    }

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(point_cloud_mutex);
        pcl::fromROSMsg(*msg, velodyne_cloud);
        received_lidar_data = updateTF();
    }

    void timer_callback()
    {
        if(received_lidar_data == false)
        {
            RCLCPP_INFO(this->get_logger(), "wait for lidar");
            return;
        }
        robot_position2d << base_link_transform.transform.translation.x, base_link_transform.transform.translation.y;

        pcd_transform();

        // 【本项目新增】空点云保护。
        // 引入 min_pass_z 滤除地面回波后，若视野内没有任何高于该阈值的点
        // （空场景、或正对开阔地面），velodyne_cloud_global 会为空；而下游
        // map_interpolation / gradient_map_processing 对全 NaN / 全零矩阵做运算
        // 会触发 SIGFPE（除零），节点直接退出（实测 exit code -8 = SIGFPE）。
        // 原版 z 下限硬编码为 -1，地面点总会留下一批，因此该缺陷从未暴露。
        // 这里跳过本帧：不更新地图，等下一帧有有效点再处理。
        if (velodyne_cloud_global.empty())
        {
            received_lidar_data = false;
            return;
        }

        map_.setPosition(robot_position2d);
        map_.clear("local_lidar");
        lidar_pcd_matrix = map_.get("local_lidar");
        lidar2gridmap(lidar_pcd_matrix);
        map_.add("local_lidar", lidar_pcd_matrix);

        map_interpolate = map_interpolation(lidar_pcd_matrix);
        map_interpolate = map_inflate(map_interpolate);
        map_.add("elevation", map_interpolate);

        // obs map
        vector<DBSCAN::Point> non_clustered_obs;
        gradient_map = gradient_map_processing(map_interpolate, non_clustered_obs);

        // DBSCAN
        DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
        vector<Obstacle> clustered_obs(DS.cluster_num);

        for (const auto &obs : non_clustered_obs)
        {
            if (obs.obsID > 0)
            {
                gradient_map(obs.x, obs.y) = -0.3;
                clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
            }
        }

        vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, map_, MapParam);

        KM.tracking(ellipses_array);

        ab_variance_calculation(ellipses_array);

        // publish
        std_msgs::msg::Float32MultiArray for_obs_track;
        for (const auto &ellipse : ellipses_array)
        {
            if (ellipse.label == 0)
                continue;

            for_obs_track.data.push_back(ellipse.cx);
            for_obs_track.data.push_back(ellipse.cy);
            for_obs_track.data.push_back(ellipse.semimajor);
            for_obs_track.data.push_back(ellipse.semiminor);
            for_obs_track.data.push_back(ellipse.theta);
            for_obs_track.data.push_back(ellipse.label);
            for_obs_track.data.push_back(ellipse.variance);
        }

        for_obs_track_pub->publish(for_obs_track);

        // 【本项目修正】发布给 GridMap 的 local_pcd 时，在这一步按 min_pass_z 滤掉地面回波。
        // 原因：GridMap 的 raycastProcess（grid_map.cpp:647-677）对"5 m 内且在地图内"的
        // 点**无条件标 hit**，没有任何地面提取；若不滤，地面会被写成 occupied，再经
        // obstacles_inflation_z_down=0.4 覆盖到机体碰撞高度 → 处处是障碍 → A* 搜不到路。
        // 只在发布这一步过滤：上游 velodyne_cloud_global 仍保持完整点云供障碍检测使用
        // （见 pcd_transform 中 pass_z 的说明）。
        pcl::PointCloud<pcl::PointXYZ> pcd_for_gridmap;
        pcd_for_gridmap.reserve(velodyne_cloud_global.size());
        for (const auto &grid_pt : velodyne_cloud_global)
        {
            if (grid_pt.z >= min_pass_z)
                pcd_for_gridmap.push_back(grid_pt);
        }
        sensor_msgs::msg::PointCloud2 local_velodyne_msg;
        pcl::toROSMsg(pcd_for_gridmap, local_velodyne_msg);
        local_velodyne_msg.header.stamp = this->get_clock()->now();
        local_velodyne_msg.header.frame_id = "world";
        local_pcd_pub->publish(local_velodyne_msg);

        // grid_map_msgs::msg::GridMap gridMapMessage;
        std::shared_ptr<grid_map_msgs::msg::GridMap> gridMapMessage;
        // grid_map_ros2::GridMapRosConverter::toMessage(map_, gridMapMessage);
        gridMapMessage = GridMapRosConverter::toMessage(map_);
        gridmap_pub->publish(*gridMapMessage);

        visEllipse(ellipses_array);

        received_lidar_data = false;
    }

    void pcd_transform()
    {
        std::lock_guard<std::mutex> lock(point_cloud_mutex);
        pass_x.setInputCloud(velodyne_cloud.makeShared());
        pass_x.filter(velodyne_cloud);

        pass_y.setInputCloud(velodyne_cloud.makeShared());
        pass_y.filter(velodyne_cloud);

        pass_z.setInputCloud(velodyne_cloud.makeShared());
        pass_z.filter(velodyne_cloud);

        // 【本项目新增】空点云保护 —— 必须放在 VoxelGrid 之前。
        // pcl::VoxelGrid::applyFilter() 对空输入会调用 getMinMax3D() 却拿不到有效
        // 包围盒，min_b_/div_b_ 保持未初始化值，随后体素索引计算除零/越界
        // → SIGFPE（实测 local_map exit code -8，且崩溃发生在任何后续日志之前）。
        // 引入 min_pass_z 滤除地面回波后，"空场景 / 正对开阔地面"时 pass_z 的结果
        // 可能为空，这条路径才被走到 —— 原版 z 下限为 -1，地面点总留下一批。
        if (velodyne_cloud.empty())
        {
            velodyne_cloud_filter.clear();
            velodyne_cloud_global.clear();
            return;
        }

        velodyne_cloud_filter.clear();
        sor.setInputCloud(velodyne_cloud.makeShared());
        sor.filter(velodyne_cloud_filter);

        Eigen::Affine3d affine_transform;
        affine_transform = tf2::transformToEigen(lidar_link_transform);
        pcl::transformPointCloud(velodyne_cloud_filter, velodyne_cloud_global, affine_transform);
    }

    void lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix)
    {
        int col = lidar_data_matrix.cols();
        int row = lidar_data_matrix.rows();
        for (const auto &pt : velodyne_cloud_global)
        {
            int j = (pt.x - robot_position2d.x()) * _inv_resolution + col * 0.5;
            j = min(max(j, 0), row - 1);
            int k = (pt.y - robot_position2d.y()) * _inv_resolution + row * 0.5;
            k = min(max(k, 0), col - 1);

            if (std::isnan(lidar_data_matrix(row - 1 - j, col - 1 - k)))
                lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
            if (lidar_data_matrix(row - 1 - j, col - 1 - k) < pt.z)
                lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
        }
    }

    void map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius)
    {
        for (int k = 0; k <= 2 * radius; k++)
        {
            for (int q = 0; q <= 2 * radius; q++)
            {
                if (isnan(src(startRow - radius + k, startCol - radius + q)))
                {
                    dst(startRow - radius + k, startCol - radius + q) = src(startRow, startCol);
                }
            }
        }
    }

    void visEllipse(const std::vector<Ellipse> &obs_ellipses)
    {
        // ellipse
        visualization_msgs::msg::MarkerArray ellipse_vis;

        visualization_msgs::msg::Marker shape_vis;
        shape_vis.header.frame_id = "world";
        shape_vis.header.stamp = this->get_clock()->now();
        shape_vis.ns = "ellipse";
        shape_vis.type = visualization_msgs::msg::Marker::CYLINDER;
        shape_vis.action = visualization_msgs::msg::Marker::ADD;
        shape_vis.lifetime = rclcpp::Duration::from_seconds(0.15);

        // number
        visualization_msgs::msg::Marker text_vis;
        text_vis.header.frame_id = "world";
        text_vis.header.stamp = this->get_clock()->now();
        text_vis.ns = "text";
        text_vis.action = visualization_msgs::msg::Marker::ADD;
        text_vis.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_vis.lifetime = rclcpp::Duration::from_seconds(0.1);
        text_vis.scale.z = 0.3;
        text_vis.color.r = 1;
        text_vis.color.g = 1;
        text_vis.color.b = 1;
        text_vis.color.a = 1;

        for (size_t i = 0; i < obs_ellipses.size(); i++)
        {
            shape_vis.color.a = 0.9;
            if (obs_ellipses[i].variance < 0.0007)
            {
                shape_vis.color.r = 0;
                shape_vis.color.g = 0;
                shape_vis.color.b = 1;
            }
            else if (obs_ellipses[i].variance > 0.005)
            {
                shape_vis.color.r = 1;
                shape_vis.color.g = 0;
                shape_vis.color.b = 0;
            }
            else
            {
                float dt = (obs_ellipses[i].variance - 0.0007) / (0.005 - 0.0007);
                if (dt > 0.5)
                {
                    shape_vis.color.r = 1;
                    shape_vis.color.g = 1 - dt;
                    shape_vis.color.b = 1 - dt;
                }
                else
                {
                    shape_vis.color.r = dt;
                    shape_vis.color.g = dt;
                    shape_vis.color.b = 1;
                }
            }

            shape_vis.id = i;
            shape_vis.pose.orientation.x = 0.0;
            shape_vis.pose.orientation.y = 0.0;
            shape_vis.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
            shape_vis.pose.orientation.w = cos(obs_ellipses[i].theta / 2);
            shape_vis.pose.position.x = obs_ellipses[i].cx;
            shape_vis.pose.position.y = obs_ellipses[i].cy;
            shape_vis.pose.position.z = -0.0;
            shape_vis.scale.x = 2 * obs_ellipses[i].semimajor;
            shape_vis.scale.y = 2 * obs_ellipses[i].semiminor;
            shape_vis.scale.z = 0.75;

            text_vis.id = obs_ellipses[i].label;

            text_vis.text = to_string(obs_ellipses[i].label);
            text_vis.pose.position.x = obs_ellipses[i].cx - 0.3;
            text_vis.pose.position.y = obs_ellipses[i].cy;
            text_vis.pose.position.z = 0.2;

            // publish
            ellipse_vis.markers.push_back(shape_vis);
            ellipse_vis.markers.push_back(text_vis);
        }

        ellipse_vis_pub->publish(ellipse_vis);
    }

    Eigen::MatrixXf map_inflate(const Eigen::MatrixXf &map_data)
    {
        int col = map_data.cols(), row = map_data.rows();
        Eigen::MatrixXf map_inflated(map_data);
        for (int i = 3; i < row - 3; i++)
        {
            for (int j = 3; j < col - 3; j++)
            {
                if (isnan(map_data(i, j)))
                    continue;

                double dis = sqrt((i - col / 2) * (i - col / 2) + (j - col / 2) * (j - col / 2));
                int radius;
                if (dis < col / 3)
                    radius = 1;
                else if (dis < col * 0.45)
                    radius = 2;
                else
                    radius = 3;
                map_inflate_block(map_inflated, map_data, i, j, radius);
            }
        }
        return map_inflated;
    }

    Eigen::MatrixXf map_interpolation(const Eigen::MatrixXf &map_data)
    {
        int col = map_data.cols(), row = map_data.rows();
        Eigen::MatrixXf map_interpolation(map_data);
        for (int i = 1; i < row - 1; i++)
        {
            for (int j = 1; j < col - 1; j++)
            {
                if (isnan(map_data(i, j)))
                {
                    int count = 0;
                    float height = 0;
                    for (int k = 0; k <= 2; k++)
                    {
                        for (int q = 0; q <= 2; q++)
                        {
                            if (!isnan(map_data(i - 1 + k, j - 1 + q)))
                            {
                                count++;
                                height += map_data(i - 1 + k, j - 1 + q);
                            }
                        }
                    }
                    map_interpolation(i, j) = (count > 0) ? height / count : NAN;
                }
            }
        }
        return map_interpolation;
    }

    Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset)
    {
        const float threshold = -1.25;
        int col = map_data.cols(), row = map_data.rows();
        Eigen::MatrixXf gradient_map(row, col);
        gradient_map.setOnes();
        gradient_map *= threshold;
        DBSCAN::Point obs;

        for (int i = 1; i < row - 1; i++)
        {
            for (int j = 1; j < col - 1; j++)
            {
                bool has_nan_value = false;
                for (int p = -1; p <= 1; p++)
                {
                    for (int q = -1; q <= 1; q++)
                    {
                        if (isnan(map_data(i + p, j + q)))
                        {
                            gradient_map(i, j) = threshold;
                            has_nan_value = true;
                        }
                    }
                }
                if (!has_nan_value)
                {
                    float sobel_x = map_data(i + 1, j + 1) - map_data(i - 1, j + 1) + map_data(i + 1, j) - map_data(i - 1, j) +
                                    map_data(i + 1, j - 1) - map_data(i - 1, j - 1);
                    float sobel_y = map_data(i - 1, j + 1) - map_data(i - 1, j - 1) + map_data(i, j + 1) - map_data(i, j - 1) +
                                    map_data(i + 1, j + 1) - map_data(i + 1, j - 1);
                    gradient_map(i, j) = sqrt(sobel_x * sobel_x + sobel_y * sobel_y) + threshold;
                    if (gradient_map(i, j) > obs_height + threshold)
                    {
                        obs.x = i;
                        obs.y = j;
                        dataset.push_back(obs);
                    }
                    else
                    {
                        const int safe_block_size = std::max(1, std::min(
                            block_size, std::min(row, col)));
                        const int start_row = std::max(
                            0, std::min(i - safe_block_size / 2, row - safe_block_size));
                        const int start_col = std::max(
                            0, std::min(j - safe_block_size / 2, col - safe_block_size));
                        const float min = map_data.block(
                            start_row, start_col, safe_block_size, safe_block_size).minCoeff();
                        if (map_data(i, j) - min > step_height)
                        {
                            obs.x = i;
                            obs.y = j;
                            dataset.push_back(obs);
                        }
                    }
                }
            }
        }

        return gradient_map;
    }

    // 成员变量

    pcl::PointCloud<pcl::PointXYZ> velodyne_cloud;
    pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_filter;
    pcl::PointCloud<pcl::PointXYZ> velodyne_cloud_global;

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    LocalMapParam MapParam;
    float localmap_x_size, localmap_y_size, max_heigh, resolution, _inv_resolution;
    float obs_height, step_height, min_pass_z, DBSCAN_R;
    int DBSCAN_N;
    int block_size, block_num;
    int map_index_len;

    Eigen::Vector2d robot_position2d;
    KMAlgorithm KM;

    Eigen::MatrixXf lidar_pcd_matrix;
    Eigen::MatrixXf map_interpolate;
    Eigen::MatrixXf gradient_map;

    geometry_msgs::msg::TransformStamped base_link_transform;
    geometry_msgs::msg::TransformStamped lidar_link_transform;

    // 定时器/pub/sub
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr TF_timer_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_sub;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pcd_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ellipse_vis_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr for_obs_track_pub;

    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pc_pub;

    //
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;

    bool received_lidar_data;

    std::string lidar_topic;
    std::string base_link;
    std::string base_scan;

    std::mutex point_cloud_mutex;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalMapPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
