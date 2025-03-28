/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


/**
 * @class PCLAnalysis
 * @brief A class for analyzing and segmenting point clouds
 */
class PCLAnalysis : public rclcpp::Node {

    public:
        PCLAnalysis();
        ~PCLAnalysis();

        void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    private: 
        std::string                                                         point_cloud_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_local_pos_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        mavros_global_pos_subscriber_;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr            mavros_state_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         planning_pcl_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                percent_above_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr          density_grid_pub_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Main input pointcloud holder
        bool cloud_init_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_regional_{new pcl::PointCloud<pcl::PointXYZI>()};
        double planning_horizon_;

        // Params
        float       voxel_grid_leaf_size_;
        bool        save_pcl_;
        bool        pcl_saved_;
        std::string data_dir_;

        double utm_rotation_;
        geometry_msgs::msg::PoseStamped current_pose_;
        sensor_msgs::msg::NavSatFix     current_ll_;
        mavros_msgs::msg::State         current_state_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_save_;
        std::string pcl_save_filename_;

        void makeRegionalCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void makeRegionalGrid(const std_msgs::msg::Header header);

        // Runs a voxel grid filter to downsample pointcloud into 3D grid of leaf_size
        void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size);

        // Determines percentage of pointcloud points above drone
        float get_percent_above(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        // Save pointcloud to file
        void savePcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

}; // class PCLAnalysis

#endif