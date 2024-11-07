/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"


/**
 * @class PCLAnalysis
 * @brief A class for analyzing and segmenting point clouds
 */
class PCLAnalysis : public rclcpp::Node {

    public:
        PCLAnalysis();
        ~PCLAnalysis();

        void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    private: 
        std::string     point_cloud_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_local_pos_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         planning_pcl_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                percent_above_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr          density_grid_pub_;

        // Main input pointcloud holder
        bool cloud_init_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_regional_{new pcl::PointCloud<pcl::PointXYZ>()};
        double planning_horizon_;

        // Params
        float   voxel_grid_leaf_size_;

        geometry_msgs::msg::PoseStamped current_pose_;

        void makeRegionalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std_msgs::msg::Header header);
        void makeRegionalGrid(const std_msgs::msg::Header header);

        // Runs a voxel grid filter to downsample pointcloud into 3D grid of leaf_size
        void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);

        // Determines percentage of pointcloud points above drone
        float get_percent_above(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

}; // class PCLAnalysis

#endif