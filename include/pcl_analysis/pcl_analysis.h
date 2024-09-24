/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "visualization_msgs/msg/marker.hpp"

/**
 * @class PCLAnalysis
 * @brief A class for analyzing and segmenting point clouds
 */
class PCLAnalysis : public rclcpp::Node {

    public:
        PCLAnalysis();
        ~PCLAnalysis();

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void timerCallback();

    private: 
        bool pcl_time_;
        int count_;
        std::string     point_cloud_topic_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_ground_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_nonground_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_cluster_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       trail_line_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        // Main input pointcloud holder
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_latest_{new pcl::PointCloud<pcl::PointXYZ>()};

        // Trail line
        visualization_msgs::msg::Marker trail_marker_;

        // Params
        double  pub_rate_;
        double  segment_distance_threshold_;
        float   voxel_grid_leaf_size_;
        int     pmf_max_window_size_;
        float   pmf_slope_;
        float   pmf_initial_distance_;
        float   pmf_max_distance_;

        void findTrail(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr findMaximumPlanar(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered,
                                                         const std::vector<pcl::PointIndices> cluster_indices);
        void extractLineSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered);

        // Segments out a plane from a pointcloud including points within segment_distance_threshold_ of plane model
        void segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane);

        void segment_cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinders,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noncylinders);

        // Runs a PMF filter to extract ground points based on pmf_* params
        void pmf_ground_extraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground);

        // Runs a voxel grid filter to downsample pointcloud into 3D grid of leaf_size
        void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);

}; // class PCLAnalysis

#endif