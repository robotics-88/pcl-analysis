/* 
© 2025 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"


/**
 * @class PowerlineDetector
 * @brief A class for detecting powerlines in point clouds
 */
class PowerlineDetector : public rclcpp::Node {

    public:
        PowerlineDetector();
        ~PowerlineDetector();

    private: 
        std::string     point_cloud_topic_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         powerline_pub_;

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void detectPowerLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

}; // class PowerlineDetector

#endif