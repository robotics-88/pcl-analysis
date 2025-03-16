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
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <opencv2/opencv.hpp>

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
        cv::Mat distance_matrix_;
        cv::Size image_size_;
        double meters_per_pixel_;
        double origin_x_, origin_y_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         powerline_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         distance_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               image_pub_;

        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void detectPowerLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void updateImage(const pcl::PointXYZ &point, float distance);
        void saveGeoTIFF();

}; // class PowerlineDetector

#endif