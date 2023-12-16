/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

/**
 * @class PCLAnalysis
 * @brief A class for analyzing point clouds
 */
class PCLAnalysis {

    public:
        PCLAnalysis(ros::NodeHandle& node);
        ~PCLAnalysis();

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    private: 
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        std::string point_cloud_topic_;
        ros::Subscriber point_cloud_subscriber_;

        ros::Publisher cloud_plane_pub_;
        ros::Publisher cloud_nonplane_pub_;

        double segment_distance_threshold_;

        void segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane);

}; // class PCLAnalysis

#endif