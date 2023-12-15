/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "pcl_analysis/pcl_analysis.h"

PCLAnalysis::PCLAnalysis(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , point_cloud_topic_("")
{
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PCLAnalysis::pointCloudCallback, this);

}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

}