/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "pcl_analysis/pcl_analysis.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

PCLAnalysis::PCLAnalysis(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , point_cloud_topic_("")
    , segment_distance_threshold_(0.01)
{
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<double>("segment_distance_threshold", segment_distance_threshold_, segment_distance_threshold_);
    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PCLAnalysis::pointCloudCallback, this);

    std::string cloud_plane_topic = point_cloud_topic_ + "_plane";
    cloud_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_plane_topic, 10);

    std::string cloud_nonplane_topic = point_cloud_topic_ + "_nonplane";
    cloud_nonplane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_nonplane_topic, 10);

}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane(new pcl::PointCloud<pcl::PointXYZ>());
    
    segment_plane(cloud, cloud_plane, cloud_nonplane);

    sensor_msgs::PointCloud2 cloud_plane_msg, cloud_nonplane_msg;

    pcl::toROSMsg(*cloud_plane, cloud_plane_msg);
    pcl::toROSMsg(*cloud_nonplane, cloud_nonplane_msg);

    cloud_plane_pub_.publish(cloud_plane_msg);
    cloud_nonplane_pub_.publish(cloud_nonplane_msg);

}

void PCLAnalysis::segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane) {
    

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (segment_distance_threshold_);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        ROS_DEBUG("Could not estimate a planar model from pointcloud.");
        return;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    extract.filter(*cloud_plane);

    extract.setNegative(true);
    extract.filter(*cloud_nonplane);

}