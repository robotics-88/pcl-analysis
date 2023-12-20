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
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/progressive_morphological_filter.h>

PCLAnalysis::PCLAnalysis(ros::NodeHandle& node)
    : private_nh_("~")
    , nh_(node)
    , pub_rate_(2.0)
    , point_cloud_topic_("")
    , segment_distance_threshold_(0.01)
    , voxel_grid_leaf_size_(0.1)
    , pmf_max_window_size_(10)
    , pmf_slope_(1.0)
    , pmf_initial_distance_(0.5)
    , pmf_max_distance_(3.0)
{
    // Get params
    private_nh_.param<double>("pub_rate", pub_rate_, pub_rate_);
    private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    private_nh_.param<double>("segment_distance_threshold", segment_distance_threshold_, segment_distance_threshold_);
    private_nh_.param<float>("voxel_grid_leaf_size", voxel_grid_leaf_size_, voxel_grid_leaf_size_);

    private_nh_.param<int>("pmf_max_window_size", pmf_max_window_size_, pmf_max_window_size_);
    private_nh_.param<float>("pmf_slope", pmf_slope_, pmf_slope_);
    private_nh_.param<float>("pmf_initial_distance", pmf_initial_distance_, pmf_initial_distance_);
    private_nh_.param<float>("pmf_max_distance", pmf_max_distance_, pmf_max_distance_);

    // Set up pubs and subs
    point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic_, 10, &PCLAnalysis::pointCloudCallback, this);

    cloud_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_ground", 10);
    cloud_nonground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_nonground", 10);

    // Set up timer for pointcloud processing and publication
    timer_ = nh_.createTimer(ros::Duration(1.0/pub_rate_), &PCLAnalysis::timerCallback, this);

}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::timerCallback(const ros::TimerEvent&) {
    
    // Downsample cloud for processing
    voxel_grid_filter(cloud_latest_, voxel_grid_leaf_size_);

    // Extract ground returns
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>());
    pmf_ground_extraction(cloud_latest_, cloud_ground, cloud_nonground);

    // Convert to ROS msg and publish
    sensor_msgs::PointCloud2 cloud_ground_msg, cloud_nonground_msg;
    pcl::toROSMsg(*cloud_ground, cloud_ground_msg);
    pcl::toROSMsg(*cloud_nonground, cloud_nonground_msg);
    cloud_ground_pub_.publish(cloud_ground_msg);
    cloud_nonground_pub_.publish(cloud_nonground_msg);
}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Convert ROS msg to PCL and store
    pcl::fromROSMsg(*msg, *cloud_latest_);
}

void PCLAnalysis::segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane) {
    

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Set up segmentation
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

    // Extract segment from cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*cloud_plane);

    // Extract opposite segment
    extract.setNegative(true);
    extract.filter(*cloud_nonplane);

}

void PCLAnalysis::pmf_ground_extraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground) {

    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cloud);
    pmf.setMaxWindowSize (pmf_max_window_size_);
    pmf.setSlope (pmf_slope_);
    pmf.setInitialDistance (pmf_initial_distance_);
    pmf.setMaxDistance (pmf_max_distance_);

    pmf.extract (ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    extract.filter (*cloud_ground);

    // Extract non-ground returns
    extract.setNegative (true);
    extract.filter (*cloud_nonground);

}

void PCLAnalysis::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*cloud);
}