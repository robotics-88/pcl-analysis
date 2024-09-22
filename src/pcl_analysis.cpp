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
#include <pcl/common/io.h>

#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>

using std::placeholders::_1;

PCLAnalysis::PCLAnalysis()
    : Node("pcl_analysis")
    , pcl_time_(false)
    , count_(0)
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
    this->declare_parameter("pub_rate", pub_rate_);
    this->declare_parameter("point_cloud_topic", point_cloud_topic_);
    this->declare_parameter("segment_distance_threshold", segment_distance_threshold_);
    this->declare_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->declare_parameter("pmf_max_window_size", pmf_max_window_size_);
    this->declare_parameter("pmf_slope", pmf_slope_);
    this->declare_parameter("pmf_initial_distance", pmf_initial_distance_);
    this->declare_parameter("pmf_max_distance", pmf_max_distance_);

    this->get_parameter("pub_rate", pub_rate_);
    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("segment_distance_threshold", segment_distance_threshold_);
    this->get_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->get_parameter("pmf_max_window_size", pmf_max_window_size_);
    this->get_parameter("pmf_slope", pmf_slope_);
    this->get_parameter("pmf_initial_distance", pmf_initial_distance_);
    this->get_parameter("pmf_max_distance", pmf_max_distance_);

    // private_nh_.param<double>("pub_rate", pub_rate_, pub_rate_);
    // private_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, point_cloud_topic_);
    // private_nh_.param<double>("segment_distance_threshold", segment_distance_threshold_, segment_distance_threshold_);
    // private_nh_.param<float>("voxel_grid_leaf_size", voxel_grid_leaf_size_, voxel_grid_leaf_size_);

    // private_nh_.param<int>("pmf_max_window_size", pmf_max_window_size_, pmf_max_window_size_);
    // private_nh_.param<float>("pmf_slope", pmf_slope_, pmf_slope_);
    // private_nh_.param<float>("pmf_initial_distance", pmf_initial_distance_, pmf_initial_distance_);
    // private_nh_.param<float>("pmf_max_distance", pmf_max_distance_, pmf_max_distance_);

    // Set up pubs and subs
    // point_cloud_subscriber_ = nh_.subscribe<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10, &PCLAnalysis::pointCloudCallback, this);
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10, std::bind(&PCLAnalysis::pointCloudCallback, this, _1));

    cloud_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_ground", 10);
    cloud_nonground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_nonground", 10);
    cloud_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_clusters", 10);
    // cloud_ground_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>("cloud_ground", 10);
    // cloud_nonground_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>("cloud_nonground", 10);

    // Set up timer for pointcloud processing and publication
    timer_ = this->create_wall_timer(std::chrono::duration<float>(1.0/pub_rate_), std::bind(&PCLAnalysis::timerCallback, this));
    // timer_ = nh_.createTimer(ros::Duration(1.0/pub_rate_), &PCLAnalysis::timerCallback, this);

}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::timerCallback() {
    if (!pcl_time_) {
        return;
    }
    
    // Downsample cloud for processing
    voxel_grid_filter(cloud_latest_, voxel_grid_leaf_size_);

    // Extract ground returns
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>());
    pmf_ground_extraction(cloud_latest_, cloud_ground, cloud_nonground);

    // Cluster ground returns to find the trail
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>());
    findTrail(cloud_ground, cloud_clustered);

    // Convert to ROS msg and publish
    sensor_msgs::msg::PointCloud2 cloud_ground_msg, cloud_nonground_msg, cloud_clustered_msg;
    pcl::toROSMsg(*cloud_ground, cloud_ground_msg);
    pcl::toROSMsg(*cloud_nonground, cloud_nonground_msg);
    pcl::toROSMsg(*cloud_clustered, cloud_clustered_msg);
    cloud_ground_pub_->publish(cloud_ground_msg);
    cloud_nonground_pub_->publish(cloud_nonground_msg);
    cloud_clustered_msg.header = cloud_ground_msg.header;
    cloud_cluster_pub_->publish(cloud_clustered_msg);
    pcl_time_ = false;
}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (count_ == 10) {
        pcl_time_ = true;
        count_ = 0;
    }
    // TODO reset cloud latest
    // Convert ROS msg to PCL and store
    pcl::PointCloud<pcl::PointXYZ>::Ptr now_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *now_cloud);
    *cloud_latest_ += *now_cloud;
    count_++;
}

void PCLAnalysis::findTrail(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered) {

    // Set up the clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1.0); // Adjust tolerance as necessary
    ec.setMinClusterSize(50); // Minimum number of points to form a cluster
    // ec.setMaxClusterSize(25000); // Maximum number of points to form a cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Create a colored point cloud for visualization
    cloud_clustered->resize(cloud->points.size());
    
    int j = 0;
    for (const auto& indices : cluster_indices) {
        uint32_t color = rand() % 0xFFFFFF; // Random color
        for (const auto& index : indices.indices) {
            cloud_clustered->points[index].x = cloud->points[index].x;
            cloud_clustered->points[index].y = cloud->points[index].y;
            cloud_clustered->points[index].z = cloud->points[index].z;
            cloud_clustered->points[index].r = (color >> 16) & 0xFF;
            cloud_clustered->points[index].g = (color >> 8) & 0xFF;
            cloud_clustered->points[index].b = color & 0xFF;
        }
        j++;
    }
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
        std::cout << "Could not estimate a planar model from pointcloud." << std::endl;
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

void PCLAnalysis::segment_cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinders,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noncylinders) {



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