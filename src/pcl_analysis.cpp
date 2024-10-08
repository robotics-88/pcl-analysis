/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "pcl_analysis/pcl_analysis.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_stick.h>
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
    , do_trail_(false)
    , trail_goal_enabled_(false)
    , pub_rate_(2.0)
    , point_cloud_topic_("")
    , segment_distance_threshold_(0.01)
    , voxel_grid_leaf_size_(0.1)
    , pmf_max_window_size_(10)
    , pmf_slope_(1.0)
    , pmf_initial_distance_(0.5)
    , pmf_max_distance_(3.0)
    , last_pub_time_(0, 0, RCL_ROS_TIME)
{
    // Get params
    this->declare_parameter("do_trail", do_trail_);
    this->declare_parameter("pub_rate", pub_rate_);
    this->declare_parameter("point_cloud_topic", point_cloud_topic_);
    this->declare_parameter("segment_distance_threshold", segment_distance_threshold_);
    this->declare_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->declare_parameter("pmf_max_window_size", pmf_max_window_size_);
    this->declare_parameter("pmf_slope", pmf_slope_);
    this->declare_parameter("pmf_initial_distance", pmf_initial_distance_);
    this->declare_parameter("pmf_max_distance", pmf_max_distance_);

    this->get_parameter("do_trail", do_trail_);
    this->get_parameter("pub_rate", pub_rate_);
    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("segment_distance_threshold", segment_distance_threshold_);
    this->get_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->get_parameter("pmf_max_window_size", pmf_max_window_size_);
    this->get_parameter("pmf_slope", pmf_slope_);
    this->get_parameter("pmf_initial_distance", pmf_initial_distance_);
    this->get_parameter("pmf_max_distance", pmf_max_distance_);

    // Set up pubs and subs
    mavros_local_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", rclcpp::SensorDataQoS(), std::bind(&PCLAnalysis::localPositionCallback, this, _1));

    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10, std::bind(&PCLAnalysis::pointCloudCallback, this, _1));

    cloud_ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_ground", 10);
    cloud_nonground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_nonground", 10);
    cloud_cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_clusters", 10);

    percent_above_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/percent_above", 10);

    trail_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/trail_line", 10);
    trail_marker_.header.frame_id = "map";
    trail_marker_.scale.x = 3.0;
    trail_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trail_marker_.action = visualization_msgs::msg::Marker::ADD;
    trail_marker_.id = 0;
    trail_goal_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/explorable_goal", 10);

    trail_enabled_service_ = this->create_service<rcl_interfaces::srv::SetParametersAtomically>("trail_enabled_service", std::bind(&PCLAnalysis::setTrailsEnabled, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // Only run processing at limited rate
    rclcpp::Duration dur = this->get_clock()->now() - last_pub_time_;
    if (dur.seconds() < (1.0 / pub_rate_)) {
        return;
    }

    // Convert ROS msg to PCL and store
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    
    // Downsample cloud for processing
    voxel_grid_filter(cloud, voxel_grid_leaf_size_);

    // Determine percentage of points above current location
    float percent = get_percent_above(cloud);
    std_msgs::msg::Float32 percent_above_msg;
    percent_above_msg.data = percent;
    percent_above_pub_->publish(percent_above_msg);

    if (do_trail_) {
        // Extract ground returns
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZ>());
        pmf_ground_extraction(cloud, cloud_ground, cloud_nonground);

        // Cluster ground returns to find the trail
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZ>());
        findTrail(cloud_ground, cloud_clustered);

        // Convert to ROS msg and publish
        sensor_msgs::msg::PointCloud2 cloud_ground_msg, cloud_nonground_msg, cloud_clustered_msg;
        pcl::toROSMsg(*cloud_ground, cloud_ground_msg);
        pcl::toROSMsg(*cloud_nonground, cloud_nonground_msg);
        pcl::toROSMsg(*cloud_clustered, cloud_clustered_msg);
        cloud_ground_pub_->publish(cloud_ground_msg);
        cloud_nonground_pub_->publish(cloud_nonground_msg);
        cloud_clustered_msg.header = cloud_ground_msg.header;
        cloud_clustered_msg.header.frame_id = "map"; // TODO why is it sometimes missing the frame?
        cloud_cluster_pub_->publish(cloud_clustered_msg);

        trail_line_pub_->publish(trail_marker_);
    }

    pcl_time_ = false;

    last_pub_time_ = this->get_clock()->now();
}

void PCLAnalysis::findTrail(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered) {
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
    if (cluster_indices.size() == 0) return;
    // TEST: Get largest cluster
    int max_index = 0, max_size = 0;
    for (int ii = 0; ii < cluster_indices.size(); ii++) {
        if (cluster_indices.at(ii).indices.size() > max_size) {
            max_index = ii;
            max_size = cluster_indices.at(ii).indices.size();
        }
    }
    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
    *cluster = cluster_indices.at(max_index);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(cluster);
    extract.filter(*cloud_clustered);

    // TEST: Get cluster best fit to shape
    // cloud_clustered = findMaximumPlanar(cloud, cluster_indices);

    // // Create a colored point cloud for visualization
    // cloud_clustered->resize(cloud->points.size());
    
    // Original, just return all clusters
    // int j = 0;
    // for (const auto& indices : cluster_indices) {
    //     uint32_t color = rand() % 0xFFFFFF; // Random color
    //     for (const auto& index : indices.indices) {
    //         cloud_clustered->points[index].x = cloud->points[index].x;
    //         cloud_clustered->points[index].y = cloud->points[index].y;
    //         cloud_clustered->points[index].z = cloud->points[index].z;
    //         cloud_clustered->points[index].r = (color >> 16) & 0xFF;
    //         cloud_clustered->points[index].g = (color >> 8) & 0xFF;
    //         cloud_clustered->points[index].b = color & 0xFF;
    //     }
    //     j++;
    // }

    // Extract line segment and append to trail marker list
    extractLineSegment(cloud_clustered);
}

void PCLAnalysis::extractLineSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered) {
    // Perform RANSAC plane fitting
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1); // Set distance threshold for inliers
    seg.setInputCloud(cloud_clustered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        RCLCPP_INFO(this->get_logger(), "Could not estimate a linear model for the given dataset.");
    }

    // Extract the line segment endpoints from the coefficients
    geometry_msgs::msg::Point point;
    point.x = coefficients->values[0];
    point.y = coefficients->values[1];
    point.z = coefficients->values[2];
    trail_marker_.points.push_back(point);

    // Second set of coefficients is a direction vector
    point.x += coefficients->values[3];
    point.y += coefficients->values[4];
    point.z += coefficients->values[5];
    trail_marker_.points.push_back(point);

    if (trail_goal_enabled_) {
        trail_goal_pub_->publish(point);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLAnalysis::findMaximumPlanar(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered,
                                    const std::vector<pcl::PointIndices> cluster_indices) {
    // Iterate through each cluster
    int index = 0, count = 0;
    float max_ratio = 0.0;
    for (int ii = 0; ii < cluster_indices.size(); ii++) {
    // for (pcl::PointIndices cluster : cluster_indices) {
        if (cluster_indices.at(ii).indices.size() < 1000) {
            continue;
        }
        pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
        *cluster = cluster_indices.at(ii);
        pcl::PointCloud<pcl::PointXYZ> cluster_points;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_clustered);
        extract.setIndices(cluster);
        extract.filter(cluster_points);

        // Perform RANSAC plane fitting
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1); // Set distance threshold for inliers
        seg.setInputCloud(cluster_points.makeShared());
        seg.segment(*inliers, *coefficients);

        // Calculate planarity metric (e.g., inlier ratio)
        float inlier_ratio = static_cast<float>(inliers->indices.size()) / cluster_points.size();

        // Store the planarity metric for comparison
        if (inlier_ratio > max_ratio) {
            index = count;
        }
        count++;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
    *cluster = cluster_indices.at(index);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_clustered);
    extract.setIndices(cluster);
    extract.filter(*cluster_points);
    
    return cluster_points;
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
        RCLCPP_INFO(this->get_logger(),"Could not estimate a planar model from pointcloud.");
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

float PCLAnalysis::get_percent_above(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if (cloud->points.size() < 1) {
        RCLCPP_ERROR(this->get_logger(), "Attempting to process invalid PCL");
        return -1.0;
    }

    int num_pts_above = 0;
    for (const auto & point : cloud->points) {
        if (point.z > current_pose_.pose.position.z) {
            num_pts_above++;
        }
    }

    return static_cast<float>(num_pts_above) / cloud->points.size();
}


bool PCLAnalysis::setTrailsEnabled(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                        const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> req,
                        const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> resp) {
  for (int ii = 0; ii < req->parameters.size(); ii++) {
    if (req->parameters.at(ii).name == "trails_enabled") {
      trail_goal_enabled_ = req->parameters.at(ii).value.bool_value;
    }
  }
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  resp->result = result;
}
