/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#include "pcl_analysis/pcl_analysis.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

#include <pcl/filters/passthrough.h>

#include <cmath>
#include <thread>

using std::placeholders::_1;

PCLAnalysis::PCLAnalysis()
    : Node("pcl_analysis")
    , planning_horizon_(6.0)
    , point_cloud_topic_("")
    , voxel_grid_leaf_size_(0.05)
    , cloud_init_(false)
{
    // Get params
    std::string pointcloud_out_topic;
    this->declare_parameter("point_cloud_topic", point_cloud_topic_);
    this->declare_parameter("point_cloud_aggregated", pointcloud_out_topic);
    this->declare_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->declare_parameter("planning_horizon", planning_horizon_);

    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("point_cloud_aggregated", pointcloud_out_topic);
    this->get_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->get_parameter("planning_horizon", planning_horizon_);

    // Set up pubs and subs
    mavros_local_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", rclcpp::SensorDataQoS(), std::bind(&PCLAnalysis::localPositionCallback, this, _1));

    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10, std::bind(&PCLAnalysis::pointCloudCallback, this, _1));

    planning_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_out_topic, 10);
    density_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occ_density_grid", 10);

    percent_above_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/percent_above", 10);
}

PCLAnalysis::~PCLAnalysis(){}

void PCLAnalysis::localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS msg to PCL and store
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    makeRegionalCloud(cloud, msg->header);

    // Determine percentage of points above current location
    float percent = get_percent_above(cloud_regional_);
    std_msgs::msg::Float32 percent_above_msg;
    percent_above_msg.data = percent;
    percent_above_pub_->publish(percent_above_msg);

}

void PCLAnalysis::makeRegionalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std_msgs::msg::Header header) {
    if (!cloud_init_) {
        cloud_regional_ = cloud;
        cloud_init_ = true;
    }
    else {
        *cloud_regional_ += *cloud;
    }
    rclcpp::Time tstart = this->get_clock()->now();
    // Cut off to local area

	pcl::PassThrough<pcl::PointXYZ> pass;
	// X
	pass.setInputCloud (cloud_regional_);
	pass.setFilterFieldName ("x");
	double lo = current_pose_.pose.position.x - planning_horizon_;
	double hi = current_pose_.pose.position.x + planning_horizon_;
	pass.setFilterLimits (lo, hi);
	pass.filter (*cloud_regional_);
	// Y
	pass.setInputCloud (cloud_regional_);
	pass.setFilterFieldName ("y");
	lo = current_pose_.pose.position.y - planning_horizon_;
	hi = current_pose_.pose.position.y + planning_horizon_;
	pass.setFilterLimits (lo, hi);
	pass.filter (*cloud_regional_);
    rclcpp::Time t2 = this->get_clock()->now();

    // Down sample, filter by voxel
    voxel_grid_filter(cloud_regional_, voxel_grid_leaf_size_);

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_regional_, cloud_msg);
    cloud_msg.header = header;
    planning_pcl_pub_->publish(cloud_msg);
    rclcpp::Time tend = this->get_clock()->now();

    makeRegionalGrid(header);
}

void PCLAnalysis::makeRegionalGrid(const std_msgs::msg::Header header) {
    auto density_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    double resolution = 0.5;
    double sz = (planning_horizon_ / resolution) * 2;
    double origin_x = current_pose_.pose.position.x - (planning_horizon_);
    double origin_y = current_pose_.pose.position.y - (planning_horizon_);
     // Initialize occupancy grid message
    density_grid->header = header;
    density_grid->info.resolution = resolution;
    density_grid->info.width = sz;
    density_grid->info.height = sz;
    density_grid->info.origin.position.x = origin_x;
    density_grid->info.origin.position.y = origin_y;
    density_grid->info.origin.position.z = 0.0;

    density_grid->data.resize(sz * sz, 0);
    for (pcl::PointXYZ p : cloud_regional_->points) {
        // Calculate grid cell indices
        int grid_x = static_cast<int>((p.x - origin_x) / resolution);
        int grid_y = static_cast<int>((p.y - origin_y) / resolution);

        // Increment count if the point falls within the grid bounds
        if (grid_x >= 0 && grid_x < sz && grid_y >= 0 && grid_y < sz) {
            density_grid->data[grid_y * sz + grid_x]++;
        }
    }

    density_grid_pub_->publish(*density_grid);
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

