/*
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com>
*/

#include "pcl_analysis/pcl_analysis.h"
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasWriter.hpp>

#include <GeographicLib/UTMUPS.hpp>

#include <boost/filesystem.hpp>
#include <cmath>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

PCLAnalysis::PCLAnalysis()
    : Node("pcl_analysis"),
      planning_horizon_(6.0),
      point_cloud_topic_(""),
      voxel_grid_leaf_size_(0.05),
      cloud_init_(false),
      utm_rotation_(0.0),
      save_laz_(false),
      laz_saved_(false),
      data_dir_(""),
      laz_save_filename_("pcl.laz") {
    // Get params
    std::string pointcloud_out_topic;
    this->declare_parameter("point_cloud_topic", point_cloud_topic_);
    this->declare_parameter("point_cloud_aggregated", pointcloud_out_topic);
    this->declare_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->declare_parameter("planning_horizon", planning_horizon_);
    this->declare_parameter("save_laz", save_laz_);
    this->declare_parameter("data_dir", data_dir_);
    this->declare_parameter("utm_rotation", utm_rotation_);

    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("point_cloud_aggregated", pointcloud_out_topic);
    this->get_parameter("voxel_grid_leaf_size", voxel_grid_leaf_size_);
    this->get_parameter("planning_horizon", planning_horizon_);
    this->get_parameter("save_laz", save_laz_);
    this->get_parameter("data_dir", data_dir_);
    this->get_parameter("utm_rotation", utm_rotation_);

    if (save_laz_) {
        cloud_save_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set up pubs and subs
    mavros_global_pos_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", rclcpp::SensorDataQoS(),
        std::bind(&PCLAnalysis::globalPositionCallback, this, _1));
    mavros_local_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", rclcpp::SensorDataQoS(),
        std::bind(&PCLAnalysis::localPositionCallback, this, _1));
    mavros_state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", rclcpp::SensorDataQoS(), std::bind(&PCLAnalysis::stateCallback, this, _1));

    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, 10, std::bind(&PCLAnalysis::pointCloudCallback, this, _1));

    recording_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "~/record", 10, std::bind(&PCLAnalysis::recordingCallback, this, _1));

    planning_pcl_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_out_topic, 10);
    density_grid_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("occ_density_grid", 10);

    percent_above_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/percent_above", 10);
}

PCLAnalysis::~PCLAnalysis() {
    if (save_laz_ && !laz_saved_) {
        saveLaz(cloud_save_);
    }
}

void PCLAnalysis::localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void PCLAnalysis::globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_ll_ = *msg;
}

void PCLAnalysis::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    if (!msg->armed && current_state_.armed && save_laz_) {
        saveLaz(cloud_save_);
    }

    current_state_ = *msg;
}

void PCLAnalysis::recordingCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data != "") {
        std::vector<std::string> parts;
        size_t pos = msg->data.find("_offset_");
        if (pos != std::string::npos) {
            parts.push_back(msg->data.substr(0, pos));
            double elevation_offset = std::stod(parts[0]);
            parts.push_back(msg->data.substr(pos + std::string("_offset_").length()));
        } else {
            RCLCPP_INFO(this->get_logger(), "Recording message did not contain altitude offset, assuming string is data directory.");
            parts.push_back(msg->data);
        }
        data_dir_ = parts[1];
        if (!boost::filesystem::exists(data_dir_)) {
            RCLCPP_INFO(this->get_logger(), "Directory didnt exist. Creating data directory: %s", data_dir_.c_str());
            boost::filesystem::create_directories(data_dir_);
        }
        cloud_save_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        save_laz_ = true;
    }
    else {
        // TODO decide whether we want to keep the reset between each takeoff
        RCLCPP_INFO(this->get_logger(), "Recording toggled off. Resetting pointcloud to save.");
        save_laz_ = false;
        cloud_save_.reset();
    }
}

void PCLAnalysis::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    } else {
        // Convert ROS msg to PCL and store
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        makeRegionalCloud(cloud);

        if (save_laz_) {
            *cloud_save_ += *cloud;
            cloud_save_->header = cloud->header;
        }
    }
    // Publish cloud either way
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_regional_, cloud_msg);
    cloud_msg.header = msg->header;
    planning_pcl_pub_->publish(cloud_msg);
    rclcpp::Time tend = this->get_clock()->now();

    // Publish grid either way
    makeRegionalGrid(msg->header);

    // Publish percentage of points above current location
    float percent = 0.0;
    if (!cloud_regional_->points.empty()) {
        // Determine percentage of points above current location
        percent = get_percent_above(cloud_regional_);
    }

    std_msgs::msg::Float32 percent_above_msg;
    percent_above_msg.data = percent;
    percent_above_pub_->publish(percent_above_msg);
}

void PCLAnalysis::makeRegionalCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (!cloud_init_) {
        cloud_regional_ = cloud;
        cloud_init_ = true;
    } else {
        *cloud_regional_ += *cloud;
    }
    rclcpp::Time tstart = this->get_clock()->now();
    // Cut off to local area

    pcl::PassThrough<pcl::PointXYZI> pass;
    // X
    pass.setInputCloud(cloud_regional_);
    pass.setFilterFieldName("x");
    double lo = current_pose_.pose.position.x - planning_horizon_;
    double hi = current_pose_.pose.position.x + planning_horizon_;
    pass.setFilterLimits(lo, hi);
    pass.filter(*cloud_regional_);
    if (cloud_regional_->points.empty()) {
        return;
    }
    // Y
    pass.setInputCloud(cloud_regional_);
    pass.setFilterFieldName("y");
    lo = current_pose_.pose.position.y - planning_horizon_;
    hi = current_pose_.pose.position.y + planning_horizon_;
    pass.setFilterLimits(lo, hi);
    pass.filter(*cloud_regional_);
    if (cloud_regional_->points.empty()) {
        return;
    }
    rclcpp::Time t2 = this->get_clock()->now();

    // Down sample, filter by voxel
    voxel_grid_filter(cloud_regional_, voxel_grid_leaf_size_);
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
    for (pcl::PointXYZI p : cloud_regional_->points) {
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

void PCLAnalysis::voxel_grid_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud);
}

float PCLAnalysis::get_percent_above(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (cloud->points.size() < 1) {
        RCLCPP_ERROR(this->get_logger(), "Attempting to process invalid PCL");
        return -1.0;
    }

    int num_pts_above = 0;
    for (const auto &point : cloud->points) {
        if (point.z > current_pose_.pose.position.z) {
            num_pts_above++;
        }
    }

    return static_cast<float>(num_pts_above) / cloud->points.size();
}

void PCLAnalysis::saveLaz(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (cloud->size() < 1) {
        RCLCPP_WARN(this->get_logger(), "LAZ save requested but no CLOUD to save");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Saving LAZ to file");

    std::string laz_save_dir = data_dir_ + "/laz/";

    if (!boost::filesystem::exists(laz_save_dir)) {
        boost::filesystem::create_directories(laz_save_dir);
    }

    // Get UTM tf
    geometry_msgs::msg::TransformStamped utm_tf;
    try {
        utm_tf = tf_buffer_->lookupTransform("utm", cloud->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        return;
    }

    if (utm_rotation_ != 0.0) {
        RCLCPP_INFO(this->get_logger(), "Rotating PCL manually by %f degrees", utm_rotation_);
    }

    // Original quaternion as TF
    tf2::Quaternion q_tf;
    tf2::convert(utm_tf.transform.rotation, q_tf);

    // Get rotation quaternion
    tf2::Quaternion q_rot;
    q_rot.setRPY(0, 0, utm_rotation_ * M_PI / 180.0);
    q_rot.normalize();

    tf2::Quaternion q_res = q_rot * q_tf;
    q_res.normalize();

    geometry_msgs::msg::Quaternion q_final;
    tf2::convert(q_res, q_final);
    utm_tf.transform.rotation = q_final;

    pdal::PointTable table;
    table.layout()->registerDim(pdal::Dimension::Id::X);
    table.layout()->registerDim(pdal::Dimension::Id::Y);
    table.layout()->registerDim(pdal::Dimension::Id::Z);
    table.layout()->registerDim(pdal::Dimension::Id::Intensity);

    pdal::PointViewPtr pointView(new pdal::PointView(table));

    // Convert PCL points to PDAL points
    for (const auto &point : cloud->points) {
        // Skip points with NaN or Inf coordinates
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
            continue;
        }

        // Create ROS point for UTM transformation
        geometry_msgs::msg::Point point_ros;
        point_ros.x = point.x;
        point_ros.y = point.y;
        point_ros.z = point.z + elevation_offset_;
        
        tf2::doTransform(point_ros, point_ros, utm_tf);

        // Save point
        pdal::PointId id = pointView->size();
        pointView->setField(pdal::Dimension::Id::X, id, point_ros.x);
        pointView->setField(pdal::Dimension::Id::Y, id, point_ros.y);
        pointView->setField(pdal::Dimension::Id::Z, id, point_ros.z);
        pointView->setField(pdal::Dimension::Id::Intensity, id, point.intensity);
        pointView->appendPoint(*pointView, id);
    }

    // Write to LAS file
    laz_save_dir += laz_save_filename_;

    int utm_zone = GeographicLib::UTMUPS::StandardZone(current_ll_.latitude, current_ll_.longitude);

    std::string code_pref = current_ll_.latitude > 0 ? "EPSG:326" : "EPSG:327";
    std::string utm_zone_str = code_pref + std::to_string(utm_zone);

    pdal::BufferReader reader;
    reader.addView(pointView);

    pdal::Options opts;
    opts.add("filename", laz_save_dir);
    opts.add("a_srs", utm_zone_str.c_str());
    opts.add("compression", "true");
    pdal::LasWriter writer;
    writer.setInput(reader);
    writer.setOptions(opts);
    writer.prepare(table);
    writer.execute(table);

    laz_saved_ = true;
    RCLCPP_INFO(this->get_logger(), "LAZ saved to %s", laz_save_dir.c_str());
}
