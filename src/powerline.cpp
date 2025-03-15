/* 
© 2025 Robotics 88
Author: Erin Linebarger <erin@robotics88.com> 
*/

#include "pcl_analysis/powerline.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/passthrough.h>

#include <cmath>
#include <thread>

using std::placeholders::_1;

PowerlineDetector::PowerlineDetector()
    : Node("powerline_detector")
    , point_cloud_topic_("")
{
    // Get params
    std::string pointcloud_out_topic;
    this->declare_parameter("point_cloud_topic", point_cloud_topic_);

    this->get_parameter("point_cloud_topic", point_cloud_topic_);

    // Set up pubs and subs
    point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, 10, std::bind(&PowerlineDetector::pointCloudCallback, this, _1));
    powerline_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("power_line_cloud", 10);

}

PowerlineDetector::~PowerlineDetector(){}

void PowerlineDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert ROS msg to PCL and store
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    std_msgs::msg::Header header = msg->header;

    // Only run processing at limited rate?
    detectPowerLines(cloud);

}

void PowerlineDetector::detectPowerLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    // Step 1: Cluster extraction to isolate power lines
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Step 2: Identify linear structures using RANSAC and store them
    pcl::PointCloud<pcl::PointXYZ>::Ptr power_line_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int index : indices.indices)
            cluster->push_back(cloud->points[index]);

        pcl::SACSegmentation<pcl::PointXYZ> line_seg;
        pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
        line_seg.setOptimizeCoefficients(true);
        line_seg.setModelType(pcl::SACMODEL_LINE);
        line_seg.setMethodType(pcl::SAC_RANSAC);
        line_seg.setDistanceThreshold(0.1);
        line_seg.setInputCloud(cluster);
        line_seg.segment(*line_inliers, *line_coefficients);

        if (!line_inliers->indices.empty())
        {
            // Extract direction vector (a, b, c)
            float a = line_coefficients->values[3];
            float b = line_coefficients->values[4];
            float c = line_coefficients->values[5];

            // Check if the line is horizontal (i.e., c is close to zero)
            float threshold = 0.3;
            if (std::abs(c) < threshold)
            {
                for (int index : line_inliers->indices)
                    power_line_cloud->push_back(cluster->points[index]);
            }
        }
    }

    // Publish the detected power line cloud
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*power_line_cloud, output);
    output.header.frame_id = "map";  // Adjust to match your frame
    output.header.stamp = this->now();
    powerline_pub_->publish(output);

}
