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
 * @brief A class for analyzing and segmenting point clouds
 */
class PCLAnalysis {

    public:
        PCLAnalysis(ros::NodeHandle& node);
        ~PCLAnalysis();

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

        void timerCallback(const ros::TimerEvent&);

    private: 
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        std::string     point_cloud_topic_;
        ros::Subscriber point_cloud_subscriber_;

        ros::Publisher cloud_ground_pub_;
        ros::Publisher cloud_nonground_pub_;

        ros::Timer timer_;

        // Main input pointcloud holder
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_latest_{new pcl::PointCloud<pcl::PointXYZ>()};

        // Params
        double  pub_rate_;
        double  segment_distance_threshold_;
        float   voxel_grid_leaf_size_;
        int     pmf_max_window_size_;
        float   pmf_slope_;
        float   pmf_initial_distance_;
        float   pmf_max_distance_;

        // Segments out a plane from a pointcloud including points within segment_distance_threshold_ of plane model
        void segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane);

        // Runs a PMF filter to extract ground points based on pmf_* params
        void pmf_ground_extraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground);

        // Runs a voxel grid filter to downsample pointcloud into 3D grid of leaf_size
        void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);

}; // class PCLAnalysis

#endif