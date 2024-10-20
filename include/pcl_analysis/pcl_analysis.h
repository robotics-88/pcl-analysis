/* 
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com> 
*/

#ifndef PCL_ANALYSIS_H_
#define PCL_ANALYSIS_H_

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "visualization_msgs/msg/marker.hpp"

/**
 * @class PCLAnalysis
 * @brief A class for analyzing and segmenting point clouds
 */
class PCLAnalysis : public rclcpp::Node {

    public:
        PCLAnalysis();
        ~PCLAnalysis();

        void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Helper classes for grid
        class PointXY
        {
        public:
            int x;
            int y;
        };

        class PointXYZI
        {
        public:
            double x;
            double y;
            double z;
            double intensity;
        };

        class GridMap
        {
        public:
            float position_x;
            float position_y;
            float cell_size;
            float length_x;
            float length_y;
            std::string cloud_in_topic;
            std::string frame_out;
            std::string mapi_topic_name;
            std::string maph_topic_name;
            float topleft_x;
            float topleft_y;
            float bottomright_x;
            float bottomright_y;
            int cell_num_x;
            int cell_num_y;
            float intensity_factor;
            float height_factor;

            void initGrid(std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid, float center_x, float center_y)
            {
                // grid->header.seq = 1;
                grid->header.frame_id = "map"; // TODO
                grid->info.origin.position.z = 0;
                grid->info.origin.orientation.w = 0;
                grid->info.origin.orientation.x = 0;
                grid->info.origin.orientation.y = 0;
                grid->info.origin.orientation.z = 1;
                grid->info.origin.position.x = center_x + length_x / 2;
                grid->info.origin.position.y = center_y + length_y / 2;
                grid->info.width = length_x / cell_size;
                grid->info.height = length_y / cell_size;
                grid->info.resolution = cell_size;
                position_x = center_x;
                position_y = center_y;
                // resolution/grid size [m/cell]
            }

            void paramRefresh()
            {
                topleft_x = position_x + length_x / 2;
                bottomright_x = position_x - length_x / 2;
                topleft_y = position_y + length_y / 2;
                bottomright_y = position_y - length_y / 2;
                cell_num_x = int(length_x / cell_size);
                cell_num_y = int(length_y / cell_size);
            }

            // x and y are in meters, it returs the cell index
            PointXY getIndex(double x, double y)
            {
                PointXY ret;
                ret.x = int(fabs(x - topleft_x) / cell_size);
                ret.y = int(fabs(y - topleft_y) / cell_size);
                return ret;
            }
        };
        // End GridMap

    private: 
        bool pcl_time_;
        int count_;
        std::string     point_cloud_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    mavros_local_pos_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      point_cloud_subscriber_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_ground_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_nonground_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         cloud_cluster_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         planning_pcl_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr                percent_above_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr       trail_line_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       trail_goal_pub_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr          density_grid_pub_;

        rclcpp::Time last_pub_time_;

        // Main input pointcloud holder
        bool cloud_init_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_regional_{new pcl::PointCloud<pcl::PointXYZ>()};
        GridMap grid_map_;
        double planning_horizon_;

        // Trail line
        visualization_msgs::msg::Marker trail_marker_;
        bool do_trail_;
        bool trail_goal_enabled_;
        rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr trail_enabled_service_;

        // Params
        double  pub_rate_;
        double  segment_distance_threshold_;
        float   voxel_grid_leaf_size_;
        int     pmf_max_window_size_;
        float   pmf_slope_;
        float   pmf_initial_distance_;
        float   pmf_max_distance_;

        geometry_msgs::msg::PoseStamped current_pose_;

        void initGridParams();
        void makeRegionalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void makeRegionalGrid();

        void findTrail(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered);
        pcl::PointCloud<pcl::PointXYZ>::Ptr findMaximumPlanar(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered,
                                                         const std::vector<pcl::PointIndices> cluster_indices);
        void extractLineSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered);

        // Segments out a plane from a pointcloud including points within segment_distance_threshold_ of plane model
        void segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonplane);

        void segment_cylinders(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinders,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noncylinders);

        // Runs a PMF filter to extract ground points based on pmf_* params
        void pmf_ground_extraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonground);

        // Runs a voxel grid filter to downsample pointcloud into 3D grid of leaf_size
        void voxel_grid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);

        // Determines percentage of pointcloud points above drone
        float get_percent_above(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        bool setTrailsEnabled(const std::shared_ptr<rmw_request_id_t>/*request_header*/,
                        const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> req,
                        const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> resp);

}; // class PCLAnalysis

#endif