# pcl-analysis
ROS node for doing point cloud analysis and segmentation. See [PCL library documentation](https://pcl.readthedocs.io/projects/tutorials/en/pcl-1.11.0/) for further information on the PCL methods used in this package. 

## Description
Subscribes to a point cloud topic, does processing on that point cloud, and publishes processed point cloud(s). Currently used for segmenting out ground/non-ground clouds from input pointcloud using PMF filtering. 

## Subscriptions
`[point_cloud_topic]`<sensor_msgs::PointCloud2>: Input point cloud ROS message

## Publications
`/cloud_ground`<sensor_msgs::PointCloud2>: Point cloud of extracted ground points from the PMF filter.
`/cloud_nonground`<sensor_msgs::PointCloud2>: Inverse of cloud_ground - only includes non-ground points (trees, objects, etc).

## Parameters
`pub_rate`: Loop rate for processing and publication, in hz. Higher rate increases CPU usage proportionally up to maximum rate (which is same rate that point_cloud_topic gets published).

`point_cloud_topic`: Rostopic name for input pointcloud.

`voxel_grid_leaf_size`: Leaf size for voxel grid filter. In other words, distance between points in downsampling. Increase for faster processing, decrease for more precise output pointclouds. 

`pmf_max_window_size`: Maximum window size on the PMF filter. Increase for more accurate/robust filtering, decrease for faster processing. 

`pmf_slope`: Slope of PMF used for computing height threshold for filtered height.

# color-projection
ROS node for projecting colors from an image to its corresponding point cloud.

## Description
Subscribes to both the pointcloud2 and the image topics and takes in the intrinsic and extrinsic parameters from the param.yml files. Using these matrices, the transformation matrix is calculated between the image and the pointcloud. The FOV of the pointcloud is set based on the FOV of the image. Using the computed transformation matrix, the colors are projected from the image to its corresponding point in the pointcloud.

To run this color projection node:

`roslaunch pcl_analysis sync_fusion_modified.launch`

Reference repo for the color projection node is: `https://github.com/GCaptainNemo/Fusion-Lidar-Camera-ROS/tree/main`
