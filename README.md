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

`point_cloud_topic`: Rostopic name for input pointcloud.

`voxel_grid_leaf_size`: Leaf size for voxel grid filter. In other words, distance between points in downsampling. Increase for faster processing, decrease for more precise output pointclouds. 

`pmf_max_window_size`: Maximum window size on the PMF filter. Increase for more accurate/robust filtering, decrease for faster processing. 

`pmf_slope`: Slope of PMF used for computing height threshold for filtered height.
