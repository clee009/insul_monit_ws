# Capstone: Insulation Monitoring

## Overview
This is a ROS workspace that monitors the insulation level of horizontal cavities during insulation blowing and publishes commands for adjusting the aim. It  takes in RGBD and lidar sensor point cloud data and filters out the airborne insulation and calculates the centroid, variance, and bounding box of the region where insulation is being blown. It then creates an elevation map of accumulated insulation. It publishes a command for how much the centroid of the insulation region of interest (ROI) should move in the x,y plane.

## Installation
This software requires the installation of the Robotic Operating System (ROS) as well as Grid Map and Eigen, which are Elevation Mapping dependencies.

### Building
Clone respository and compile.
```bash
cd catkin_ws/src
git clone https://github.com/clee009/insul_monit_ws.git
cd ../
catkin build
```

## Usage
You will need to run two separate nodes, one for filtering/insulation ROI/aim adjustment and the other for elevation mapping
```bash
roslaunch insul_monit insul_monit.launch
roslaunch elevation_mapping elevation_mapping.launch
```

## Nodes
### Depth to xyz point cloud: Realsense
Description: This is a PCL library nodelet that converts realsense depth image to xyz point cloud.

Subscribed topics:
    /camera/aligned_depth_to_color/camera_info: camera information (including camera TFs and intrinsics)
        remapped from camera_info
    /camera/aligned_depth_to_color/image_raw: rgbd depth raw image
        remapped from image_rect

Published topics:
    /camera/depth/points: output point cloud
        remapped from points

### Crop box filter: Depth
Description: This is a PCL library nodelet filter that crops the depth point cloud to the cavity region. 

Subscribed topics:
    /camera/depth/points: input raw depth point cloud
        remapped from ~input

Published topics:
    /crop_box/depth/output: output cropped depth point cloud
        remapped from ~output

Parameters:
    min_x, max_x, min_y, max_y, min_z, max_z: I assume that the cavity region of interest will be right in front of the robot.
        value: crop box coordinates based on input frame (base_link)
    input_frame: input point cloud frame.
        value: base_link
    output_frame: output point cloud frame.
        value: base_link

### Crop box filter: Velodyne
Description: Similar to depth crop box filter but using velodyne point cloud.

Subscribed topics:
    /velodyne_points: input velodyne point cloud
        remapped from ~input

Published topics:
    /crop_box/velodyne/output: output cropped velodyne point cloud
        remapped from ~output

### Crop box filter: ROI
Description: Crops the velodyne point cloud right above the cavity region to estimate airborne insulation distribution.

Subscribed topics:
    /velodyne_points: input velodyne point cloud
        remapped from ~input

Published topics:
    /crop_box/roi/output: output cropped ROI velodyne point cloud
        remapped from ~output

Parameters:
    max_x should be within the cavity and not include any non-airborne-insulation points.
    min_x should be at the cavity min x.
    min_z should be just above the cavity height (~5-10 cm offset)
    max_z is set at 20cm above min_z. We don't want this to be too high since that would skew the airborne insulation centroid toward the robot. We also don't want to be too low since we want enough points to estimate the centroid and bounding box of the airborne insulation region.

### Height filter
Description: A custom filter that filters out points corresponding to airborne insulation. For each point, it takes the point's x,y coordinates and finds N nearest neighbors in terms of x,y coordinates. It then finds the minimum z value of that set. If the point's z value is above a tolerance of the minimum z value, then the point is considered an airborne insulation and filtered out.

Subscribed topics:
    /voxel_grid/velodyne/output: input cropped velodyne point cloud
        remapped from /input

Published topics:
    /height_filter/velodyne/output: output height filtered point cloud
        remapped from /output

Parameters:
    N: Number of neighboring parameters based on x,y coordinates to consider
    epsilon: Distance above minimum z height at which to consider the point an airborne insulation.

### Persistent filter
Description: A custom filter that filters out points corresponding to airborne insulation. It stores the past N point clouds and for each voxel, if the voxel is occupied for at least M frames, then the point is not considered an airborne insulation. It applies the filter on the output of the height filter.

Subscribed topics:
    /height_filter/velodyne/output: input height filtered point cloud
        remapped from /input

Published topics:
    /filtered_cloud/velodyne/output: output persistent filtered point cloud
        remapped from /output

Parameters:
    N: Number of previous frames to store in a rolling buffer
    M: Threshold of number of previous frames to be considered not an airborne insulation
    voxel_size: voxel size of the input point cloud

### Insulation ROI
Description: Finds the centroid, bounding box, and variance of the airborne insulation. It uses moving average window to minimize noise. If the input cloud is below a threshold or if the time between valid point clouds is greater than a threshold, it does not incorporate that frame in the moving average. The centroid is manually biased in the x axis away from the robot to correct lidar's tendency to capture more points closer to the robot.

Subscribed topics:
    /voxel_grid/roi/output: input point cloud from the crop box roi node.
        remapped from /input

Published topics:
    /roi: output marker of centroid, bounding box, and ellipse representing variance.
        remapped from /output
    /centroid_info: custom CentroidInfo message containing the x, y, z points of the centroid, x offset from the bounding box max x, density, and whether the point cloud is valid.

Parameters:
    moving_average_window: the moving average window size
    centroid_bias: degree of bias away from robot in x direction. 0.0 for no bias, 1.0 for biasing centroid to bounding box max x.
    cloud_size: cloud size threshold to incorporate the cloud in moving average.
    timeout: max time (sec) that consecutive valid point clouds can have before the node clears the moving average buffer.

### Insulation cost map (velodyne)
Description: Computes the cost map of the velodyne elevation map of insulation in cavity. We want the cost to be such that the robot fills from far end of the channel to the front. Grids closer to the robot in x direction have higher costs. Grids that have higher elevations have higher cost. If a grid has elevation above the filled threshold, it is considered filled and is given max cost.

Subscribed topics:
    /elevation_mapping/elevation_map: input elevation map of insulation levels in cavity
        remapped from /input

Published topics:
    /insulation_cost_map/velodyne: output cost map of insulation levels in cavity
        remapped from /output

Parameters:
    layer: name of the elevation layer in the input elevation map
    min_height: minimum height (m) of elevation map
    max_height: height (m) at which grid is considered filled
    wt_height: weight for the elevation component of the cost
    wt_dist: weight for the distance component of the cost
    max_cost: max cost of the cost map

### Insulation cost map (depth)
Description: Same as velodyne insulation cost map, except the input elevation map is from the depth point cloud.

Subscribed topics:
    /elevation_mapping_depth/elevation_map: input elevation map of insulation levels in cavity
        remapped from /input

Published topics:
    /insulation_cost_map/depth: output cost map of insulation levels in cavity
        remapped from /output

### Insulation target
Description: Publishes a /target topic that provides the robot with relative displacements in x, y coordinates of how much to move the centroid based on where the insulation has been filled already. It first crops the cost map to the region in front of the robot. Then it monitors the cost map and consider a depth (y axis) filled only when a certain threshold of grids at that depth are filled. All depths further beyond a filled depth are considered filled. The node tries to move the boundary box of the airborne insulation to the closest filled depth. If none of the cavity is filled, it tries to aim at the far end of the cavity.

Subscribed topics:
    /insulation_cost_map/velodyne: input velodyne elevation cost map
        remapped from /input
    /centroid_info: input CentroidInfo message containing the current centroid information

Published topics:
    /target: geometry_msgs/point message that gives the relative displacements in x, y coordinates for adjusting the aim of the centroid

Parameters:
    y_min, y_max: region to consider for filling cavity
    fill_threshold: 0.0-1.0. The percentage of grid cells at a given depth beyond which the depth is considered filled.

### Static transform publishers
Description: I have manually included static transform publishers since I removed the tf_static topic from the rosbag.
