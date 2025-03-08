# Capstone: Insulation Monitoring

## Overview
This is a ROS workspace that monitors the insulation level of horizontal cavities during insulation blowing and publishes commands for adjusting the aim. It takes in RGBD and LiDAR sensor point cloud data, filters out airborne insulation, and calculates the centroid, variance, and bounding box of the region where insulation is being blown. It then creates an elevation map of accumulated insulation and publishes a command for how much the centroid of the insulation region of interest (ROI) should move in the x, y plane.

## Installation
This software requires the installation of the Robot Operating System (ROS) as well as Grid Map and Eigen, which are dependencies for Elevation Mapping.

### Building
Clone the repository and compile:

```bash
cd catkin_ws/src
git clone https://github.com/clee009/insul_monit_ws.git
cd ../
catkin build
```

## Usage
You will need to run two separate nodes: one for filtering/insulation ROI/aim adjustment and another for elevation mapping.

```bash
roslaunch insul_monit insul_monit.launch
roslaunch elevation_mapping elevation_mapping.launch
```

## Nodes

### Depth to XYZ Point Cloud: Realsense
**Description:** Converts the RealSense depth image to an XYZ point cloud using a PCL library nodelet.

**Subscribed Topics:**
- `/camera/aligned_depth_to_color/camera_info`: Camera information, including TFs and intrinsics  
  *(remapped from `camera_info`)*
- `/camera/aligned_depth_to_color/image_raw`: RGBD depth raw image  
  *(remapped from `image_rect`)*

**Published Topics:**
- `/camera/depth/points`: Output point cloud  
  *(remapped from `points`)*

---

### Crop Box Filter: Depth
**Description:** Crops the depth point cloud to the cavity region using a PCL filter.

**Subscribed Topics:**
- `/camera/depth/points`: Input raw depth point cloud  
  *(remapped from `~input`)*

**Published Topics:**
- `/crop_box/depth/output`: Output cropped depth point cloud  
  *(remapped from `~output`)*

**Parameters:**
- `min_x, max_x, min_y, max_y, min_z, max_z`: Defines the cavity region of interest.  
  - **Value:** Crop box coordinates in the input frame (`base_link`).
- `input_frame`: Input point cloud frame.  
  - **Value:** `base_link`
- `output_frame`: Output point cloud frame.  
  - **Value:** `base_link`

---

### Crop Box Filter: Velodyne
**Description:** Similar to the depth crop box filter but using Velodyne point cloud.

**Subscribed Topics:**
- `/velodyne_points`: Input Velodyne point cloud  
  *(remapped from `~input`)*

**Published Topics:**
- `/crop_box/velodyne/output`: Output cropped Velodyne point cloud  
  *(remapped from `~output`)*

---

### Crop Box Filter: ROI
**Description:** Crops the Velodyne point cloud right above the cavity region to estimate airborne insulation distribution.

**Subscribed Topics:**
- `/velodyne_points`: Input Velodyne point cloud  
  *(remapped from `~input`)*

**Published Topics:**
- `/crop_box/roi/output`: Output cropped ROI Velodyne point cloud  
  *(remapped from `~output`)*

**Parameters:**
- `max_x`: Should be within the cavity and not include non-airborne-insulation points.
- `min_x`: Should be at the cavity's minimum x value.
- `min_z`: Should be just above the cavity height (~5-10 cm offset).
- `max_z`: Set at 20 cm above `min_z` to balance centroid estimation.

---

### Height Filter
**Description:** Filters out airborne insulation points based on their height relative to nearby points.

**Subscribed Topics:**
- `/voxel_grid/velodyne/output`: Input cropped Velodyne point cloud  
  *(remapped from `/input`)*

**Published Topics:**
- `/height_filter/velodyne/output`: Output height-filtered point cloud  
  *(remapped from `/output`)*

**Parameters:**
- `N`: Number of neighboring points considered.
- `epsilon`: Height difference threshold for airborne insulation classification.

---

### Persistent Filter
**Description:** Filters out airborne insulation points by checking their persistence over multiple frames.

**Subscribed Topics:**
- `/height_filter/velodyne/output`: Input height-filtered point cloud  
  *(remapped from `/input`)*

**Published Topics:**
- `/filtered_cloud/velodyne/output`: Output persistent-filtered point cloud  
  *(remapped from `/output`)*

**Parameters:**
- `N`: Number of previous frames stored in a rolling buffer.
- `M`: Minimum frames a voxel must be occupied to not be considered airborne insulation.
- `voxel_size`: Size of the input point cloud voxel.

---

### Insulation ROI
**Description:** Finds the centroid, bounding box, and variance of the airborne insulation. Uses a moving average to minimize noise.

**Subscribed Topics:**
- `/voxel_grid/roi/output`: Input point cloud from the crop box ROI node.  
  *(remapped from `/input`)*

**Published Topics:**
- `/roi`: Output marker showing centroid, bounding box, and variance.  
  *(remapped from `/output`)*
- `/centroid_info`: Custom `CentroidInfo` message containing centroid coordinates, bounding box offset, density, and validity status.

**Parameters:**
- `moving_average_window`: Size of the moving average window.
- `centroid_bias`: Bias applied to x-axis centroid estimation.
- `cloud_size`: Minimum cloud size to include in the moving average.
- `timeout`: Maximum time between valid point clouds before clearing the moving average buffer.

---

### Insulation Cost Map
**Description:** Computes a cost map of insulation elevation to optimize the filling process.

**Subscribed Topics:**
- `/elevation_mapping/elevation_map`: Input elevation map of insulation levels.  
  *(remapped from `/input`)*

**Published Topics:**
- `/insulation_cost_map/velodyne`: Output cost map of insulation levels.  
  *(remapped from `/output`)*

**Parameters:**
- `layer`: Name of the elevation layer in the input map.
- `min_height`: Minimum height (m) in the elevation map.
- `max_height`: Height threshold for filled status.
- `wt_height`: Weight for elevation in cost calculation.
- `wt_dist`: Weight for distance in cost calculation.
- `max_cost`: Maximum cost value.

---

### Insulation Target
**Description:** Publishes a `/target` topic with x, y displacement commands to adjust insulation aiming based on filled regions.

**Subscribed Topics:**
- `/insulation_cost_map/velodyne`: Input Velodyne elevation cost map.  
  *(remapped from `/input`)*
- `/centroid_info`: Input `CentroidInfo` message containing centroid information.

**Published Topics:**
- `/target`: Output target displacement command for centroid adjustment.

**Parameters:**
- `y_min, y_max`: Defines the fillable region.
- `fill_threshold`: 0.0-1.0 percentage of filled grid cells required to consider a depth filled.

---

### Static Transform Publishers
**Description:** Manually included static transform publishers due to the removal of the `tf_static` topic from the ROS bag.

