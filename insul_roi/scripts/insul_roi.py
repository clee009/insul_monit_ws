#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.ndimage import convolve, label
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from threading import Lock

class ElevationDifferenceNode:
    def __init__(self):
        rospy.init_node("insul_roi", anonymous=True)

        # Parameters
        self.gridmap_topic = rospy.get_param("~gridmap_topic", "/elevation_mapping/elevation_map")
        self.update_interval = rospy.get_param("~update_interval", 1000)  # Milliseconds
        self.buffer_size = rospy.get_param("~buffer_size", 5)  # Number of stored difference maps
        self.output_topic = rospy.get_param("~output_topic", "/roi_map")
        self.diff_lower = rospy.get_param("~diff_lower", 0.01)
        self.diff_upper = rospy.get_param("~diff_upper", 0.1)
        self.neighbor_threshold = rospy.get_param("~neighbor_threshold", 4)
        self.kernel = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
        self.layers = {}

        # Data Storage
        self.prev_elevation = None
        self.latest_msg = None  # Store the latest GridMap message
        self.diff_buffer = []  # Circular buffer for storing elevation differences
        self.lock = Lock()  # Prevent race conditions

        # Subscriber & Publisher
        rospy.Subscriber(self.gridmap_topic, GridMap, self.gridmap_callback)
        self.map_pub = rospy.Publisher(self.output_topic, GridMap, queue_size=1)

        # Timer for processing buffer at fixed intervals
        self.timer = rospy.Timer(rospy.Duration(self.update_interval / 1000.0), self.process_buffer)

    def gridmap_callback(self, msg):
        """ Stores the latest GridMap message but does not process it immediately. """
        with self.lock:
            self.latest_msg = msg  # Store latest message to be processed in the timer callback

    def process_gridmap(self):
        """ Processes the stored GridMap message, computes the difference map, and updates buffer. """
        with self.lock:
            if self.latest_msg is None:
                # rospy.logwarn("No new elevation map received yet.")
                return
            
            msg = self.latest_msg  # Use the latest received message
            self.latest_msg = None  # Reset to avoid redundant processing

            try:
                # Extract elevation layer index
                if "elevation" not in msg.layers:
                    rospy.logwarn("Elevation layer missing in GridMap!")
                    return
                
                self.msg_info = msg.info
                idx = msg.layers.index("elevation")
                elevation_data = np.array(msg.data[idx].data).reshape(
                    (msg.data[idx].layout.dim[1].size, msg.data[idx].layout.dim[0].size)
                )

                elevation_data = np.nan_to_num(elevation_data, nan=0.0)
                
                # Apply shift correction using GridMap circular buffer indices
                outer_index = msg.outer_start_index
                inner_index = msg.inner_start_index

                # Rotate the array to compensate for the buffer shift
                elevation_data = np.roll(elevation_data, shift=-inner_index, axis=0)
                elevation_data = np.roll(elevation_data, shift=-outer_index, axis=1)
                
                # If it's the first frame, store it and return
                if self.prev_elevation is None:
                    self.prev_elevation = elevation_data
                    return
                
                # Compute elevation difference
                diff_map = elevation_data - self.prev_elevation

                # Store the difference in a circular buffer
                if len(self.diff_buffer) >= self.buffer_size:
                    self.diff_buffer.pop(0)  # Remove the oldest entry
                
                self.diff_buffer.append(diff_map)

                # Update previous elevation
                self.prev_elevation = elevation_data

            except Exception as e:
                rospy.logerr(f"Error processing GridMap: {e}")
    
    def get_roi_bbox(self, neighbor_map, centroid):
        """Returns the region of interest bounding box."""

        # Find all indices where binary_array is 1
        ones_indices = np.argwhere(neighbor_map == 1)

        if ones_indices.size == 0:
            return None  # No ones found in the array

        # Find the nearest '1' to the centroid
        distances = np.linalg.norm(ones_indices - centroid, axis=1)
        closest_idx = ones_indices[np.argmin(distances)]

        # Label contiguous regions (horizontal & vertical)
        structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])
        labeled_map, num_features = label(neighbor_map, structure)

        # Identify the label of the connected component that contains the closest '1'
        roi_label = labeled_map[tuple(closest_idx)]

        if roi_label == 0:
            return None  # No valid region found

        # Find the bounding box (min/max row and column)
        roi_indices = np.argwhere(labeled_map == roi_label)
        min_row, min_col = roi_indices.min(axis=0)
        max_row, max_col = roi_indices.max(axis=0)

        # Return bounding box as (top-left row, top-left col, bottom-right row, bottom-right col)
        return (min_row, min_col, max_row, max_col)


    def process_buffer(self, event):
        """ Calls process_gridmap() and computes temporal moving average before publishing. """
        self.process_gridmap()  # Compute difference map

        with self.lock:
            if len(self.diff_buffer) == 0:
                # rospy.logwarn("No elevation difference data available yet.")
                return

            # Compute moving average across the buffer
            moving_avg = np.mean(np.stack(self.diff_buffer), axis=0)
            moving_avg[moving_avg == 0] = np.nan
            self.layers["elevation_difference"] = moving_avg

            # Compute region of interest
            roi_map = ((moving_avg > self.diff_lower) & (moving_avg < self.diff_upper)).astype(int)
            self.layers["roi"] = roi_map

            # Compute neighbor count
            neighbor_map = convolve(roi_map, self.kernel, mode='constant', cval=0)
            neighbor_map = (neighbor_map >= self.neighbor_threshold).astype(int)
            self.layers["neighbor_count"] = neighbor_map

            # Compute centroid and ROI bounding box
            rows, cols = np.indices(neighbor_map.shape)
            total = neighbor_map.sum()
            if total != 0:
                # Get centroid
                centroid_row = (rows * neighbor_map).sum() / total
                centroid_col = (cols * neighbor_map).sum() / total
                centroid = np.array([round(centroid_row), round(centroid_col)])
                # rospy.loginfo(centroid)
                centroid_map = np.zeros_like(neighbor_map)
                centroid_map[centroid[0], centroid[1]] = 1
                self.layers["centroid"] = centroid_map

                # Get ROI bounding box
                roi_bbox = self.get_roi_bbox(neighbor_map, centroid)
                if roi_bbox is not None:
                    roi_bbox_map = np.zeros_like(neighbor_map)
                    roi_bbox_map[roi_bbox[0]:roi_bbox[2]+1, roi_bbox[1]:roi_bbox[3]+1] = 1
                    self.layers["roi_bbox"] = roi_bbox_map
            # elif "centroid" not in self.layers:
            #     self.layers["centroid"] = np.zeros_like(neighbor_map)

            # Publish the computed moving average as a new GridMap
            self.publish_map(self.layers)


    def publish_map(self, layers):
        """ Converts the moving average difference map into a GridMap message and publishes it. """
        map_msg = GridMap()
        map_msg.info = self.msg_info
        map_msg.info.header.stamp = rospy.Time.now()

        for layer in layers:
            # Copy metadata from input grid map
            map_msg.layers.append(layer)
            map_msg.basic_layers.append(layer)

            # Convert moving_avg (NumPy array) into a ROS message
            grid_data = Float32MultiArray()

            # Create and define the layout
            layout = MultiArrayLayout()
            
            layer_map = layers[layer]
            dim_col = MultiArrayDimension()
            dim_col.label = "column_index"
            dim_col.size = layer_map.shape[1]
            dim_col.stride = layer_map.shape[1] * layer_map.shape[0]  # Total elements

            dim_row = MultiArrayDimension()
            dim_row.label = "row_index"
            dim_row.size = layer_map.shape[0]
            dim_row.stride = layer_map.shape[1]  # Columns per row

            layout.dim = [dim_col, dim_row]
            layout.data_offset = 0
            
            # Assign layout and data
            grid_data.layout = layout
            grid_data.data = layer_map.flatten().tolist()

            # Append processed data to GridMap message
            map_msg.data.append(grid_data)

        # Publish the GridMap message
        self.map_pub.publish(map_msg)

if __name__ == "__main__":
    try:
        ElevationDifferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
