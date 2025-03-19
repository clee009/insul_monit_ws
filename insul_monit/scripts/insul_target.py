#!/usr/bin/env python3

import rospy
import numpy as np
from grid_map_msgs.msg import GridMap
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from copy import deepcopy
from insul_monit.msg import CentroidInfo

class InsulationTarget:
    def __init__(self):
        rospy.init_node("insulation_target")

        # Get parameters (Y range in real-world coordinates)
        self.y_min_world = rospy.get_param("~y_min", 0.0)  # Min y in meters
        self.y_max_world = rospy.get_param("~y_max", 10.0)  # Max y in meters
        self.fill_threshold = rospy.get_param("~fill_threshold", 0.3)  # Percentage of a row of cells that need to be filled for a given depth to be considered filled
        self.max_cost = rospy.get_param("~max_cost", 200)  # Max cost for filled cells

        # Subscribers
        self.costmap_sub = rospy.Subscriber("/input", GridMap, self.costmap_callback)
        self.centroid_info_sub = rospy.Subscriber("/centroid_info", CentroidInfo, self.centroid_info_callback)

        # Publisher
        self.costmap_pub = rospy.Publisher("/subset_costmap", GridMap, queue_size=1)
        self.target_pub = rospy.Publisher("/target", Point, queue_size=1)
        self.marker_pub = rospy.Publisher("/target_marker", Marker, queue_size=1)

        # Store the latest centroid info
        self.latest_timestamp = None
        self.latest_centroid = None
        self.latest_x_offset = None
        self.latest_density = None

    def centroid_info_callback(self, msg):
        """Stores the latest CentroidInfo data."""
        if msg.valid:
            self.latest_centroid = (msg.biased_centroid.x, msg.biased_centroid.y, msg.biased_centroid.z)
            self.latest_x_offset = msg.x_offset
            self.latest_density = msg.density
        else:
            self.latest_centroid = None
            self.latest_x_offset = None
            self.latest_density = None

    def costmap_callback(self, msg):
        """Processes cost map data along with the latest centroid info."""
        if self.latest_centroid is None or self.latest_x_offset is None:
            # rospy.logwarn("No CentroidInfo data received!")
            return

        centroid_x, centroid_y, centroid_z = self.latest_centroid
        rospy.loginfo(f"{centroid_x}, {centroid_y}")

        # Extract metadata
        width = msg.data[0].layout.dim[0].size
        height = msg.data[0].layout.dim[1].size
        length_x = msg.info.length_x
        length_y = msg.info.length_y
        resolution = msg.info.resolution
        x_origin = msg.info.pose.position.x
        y_origin = msg.info.pose.position.y

        # Convert world coordinates to grid indices
        y_min_idx = int((-self.y_max_world + y_origin + length_y/2) / resolution)
        y_max_idx = int((-self.y_min_world + y_origin + length_y/2) / resolution)

        # Clamp indices to valid range
        y_min_idx = max(0, min(y_min_idx, height - 1))
        y_max_idx = max(0, min(y_max_idx, height - 1))
        # rospy.loginfo(y_min_idx)
        # rospy.loginfo(y_max_idx)

        if y_min_idx >= y_max_idx:
            rospy.logwarn("y_min is greater than or equal to y_max after conversion! No subset will be published.")
            return

        # Convert to 2D NumPy array
        subset_cost_map = np.array(msg.data[0].data).reshape((height, width))

        # Select the subset based on the Y range
        subset_cost_map[:y_min_idx, :] = np.nan
        subset_cost_map[y_max_idx+1:, :] = np.nan
        submap_height = y_max_idx - y_min_idx
        # rospy.loginfo(submap_height)
        fill_count = np.sum(subset_cost_map == 200, axis=0)
        # rospy.loginfo(fill_count / submap_height)
        filled_indices = np.where(fill_count / submap_height >= self.fill_threshold)
        # rospy.loginfo(filled_indices)
        if filled_indices[0].size != 0:
            filled_depth = np.max(filled_indices)
        else:
            masked_cost_map = np.ma.masked_invalid(subset_cost_map)
            min_cost_idx = np.argmin(masked_cost_map)
            _, filled_depth = np.unravel_index(min_cost_idx, masked_cost_map.shape)
        rospy.loginfo(f"filled depth: '{filled_depth}'")

        # Publish target message
        # rospy.loginfo(self.latest_x_offset)
        target_msg = Point()
        target_msg.x = x_origin + length_x/2 - filled_depth*resolution - self.latest_x_offset - centroid_x
        target_msg.y = y_origin + length_y/2 - (y_min_idx + y_max_idx)/2*resolution - centroid_y
        target_msg.z = 0.0
        self.target_pub.publish(target_msg)

        # Publish arrow marker at centroid
        marker = Marker()
        marker.header.frame_id = "cavity"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set arrow start at centroid position
        marker.points = [
            Point(centroid_x, centroid_y, centroid_z),  # Start at centroid
            Point(centroid_x + target_msg.x, centroid_y + target_msg.y, centroid_z + target_msg.z)  # End at target
        ]

        # Define arrow scale
        marker.scale = Vector3(0.05, 0.1, 0.1)  # Shaft width, head width, head length

        # Define marker color (blue)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        self.marker_pub.publish(marker)

        # Convert back to 1D list for ROS message
        new_data = subset_cost_map.flatten().astype(np.float32).tolist()

        # Create new OccupancyGrid message
        new_msg = deepcopy(msg)
        new_msg.data[0].data = new_data
        

        # Publish the new map
        self.costmap_pub.publish(new_msg)

        
if __name__ == "__main__":
    InsulationTarget()
    rospy.spin()
