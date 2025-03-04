#!/usr/bin/env python3

import rospy
import numpy as np
from grid_map_msgs.msg import GridMap

class InsulCostMap:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('insul_cost_map', anonymous=True)

        # Parameters
        self.layer = rospy.get_param("~layer", "elevation")
        self.min_height = rospy.get_param("~min_height", 0.0)
        self.max_height = rospy.get_param("~max_height", 4.0)
        self.wt_height = rospy.get_param("~wt_height", 1)
        self.wt_dist = rospy.get_param("~wt_dist", 100)
        self.max_cost = rospy.get_param("~max_cost", 1000)

        # Subscribe to the input grid map
        self.gridmap_sub = rospy.Subscriber('/input', GridMap, self.costmap_callback)

        # Publisher for the processed grid map (with cost layer)
        self.gridmap_pub = rospy.Publisher('/output', GridMap, queue_size=1)

        rospy.loginfo("InsulCostMap Node Initialized")

    def costmap_callback(self, msg):
        """ Callback function to compute and publish a cost grid map """
        # Check if the specified layer exists in the input GridMap
        if self.layer not in msg.layers:
            rospy.logwarn(f"Layer '{self.layer}' not found in GridMap!")
            return

        # Get elevation data from the specified layer
        idx = msg.layers.index(self.layer)
        row_size = msg.data[idx].layout.dim[1].size
        col_size = msg.data[idx].layout.dim[0].size
        data = np.array(msg.data[idx].data).reshape((row_size, col_size))

        # Correct for the circular buffer indices provided by GridMap
        outer_index = msg.outer_start_index
        inner_index = msg.inner_start_index
        data = np.roll(data, shift=-inner_index, axis=0)
        data = np.roll(data, shift=-outer_index, axis=1)
        nan_mask = np.isnan(data)

        # Create an x-distance array
        x_dist, _ = np.meshgrid(np.arange(col_size), np.arange(row_size))

        # Compute the cost based on a combination of normalized height and distance.
        normalized_height = (data - self.min_height) / (self.max_height - self.min_height)
        normalized_dist = x_dist / (col_size - 1)
        cost_data = self.wt_height * normalized_height + self.wt_dist * normalized_dist

        # Replace cells with elevation above max_height with max_cost
        filled_indices = np.where(data >= self.max_height)
        cost_data[filled_indices] = self.max_cost

        # Set cells with no valid data to NaN
        cost_data[nan_mask] = np.nan

        # Create a new GridMap message with the computed cost layer.
        grid_map_out = GridMap()
        grid_map_out.info = msg.info

        # Set the layers field
        grid_map_out.layers = ["cost"]

        # Assign the cost data as a flat list (row-major order)
        grid_map_out.data = [msg.data[idx]]
        grid_map_out.data[0].data = cost_data.flatten().astype(np.float32).tolist()

        # Publish the new GridMap with the cost layer
        self.gridmap_pub.publish(grid_map_out)

if __name__ == '__main__':
    try:
        node = InsulCostMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("InsulCostMap Node Shutting Down")
