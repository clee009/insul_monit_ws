#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

def depth_callback(msg):
    global frame_queue, camera_info_msg, N_FRAMES, THRESHOLD

    # Convert ROS Image message to OpenCV format
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Convert 16-bit depth to meters (if needed)
    if msg.encoding == "16UC1":
        depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters

    # Store initial frames before processing
    if len(frame_queue) < N_FRAMES:
        frame_queue.append(depth_image)
        return

    # Get the reference frame (N frames ago)
    reference_frame = frame_queue[0]

    # Compute absolute difference
    depth_diff = np.abs(depth_image - reference_frame)

    # Create mask for airborne insulation (moving particles)
    mask = depth_diff > (THRESHOLD / 1000.0)  # Convert mm threshold to meters

    # Apply mask: Set moving pixels to 0 (filtered out)
    filtered_depth = depth_image.copy()
    filtered_depth[mask] = 0

    # Convert back to 16-bit format for publishing
    filtered_depth_mm = (filtered_depth * 1000).astype(np.uint16)  # Convert meters to mm

    # Convert back to ROS Image message
    filtered_msg = bridge.cv2_to_imgmsg(filtered_depth_mm, encoding="16UC1")
    filtered_msg.header = msg.header  # Preserve original timestamp and frame ID

    # Publish the filtered depth image
    filtered_depth_pub.publish(filtered_msg)

    # Publish corresponding CameraInfo message (if available)
    if camera_info_msg:
        camera_info_msg.header.stamp = msg.header.stamp  # Sync timestamps
        filtered_camera_info_pub.publish(camera_info_msg)

    # Update the frame queue
    frame_queue.append(depth_image)

def camera_info_callback(msg):
    """ Store the latest CameraInfo message to synchronize it with filtered depth images. """
    global camera_info_msg
    camera_info_msg = msg

if __name__ == "__main__":
    try:
        # Initialize ROS Node
        rospy.init_node("filtered_depth_node")

        # Load parameters from launch file
        N_FRAMES = rospy.get_param("~n_frames", 10)  # Default is 10
        THRESHOLD = rospy.get_param("~threshold", 500)  # Default is 500 mm

        rospy.loginfo(f"Starting filtered_depth_node with N_FRAMES={N_FRAMES}, THRESHOLD={THRESHOLD} mm")

        # Initialize Global Variables
        frame_queue = deque(maxlen=N_FRAMES)  # Rolling buffer for past N frames
        bridge = CvBridge()
        camera_info_msg = None  # Stores latest CameraInfo

        # ROS Publishers
        filtered_depth_pub = rospy.Publisher("/output/image_raw", Image, queue_size=1)
        filtered_camera_info_pub = rospy.Publisher("/output/camera_info", CameraInfo, queue_size=1)

        # Subscribe to depth image and camera info topics
        rospy.Subscriber("/input/image_raw", Image, depth_callback)
        rospy.Subscriber("/input/camera_info", CameraInfo, camera_info_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
