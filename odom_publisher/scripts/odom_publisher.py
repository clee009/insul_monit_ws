#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry

class OdomPublisher:
    def __init__(self):
        # Initialize the ROS node.
        rospy.init_node('odom_publisher', anonymous=True)
        
        # Initialize a variable to store the latest camera info timestamp.
        self.latest_cam_stamp = rospy.Time(0)
        
        # Set up the publisher for the odometry message.
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Subscribe to the camera info topic.
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)
        
        # Set up a timer to call the odometry publishing function at 10 Hz.
        self.timer = rospy.Timer(rospy.Duration(0.1), self.odom_timer_callback)
        
        rospy.loginfo("Odom Publisher Node Initialized.")

    def camera_info_callback(self, msg):
        """
        Callback function that updates the latest timestamp from the camera info message.
        """
        self.latest_cam_stamp = msg.header.stamp
        # rospy.loginfo("Camera info timestamp updated: %s", self.latest_cam_stamp.to_sec())

    def odom_timer_callback(self, event):
        """
        Timer callback that creates and publishes the odometry message.
        """
        odom_msg = Odometry()

        # Use the latest camera info timestamp if available; otherwise, use current time.
        if self.latest_cam_stamp == rospy.Time(0):
            odom_msg.header.stamp = rospy.Time.now()
        else:
            odom_msg.header.stamp = self.latest_cam_stamp

        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set pose: position at the origin and default orientation.
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Set twist: zero linear and angular velocities.
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Publish the odometry message.
        self.odom_pub.publish(odom_msg)
        # rospy.loginfo("Published odometry with timestamp: %s", odom_msg.header.stamp.to_sec())

if __name__ == '__main__':
    try:
        # Instantiate the class which sets up the node, subscriptions, and timers.
        odom_publisher_node = OdomPublisher()
        # Keep the node running and processing callbacks.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
