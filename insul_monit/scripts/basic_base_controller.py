#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point, Twist

class TargetToCmdVel:
    def __init__(self):
        rospy.init_node('target_to_cmd_vel', anonymous=True)
        
        # Subscribers and Publishers
        self.target_sub = rospy.Subscriber('/target', Point, self.target_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Parameters
        self.linear_speed = 0.2  # Scaling factor for speed
        
    def target_callback(self, msg):
        """Callback function to process the target displacement."""
        x_target = msg.x
        y_target = msg.y
        
        # Wait for user input before processing
        input("Press Enter to process the target displacement...")
        
        # Compute movement duration based on the maximum displacement component
        max_displacement = max(abs(x_target), abs(y_target))
        if max_displacement == 0:
            return  # No movement needed
        
        duration = max_displacement / self.linear_speed
        
        # Compute velocity components
        twist = Twist()
        twist.linear.x = (x_target / max_displacement) * self.linear_speed
        twist.linear.y = (y_target / max_displacement) * self.linear_speed
        
        # Publish velocity command
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(duration)  # Move for required duration
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TargetToCmdVel()
        node.run()
    except rospy.ROSInterruptException:
        pass
