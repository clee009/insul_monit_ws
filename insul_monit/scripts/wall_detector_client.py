#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def call_service():
    rospy.wait_for_service('/detect_wall_x')
    try:
        detect_wall = rospy.ServiceProxy('/detect_wall_x', Trigger)
        response = detect_wall()
        rospy.loginfo("Service response: Success=%s, Message=%s", response.success, response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node("wall_detector_client")
    call_service()