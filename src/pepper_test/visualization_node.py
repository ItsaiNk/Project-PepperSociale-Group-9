#!/usr/bin/env python3
import rospy
import cv2
import ros_numpy
from std_msgs.msg import Bool

def pepper_vision_client():
    rospy.wait_for_service("pepper_vision")
    try:
        pepper_vision = rospy.ServiceProxy('pepper_vision', Shot)
        resp = pepper_vision()
        return resp.image
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def visualize_photo(data):
    im = ros_numpy.numpify(pepper_vision_client())
    cv2.imshow('Image', im)
    cv2.waitKey(100)

#Initialize node

rospy.init_node('visualization_node')
sd = rospy.Subscriber("pepper_take_photo", Bool, visualize_photo)
rospy.spin()
