#!/usr/bin/env python3
import rospy
import cv2
import ros_numpy
from std_msgs.msg import Bool
from pepper_test.srv import Shot, ShotResponse
from cv_bridge import CvBridge


def pepper_vision_client():
    rospy.wait_for_service("pepper_vision_service")
    try:
        pepper_vision = rospy.ServiceProxy('pepper_vision_service', Shot)
        resp = pepper_vision()
        return resp.image
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def visualize_photo(data):
    bridge = CvBridge()
    im = bridge.imgmsg_to_cv2(pepper_vision_client())
    #im = ros_numpy.numpify(pepper_vision_client())
    cv2.imshow('Image', im)
    cv2.waitKey(100)

#Initialize node

rospy.init_node('visualization_node')
sd = rospy.Subscriber("pepper_take_photo", Bool, visualize_photo)
rospy.spin()
