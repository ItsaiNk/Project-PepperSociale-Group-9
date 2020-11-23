#!/usr/bin/env python3
import os
import rospy
from pepper_group9.msg import DetectorMessage
from detector import Detector
from pepper_group9.srv import Say
import ros_numpy
from classmap import category_map as classmap

def animated_say_client(labels, position):
    rospy.wait_for_service('animated_say')
    try:
        animated_say = rospy.ServiceProxy('animated_say', Say)
        resp = animated_say(labels, position)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def rcv_image(msg):
    image = ros_numpy.numpify(msg.image)
    detections = mydetector(image)
    detected_classes = []
    for c in detections['detection_classes']:
        detected_classes.append(classmap[c])
    animated_say_client(detected_classes, msg.position)


#Initialize node
DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)
rospy.init_node('detector_node')
rospy.loginfo("Detector avviato correttamente")
si = rospy.Subscriber("take_image_topic", DetectorMessage, rcv_image)

rospy.spin()
