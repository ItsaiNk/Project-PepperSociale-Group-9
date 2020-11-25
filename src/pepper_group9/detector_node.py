#!/usr/bin/env python3
import os
import rospy
from pepper_group9.msg import DetectorMessage
from std_msgs.msg import Bool
from detector import Detector
from pepper_group9.srv import Say
import ros_numpy
from classmap import category_map as classmap

# Calls the service "text_to_speech_service",
# which lets Pepper say something you want 
# param: labels - Specifies the labels of the objects detected by Pepper
# param: position - Specifies where the objects are located in the scene (left, right or center)
# returns: a Bool representing the success of the operation
# throws: rospy.ServiceException if service call failed
def text_to_speech_client(labels, position):
    rospy.wait_for_service('text_to_speech_service')
    try:
        text_to_speech = rospy.ServiceProxy('text_to_speech_service', Say)
        resp = text_to_speech(labels, position)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Receives an image and executes an object detection
# It uses the classmap in the file classmap.py to map the detected objects with the correct label
# param: msg - The message containing the Image on which perform the detection
# throws: rospy.ServiceException if service call failed
def rcv_image(msg):
    image = ros_numpy.numpify(msg.image)
    detections = mydetector(image)
    detected_classes = []
    for c in detections['detection_classes']:
        detected_classes.append(classmap[c])
    text_to_speech_client(detected_classes, msg.position)


# Initializes node with name "detector_node"
pub = rospy.Publisher("detector_loaded", Bool, queue_size=1)
DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)
rospy.init_node('detector_node')
si = rospy.Subscriber("take_image_topic", DetectorMessage, rcv_image)
rospy.loginfo("Detector node successfully started")
rospy.sleep(0.01) #necessary in order to let the publish work
pub.publish(True)
rospy.spin()
