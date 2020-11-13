#!/usr/bin/env python3
import os
import rospy
from pepper_acquire_stream.msg import detector_msg
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from detector import Detector
from pepper_acquire_stream.srv import Say
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
    message = Detection2DArray()
    for clabel,score,box in zip(detections['detection_classes'], detections['detection_scores'], detections['detection_boxes']):
        d = Detection2D()
        d.bbox.size_x = box[3]-box[1]
        d.bbox.size_y = box[2]-box[0]
        d.bbox.center.x = box[1]+d.bbox.size_x/2
        d.bbox.center.y = box[0]+d.bbox.size_y/2
        o = ObjectHypothesisWithPose()
        o.score = score
        o.id = clabel
        d.results.append(o)
        message.detections.append(d)
    pub.publish(message)
    detected_classes = []
    for c in detections['detection_classes']:
        detected_classes.append(classmap[c])
    animated_say_client(detected_classes, msg.position)
    rospy.loginfo("published")

#Initialize node
DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)

rospy.init_node('detector_node')

pub = rospy.Publisher('detection', Detection2DArray, queue_size=3)
si = rospy.Subscriber("take_image_topic", detector_msg, rcv_image)

rospy.spin()