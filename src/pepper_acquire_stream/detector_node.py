#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from detector import Detector
import ros_numpy # pip3 install git+https://github.com/eric-wieser/ros_numpy

DET_PATH=os.path.join(os.path.dirname(__file__),'efficientdet_d1_coco17_tpu-32')
mydetector = Detector(DET_PATH)

rospy.init_node('detector_node')
pub = rospy.Publisher('detection', Detection2DArray, queue_size=2)


def rcv_image(msg):
    image = ros_numpy.numpify(msg)
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
    rospy.loginfo("published")


si = rospy.Subscriber("take_image_topic", Image, rcv_image)



try:
    rospy.spin()

except KeyboardInterrupt:
    print("Shutting down")