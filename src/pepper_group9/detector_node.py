#!/usr/bin/env python3
import os
import rospy
from detector import Detector
from pepper_group9.srv import DetectService, DetectServiceResponse
import ros_numpy
from classmap import category_map as classmap
import time

# Class DetectorService
# Detect objects in an image
# Class attributes:
# mydetector - attribute of class Detector
# service - the detection service
class DetectorService():

    # Constructor of the class
    def __init__(self):
        DET_PATH=os.path.join(os.path.dirname(__file__),'ssd_mobilenet_v2_fpnlite_640x640_coco17_tpu-8')
        self.mydetector = Detector(DET_PATH)
        self.service = rospy.Service('detector_service', DetectService, self.detect_handle)

    # Receives an image and executes an object detection
    # It uses the classmap in the file classmap.py to map the detected objects with the correct label
    # param: req - The request message containing the Image on which perform the detection
    # returns: DetectServiceResponse - string[] labels, float32[] boxes, response of DetectService.srv
    def detect_handle(self, req):
        image = ros_numpy.numpify(req.image)
        start_time = time.time()
        detections = self.mydetector(image)
        end_time = time.time()
        rospy.loginfo("DETECTION TIME: " + str((end_time-start_time) % 60) + " SECONDS.")
        detected_classes = []
        detected_boxes = []
        for c in detections['detection_classes']:
            detected_classes.append(classmap[c])
        for c in detections['detection_boxes']:
            for v in c:
                detected_boxes.append(float(v))
        return DetectServiceResponse(detected_classes, detected_boxes)

if __name__ == "__main__":
    rospy.init_node('detector_node')
    start_time = time.time()
    detector = DetectorService()
    end_time = time.time()
    rospy.loginfo("DETECTION LOADING TIME: " + str((end_time-start_time) % 60) + " SECONDS.")
    rospy.loginfo("Detector node successfully started")
    rospy.spin()