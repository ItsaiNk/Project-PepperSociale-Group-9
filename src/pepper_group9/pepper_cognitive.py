#!/usr/bin/env python
import rospy
from pepper_group9.msg import DetectorMessage
from pepper_group9.srv import DetectService, DetectServiceResponse, Say, SayResponse
from std_msgs.msg import Bool

# Class PepperCognitive
# Handle the cognitive part og the project, i.e. detection and speech
# Class attributes:
# si - subscriber for the topic "take_image_topic"
# pub - publisher for the topic "detector_loaded"
# center_left - dictionary (key: label, value: bounding box) with objects in the left border of the central image
# center_right - dictionary (key: label, value: bounding box) with objects in the right border of the central image
# left - dictionary (key: label, value: bounding box) with objects in the right border of the left image
# right - dictionary (key: label, value: bounding box) with objects in the left border of the right image
class PepperCognitive():

    # Constructor of the class
    def __init__(self):
        self.si = rospy.Subscriber("take_image_topic", DetectorMessage, self.rcv_image)
        self.pub = rospy.Publisher("detector_loaded", Bool, queue_size=1)
        self.center_left = {}
        self.center_right = {}
        self.left = {}
        self.right = {}
        self.wait_detector_service()

    # Wait for the detector service, publish a message on the "detector_loaded" topic when ready
    def wait_detector_service(self):
        rospy.wait_for_service('detector_service')
        self.pub.publish(True)

    # Check if an object is on a border of the image
    # param: classes - list of labels of objects to examine
    # param: boxes - bounding box of the objects
    # param: mode - indicates the image in which the objects appear
    # return: the dictionaries (key: label, value: bounding box) containing them.
    def border_items(self, classes, boxes, mode):
        if mode == "center":
            th_left = 0.05
            th_right = 0.95
        elif mode == "left":
            th_left = -2.00
            th_right = 0.95
        else:
            th_left = 0.05
            th_right = 2.00

        left_border_items = {}
        right_border_items = {}
        for i in range(len(boxes)):
            if boxes[i][1] < th_left:
                left_border_items[classes[i]] = boxes[i]
            elif boxes[i][3] > th_right:
                right_border_items[classes[i]] = boxes[i]
        return left_border_items, right_border_items

    # Calls the service "detector_service",
    # which detects objects in an image
    # param: msg - message containing the image and the position
    # throws: rospy.ServiceException if service call failed
    def rcv_image(self, msg):
        position = msg.position
        rospy.wait_for_service('detector_service')
        try:
            detector_service = rospy.ServiceProxy('detector_service',DetectService)
            resp = detector_service(msg.image)
            boxes = [resp.boxes[i:i + 4] for i in range(0, len(resp.boxes), 4)]
            if position == "center":
                self.center_left, self.center_right = self.border_items(resp.labels, boxes, position)
                labels = resp.labels
            elif position == "left":
                _, self.left = self.border_items(resp.labels, boxes, position)
                labels = self.check_border_items(self.center_left, self.left)
            else:
                self.right, _ = self.border_items(resp.labels, boxes, position)
                labels = self.check_border_items(self.center_right, self.right)
            self.text_to_speech_client(labels, position)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    # Checks if the same object appears in two adjacent images.
    # param: center - dictionaries (key: label, value: bounding box) of objects on the common border of the central image
    # param: side - dictionaries (key: label, value: bounding box) of objects on the common border of the side (left or right) image
    # return: the updated list of object of the side image, without the repeated elements
    def check_border_items(self, center, side):
        for item in side.keys():
            if item in center:
                if abs(side[item][0]-center[item][0]) < 0.10 and abs(side[item][2]-center[item][2]) < 0.10:
                    del side[item]
        return side.keys()

    # Calls the service "text_to_speech_service",
    # which lets Pepper say something you want 
    # param: labels - Specifies the labels of the objects detected by Pepper
    # param: position - Specifies where the objects are located in the scene (left, right or center)
    # returns: a Bool representing the success of the operation
    # throws: rospy.ServiceException if service call failed
    def text_to_speech_client(self, labels, position):
        rospy.wait_for_service('text_to_speech_service')
        try:
            text_to_speech = rospy.ServiceProxy('text_to_speech_service', Say)
            resp = text_to_speech(labels, position)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
if __name__ == "__main__":
    rospy.init_node("pepper_cognitive")
    cognitive = PepperCognitive()
    rospy.loginfo("Pepper Cognitive node successfully started.")
    rospy.spin()