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
class PepperCognitive():

    # Constructor of the class
    def __init__(self):
        self.si = rospy.Subscriber("take_image_topic", DetectorMessage, self.rcv_image)
        self.pub = rospy.Publisher("detector_loaded", Bool, queue_size=1)
        self.wait_detector_service()

    # Wait for the detector service, publish a message on the "detector_loaded" topic when ready
    def wait_detector_service(self):
        rospy.wait_for_service('detector_service')
        self.pub.publish(True)

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
            self.text_to_speech_client(resp.labels, position)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
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