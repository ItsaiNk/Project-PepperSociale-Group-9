#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_group9.srv import Shot, ShotResponse
from sensor_msgs.msg import Image

# Class Vision extends NaoqiNode
# Node that gives connection with the pepper camera and acquires image
#
# Class Attributes:
# - nameId: ID of the pepper camera
# - vision: proxy to the core module of ALVideoDevice
# - s: Service pepper_vision_service related to the acquisition from camera
#
class Vision(NaoqiNode):
    # Constructor of the class,
    # which initializes pepper_vision node and 
    # calls the connectNaoQi function
    def __init__(self):
        NaoqiNode.__init__(self,'pepper_vision')
        self.nameId = None
        self.connectNaoQi()

    # Uses the vision proxy to subscribe the pepper camera
    # and instantiates pepper_vision_service.
    # error: if the acquisition of ALVideoDevice proxy fails
    def connectNaoQi(self):
        self.vision=self.get_proxy("ALVideoDevice")
        if self.vision is None:
            rospy.logerr("Error in initialization of ALVideoDevice proxy!")
        kTopCamera = 0
        resolution = 2
        framerate = 5
        color_space = 11
        self.nameId = self.vision.subscribeCamera("pepper_rgb_camera", kTopCamera, resolution, color_space, framerate)
        rospy.loginfo('Using camera: rgb camera. Subscriber name is %s .' % (self.nameId))
        self.s = rospy.Service('pepper_vision_service', Shot, self.shot)

    # Takes a photo and sends it as Shot service's response
    # param: 
    # - data: set to None because is used as a trigger
    # return: ShotResponse that contains the acquired image
    def shot(self, data=None):
        img_to_send = Image()
        image = self.vision.getImageRemote(self.nameId)
        img_to_send.header.stamp = rospy.Time.now()
        img_to_send.height = image[1]
        img_to_send.width = image[0]
        layers = image[2] #3 for RGB
        img_to_send.encoding = "8UC3" #8 unsigned bit 3 channel
        img_to_send.step = img_to_send.width * layers
        img_to_send.data = image[6]
        rospy.loginfo('Sending image...')
        return ShotResponse(img_to_send)
    
    #Unsubscribes the camera
    def shutdown_handle(self):
        self.vision.unsubscribe(self.nameId)

# Initializes the vision_node and define the shutdown_handle        
if __name__=="__main__":
    vision_node = Vision()
    rospy.on_shutdown(vision_node.shutdown_handle)
    rospy.loginfo("Pepper Vision node successfully started")
    rospy.spin()


