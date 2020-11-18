#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from pepper_group9.msg import DetectorMessage
from pepper_group9.srv import StiffnessSrv, Shot

class PepperStart():

    def __init__(self):
        self.pub_head_move = rospy.Publisher("head_movement_start", String, queue_size=3)
        self.sub_movement_done = rospy.Subscriber("head_movement_done", Bool, self.controller)
        self.pub_take_image = rospy.Publisher("take_image_topic", DetectorMessage, queue_size=3)
        self.count = 0
        self.pepper_stiffness_client(True)
        #self.pub_head_move.publish("reset")

    def pepper_stiffness_client(self, flag):
        rospy.wait_for_service("pepper_stiffness_service")
        try:
            pepper_stiffness = rospy.ServiceProxy('pepper_stiffness_service', StiffnessSrv)
            resp = pepper_stiffness(flag)
            return resp.response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def take_photo(self):
        rospy.wait_for_service("pepper_vision_service")
        try:
            pepper_vision = rospy.ServiceProxy('pepper_vision_service', Shot)
            resp = pepper_vision()
            return resp.image
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_head_position(self):
        if self.count == 0:
            return "center"
        elif self.count == 1:
            return "left"
        elif self.count == 2:
            return "right"
        elif self.count == 3:
            return "reset"


    def controller(self, data):
        if self.count < 3:
            img = self.take_photo()
            msg = DetectorMessage()
            msg.position = self.get_head_position()
            msg.image = img
            self.pub_take_image.publish(msg)
            self.count += 1
            self.pub_head_move.publish(self.get_head_position())
        else:
            self.pepper_stiffness_client(False)


if __name__ == "__main__":
    rospy.init_node("pepper_start")
    start = PepperStart()
    rospy.spin()