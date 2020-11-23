#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from pepper_group9.msg import DetectorMessage
from pepper_group9.srv import StiffnessSrv, Shot

# Class PepperStart
# Acts as central node to synchronize head movement with image acquisition
#
# Class Attributes:
# - count: counts the head movements performed
# - pub_head_move: publisher for the topic "head_movement_start"
# - sub_movement_done: subscriber for the topic "head_movement_done"
# - pub_take_image: publisher for the topic "take_image_topic"
#
class PepperStart():
    # Constructor of the class
    def __init__(self):
        self.pub_head_move = rospy.Publisher("head_movement_start", String, queue_size=3)
        self.sub_movement_done = rospy.Subscriber("head_movement_done", Bool, self.controller)
        self.pub_take_image = rospy.Publisher("take_image_topic", DetectorMessage, queue_size=3)
        self.count = 0
        self.pepper_stiffness_client()
        self.pub_head_move.publish("reset")

    # Calls the service "pepper_stiffness_service",
    # which wakes up Pepper 
    # error: if the service response is False (Pepper is in rest)
    # throws: rospy.ServiceException if service call failed
    def pepper_stiffness_client(self):
        rospy.wait_for_service("pepper_stiffness_service")
        try:
            pepper_stiffness = rospy.ServiceProxy('pepper_stiffness_service', StiffnessSrv)
            resp = pepper_stiffness(True)
            if not resp.response:
                rospy.logerr("Error in robot wakeup!")
                exit(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # Calls the service "pepper_vision_service",
    # which takes an image from Pepper front camera
    # returns: the acquired image
    # throws: rospy.ServiceException if service call failed
    def take_photo(self):
        rospy.wait_for_service("pepper_vision_service")
        try:
            pepper_vision = rospy.ServiceProxy('pepper_vision_service', Shot)
            resp = pepper_vision()
            return resp.image
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # Returns the current head position
    def get_head_position(self):
        if self.count == 0:
            return "center"
        elif self.count == 1:
            return "left"
        elif self.count == 2:
            return "right"
        elif self.count == 3:
            return "reset"

    # Callback of sub_movement_done, called when a head movement has completed.
    # Publishes an image and the relative position on the topic "take_image_topic".
    # Publishes also the next head movement to perform on the topic "head_movement_start".
    # 
    # If Pepper turned its head left, right and returned to center position,
    # resets the counter to perform another cycle of detections.
    # param: data - Bool
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
            self.count = 0

# Initializes the node with name "pepper_start"
if __name__ == "__main__":
    rospy.init_node("pepper_start")
    start = PepperStart()
    rospy.spin()