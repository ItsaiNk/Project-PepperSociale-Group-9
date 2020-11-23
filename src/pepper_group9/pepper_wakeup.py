#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_group9.srv import StiffnessSrv, StiffnessSrvResponse

# Class PepperStart, extends NaoqiNode
# Allows to wake up or rest Pepper
#
# Class Attributes:
# - motion: proxy to Naoqi core module "ALMotion"
# - service: rospy Service to wake up or rest Pepper
#
class StiffnessController(NaoqiNode):
    # Initializes the NaoQi node with name "pepper_stiffness"
    def __init__(self):
        NaoqiNode.__init__(self,'pepper_stiffness')
        self.motion=self.get_proxy("ALMotion")
        self.service = rospy.Service("pepper_stiffness_service", StiffnessSrv, self.set_stiffness)

    # Wakes up or rests Pepper
    # param: flag - Bool, determines if Pepper will be woken up or rested
    # returns: True if Pepper is woken up, False if it's rested
    def set_stiffness(self, flag):
        if flag.request:
            self.motion.wakeUp()
        else:
            self.motion.rest()
        return self.motion.robotIsWakeUp()
    
    # Called when the node is destroyed
    # Automatically rests Pepper
    def shutdown_handle(self): 
        self.motion.rest()

if __name__=="__main__":
    controller = StiffnessController()
    rospy.on_shutdown(controller.shutdown_handle)
    rospy.spin()
