#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_group9.srv import StiffnessSrv, StiffnessSrvResponse

class StiffnessController(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'pepper_stiffness')
        self.motion=self.get_proxy("ALMotion")
        self.service = rospy.Service("pepper_stiffness_service", StiffnessSrv, self.set_stiffness)

    def set_stiffness(self, flag):
        if flag.request:
            self.motion.wakeUp()
        else:
            self.motion.rest()
        
        return StiffnessSrvResponse(True)

if __name__=="__main__":
    controller = StiffnessController()
    rospy.spin()
