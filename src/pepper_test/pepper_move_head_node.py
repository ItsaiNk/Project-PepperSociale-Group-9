#!/usr/bin/env python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_msgs.msg import Bool
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

class HeadController(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'pepper_move_head')
        self.connectNaoQi()
        self.s = JointAnglesWithSpeed()
        self.s.joint_names=['HeadPitch', 'HeadYaw']
        self.s.relative=0
        self.s.speed=0.2
        self.pub_pepper = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0)
        self.pub_node = rospy.Publisher('head_movement_done', Bool, queue_size=1)
        #self.move_head(0.0, 0.0, False)
        self.sub = rospy.Subscriber('head_movement_start', String, self.move_head_listener)

    def connectNaoQi(self):
        self.motion=self.get_proxy("ALMotion")
        self.motion.wakeUp()

    def move_head_listener(self, msg):
        if msg.data == "left":
            self.move_head(0.2, 1.0)
        elif msg.data == "right":
            self.move_head(0.2, -1.0)
        elif msg.data == "reset":
            self.move_head(0.0, 0.0)

    def move_head(self, pitch, yaw, done=True):
        self.s.joint_angles=[pitch, yaw]
        rospy.loginfo(self.s.joint_angles)
        self.pub_pepper.publish(self.s)
        rospy.sleep(rospy.Duration(5.0))
        if done:
            self.pub_node.publish(done)


if __name__=='__main__':
    controller = HeadController()
    rospy.spin()
