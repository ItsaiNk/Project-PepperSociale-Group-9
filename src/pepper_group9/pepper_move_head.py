#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

class HeadController():

    def __init__(self):
        self.s = JointAnglesWithSpeed()
        self.s.joint_names=['HeadPitch', 'HeadYaw']
        self.s.relative=0
        self.s.speed=0.2
        self.pub_pepper = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0)
        self.pub_node = rospy.Publisher('head_movement_done', Bool, queue_size=1)
        self.sub = rospy.Subscriber('head_movement_start', String, self.move_head_listener)

    def move_head_listener(self, msg):
        if msg.data == "left":
            self.move_head(0.2, 0.9)
        elif msg.data == "right":
            self.move_head(0.2, -0.9)
        elif msg.data == "reset":
            self.move_head(0.2, 0.0)

    def move_head(self, pitch, yaw, done=True):
        self.s.joint_angles=[pitch, yaw]
        rospy.loginfo(self.s.joint_angles)
        self.pub_pepper.publish(self.s)
        rospy.sleep(rospy.Duration(5.0))
        if done:
            self.pub_node.publish(done)


if __name__=='__main__':
    rospy.init_node("head_movement_controller")
    controller = HeadController()
    rospy.spin()
