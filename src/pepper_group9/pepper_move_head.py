#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

# Class HeadController
# Manage the Pepper head movement, regulates joints angles and speed through the "JointAnglesWithSpeed" message of the package "naoqi_bridge_msgs"
#
# Class Attributes:
# - s: the message of type "JointAnglesWithSpeed"
# - pub_pepper: publisher for the topic "pepper_robot/pose/joint_angles"
# - pub_node: publisher for the topic "head_movement_done"
# - sub: subscriber for the topic "head_movement_start"
#
class HeadController():
    # Constructor of the class
    def __init__(self):
        self.s = JointAnglesWithSpeed()
        self.s.joint_names=['HeadPitch', 'HeadYaw']
        self.s.relative=0
        self.s.speed=0.2
        self.pub_pepper = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0)
        self.pub_node = rospy.Publisher('head_movement_done', Bool, queue_size=1)
        self.sub = rospy.Subscriber('head_movement_start', String, self.move_head_listener)

    # Callback of the subscriber on the topic "head_movement_start"
    # Calls the method "move_head", giving in input the appropriate angles based on the position received
    # param: msg - std_msgs.msg/String -> message that contains the desired position.
    def move_head_listener(self, msg):
        if msg.data == "left":
            self.move_head(0.2, 0.9)
        elif msg.data == "right":
            self.move_head(0.2, -0.9)
        elif msg.data == "reset":
            self.move_head(0.2, 0.0)

    # Publish on the topic "/pepper_robot/pose/joint_angles" the desired angles for the joints.
    # Publish on the topic "head_movement_done" when the head movement is completed.
    # param: pitch: angle relative to the "HeadPithch" joint
    # param: yaw: angle relative to the "HeadYaw" joint
    def move_head(self, pitch, yaw):
        self.s.joint_angles=[pitch, yaw]
        rospy.loginfo(self.s.joint_angles)
        self.pub_pepper.publish(self.s)
        rospy.sleep(rospy.Duration(5.0))
        self.pub_node.publish(done)

# Initializes the node with name "head_movement_controller"
if __name__=='__main__':
    rospy.init_node("head_movement_controller")
    controller = HeadController()
    rospy.spin()
