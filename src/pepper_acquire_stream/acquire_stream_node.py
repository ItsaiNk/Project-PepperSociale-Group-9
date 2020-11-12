#!/usr/bin/env python
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Bool

class StreamController:
    def __init__(self):
        self.br = CvBridge()
        self.pub = rospy.Publisher("take_image_topic", Image, queue_size=3)
        self.sub = rospy.Subscriber("head_movement_done", Bool, self.callback)        

    def callback(self, msg):
        if msg.data:
            self.take_frame()
            
    def take_frame(self):
        cap = cv.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
        cap.release()
        self.pub.publish(self.br.cv2_to_imgmsg(frame))


if __name__ == "__main__":
    rospy.init_node('stream_acquire')
    stream_controller = StreamController()
    rospy.spin()