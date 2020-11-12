#!/usr/bin/env python
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Bool

class StreamController:
    def __init__(self):
        #self.cap = cv.VideoCapture(0)
        self.br = CvBridge()
        self.pub = rospy.Publisher("take_image_topic", Image, queue_size=3)
        self.sub = rospy.Subscriber("head_movement_done", Bool, self.callback)
        #self.sub2 = rospy.Subscriber("take_image_topic", Image, self.show_frame)
        

    def callback(self, msg):
        if msg.data:
            self.take_frame()
        
    # Sostituire con codice per acquisire stream da Pepper
    def show_stream(self):
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            # Our operations on the frame come here
            # Display the resulting frameq
            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):
                break
        # When everything done, release the capture
        self.cap.release()
        cv.destroyAllWindows()
    
    def show_frame(self, msg):
        img = self.br.imgmsg_to_cv2(msg)
        cv.imshow("prova", img)
        cv.waitKey(0)  
        cv.destroyAllWindows()
            
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
        # self.cap.release()
        self.pub.publish(self.br.cv2_to_imgmsg(frame))


if __name__ == "__main__":
    rospy.init_node('stream_acquire')
    stream_controller = StreamController()
    rospy.spin()