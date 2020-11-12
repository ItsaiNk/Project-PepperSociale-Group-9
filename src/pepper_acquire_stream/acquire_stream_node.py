#!/usr/bin/env python
import cv2 as cv
import rospy

class StreamController:
    def __init__(self):
        self.cap = cv.VideoCapture(0)
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()

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

if __name__ == "__main__":
    rospy.init_node('stream_acquire')
    stream_controller = StreamController()
    stream_controller.show_stream()
    rospy.spin()