import cv2 as cv
import rospy

cam = cv.VideoCapture(-1)
cv.namedWindow("test")
while True:
    ret, frame = cam.read()
    cv.imshow("test", frame)