#!/usr/bin/env python
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

cv_image = cv.imread("circulos.png")

hsv_img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

frame_thresholdRed = cv.inRange(hsv_img, (0, 88, 179), (33, 255, 255))
frame_thresholdGreen = cv.inRange(hsv_img, (49, 39, 130), (98, 255, 255))


cv.imshow("Keypoints", frame_thresholdGreen)
cv.waitKey(0)

ret,th = cv.threshold(frame_thresholdGreen,98,255,cv.THRESH_BINARY_INV)

circles = cv.HoughCircles(th, cv.HOUGH_GRADIENT, 1, minDist=150, param1=200, param2=18, minRadius=5)

# Draw circles
if circles is not None:    
    print("Entro")
    circles = np.round(circles[0, :]).astype("int")
    for (x,y,r) in circles:
        cv.circle(th, (x,y), r, (36,255,12), 3)

cv.imshow('image', th)
cv.waitKey(0)
