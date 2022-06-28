#!/usr/bin/env python
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

cv_image = cv.imread("BlobTest.webp")

cv.imshow("image",cv_image)

img_resized = cv.resize(cv_image, (720,480), interpolation = cv.INTER_AREA)

#converted_img = cv.cvtColor(img_resized, cv.COLOR_BGRA2BGR);
#converted_img = cv.cvtColor(img_resized, cv.COLOR_GRAY2BGR)
   
flt_img = cv.fastNlMeansDenoisingColored(img_resized, None, 10, 10,7,21)

hsv_img = cv.cvtColor(flt_img, cv.COLOR_BGR2HSV)

frame_thresholdRed = cv.inRange(hsv_img, (0, 88, 179), (33, 255, 255))
frame_thresholdGreen = cv.inRange(hsv_img, (49, 39, 130), (98, 255, 255))
msk = cv.bitwise_or(frame_thresholdRed,frame_thresholdGreen)
colors_img = cv.bitwise_and(flt_img,flt_img,mask=msk)

cv.imshow("Colors", colors_img)
cv.waitKey(0)

ret,th = cv.threshold(msk,98,255,cv.THRESH_BINARY_INV)
#th = cv.adaptiveThreshold(colors_img,255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY,11,2)
#flt_img = cv.GaussianBlur(img_resized, (5,5), 0)
cv.imshow("Image Window", th)
cv.waitKey(0)



params = cv.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 10

params.filterByCircularity = True
params.minCircularity = 0.9

params.filterByConvexity = True
#params.minConvexity = 0.2

#params.filterByInertia = True
#params.minInertiaRatio = 0.01

#gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)



detector = cv.SimpleBlobDetector_create()
keypoints = detector.detect(cv_image)

if keypoints:
    print "found blobs"

#blank = np.zeros((1, 1))
#im_with_keypoints = cv.drawKeypoints(th, keypoints, blank, (0, 0, 255),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

#number_of_blobs = len(keypoints)
#text = "Circular Blobs: " + str(len(keypoints))
#cv.putText(im_with_keypoints, text, (10, 100),cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

#cv.imshow("Keypoints", im_with_keypoints)
#cv.waitKey(0)


im_with_keypoints = cv.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv.imshow("Keypoints", im_with_keypoints)
cv.waitKey(0)

print(detector)


cv.destroyAllWindows()
