#!/usr/bin/env python

#TRAFFIC LIGHTS

import imghdr
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

bridge = CvBridge()
class TrafficLights():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        image_topic = "/video_source/raw"  # robot
        #image_topic = "/camera/image_raw"  # simulacion
        
        #SUBSCRIBERS
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        
        #PUBLISHERS
        self.light_pub = rospy.Publisher("traffic_light_topic", String, queue_size=1)
        
        #VARIABLES
        ra = rospy.Rate(10) #10Hz  
        self.bandera = 0
        Param1 = 100
        Param2 = 30
        MinRadius = 0
        MaxRadius = 40
        cx = 0
        cy = 0

        self.light = ""
        
        while not rospy.is_shutdown():  
            if self.bandera: 
                self.img = self.cv_image
                # Cambiamos la imagen a escala de grises y aplicamos un filtro
                imgGray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
                imgGray = cv2.medianBlur(imgGray, 5)
                rows = imgGray.shape[0]
                # Deteccion de circulos con los parametros definidos
                circles = cv2.HoughCircles(imgGray,cv2.HOUGH_GRADIENT,1,rows/8,param1 = Param1,param2 = Param2,minRadius = MinRadius, maxRadius = MaxRadius)
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for i in circles[0, :]:
                        cx = i[0]
                        cy = i[1]
                        center = (cx,cy)
                        # circle center
                        # Trazamos el centro del circulo
                        cv2.circle(self.img,center,1,(0,100,100),3)
                        #circle outline
                        radius = i[2]
                        # Trazamos el contorno del circulo
                        cv2.circle(self.img,center,radius, (255,0,255),3) 
                    self.colors(self.img)
                    if (cv2.countNonZero(self.frame_thresholdRed) > 50):
                        self.light = "Red"  
                    elif (cv2.countNonZero(self.frame_thresholdGreen) > 50):
                        self.light = "Green"
                    else:
                        self.light = "Other"
                        
                self.light_pub.publish(self.light)
                cv2.imshow('img',self.img)
                cv2.waitKey(0)
            ra.sleep() 

    def callback(self,data):
        ############ Cargamos la imagen ####################
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bandera = 1

    def cleanup(self):
        cv2.destroyAllWindows()

    def colors(self, img):
        flt_img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10,7,21)
                
        ############### La pasamos a formato hsv ###############
        hsv_img = cv2.cvtColor(flt_img, cv2.COLOR_BGR2HSV)

        ################ Reconocemos el rojo y el verde ####################
        self.frame_thresholdGreen = cv2.inRange(hsv_img, (49, 80, 20) , (98, 255, 255))
        self.frame_thresholdRed = cv2.inRange(hsv_img, (0, 50, 50) , (10, 255, 255))
        
        (T,frameRed) = cv2.threshold(self.frame_thresholdRed,127,255,cv2.THRESH_BINARY_INV)
        (T,frameGreen) = cv2.threshold(self.frame_thresholdGreen,127,255,cv2.THRESH_BINARY_INV)
        
        ############### Resaltamos los dos colores #############################
        msk = cv2.bitwise_or(self.frame_thresholdRed,self.frame_thresholdGreen)
        colors_img = cv2.bitwise_and(flt_img,flt_img,mask=msk)
                        
        ###################  ###########################
        ret,self.thresh2 = cv2.threshold(msk,127,255,cv2.THRESH_BINARY_INV)

if __name__ == '__main__':
    rospy.init_node('TrafficLights', anonymous=True)
    TrafficLights()
