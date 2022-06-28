#!/usr/bin/env python

#LINE FOLLOWER

import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

bridge = CvBridge()
class LineFollower():
    def __init__(self):
        #rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        kw = 0.04#0.04#0.001
        image_topic = "/video_source/raw"  # robot
        
        #SUBSCRIBERS
        #image_topic = "/camera/image_raw"  # simulacion
        #self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        
        #PUBLISHERS
        self.line_pub = rospy.Publisher("line_topic", Twist, queue_size=1)        
        
        #VARIABLES
        self.vel = Twist()
        ra = rospy.Rate(10) #10Hz  
        self.bandera = 0
        
        while not rospy.is_shutdown():  
            if self.bandera: 
                ############# Le cambiamos el tamano y escala de grises ####################
                gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                img_resizedd = cv.resize(gray, (720,480), interpolation = cv.INTER_AREA)
                (T, img_resized) = cv.threshold(img_resizedd, 100, 255, cv.THRESH_BINARY)
                
                ######## Le aplicamos filtros ##############################
                kernel = np.ones((4,4), np.uint8)
                gaus_img = cv.GaussianBlur(img_resized, (5,5),0)
                img_erosion = cv.erode(gaus_img, kernel, iterations=4) #quita los blancos
                img_dilation = cv.dilate(img_erosion , kernel, iterations=4) #aumenta los blancos
                
                
                imageOut = img_dilation[0:480,200:520]
                ################# Detectamos el punto mas oscuro ##############################
                data = np.array(imageOut)
                data2 = data.sum(axis=0)
                datoMin = np.min(data2)
                val = np.where(data2==datoMin)[0][0]
                
             
                ################### control ############################
                kw = 0.001 #0.0035
                self.vel.linear.x = 0.08
                
                if val > 100 and val < 220:
                    val = 160
                
                ################# publicamos velocidad ###########################
                self.vel.angular.z = kw*(160 - val) # velocidad angular
                self.line_pub.publish(self.vel)
    
                print("pixel mas oscuro: " + str(val))
                ################### para probar y ver las imagenes ####################3
                #rectangle = cv.rectangle(imageOut, (val-5, 460),(val+5,480),(255,0,0), 2)
                #cv.imshow("hola", rectangle)
                #cv.waitKey(0)
                #cv.destroyAllWindows()
                #cv.imshow("hola", edges)


            ra.sleep() 
        cv.destroyAllWindows() 

    def callback(self,data):
        ############ Cargamos la imagen ####################
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bandera = 1

    #def cleanup(self):
        #vel = Twist()
        #self.pub_vel.publish(vel)
        #cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('LineFollower', anonymous=True)
    LineFollower()
