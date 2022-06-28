#!/usr/bin/env python

#Este programa hace que el robot detecte blob de color rojo y verde y determine si debe avanzar o parar
#Puede trabajar con goal.py
import rospy, sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

bridge = CvBridge()
class StopAndGo():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        image_topic = "/camera/image_raw"
        #image_topic = "/camera/image_raw"
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_semaforo = rospy.Publisher('cflag', String, queue_size=10)
        rospy.Subscriber('cflag', String, self.prt_cb)
        self.bandera = 0
        self.vel = Twist()
        
        ra = rospy.Rate(10) #10Hz  
        while not rospy.is_shutdown():  
            #self.pub_semaforo.publish("Hola")
            if self.bandera: 
                ############# Le cambiamos el tamano ####################
                img_resized = cv.resize(self.cv_image, (720,480), interpolation = cv.INTER_AREA)
                
                ######## Le aplicamos un filtro ##############################
                flt_img = cv.fastNlMeansDenoisingColored(img_resized, None, 10, 10,7,21)
                
                ############### La pasamos a formato hsv ###############
                hsv_img = cv.cvtColor(flt_img, cv.COLOR_BGR2HSV)
                
                ################ Reconocemos el rojo y el verde ####################
                frame_thresholdRed = cv.inRange(hsv_img, (0,88, 179), (20, 255, 255))
                frame_thresholdGreen = cv.inRange(hsv_img, (49, 39, 130), (98, 255, 255))
                
                ############### Resaltamos los dos colores #############################
                msk = cv.bitwise_or(frame_thresholdRed,frame_thresholdGreen)
                colors_img = cv.bitwise_and(flt_img,flt_img,mask=msk)
                                
                ################### Deteccion de circulos ###########################
                ret,thresh2 = cv.threshold(msk,127,255,cv.THRESH_BINARY_INV)

                params = cv.SimpleBlobDetector_Params()
                params.filterByArea = True
                params.minArea = 10000
                
                detector = cv.SimpleBlobDetector_create()
                
                ################# Cicrulo rojo, detenemos #####################
                print("antes del if")
                if (cv.countNonZero(frame_thresholdRed)>200):
                    print("ya entre al primer if")
                    keypoints = detector.detect(thresh2)
                    if keypoints != 0:
                        var = "stop"
                        self.pub_semaforo.publish(var)
                        #self.vel.linear.x = 0.0
                        im_with_keypoints = cv.drawKeypoints(thresh2, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                        print("estoy detenido")
                        #cv.imshow("Colores", im_with_keypoints)
                        #cv.waitKey(0)

                ################# Cicrulo verde, avanzamos #####################    
                elif (cv.countNonZero(frame_thresholdGreen)>200):
                    print('segundo if')
                    keypoints = detector.detect(thresh2)
                    if keypoints != 0:
                        im_with_keypoints = cv.drawKeypoints(thresh2, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                        print("estoy avanzando")
                        var = "go"
                        self.pub_semaforo.publish(var)
                        #self.vel.linear.x = 0.1
                        #cv.imshow("Colores", im_with_keypoints)
                        #cv.waitKey(0)
                else:
                    print("no detecto nada")
                    var = "stop"
                    self.pub_semaforo.publish(var)
                    
                self.pub_cmd_vel.publish(self.vel)
            ra.sleep() 
        cv.destroyAllWindows() 

    def callback(self,data):
        ############ Cargamos la imagen ####################
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bandera = 1

    def cleanup(self):
        self.vel.linear.x = 0.0
        self.pub_cmd_vel.publish(self.vel)
        cv.destroyAllWindows()

    def prt_cb(self, msg):
        self.msg = msg.data
        print(self.msg)

if __name__ == '__main__':
    rospy.init_node('StopAndGo', anonymous=True)
    StopAndGo()
