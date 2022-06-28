#!/usr/bin/env python
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
        rospy.on_shutdown(self.cleanup) 
        self.bridge = CvBridge()
        kw = 0.04#0.04#0.001
        #image_topic = "/video_source/raw"  # robot
        image_topic = "/camera/image_raw"  # simulacion
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
        self.vel = Twist()
        ra = rospy.Rate(10) #10Hz  
        self.bandera = 0
        while not rospy.is_shutdown():  
            if self.bandera: 
                ############# Le cambiamos el tamano y escala de grises ####################
                gray = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                img_resized = cv.resize(gray, (720,480), interpolation = cv.INTER_AREA)
                
                ######## Le aplicamos filtros ##############################
                (thresh, im_bw) = cv.threshold(img_resized, 128, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)                
                kernel = np.ones((5,5), np.uint8)
                gaus_img = cv.GaussianBlur(im_bw, (5,5),0)

                #Invert the image to change white to black and vice versa
                image_inv = 255-gaus_img

                #Define kernels for horizontal and vertical lines
                kernel_len = np.array(gaus_img).shape[1]//100
                horizontal_kernel = cv.getStructuringElement(cv.MORPH_RECT, (kernel_len, 1))
                kernel = cv.getStructuringElement(cv.MORPH_RECT, (2, 2))

                #Remove anything that is not a horizontal line
                image_inv2 = cv.erode(image_inv, horizontal_kernel, iterations=3)
                horizontal_lines = cv.dilate(image_inv2, horizontal_kernel, iterations=1) #3

                #Add horizontal and vertical lines to get all lines
                image_vh = horizontal_lines
                image_vh = cv.erode(~image_vh, kernel, iterations=2)
                threshold, image_vh = cv.threshold(image_vh, 128, 255, cv.THRESH_BINARY|cv.THRESH_OTSU)

                # Make a inverted copy of original grayscale image
                org_img_inv = cv.bitwise_not(gaus_img)
                #Apply mask of all lines
                final_image_inv = cv.bitwise_and(org_img_inv, org_img_inv, mask=image_vh)
                #Invert again to get clean image without lines
                f_image = cv.bitwise_not(final_image_inv) 
                
                
                
                ################# Data ##############################
                data = np.array(f_image)
                data2 = data.sum(axis=0)
                #plt.plot(data2)
                #cv.imshow("IMG", img_dilation)
                #cv.waitKey(0)
                #plt.show()
                #cv.waitKey(0)
                #cv.destroyAllWindows()

                datoMin = np.min(data2)
                
                val = np.where(data2==datoMin)[0][0]
                
             
            
                if val > 355 and val < 365:
                    kw = 0.009
                else:
                    kw = 0.009
                
                
                self.vel.linear.x = 0.009
                self.vel.angular.z = kw*(360 - val) # velocidad angular
            
                self.pub_vel.publish(self.vel)
            
                
                #print("velocidad angular: " + str(self.vel.angular.z))
                print("pixel mas oscuro: " + str(val))
                
                #edges = cv.Canny(image=img_dilation,threshold1=100, threshold2=200) # Canny Edge
                
                rectangle = cv.rectangle(f_image, (val-5, 460),(val+5,480),(255,0,0), 2)
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

    def cleanup(self):
        
        vel = Twist()
        print(vel)
        self.pub_vel.publish(vel)
        #cv.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('LineFollower', anonymous=True)
    LineFollower()
