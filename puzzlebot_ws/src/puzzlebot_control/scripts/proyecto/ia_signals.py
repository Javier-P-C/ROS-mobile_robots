#!/usr/bin/env python3

#SIGNALS

from matplotlib import image
import rospy
from tensorflow.keras.models import load_model
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import numpy as np
import cv2

class Signals2():
    def __init__ (self):
        rospy.on_shutdown(self.cleanup)
        self.bridge_object = CvBridge()
        self.image_received = 0
        rute = "/video_source/raw"
        
        #SUBSCRIBERS
        image_sub = rospy.Subscriber(rute, Image, self.image_cb)

        #PUBLISHERS
        self.signal_pub = rospy.Publisher("signal_topic", String, queue_size=1)
        
        #VARIABLES
        self.detected = False
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image_received:
                self.ia_signal()
            r.sleep()

    def image_cb(self, ros_image):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            self.image_received = 1
        except CvBridgeError as e:
            print("Error",e)

    def ia_signal(self):
      	
        self.detected = False #Bandera auxiliar
      
        # Cortar contorono de un circulo
        img = self.cv_image
        img = img[0:400,640:1240]
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        blur_img = cv2.medianBlur(gray_img,5)
        circles = cv2.HoughCircles(blur_img,cv2.HOUGH_GRADIENT,1,20,param1=70,param2=67,minRadius=0,maxRadius=150)
        #Primer circulo centro en 356,160 y con un radio de 52
        if circles is not None:
            circles = np.uint16(np.around(circles))
            data_circle = circles[0]
            arr = data_circle[0]
            x = arr[0]
            y = arr[1]
            r = arr[2]

            desp = r * 4
            orig_x = x - desp/2
            orig_y = y - desp/2

            final_x = orig_x + desp
            final_y = orig_y + desp
            
            self.CNN_img = img[int(orig_y):int(final_y),int(orig_x):int(final_x)]
            
            # Red Neuronal
            self.RedCNN()
            
            self.detected = True

        stop_sign = cv2.CascadeClassifier(r"/home/neri/Neutron/cascade_stop_sign.xml") #Path del XML
        stop_sign_scaled = stop_sign.detectMultiScale(gray_img, 1.3, 5)
        band = np.size(stop_sign_scaled)
        if band != 0:
            stop_cord = stop_sign_scaled[0]

            stop_orig_x = stop_cord[0] - 25
            stop_final_x = stop_orig_x + 150
            stop_orig_y = stop_cord[1] - 25
            stop_final_y = stop_orig_y + 150

            self.CNN_img = img[stop_orig_y:stop_final_y,stop_orig_x:stop_final_x]

            # Red Neuronal
            self.RedCNN()
            
            self.detected = True
            
        if self.detected == False:
        		self.signal_pub.publish("no signal")

    def RedCNN(self):
        # Implementacion de red neuronal
        modeloCNN2 = load_model(r"/home/neri/puzzlebot_ws/src/twist_practice/scripts/neri/senperf.h5")
        TAMANO_IMG = 72
        neu_img = cv2.resize(self.CNN_img, (TAMANO_IMG,TAMANO_IMG))
        neu_img = cv2.cvtColor(neu_img, cv2.COLOR_BGR2GRAY)
        neu_img = neu_img.reshape(1,TAMANO_IMG, TAMANO_IMG,1)
        resultado = modeloCNN2.predict(neu_img)

        signals = {0: "Stop",
        1:"NoLimit",
        2:"TurnRight",
        3:"GoAhead"}

        print(resultado)
        resultado_str = signals[np.argmax(resultado)]
        print(resultado_str)
        
        self.signal_pub.publish(resultado_str)

    def cleanup(self):
        cv2.destroyAllWindows()
        #pass

if __name__ == "__main__":
    rospy.init_node("image_signal", anonymous=True)
    print("AAAAAAAAAAAA")
    Signals2()
