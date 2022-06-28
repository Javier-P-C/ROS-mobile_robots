#!/usr/bin/env python 

#Codigo para hacer un cuadrado con el puzzlebot
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class SquareClass(): 
    def __init__(self): 
        ############ CONSTANTS ################ 
        r = 0.05 # wheel radius [m]
        L = 0.18 # wheel speparation [m]
        self.d = 0 # distance 
        self.theta = 0 # angle
        self.vel = Twist() # Robot's speed
        self.wr = 0
        self.wl = 0
        self.vel.linear.x = 0.3
        self.vel.angular.z = 0.2
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        #********** INIT NODE **********### 
        freq = 20
        rate = rospy.Rate(freq) #20Hz 
        Dt = 1/20.0 # Dt is the time between one calculation and the next one
        print("Node initialized 20hz")
        state = 1
        count = 0
        while not rospy.is_shutdown(): 
            v = r*(self.wr + self.wl)/2
            w = r*(self.wr-self.wl)/L
            self.d = v*Dt+self.d
            self.theta = w*Dt+self.theta
            
            if state == 1:
                self.vel.linear.x = 0.2
                self.vel.angular.z = 0
                if self.d > 1:
                    self.d = 0.0
                    state = 2

            elif state == 2:
                self.vel.linear.x = 0
                self.vel.angular.z = 0.1
                if self.theta > np.pi/2 - 0.1:
                    if count < 3:
                        state = 1
                        count = count +1
                        self.theta = 0.0
                    else :
                        state = 3

            elif state == 3:
                self.vel.linear.x = 0
                self.vel.angular.z = 0 
               
            
            

            self.pub_cmd_vel.publish(self.vel) #publish the number 
    
            rate.sleep() 
    def wl_cb(self, wl): 
        
        self.wl = wl.data
        
    def wr_cb(self, wr): 
  
        self.wr = wr.data
        
    def cleanup(self, ): 
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub_cmd_vel.publish(self.vel)
        print("\nbye bye")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("square", anonymous=True) 
    SquareClass() 
