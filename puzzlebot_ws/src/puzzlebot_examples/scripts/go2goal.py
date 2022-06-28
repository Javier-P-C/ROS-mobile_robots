#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
from std_msgs.msg import String
import math

class PositionClass(): 
    def __init__(self): 
        ############ CONSTANTS ################ 
        rospy.on_shutdown(self.cleanup)   
      
        r = 0.05 # wheel radius [m]
        L = 0.18 # wheel speparation [m]
        self.d = 0 # distance 
        self.theta = 0 # angle
        self.vel = Twist() # Robot's speed
        self.wr = 0
        self.wl = 0
        
        ###*** INIT PUBLISHERS ***### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        self.pubx = rospy.Publisher('x_inst', String, queue_size=10)
        self.puby = rospy.Publisher('y_inst', String, queue_size=10)
        self.pub_theta = rospy.Publisher('theta_inst', String, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        #**** INIT NODE ****### 
        freq = 20
        rate = rospy.Rate(freq) #20Hz 
        
        self.theta_inst=0.0 #Theta actual
        self.x_inst=0.0 #X actual
        self.y_inst=0.0 #y actual

        self.theta_target= 0
        self.x_target= -0.8
        self.y_target= 0

        self.velocity=Twist()
        self.vel_k=0.12
        self.omega_k=0.3
    
        Dt = 1/20.0 # Dt is the time between one calculation and the next one

        while not rospy.is_shutdown():
            self.theta_inst=self.theta_inst+(r*(self.wr-self.wl)/L)*Dt
            if (self.theta_inst > np.pi*2):
                self.theta_inst=0
            elif (self.theta_inst < -np.pi*2):
                self.theta_inst=0
            
            self.x_inst=self.x_inst+(r*(self.wr+self.wl)/2)*Dt*np.cos(self.theta_inst)
            self.y_inst=self.y_inst+(r*(self.wr+self.wl)/2)*Dt*np.sin(self.theta_inst)
            
            self.e_theta=np.arctan2(self.y_target-self.y_inst,self.x_target-self.x_inst)-self.theta_inst #Error en theta
            self.e_dist=np.sqrt(math.pow(self.x_target-self.x_inst,2)+math.pow(self.y_target-self.y_inst,2))
            
            print("")
            print("x:",self.x_inst)
            print("y:",self.y_inst)
            print("theta:",self.theta_inst)
            print("e_theta:",self.e_theta)
            print("e_dist:",self.e_dist)

            ###CONTROL
            if (self.e_theta < 0.006) and (self.e_theta > -0.006):
                self.velocity.linear.x = self.vel_k*self.e_dist
                self.velocity.angular.z = 0
            else:
                self.velocity.angular.z = self.omega_k*self.e_theta
                self.velocity.linear.x = 0 
            if (self.e_theta < 0.06) and (self.e_theta > -0.06) and (self.e_dist < 0.005) and (self.e_dist > -0.005):
                self.velocity.angular.z = 0
                self.velocity.linear.x = 0 

            self.pub_cmd_vel.publish(self.velocity)
            rate.sleep() 

    def wl_cb(self, wl): 
        
        self.wl = wl.data
        
    def wr_cb(self, wr): 
  
        self.wr = wr.data
        
    def cleanup(self, ): 
        self.velocity.linear.x=0
        self.velocity.angular.z=0
        self.pub_cmd_vel.publish(self.velocity)
        print("\nbye bye")

if __name__ == "__main__": 
    rospy.init_node("position", anonymous=True) 
    PositionClass()
