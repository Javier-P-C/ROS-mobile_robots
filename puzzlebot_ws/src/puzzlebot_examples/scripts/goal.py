#!/usr/bin/env python 

#Codigo para llegar a dos puntos deseaados

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
      
        self.r = 0.05 # wheel radius [m]
        self.L = 0.18 # wheel speparation [m]
        self.d = 0 # distance 
        self.theta = 0 # angle
        self.vel = Twist() # Robot's speed
        self.omega_k=2.1 ################################################ kkkkkkkkkkkkkkkkkkkwwwwwwwwwwwwww
        self.wr = 0
        self.wl = 0
        self.a = 1
        self.k_max = 0.89
        self.cflag = ""
        ###*** INIT PUBLISHERS ***### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        self.pubx = rospy.Publisher('x_inst', String, queue_size=10)
        self.puby = rospy.Publisher('y_inst', String, queue_size=10)
        self.pub_theta = rospy.Publisher('theta_inst', String, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("wl", Float32, self.wl_cb) 
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        rospy.Subscriber("cflag", String, self.semaforo_cb)

        #**** INIT NODE ****### 
        freq = 20
        rate = rospy.Rate(freq) #20Hz 
        
        self.theta_inst=0.0 #Theta actual
        self.x_inst=0.0 #X actual
        self.y_inst=0.0 #y actual

        self.theta_target= 0
        #self.x_target= -0.8
        #self.y_target= 0

        self.velocity=Twist()
        self.vel_k=0.12 ########################### Kkkkkkkkkkkkkkkkkkkkkkkkkkkkvvvvvvvvvvvvvvvv
        
    
        self.Dt = 1/20.0 # Dt is the time between one calculation and the next one

        self.flag = 0

        while not rospy.is_shutdown():
            while not self.flag == 1 :
                if self.cflag == "go":
                    self.go_to_goal(x_target=1,y_target=1)
                elif self.cflag == "stop":
                    self.velocity.angular.z = 0
                    self.velocity.linear.x = 0
                    self.pub_cmd_vel.publish(self.velocity)
                rate.sleep()
            self.flag = 0
            while not self.flag == 1 :
                if self.cflag == "go":
                    self.go_to_goal(x_target=0,y_target=1)
                elif self.cflag == "stop":
                    self.velocity.angular.z = 0
                    self.velocity.linear.x = 0
                    self.pub_cmd_vel.publish(self.velocity)
                rate.sleep()
            print('DONE')
            #rospy.signal_shutdown('se apago')
            #while not self.e_dist < 0.005:
            #    self.go_to_goal(x_target=-1,y_target=-1)


    def wl_cb(self, wl): 
        
        self.wl = wl.data
        
    def wr_cb(self, wr): 
  
        self.wr = wr.data
        
    def semaforo_cb(self, cflag):
        
        self.cflag = cflag.data

    def go_to_goal(self, x_target, y_target):
        self.theta_inst=self.theta_inst+(self.r*(self.wr-self.wl)/self.L)*self.Dt
        if (self.theta_inst > np.pi*2):
            self.theta_inst=0
        elif (self.theta_inst < -np.pi*2):
            self.theta_inst=0
        
        self.x_inst=self.x_inst+(self.r*(self.wr+self.wl)/2)*self.Dt*np.cos(self.theta_inst)
        self.y_inst=self.y_inst+(self.r*(self.wr+self.wl)/2)*self.Dt*np.sin(self.theta_inst)
        
        self.e_theta=np.arctan2(y_target-self.y_inst,x_target-self.x_inst)-self.theta_inst #Error en theta
        self.e_theta = np.arctan2(np.sin(self.e_theta),np.cos(self.e_theta))
        self.e_dist=np.sqrt(math.pow(x_target-self.x_inst,2)+math.pow(y_target-self.y_inst,2))
        
        print("")
        print("x:",self.x_inst)
        print("y:",self.y_inst)
        print("theta:",self.theta_inst)
        print("e_theta:",self.e_theta)
        print("e_dist:",self.e_dist)

        ###CONTROL
        if (self.e_theta < 0.6) and (self.e_theta > -0.6):
            self.vel.linear.x = self.vel_k*self.e_dist*2
            self.vel.angular.z = 0.0

            if(self.e_theta < 0.2) or (self.e_theta < -0.2):
                if(self.e_dist < 0.2) or (self.e_dist < -0.2):
                    self.vel = Twist()
                    self.pub_cmd_vel.publish(self.vel)
                    print("DONE BITCH")
                    self.flag = 1
                    #exit()
            
        else:
            self.vel.angular.z = self.omega_k*self.e_theta
            self.vel.linear.x = 0.0
        self.pub_cmd_vel.publish(self.vel)
        
        
    def cleanup(self): 
        self.velocity.linear.x=0
        self.velocity.angular.z=0
        self.pub_cmd_vel.publish(self.velocity)
        print("\nbye bye")

if __name__ == "__main__": 
    rospy.init_node("position", anonymous=True) 
    PositionClass()
