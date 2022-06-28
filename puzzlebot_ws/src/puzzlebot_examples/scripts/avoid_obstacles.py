#!/usr/bin/env python

#NODO MAESTRO

import rospy, sys
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Master():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        #SUBSCRIBERS
        self.signals_sub = rospy.Subscriber("signal_topic",String,self.signal_cb)
        self.line_sub = rospy.Subscriber("line_topic",Twist, self.line_cb)
        self.traffic_light_sub = rospy.Subscriber("traffic_light_topic",String,self.light_cb)
        
        #PUBLISHERS
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 
        
        #VARIABLES
        self.line_vel = Twist()
        self.vel = Twist()
        self.signal = ""
        self.light = ""
        
        ra = rospy.Rate(10) #10Hz  
        
        while not rospy.is_shutdown():  
          	print("Luz: "+str(self.light)+" Senal: "+str(self.light))
        	if self.light == "Green" or self.light == "Other":
                if self.signal == "no signal"
                    self.vel = self.line_vel
                elif self.signal == "GoAhead":
                    self.vel.linear.x = 0.08
                    self.vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.vel)
                    sleep(1)
                elif self.signal == "NoLimit":
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.0
                elif self.signal == "TurnRight":
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.vel)
                    sleep(0.5)

                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.002
                    self.cmd_vel_pub.publish(self.vel)
                    sleep(1)
                    self.vel = self.line_vel

                elif self.signal == "Stop":
                    self.vel.linear.x = 0.0
                	self.vel.angular.z = 0.0
                    
            elif self.light == "Red":
              	self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(self.vel)
            ra.sleep() 
        
    def signal_cb(self, signal):
    	self.signal = signal
        
    def line_cb(self, vel):
    	self.line_vel = vel.data
      
    def light_cb(self, light):
    	self.light = light
        
    def color_detection(self):
    	pass

    def cleanup(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub_cmd_vel.publish(self.vel)
        

if __name__ == '__main__':
    rospy.init_node('Master_node', anonymous=True)
    Master()                
