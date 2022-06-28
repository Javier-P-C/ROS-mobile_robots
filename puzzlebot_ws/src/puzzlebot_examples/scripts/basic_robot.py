#!/usr/bin/env python  
#MUeve el robot al punto más cercano detectado con el lidar por medio de un control con una k variable
import rospy  
import numpy as np
import math

from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist

# This class receives a LaserScan and finds the closest object  
class ClosestDetectorClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
 
        ############ CONSTANTS ################  
        self.closest_range = 0.0
        self.closest_angle = 0.0
        kmax = 0.89012
        alpha = 1
        kw = 1.0
        kv = 0
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        vel_msg = Twist()
     
        #********** INIT NODE **********###  
        r = rospy.Rate(10) #10Hz is the lidar frequency
        print("Node initialized 10hz") 
        while not rospy.is_shutdown():  
            range_ = self.closest_range
            theta = self.closest_angle
            theta = np.arctan2(np.sin(theta),np.cos(theta))
            if np.isinf(range_):
                print("No object detected")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
            else:
                if self.closest_range != 0:
                    kv = kmax*(1-math.exp(-alpha*pow(abs(self.closest_range),2)))/abs(self.closest_range)
                if self.closest_range >= 0.2:
                    vel_msg.linear.x = kv*range_ 
                else:
                    vel_msg.linear.x = 0.0
                vel_msg.angular.z = kw*theta #rad/s
            self.cmd_vel_pub.publish(vel_msg)
            r.sleep()  


    def laser_cb(self, msg):  
        ## This function receives a number   
        #For hls lidar  
        self.closest_range = min(msg.ranges) 
        idx = msg.ranges.index(self.closest_range) 
        self.closest_angle = msg.angle_min + idx * msg.angle_increment 
        print("closest object distance: " + str(self.closest_range)) 
        print("closest object direction: " + str(self.closest_angle)) 
         

    def cleanup(self):  
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("closest_detector", anonymous=True)
    ClosestDetectorClass() 
