#!/usr/bin/env python  
import rospy  
import numpy as np 
from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist 

# This class implements a simple obstacle avoidance algorithm 
class AvoidObstacleClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  

        ####################### PUBLISEHRS AND SUBSCRIBERS ############################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1) 

        ######################## CONSTANTS AND VARIABLES ##############################   
        self.ranges_list = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        self.theta_T = 0
        self.dist_T = 0

        kv = 0.0005
        kw = 3
        
        v_desired = 0.4 #[m/s] desired speed when there are no obstacles
        kw = 0.3 #Angular speed gain
        self.closest_angle = 0.0 #Angle to the closest object 
        self.closest_range = np.inf #Distance to the closest object 
        vel_msg = Twist() 
        r = rospy.Rate(10) #10Hz is the lidar's frequency  
        print("Node initialized 1hz") 

        ############################### MAIN LOOP ##################################### 
        while not rospy.is_shutdown():  
            range=self.closest_range 
            theta_closest=self.closest_angle  
            
            self.coordenadas()
            self.vector_ref()

            vel_msg.linear.x = kv * self.dist_T
            vel_msg.angular.z = kw * self.theta_T
            
            print("closest object distance: " + str(self.closest_range)) 
            print("theta_closest: " + str(theta_closest)) 
            print("")
            self.cmd_vel_pub.publish(vel_msg) 
            r.sleep()  


    def laser_cb(self, msg):  
        self.ranges_list = msg.ranges
        self.angle_min = float(msg.angle_min)
        #print("self.angle_min ",self.angle_min)
        self.angle_increment = float(msg.angle_increment)
        #print("self.angle_increment",self.angle_increment)
        #print("cantidad: ",len(self.ranges_list))
        ## This function receives a message of type LaserScan and computes the closest object direction and range 
        closest_range = min(msg.ranges) 
        idx = msg.ranges.index(closest_range) 
        closest_angle = msg.angle_min + idx * msg.angle_increment 

        # Limit the angle to [-pi,pi] 
        closest_angle = np.arctan2(np.sin(closest_angle),np.cos(closest_angle)) 
        self.closest_range = closest_range 
        self.closest_angle = closest_angle 
    
    def coordenadas(self):
        angle = self.closest_angle
        range = self.closest_range
        x = range * np.cos(angle)
        y = range * np.sin(angle)
        print("x: ",x)    
        print("y: ",y)

    def vector_ref(self):
        ranges = self.ranges_list
        
        Xi = []
        Yi = []
        angle = 0.0
        idx = 0
        for value in ranges:
            angle = self.angle_min + self.angle_increment*idx
            if (np.isposinf(value)):
                Xi.append(8.0 * np.cos(angle))
                Yi.append(8.0 * np.sin(angle))
            else:
                Xi.append(value * np.cos(angle))
                Yi.append(value * np.sin(angle))
            idx = idx + 1

        Xi_F = float(np.sum(Xi))
        Yi_F = float(np.sum(Yi))

        theta_T = np.arctan2(Yi_F,Xi_F)
        dist_T = np.sqrt(np.power(Xi_F,2) + np.power(Yi_F,2))
        print("theta_T: ",theta_T)
        print("dist_T: ",dist_T)
        
        self.theta_T = theta_T
        self.dist_T = dist_T
         
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.cmd_vel_pub.publish(vel_msg)  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("avoid_obstacle", anonymous=True)  
    AvoidObstacleClass() 
