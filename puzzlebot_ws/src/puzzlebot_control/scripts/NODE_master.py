#!/usr/bin/env python  

#Codigo para controlar el comportamiento de direccion

import rospy  
from sensor_msgs.msg import LaserScan     
from geometry_msgs.msg import Twist 

#This class will receive a number and an increment and it will publish the   
# result of adding number + increment in a recursive way.  

class MasterClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  

        ###******* INIT PUBLISHERS *******###  
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
        rospy.Subscriber("avoid_obst_vel", Twist, self.avoid_obst_cb) 
        rospy.Subscriber("go2goal_vel", Twist, self.go2goal_cb)
        
        ############ CONSTANTS ################  
        self.velocity = Twist()        
        self.vel_avoid = Twist()
        self.vel_goal = Twist()
        self.nearest_range = 0.0

        range_temp = 0.0
        avoid_temp = 0.0
        goal_temp = 0.0
        
        #********** INIT NODE **********###  
        r = rospy.Rate(1) #1Hz  
        print("Node initialized 1hz") 
        while not rospy.is_shutdown():  
            range_temp = self.nearest_range
            avoid_temp = self.vel_avoid
            goal_temp = self.vel_goal

            if range_temp > 0.6:
                self.velocity = goal_temp
                print("goal")
            #elif (range_temp < 2) and (range_temp > 0.5):
            #    self.velocity = 
            else:
                self.velocity = avoid_temp
                print("avoid")

            self.pub_vel.publish(self.velocity)
            r.sleep()  

    def laser_cb(self,msg):
        self.nearest_range = min(msg.ranges)
    
    def avoid_obst_cb(self, msg):  
        self.vel_avoid = msg
         
    def go2goal_cb(self, msg):  
        self.vel_goal = msg

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.   
        self.velocity.angular.z = 0
        self.velocity.linear.x = 0
        self.pub_vel.publish(self.velocity)

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("master_node", anonymous=True)  
    MasterClass() 
