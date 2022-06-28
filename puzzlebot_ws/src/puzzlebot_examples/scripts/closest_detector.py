#!/usr/bin/env python  

#Avanza al punto más cercano identificado por el lidar

import rospy  
from sensor_msgs.msg import LaserScan   

#This class will receive a number and an increment and it will publish the   
# result of adding number + increment in a recursive way.  
class CloserDetectorClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)    
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)  
        ############ CONSTANTS ################  

        #********** INIT NODE **********###  
        r = rospy.Rate(1) #1Hz  
        print("Node initialized 1hz") 
        while not rospy.is_shutdown():  
            
            r.sleep()  

    def laser_cb(self, msg):  
        ## This function receives a number   

        closest_range= min(msg.ranges)
        idx = msg.ranges.index(closest_range)
        closest_angle = msg.angle_min + idx * msg.angle_increment
        print("closest range: " + str(closest_range))
        print("closest angle: " + str(closest_angle))

        #print(msg.angle_increment)    
        #print(msg.angle_min)      
        #print(msg.range_min)   
        print("")
         

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    rospy.init_node("closer_detector_node", anonymous=True)  
    CloserDetectorClass() 
