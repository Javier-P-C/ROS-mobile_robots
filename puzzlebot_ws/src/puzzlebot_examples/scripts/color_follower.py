#!/usr/bin/env python  

import rospy  
from std_msgs.msg import Int32   
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

#Recive the radius and center of a detected object and make the robot to move towards it 
#Este codigo trabaja con ball_tracker_from_topic.py
class ColorFollower():  

    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("radius", Int32, self.radius_cb)  
        rospy.Subscriber("center", Point, self.center_cb)  
        ############ CONSTANTS ################  
        self.radius = Int32()
        self.center = Point()
        self.vel=Twist()
        
        kv=0.89
        kw=0.005
        xm = 300 #Centro de la imagen (px)
        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        print("Node initialized 10hz") 
        self.vel=Twist()
        while not rospy.is_shutdown():  
            rad_aux = self.radius.data
            center_aux = self.center
            
            if rad_aux != 0:
                self.vel.linear.x=kv*1/rad_aux
            else:
                self.vel.linear.x = 0
            self.vel.angular.z = kw*(xm - center_aux.x)
            
            self.cmd_vel_pub.publish(self.vel) 
            print(self.vel) 
            r.sleep()  

    def radius_cb(self, radius):  
        self.radius=radius  

    def center_cb(self, center):  
        self.center=center  
         

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.  
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.cmd_vel_pub.publish(self.vel)

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("color_follower", anonymous=True)  
    ColorFollower() 
