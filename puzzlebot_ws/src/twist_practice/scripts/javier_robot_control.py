#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#Class to control robot with commands
class Control(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        self.pub_control = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("string_command", String, self.control_cb) 
        ############ CONSTANTS ################ 
        self.control = Twist() 
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        self.control.angular.z=0;            
        self.control.linear.x=0;
        print("Node initialized 1hz")
        while not rospy.is_shutdown(): 
            self.pub_control.publish(self.control) #publish twist 
            print(self.control)
            r.sleep() 

    def control_cb(self,aux):
        if (aux.data == 'forward'):
            self.forward_cb()
        elif (aux.data == 'back'):
            self.back_cb()
        elif (aux.data == 'left'):
            self.left_cb()
        elif (aux.data == 'right'):
            self.right_cb()
        elif (aux.data == 'stop'):
            self.stop_cb()
        elif (aux.data == 's'):
            self.S_cb()
        return

    def forward_cb(self):
        self.control.angular.z=0;            
        self.control.linear.x=0.3;
        return
        
    def back_cb(self): 
        self.control.angular.z=0;           
        self.control.linear.x=-0.3;
        return

    def left_cb(self):            
        self.control.angular.z=0.2;
        self.control.linear.x=0;
        return

    def right_cb(self):            
        self.control.angular.z=-0.2;
        self.control.linear.x=0;
        return

    def stop_cb(self):            
        self.control.angular.z=0;
        self.control.linear.x=0;
        return

    def S_cb(self):            
        self.control.angular.z=0.5;
        self.control.linear.x=0.5;
        self.pub_control.publish(self.control)
        rospy.sleep(2.)
        self.control.angular.z=-0.5;
        self.control.linear.x=0.5;
        self.pub_control.publish(self.control)
        rospy.sleep(2.)
        self.control.angular.z=0;
        self.control.linear.x=0;
        return
        
    def cleanup(self):
        print("bye bye")
        self.turn.linear.x=0;
        self.turn.linear.y=0;
        self.turn.linear.z=0;
        self.turn.angular.x=0;
        self.turn.angular.y=0;
        self.turn.angular.z=0;
        self.pub_turn.publish(self.turn)
        return 
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("turn_node", anonymous=True) 
    Control()
