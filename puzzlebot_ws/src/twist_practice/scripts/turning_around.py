#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#This class will receive a number and an increment and it will publish the  
# result of adding number + increment in a recursive way. 
class TurningAround(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.pub_turn = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("direction", String, self.turn_cb) 
        ############ CONSTANTS ################ 
        self.turn = Twist() #The number to be use in the sum 
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        print("Node initialized 1hz")
        while not rospy.is_shutdown(): 
            self.turn.linear.x=0.3;
            self.turn.linear.z=0;
            self.pub_turn.publish(self.turn) #publish twist 
            print(self.turn)
            r.sleep() 
    def turn_cb(self, aux): 
        ## This function receives a number  
        if (aux.data == 'turn'):
            self.turn.linear.x=0;
            self.turn.angular.z=1;
            self.pub_turn.publish(self.turn)
            rospy.sleep(3.)
            self.turn.linear.x=0.3;
            self.turn.angular.z=0;
            self.pub_turn.publish(self.turn)
        return
        
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        print("bye bye")
        self.turn.linear.x=0
        self.turn.angular.z=0;
        self.pub_turn.publish(self.turn)
        return 
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("turn_node", anonymous=True) 
    TurningAround()
