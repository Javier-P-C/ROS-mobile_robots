#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    #inicializa un nodo de ROS
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    #Se indica el topico al que se va a publicas
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #Determina la velocidad de publicacion
    rate = rospy.Rate(10) # 10hz
    #Se crea un mensaje de tipo Twist
    robot_dir=Twist()
    #Se indica velocidad en x
    robot_dir.linear.x = 1
    #Se publica el mensaje mientras rospy se encuentre activo
    while not rospy.is_shutdown():
        pub.publish(robot_dir)
        rate.sleep()

