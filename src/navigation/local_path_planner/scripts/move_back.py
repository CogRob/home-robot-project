#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_x_secs(secs):
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(2.0)
    move=Twist()
    move.linear.x = -0.5
    for i in range(10):
        print("Publishing move : ", move)
        pub.publish(move)
        rospy.sleep(0.5)



if __name__ == "__main__":

    rospy.init_node('lastforward')
    rate=rospy.Rate(4)
    

    
    move_x_secs(50)
    rate.sleep()
    rospy.spin()