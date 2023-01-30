#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class diff_drive_remapper:
    def __init__(self):
        self.pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, callback=self.callback)
    
    def callback(self, data):
        self.pub.publish(data)


if __name__ == '__main__':
    ddr = diff_drive_remapper()
    rospy.init_node('diff_drive_remapper', anonymous=True)
    print "diff drive remapper start!"
    rospy.spin()