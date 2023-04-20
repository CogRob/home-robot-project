#!/usr/bin/env python  
import rospy

import tf2_ros
import geometry_msgs.msg


def handle_amcl_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    print("Received!")
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "amcl_odom"

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    # print(t)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_amclpose_broadcaster')
    rospy.Subscriber('/amcl_pose',
                     geometry_msgs.msg.PoseWithCovarianceStamped,
                     handle_amcl_pose,
                    )
    rospy.spin()