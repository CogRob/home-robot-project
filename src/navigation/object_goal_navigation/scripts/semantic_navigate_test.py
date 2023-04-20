#! /usr/bin/env python

import rospy
import actionlib
from object_goal_navigation.msg import objectNavigateAction
from home_robot_msgs.msg import NavTuple
from sensor_msgs.msg import Image
import object_goal_navigation


def create_move_base_action_client():
    semantic_navigate_client = actionlib.SimpleActionClient(
        "/navigation/object_navigate_action_server",
        objectNavigateAction,
    )
    rospy.loginfo("Waiting for server")

    semantic_navigate_client.wait_for_server()
    rospy.loginfo("Server found!")
    return semantic_navigate_client

if __name__ == '__main__':
    rospy.init_node('init')
    rospy.loginfo("Inited")
    move_client = create_move_base_action_client()
    rospy.loginfo("Created client")

    move_goal = object_goal_navigation.msg.objectNavigateGoal()
    print(move_goal)

    rospy.loginfo("Created goal")
    move_client.send_goal_and_wait(move_goal)
    rospy.loginfo("Sent goal")

    rospy.spin()
