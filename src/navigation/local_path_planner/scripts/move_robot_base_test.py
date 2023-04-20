#! /usr/bin/env python

import rospy
import actionlib
from local_path_planner.msg import moveRobotBaseAction, moveRobotBaseActionGoal, moveRobotBaseActionResult
from geometry_msgs.msg import Pose2D
import local_path_planner

def create_move_base_action_client():
    move_fetch_base_client = actionlib.SimpleActionClient(
        "move_fetch_robot_base",
        moveRobotBaseAction,
    )
    rospy.loginfo("Waiting for server")

    move_fetch_base_client.wait_for_server()
    rospy.loginfo("Server found!")
    return move_fetch_base_client

if __name__ == '__main__':
    rospy.init_node('init')
    rospy.loginfo("Inited")
    move_client = create_move_base_action_client()
    rospy.loginfo("Created client")
    move_goal = local_path_planner.msg.moveRobotBaseGoal(
        pose = Pose2D(8, -3.0, 0.0)
    )
    rospy.loginfo("Created goal")
    move_client.send_goal_and_wait(move_goal)
    rospy.loginfo("Sent goal")
    # print(dir(move_goal))
    # done
    # move_goal.pose = 
    rospy.spin()
