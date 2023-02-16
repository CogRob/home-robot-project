#! /usr/bin/env python

import rospy
import actionlib
import move_robot_base.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import sin, cos


class GoToPositionRobotBaseAction(object):
    # create messages that are used to publish feedback/result
    _result = move_robot_base.msg.FibonacciResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("move_robot_base", local_path_planner.msg.move_robot_baseActionGoal, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()


    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        x, y, theta = goal.pose.x, goal.pose.y, goal.pose.theta

        # Can execute some dynamic motion planning here and feed smaller waypoints in a loop here

        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = "map"
        move_goal.target_pose.header.stamp = rospy.Time.now()        
        
        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        success = True

        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)