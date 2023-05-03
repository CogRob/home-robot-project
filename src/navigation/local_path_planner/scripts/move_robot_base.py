#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from local_path_planner.msg import moveRobotBaseAction, moveRobotBaseActionGoal, moveRobotBaseResult
from math import sin, cos, sqrt
import tf2_ros


class GoToPositionRobotBaseAction(object):
    # create messages that are used to publish feedback/result
    

    def __init__(self):
        self._as = actionlib.SimpleActionServer("move_fetch_robot_base", moveRobotBaseAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("Got move base!")
        self.result = moveRobotBaseResult()
        rospy.loginfo("Got base")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()

        rospy.spin()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        x, y, theta = goal.pose.x, goal.pose.y, goal.pose.theta
        rospy.loginfo("Received goal!")
        print(self.result)
        # Can execute some dynamic motion planning here and feed smaller waypoints in a loop here

        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = "map"
        move_goal.target_pose.header.stamp = rospy.Time.now()        
        
        rospy.loginfo("Sent goal")
        self.client.send_goal(move_goal)
        self.client.wait_for_result()
        rospy.loginfo("Waited!")
        result = self.client.get_result()

        # count = 0
        # while True:
        #     count += 0
        #     rospy.sleep(2.0)
        #     transform_se3 = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        #     position = [transform_se3.transform.translation.x, transform_se3.transform.translation.y]
        #     print(position)
        #     diff = (position[0] - x) ** 2 + (position[1] - y) ** 2
        #     print(position, diff)
        #     if diff < 1.0:
        #         success = True
        #         break
        #     if count == 50:
        #         break

        success = True

        if success:
            print(dir(self.result))
            self.result.success = True
            rospy.loginfo('Goto position: Succeeded')
            self._as.set_succeeded(self.result)

def create_move_base_action_client():
    move_fetch_base_client = actionlib.SimpleActionClient(
        "move_fetch_robot_base",
        moveRobotBaseAction,
    )

    move_fetch_base_client.wait_for_server()
    return move_fetch_base_client


if __name__ == '__main__':
    rospy.init_node("fetch_move_base_node")
    action_server = GoToPositionRobotBaseAction()
