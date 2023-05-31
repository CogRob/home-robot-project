#! /usr/bin/env python
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import actionlib
from local_path_planner.msg import moveHeadAction, moveHeadActionGoal, moveHeadResult


class MoveHead:
    def __init__(self):

        self.head_controller_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        self._as = actionlib.SimpleActionServer("move_fetch_head", moveHeadAction, execute_cb=self.move_head_cb, auto_start = False)
        self._as.start()

        msg = FollowJointTrajectoryGoal()
        msg.trajectory.header.frame_id = ''
        msg.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        self.joint_msg = msg
        self.result = moveHeadResult()

    def move_head_cb(self, goal):

        point = JointTrajectoryPoint()
        point.positions = goal.joint_values
        point.time_from_start = rospy.Duration(2.0)

        self.joint_msg.trajectory.header.stamp = rospy.Time.now()
        self.joint_msg.trajectory.points = []
        self.joint_msg.trajectory.points.append(point)

        self.head_controller_client.send_goal_and_wait(
            self.joint_msg, execute_timeout=rospy.Duration(3.0)
        )
        rospy.sleep(1.0)
        self.result.success = True
        self._as.set_succeeded(self.result)

    def move_right(self):

        point = JointTrajectoryPoint()
        point.positions = [-1.30, 0.7250]
        point.time_from_start = rospy.Duration(5)

        self.joint_msg.trajectory.header.stamp = rospy.Time.now()
        self.joint_msg.trajectory.points = []
        self.joint_msg.trajectory.points.append(point)

        self.head_controller_client.send_goal_and_wait(
            self.joint_msg, execute_timeout=rospy.Duration(5.0)
        )

    def move_left(self):

        point = JointTrajectoryPoint()
        point.positions = [1.50, 0.75]
        point.time_from_start = rospy.Duration(5)

        self.joint_msg.trajectory.header.stamp = rospy.Time.now()
        self.joint_msg.trajectory.points = []
        self.joint_msg.trajectory.points.append(point)

        self.head_controller_client.send_goal_and_wait(
            self.joint_msg, execute_timeout=rospy.Duration(5.0)
        )


    def move_center(self):

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.80]
        point.time_from_start = rospy.Duration(5)

        self.joint_msg.trajectory.header.stamp = rospy.Time.now()
        self.joint_msg.trajectory.points = []
        self.joint_msg.trajectory.points.append(point)

        self.head_controller_client.send_goal_and_wait(
            self.joint_msg, execute_timeout=rospy.Duration(5.0)
        )




if __name__ == '__main__':
    rospy.init_node('TempNode')
    head_mover = MoveHead()
    rospy.sleep(2.0)
    # head_mover.move_right()
    # rospy.sleep(2.0)
    # head_mover.move_left()
    # rospy.sleep(5.0)
    head_mover.move_center()
    rospy.spin()
