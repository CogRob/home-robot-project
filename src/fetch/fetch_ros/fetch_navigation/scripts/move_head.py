#! /usr/bin/env python
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MoveHead:
    def __init__(self):

        self.head_controller_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        
        msg = FollowJointTrajectoryGoal()
        msg.trajectory.header.frame_id = ''
        msg.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        self.joint_msg = msg


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
        point.positions = [0.0, 0.60]
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
