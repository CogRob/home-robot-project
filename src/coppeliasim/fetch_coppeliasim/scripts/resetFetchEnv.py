#!/usr/bin/env python
# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import numpy as np
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

    import time

def moveObjectTo(clientID, objectname, targetposition, targetquaternion):
    ret, objecthandle = sim.simxGetObjectHandle(clientID, objectname, sim.simx_opmode_oneshot_wait)
    if ret == sim.simx_return_ok:
        _, position = sim.simxGetObjectPosition(clientID, objecthandle, -1, sim.simx_opmode_blocking)
        _, quaternion = sim.simxGetObjectQuaternion(clientID, objecthandle, -1, sim.simx_opmode_blocking)
        print "object", objectname, " quaternion ", quaternion
        while not np.isclose(targetposition, position).all():
            sim.simxSetObjectPosition(clientID, objecthandle, -1, targetposition, sim.simx_opmode_oneshot)
            sim.simxSetObjectQuaternion(clientID, objecthandle, -1, targetquaternion, sim.simx_opmode_oneshot)
            _, position = sim.simxGetObjectPosition(clientID, objecthandle, -1, sim.simx_opmode_blocking)
            _, quaternion = sim.simxGetObjectQuaternion(clientID, objecthandle, -1, sim.simx_opmode_blocking)
        
    else:
        print ("no such object in the sim!!")

class Arm:
    def __init__(self):
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        self.client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.init_position = [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rospy.loginfo('Waiting for joint trajectory action')    
        self.client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def reset(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # point.velocities.append(0.1)
        point.positions = self.init_position
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

    def move(self, joints):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

class Torso:
    def __init__(self):
        self.joint_names = ['torso_lift_joint']
        self.joint_value = [0.3]
        self.client = actionlib.SimpleActionClient("/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        rospy.loginfo('Waiting for joint trajectory action')    
        self.client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        
    def reset(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # point.velocities.append(0.1)
        point.positions = self.joint_value
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

class Gripper:
    def __init__(self):
        # self.traj = JointTrajectory()
        # self.traj.joint_names = ['r_gripper_finger_joint']
        # self.gripper_publisher = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=5)
        self.joint_names = ['r_gripper_finger_joint']
        self.open_joints = [0.04]
        self.close_joints = [-0.04]
        self.client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        rospy.loginfo('Waiting for joint trajectory action')    
        self.client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def open(self):

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.open_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)
    
    def close(self):

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.close_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

def resetEnv(clientID, arm, gripper, torso):
    
    moveObjectTo(clientID, "bottle_respondable", [-1.1507, 0.17481, 0.65546], [0.0, 0.0, 0.0, 1.0])
    
    # arm.reset()
    # gripper.open()


if __name__ == "__main__":
    rospy.init_node('reset_fetch_env_py')
    arm = Arm()
    gripper = Gripper()
    torso = Torso()

    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        resetEnv(clientID, arm , gripper, torso)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')