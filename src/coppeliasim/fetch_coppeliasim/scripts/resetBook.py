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
import time

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
        self.init_position = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

    # def getFingerValue(self):
    #     if not rospy.is_shutdown():
    #         js = rospy.wait_for_message('joint_states', JointState)
    #         # get index of finger
    #         index = js.name.index('r_gripper_finger_joint')
    #         return js.position[index]
    #     else:
    #         return None

    def open(self):
        # pt = JointTrajectoryPoint()
        # pt.positions = [0.04]
        # pt.time_from_start = rospy.Duration(1.0)
        # self.traj.points = [pt]

        # r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     self.traj.header.stamp = rospy.Time.now()
        #     self.gripper_publisher.publish(self.traj)
        #     r.sleep()
        #     if self.getFingerValue() > 0.036:
        #         break
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.open_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)
    
    def close(self):
        # pt = JointTrajectoryPoint()
        # pt.positions = [-0.04]
        # pt.time_from_start = rospy.Duration(1.0)
        # self.traj.points = [pt]

        # r = rospy.Rate(10)
        # last_value = self.getFingerValue()
        # stablenum = 0
        # while not rospy.is_shutdown():
        #     self.traj.header.stamp = rospy.Time.now()
        #     self.gripper_publisher.publish(self.traj)
        #     r.sleep()
        #     # print "error ", abs(last_value - self.getFingerValue()) 
        #     if abs(last_value - self.getFingerValue()) < 0.0001:
        #         stablenum+=1
        #     else:
        #         stablenum = 0
        #     if stablenum == 5 or abs(pt.positions[0] - self.getFingerValue()) < 0.0001:
        #         break
        #     last_value = self.getFingerValue()
        # return last_value
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.close_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

def resetEnv(clientID, arm, gripper, torso):

    torso.reset()
    # move the robot (need to search how to do)
    # moveObjectTo(clientID, "base_link_respondable", [-0.011241, -0.65001, 0.18444], [-9.568027599016204e-05, -0.7104805111885071, 0.003217362565919757, 0.7037094831466675])

    # move table away
    moveObjectTo(clientID, "Table", [1.526, -1.3749, 0.3944], [0.5,0.5,0.5,-0.5]) # it was [-0.025, 0.5, 0.35]
    moveObjectTo(clientID, "cup", [0.7768, 2.093, 0.8338], [0.5, 0.5, 0.5, 0.5]) # it was [0.021, 0.463, 0.7775]
    moveObjectTo(clientID, "book", [0.65, 2.093, 0.8338], [0.5, 0.5, 0.5, 0.5])
    moveObjectTo(clientID, "almonds_can", [0.4, 2.093, 0.8338], [0.5, 0.5, 0.5, 0.5])

    arm.move([-0.39, -0.2794, 0.5112, 2.11, -0.14669, -1.5567, -0.51277])
    
    # arm.reset()
    gripper.open()

    moveObjectTo(clientID, "Table", [0.77, 0.075, 0.3946], [0.5,0.5,0.5,-0.5]) # it was [-0.025, 0.5, 0.35]
    moveObjectTo(clientID, "book", [0.835, -0.01, 0.87], [0.0, 0.707, 0.0, 0.707]) 
    # moveObjectTo(clientID, "almonds_can", [0.79, 0.2, 0.97], [0.5, 0.5, 0.5, 0.5]) 

    gripper.close()

    time.sleep(1.0)

    arm.move([-0.45, -0.44, 0.645, 2.17, -0.0475, -1.444, -0.5747])


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