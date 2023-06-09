#! /usr/bin/env python

import rospy
import actionlib
from manipulation.msg import PickupAction, PlaceAction, PickupResult, PlaceResult, SetJointsToActuateAction, SetJointsToActuateResult
import random

# from coppeliasim_zmq.srv import AttachObjectToGripper, DetachObjectToGripper
from object_detector.srv import detect2DObject, detect2DObjectRequest

import tf2_ros
from ros_tensorflow_msgs.srv import *
from rail_segmentation.srv import *
from rail_manipulation_msgs.srv import *
from geometry_msgs.msg import TransformStamped, PoseStamped, TwistStamped, Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive

from sensor_msgs import point_cloud2

from sensor_msgs.msg import CameraInfo, JointState
import tf
from ros_numpy import numpify, msgify
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

import sys
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
import cv2
import math

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal,
)
def get_zmq_clients():

    rospy.wait_for_service("attach_object_to_gripper_service")
    attach_object_to_gripper_service_client = rospy.ServiceProxy(
        "attach_object_to_gripper_service", AttachObjectToGripper
    )

    rospy.wait_for_service("detach_object_to_gripper_service")
    detach_object_to_gripper_service_client = rospy.ServiceProxy(
        "detach_object_to_gripper_service", DetachObjectToGripper
    )

    return attach_object_to_gripper_service_client, detach_object_to_gripper_service_client


class Manipulation(object):
    def __init__(self, isSim = False):

        # init moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander("arm") 
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        rospy.wait_for_service("/compute_ik")
        self.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)

        print ">>> moveit interface: READY "

        # initialize the display trajectory publisher
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )
    
        # initialize the object detectiont client
        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()
        print ">>> object detection: READY "

        # start all pick and place servers
        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupAction, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        self.place_as = actionlib.SimpleActionServer("place_server", PlaceAction, execute_cb=self.place_cb, auto_start = False)
        self.place_as.start()
        self.prepare_manip_as = actionlib.SimpleActionServer("prepare_manipulation_joints", SetJointsToActuateAction, execute_cb=self.prepare_manip_cb, auto_start = False)
        self.prepare_manip_as.start()

        print ">>> Servers: READY "

        # target object pointcloud publisher
        self.target_object_pc_publisher = rospy.Publisher("/target_object_point_cloud", PointCloud2, queue_size=2)

        self._sim = isSim 
        if self._sim == True:
            self.gripper_client = actionlib.SimpleActionClient(
                "/gripper_controller/follow_joint_trajectory",
                FollowJointTrajectoryAction,
            )
        else:
            self.gripper_client = actionlib.SimpleActionClient(
                "/gripper_controller/gripper_action", GripperCommandAction
            )

        self.gripper_client.wait_for_server()
        print ">>> Gripper controller: READY "

        if isSim:
            self.attach_client, self.detach_client = get_zmq_clients()

        # prepare torso and head controller
        self.torso_controller_client = actionlib.SimpleActionClient(
            "/torso_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.head_controller_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        # init table searcher
        rospy.wait_for_service('table_searcher/search_table')
        self.table_searcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

        # init table-top object searcher
        rospy.wait_for_service('table_searcher/segment_objects')
        self.object_searcher = rospy.ServiceProxy('table_searcher/segment_objects', SegmentObjects)

        # init grasp predictor
        rospy.wait_for_service('grasp_predict')
        self.grasp_predictor = rospy.ServiceProxy('grasp_predict', Predict)
        print ">>> object, table, and grasp detector: READY"
        
        # get camera transform from tf tree
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.in_hand_object_name = None

        # Constant variables
        self.default_joints = [-1.3089969389957472, -0.08726646259971647, -2.897246558310587, 1.3962634015954636, -1.8151424220741028, 1.8151424220741028, 1.1868238913561442]
        self.grasp_shift = np.array([[1,0,0,0.02],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.pre_grasp_shift = np.array([[1,0,0,-0.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.pick_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.05],[0,0,0,1]])
        self.max_attempt_count = 25

        ## need to get the camera matrix
        camera_info = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
        self.camera_intrinsic_matrix = np.reshape(camera_info.K, (3, 3))

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1.0)
        print ">>> Everything should be ready! <<<"
        rospy.spin()

    # def move_back(self):
    #     move = Twist()
    #     move.linear.x = -0.5
    #     for _ in range(5):
    #         self.cmd_publisher.publish(move)
    #         rospy.sleep(0.5)

    def setGripperWidth(self, pos):
        if self._sim == True:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['r_gripper_finger_joint'] 
            point = JointTrajectoryPoint()
            point.positions = [pos - 0.04]
            point.time_from_start = rospy.Duration(1)
            goal.trajectory.points.append(point)
            self.gripper_client.send_goal_and_wait(
                goal, execute_timeout=rospy.Duration(2.0)
            )
        else:
            goal = GripperCommandGoal()
            goal.command.position = float(pos)
            goal.command.max_effort = 100
            self.gripper_client.send_goal_and_wait(goal)

    def openGripper(self):
        self.setGripperWidth(0.08)

    def closeGripper(self, width=0.0):
        self.setGripperWidth(width)


    def prepare_manip_cb(self, request):
        joint_names = ['shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'upperarm_roll_joint',
                        'elbow_flex_joint',
                        'forearm_roll_joint',
                        'wrist_flex_joint',
                        'wrist_roll_joint']
        joint_values = [-1.3089969389957472, -0.08726646259971647, -2.897246558310587, 1.3962634015954636, -1.8151424220741028, 1.8151424220741028, 1.1868238913561442]
        current_values = self.move_group.get_current_joint_values()
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.go()

        msg = FollowJointTrajectoryGoal()
        msg.trajectory.header.frame_id = ''
        msg.trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.35]
        point.time_from_start = rospy.Duration(1.0)

        msg.trajectory.header.stamp = rospy.Time.now()
        msg.trajectory.points = []
        msg.trajectory.points.append(point)

        self.torso_controller_client.send_goal_and_wait(
            msg, execute_timeout=rospy.Duration(2.0)
        )

        rospy.sleep(3.0)


        msg = FollowJointTrajectoryGoal()
        msg.trajectory.header.frame_id = ''
        msg.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.60]
        point.time_from_start = rospy.Duration(1.0)

        msg.trajectory.header.stamp = rospy.Time.now()
        msg.trajectory.points = []
        msg.trajectory.points.append(point)

        self.head_controller_client.send_goal_and_wait(
            msg, execute_timeout=rospy.Duration(2.0)
        )
        rospy.sleep(3.0)

        self.prepare_manip_as.set_succeeded(SetJointsToActuateResult(success=True))

    def return_failure(self, fail_message):
        self.scene.clear()
        rospy.loginfo("ERROR: " + fail_message)
        pickup_as_result = PickupResult()
        pickup_as_result.success = False
        self.pickup_as.set_aborted(pickup_as_result)

    def pickup_cb(self, request):

        self.scene.clear()

        object_id = request.object_id
        print "Receive object id = ", object_id

        ## call perception
        detections = self.object_detector_client()

        has_object_in_view = False
        for detection in detections.detections.detections:
            if detection.object_id == object_id:
                has_object_in_view = True
                break
        
        if not has_object_in_view:
            self.return_failure("The detected objects from YOLO does not contain the object id you passed!!!")
            return

        print " --- The Yolo found the object from camera view."
        
        # get the bounding box.
        center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y
        # size_x *= 1.2
        # size_y *= 1.2
        boundary = [center_x - size_x / 2, center_x + size_x / 2, center_y - size_y / 2, center_y + size_y / 2]

        # find and add the table into the planning scene
        table_info = self.table_searcher()
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.orientation.x = table_info.orientation.x
        table_pose.pose.orientation.y = table_info.orientation.y
        table_pose.pose.orientation.z = table_info.orientation.z
        table_pose.pose.orientation.w = table_info.orientation.w
        table_pose.pose.position.x = table_info.center.x
        table_pose.pose.position.y = table_info.center.y
        table_pose.pose.position.z = table_info.center.z / 2
        table_name = "table"

        # convert the table to perpendicular pose.
        oritinal_table_pose_mat = numpify(table_pose.pose)
        oritinal_table_pose_mat[0, 0] = 0
        oritinal_table_pose_mat[1, 0] = 0
        if np.linalg.norm(oritinal_table_pose_mat[:3, 0]) == 0:
            oritinal_table_pose_mat[:3, 0] = np.array([0,1,0])
        else:
            oritinal_table_pose_mat[:3, 0] = oritinal_table_pose_mat[:3, 0] / np.linalg.norm(oritinal_table_pose_mat[:3, 0])
        oritinal_table_pose_mat[:3, 1] = np.cross(oritinal_table_pose_mat[:3, 2], oritinal_table_pose_mat[:3, 0])

        table_pose.pose = msgify(geometry_msgs.msg.Pose, oritinal_table_pose_mat)

        # attach the table to the robot for avoiding the collision between the hand base and the table.
        self.scene.add_box(table_name, table_pose, size=(5.0, table_pose.pose.position.x * 2.0 - 0.6,  table_info.center.z))
        while(table_name not in self.scene.get_known_object_names()):
            rospy.sleep(0.0001)
        self.move_group.attach_object(table_name, "base_link", touch_links=['shoulder_pan_link'])

        # get the camera pose
        try:
            camera_trans = self.tfBuffer.lookup_transform('base_link', 'head_camera_rgb_optical_frame', rospy.Time(), rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.return_failure("Can't get the camera pose from the TF tree!!!")
            return

        ## initialize the target object info
        target_object_pose = PoseStamped()
        target_object_width = 0.0
        target_object_depth = 0.0
        target_object_height = 0.0
 
        for attempt_count in range(self.max_attempt_count):
            print " --- ", attempt_count, " times to plan for picking up."

            ################### 1. detect table-top objects and identify the target object with its pointcloud.#################################
            detected_objects = self.object_searcher()

            select_object_id = 0
            has_selected_object = False
            for i in range(len(detected_objects.segmented_objects.objects)):
                object_center = np.array([detected_objects.segmented_objects.objects[i].center.x, 
                                          detected_objects.segmented_objects.objects[i].center.y, 
                                          detected_objects.segmented_objects.objects[i].center.z])
                object_center_in_cam = np.dot(np.linalg.inv(numpify(camera_trans.transform)), np.append(object_center, 1))[:3]
                
                if self.is_in_box(object_center_in_cam, self.camera_intrinsic_matrix, boundary[0], boundary[1], boundary[2], boundary[3]):
                    select_object_id = i
                    has_selected_object = True
                    break

            # if there is no object selected. For this, you do not have to rerun.
            if not has_selected_object:
                self.return_failure("The predict bounding box from YOLO does not match any Objects with pointcloud from the rail-segmentation.")
                return

            self.target_object_pc_publisher.publish(detected_objects.segmented_objects.objects[select_object_id].point_cloud)
            print " -- get target object pointcloud. "
            #################################################################################################

            ################ 2. predict the grasp poses over the object.###################################################################
            try:
                # predicted_grasp_result = self.grasp_predictor(table_info.full_point_cloud, detected_objects.segmented_objects.objects[select_object_id].point_cloud, camera_trans)
                predicted_grasp_result = self.grasp_predictor(detected_objects.segmented_objects.objects[select_object_id].point_cloud, detected_objects.segmented_objects.objects[select_object_id].point_cloud, camera_trans)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            if len(predicted_grasp_result.predicted_grasp_poses) == 0:
                if attempt_count == self.max_attempt_count - 1:
                    # no way to grasp the object.
                    self.return_failure("Contact Grasp Net does not return any grasp poses over the object!")
                    return
                else:
                    rospy.sleep(1.0) # sleep for 1 second for retry.
                    continue

            print " -- number of grasps over the target object = ", len(predicted_grasp_result.predicted_grasp_poses)
            ###############################################################################################################################

            ############### 3. add all obstacles include target object into the planniner scene. #########################################
            obstacle_list = []
            for i in range(len(detected_objects.segmented_objects.objects)):
                if i != select_object_id:
                    # add the object
                    obstacle_pose = PoseStamped()
                    obstacle_pose.header.frame_id = "base_link"
                    obstacle_pose.pose.orientation.x = detected_objects.segmented_objects.objects[i].orientation.x
                    obstacle_pose.pose.orientation.y = detected_objects.segmented_objects.objects[i].orientation.y
                    obstacle_pose.pose.orientation.z = detected_objects.segmented_objects.objects[i].orientation.z
                    obstacle_pose.pose.orientation.w = detected_objects.segmented_objects.objects[i].orientation.w

                    obstacle_pose.pose.position.x = detected_objects.segmented_objects.objects[i].center.x
                    obstacle_pose.pose.position.y = detected_objects.segmented_objects.objects[i].center.y
                    obstacle_pose.pose.position.z = detected_objects.segmented_objects.objects[i].center.z
                    self.scene.add_box("obstacle_" + str(i), obstacle_pose, size=(detected_objects.segmented_objects.objects[i].width, detected_objects.segmented_objects.objects[i].depth, detected_objects.segmented_objects.objects[i].height))
                    while("obstacle_" + str(i) not in self.scene.get_known_object_names()):
                        rospy.sleep(0.0001)
                    obstacle_list.append("obstacle_" + str(i))
                else:
                    # save the target object information
                    target_object_pose.header.frame_id = "base_link"
                    target_object_pose.pose.orientation.x = detected_objects.segmented_objects.objects[i].orientation.x
                    target_object_pose.pose.orientation.y = detected_objects.segmented_objects.objects[i].orientation.y
                    target_object_pose.pose.orientation.z = detected_objects.segmented_objects.objects[i].orientation.z
                    target_object_pose.pose.orientation.w = detected_objects.segmented_objects.objects[i].orientation.w

                    target_object_pose.pose.position.x = detected_objects.segmented_objects.objects[i].center.x
                    target_object_pose.pose.position.y = detected_objects.segmented_objects.objects[i].center.y
                    target_object_pose.pose.position.z = detected_objects.segmented_objects.objects[i].center.z

                    target_object_width = detected_objects.segmented_objects.objects[i].width
                    target_object_depth = detected_objects.segmented_objects.objects[i].depth
                    target_object_height = detected_objects.segmented_objects.objects[i].height

            print " -- add all obstacles into the planning scene. "
            ###############################################################################################################################


            ################ 4. Plan to grasp the object ##########################################################
            
            got_pick_solution = False

            # we should sort the grasp based on its score.
            sort_index_list =  [i[0] for i in sorted(enumerate(predicted_grasp_result.scores), key=lambda x:x[1], reverse=True)]

            # for i in range(len(predicted_grasp_result.predicted_grasp_poses)):
            for i in sort_index_list:

                # if the grasp score is low then skip
                # print 'grasp score ', predicted_grasp_result.scores[i]
                if predicted_grasp_result.scores[i] < 0.1:
                    continue

                # # add the target object into the planning scene.
                # self.scene.add_box("target_object", target_object_pose, size=(target_object_width, target_object_depth, target_object_height))
                # while("target_object" not in self.scene.get_known_object_names()):
                #     rospy.sleep(0.0001)

                # initialize the start state.
                self.move_group.set_start_state_to_current_state()
                
                # calculate grasp, pre-grasp, and pick-up poses in numpy array format
                grasp_pose = numpify(predicted_grasp_result.predicted_grasp_poses[i].pose).dot(self.grasp_shift)
                pre_grasp_pose = grasp_pose.dot(self.pre_grasp_shift)
                pick_up_pose = self.pick_shift.dot(grasp_pose)

                # First to compute the ik solution for checking the feasibility
                ik_target_pose = PoseStamped()
                ik_target_pose.header.stamp = rospy.Time.now()
                ik_target_pose.header.frame_id = 'base_link'
                ik_target_pose.pose = msgify(geometry_msgs.msg.Pose, pre_grasp_pose)

                ik_req = GetPositionIKRequest()
                ik_req.ik_request.group_name = "arm"
                ik_req.ik_request.robot_state = self.robot.get_current_state()
                ik_req.ik_request.avoid_collisions = True
                ik_req.ik_request.pose_stamped = ik_target_pose
                ik_res = self.compute_ik_srv(ik_req)

                if not ik_res.error_code.val == 1:
                    continue

                self.move_group.clear_pose_targets()

                ik_solution_list = [ik_res.solution.joint_state.position[ik_res.solution.joint_state.name.index(joint_name)] for joint_name in self.move_group.get_joints()]
                self.move_group.set_joint_value_target(ik_solution_list)

                # plan the solution to the pre-grasp pose.
                plan_result = self.move_group.plan()

                if plan_result[0]:
                    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                    display_trajectory.trajectory_start = self.move_group.get_current_state()
                    display_trajectory.trajectory.append(plan_result[1])

                    # self.scene.remove_world_object("target_object")
                    # while("target_object" in self.scene.get_known_object_names()):
                    #     rospy.sleep(0.0001)

                    # calculate the last state of the previous planned trajectory.
                    moveit_robot_state = self.robot.get_current_state()
                    start_position_list = list(moveit_robot_state.joint_state.position)
                    for joint_name, joint_value in zip(plan_result[1].joint_trajectory.joint_names, plan_result[1].joint_trajectory.points[-1].positions):
                        start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                    moveit_robot_state.joint_state.position = tuple(start_position_list)

                    self.move_group.set_start_state(moveit_robot_state)
                    (approach_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, grasp_pose)], 0.01, 0.0)

                    # check whether you can approach the object
                    if fraction < 0.9:
                        continue

                    display_trajectory.trajectory.append(approach_plan)

                    for joint_name, joint_value in zip(approach_plan.joint_trajectory.joint_names, approach_plan.joint_trajectory.points[-1].positions):
                        start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                    moveit_robot_state.joint_state.position = tuple(start_position_list)

                    self.move_group.set_start_state(moveit_robot_state)
                    (pick_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pick_up_pose)], 0.01, 0.0)
                    # check whether you can pick the object
                    if fraction < 0.9:
                        continue

                    display_trajectory.trajectory.append(pick_plan)

                    # print "got a way to pre-grasp pose for picking up the object"
                    got_pick_solution = True
                    
                    break

            #######################################################################################################

            ############## 5. if there is no way to grasp the object, then clean the scene and reset thing #######

            self.move_group.clear_pose_targets()

            # need to clear all obstacle
            for obstacle_name in obstacle_list:
                self.scene.remove_world_object(obstacle_name)
                while(obstacle_name in self.scene.get_known_object_names()):
                    rospy.sleep(0.0001)

            # # remove the target object just in case.
            # self.scene.remove_world_object("target_object")
            # while("target_object" in self.scene.get_known_object_names()):
            #     rospy.sleep(0.0001)

            if not got_pick_solution:
                if attempt_count == self.max_attempt_count - 1:
                    # can't find a way to approach the grasp pose.
                    self.return_failure("Can't find a way to approach and pick up the object!")
                    return
            else:
                break
            #####################################################################################################

        self.display_trajectory_publisher.publish(display_trajectory)

        ## need to execute the action actually.
        # # move to pre-grasp
        # action_result = self.move_group.execute(plan_result[1])

        # # # open gripper
        # self.openGripper()

        # # # apporach the object
        # action_result = self.move_group.execute(approach_plan)

        # # # grasp the object
        # self.closeGripper()

        # # # lift up object
        # action_result = self.move_group.execute(pick_plan)

        # prepare the in-hand grasp pose and object dimension for placing later.
        object_pose = Pose()
        object_pose.position.x = detected_objects.segmented_objects.objects[select_object_id].center.x
        object_pose.position.y = detected_objects.segmented_objects.objects[select_object_id].center.y
        object_pose.position.z = detected_objects.segmented_objects.objects[select_object_id].center.z
        object_pose.orientation.x = detected_objects.segmented_objects.objects[select_object_id].orientation.x
        object_pose.orientation.y = detected_objects.segmented_objects.objects[select_object_id].orientation.y
        object_pose.orientation.z = detected_objects.segmented_objects.objects[select_object_id].orientation.z
        object_pose.orientation.w = detected_objects.segmented_objects.objects[select_object_id].orientation.w
        object_pose_mat = numpify(object_pose)

        # get the table top pose
        table_top_pose = geometry_msgs.msg.PoseStamped()
        table_top_pose.header.frame_id = "base_link"
        table_top_pose.pose.orientation.x = table_info.orientation.x
        table_top_pose.pose.orientation.y = table_info.orientation.y
        table_top_pose.pose.orientation.z = table_info.orientation.z
        table_top_pose.pose.orientation.w = table_info.orientation.w
        table_top_pose.pose.position.x = table_info.center.x
        table_top_pose.pose.position.y = table_info.center.y
        table_top_pose.pose.position.z = table_info.center.z

        # get the object in hand pose used later for placing
        in_hand_pose = np.linalg.inv(grasp_pose).dot(object_pose_mat) # input of placing

        # get the object pose should be on the table(we should consider the table rotation as well)
        table_pose_mat = numpify(table_top_pose.pose)
        object_pose_on_table = np.linalg.inv(table_pose_mat).dot(object_pose_mat)
        object_pose_on_table[0][3] = 0.0
        object_pose_on_table[1][3] = 0.0

        ## attach the target object to the hand
        target_object_pose.pose = msgify(geometry_msgs.msg.Pose, numpify(self.move_group.get_current_pose().pose).dot(in_hand_pose))
        self.scene.add_box("target_object", target_object_pose, size=(target_object_width, target_object_depth, target_object_height))
        while("target_object" not in self.scene.get_known_object_names()):
            rospy.sleep(0.0001)
        self.move_group.attach_object("target_object", "wrist_roll_link", touch_links=["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"] )

        # # reset the arm configuration
        self.move_group.clear_pose_targets()
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_joint_value_target(self.default_joints)
        self.move_group.plan()
        # plan = self.move_group.go()

        self.move_group.clear_pose_targets()
        self.move_group.detach_object("target_object")
        self.move_group.detach_object(table_name)
        self.scene.clear()

        self.in_hand_object_name = object_id

        # Change this
        pickup_as_result = PickupResult()
        pickup_as_result.success = True
        pickup_as_result.in_hand_pose = msgify(geometry_msgs.msg.Pose, in_hand_pose)
        pickup_as_result.object_pose_on_table = msgify(geometry_msgs.msg.Pose, object_pose_on_table)
        pickup_as_result.object_width = target_object_width
        pickup_as_result.object_depth = target_object_depth
        pickup_as_result.object_height = target_object_height
        rospy.loginfo("Picked up!")
        self.pickup_as.set_succeeded(pickup_as_result)

    def rotate_pose_z(self, pose, theta):
        # Create the rotation matrix
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Multiply the pose by the rotation matrix
        rotated_pose = np.dot(pose, rotation_matrix)

        return rotated_pose
    def rotate_pose_z_random(self, pose):
        theta = np.random.uniform(0, 2*np.pi)  # Random angle between 0 and 2*pi
        return self.rotate_pose_z(pose, theta)

    def place_cb(self, request):
        
        self.scene.clear()

        # perform manipulation
        rospy.loginfo("Placing object!")

        if self.in_hand_object_name == None: # you do not have object in hand yet.
            self.return_failure("You can only place the object after pick up something!")
            return

        ### 1. get the object in hand pose and its size.
        in_hand_pose = numpify(request.in_hand_pose)
        object_pose_on_table = numpify(request.object_pose_on_table)
        target_object_width = request.object_width
        target_object_depth = request.object_depth
        target_object_height = request.object_height

        print " --- try to place object. "

        ### 2. search both table and table-top objects
        table_info = self.table_searcher()
        detected_objects = self.object_searcher()
        print " -- search both table and table-top objects. "

        ### 3. add table into the planning scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.orientation.x = table_info.orientation.x
        table_pose.pose.orientation.y = table_info.orientation.y
        table_pose.pose.orientation.z = table_info.orientation.z
        table_pose.pose.orientation.w = table_info.orientation.w
        table_pose.pose.position.x = table_info.center.x
        table_pose.pose.position.y = table_info.center.y
        table_pose.pose.position.z = table_info.center.z / 2
        table_name = "table"

        # convert the table to perpendicular pose.
        oritinal_table_pose_mat = numpify(table_pose.pose)
        oritinal_table_pose_mat[0, 0] = 0
        oritinal_table_pose_mat[1, 0] = 0
        if np.linalg.norm(oritinal_table_pose_mat[:3, 0]) == 0:
            oritinal_table_pose_mat[:3, 0] = np.array([0,1,0])
        else:
            oritinal_table_pose_mat[:3, 0] = oritinal_table_pose_mat[:3, 0] / np.linalg.norm(oritinal_table_pose_mat[:3, 0])
        oritinal_table_pose_mat[:3, 1] = np.cross(oritinal_table_pose_mat[:3, 2], oritinal_table_pose_mat[:3, 0])

        table_pose.pose = msgify(geometry_msgs.msg.Pose, oritinal_table_pose_mat)

        self.scene.add_box(table_name, table_pose, size=(5.0, table_pose.pose.position.x * 2.0 - 0.6,  table_info.center.z))
        while(table_name not in self.scene.get_known_object_names()):
            rospy.sleep(0.0001)
        self.move_group.attach_object(table_name, "base_link", touch_links=['shoulder_pan_link'])
        
        table_pose_mat = numpify(table_pose.pose)

        ### 4. need to add the table top object as obstacle into planning scene.
        for i in range(len(detected_objects.segmented_objects.objects)):
            # add the object
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = "base_link"
            obstacle_pose.pose.orientation.x = detected_objects.segmented_objects.objects[i].orientation.x
            obstacle_pose.pose.orientation.y = detected_objects.segmented_objects.objects[i].orientation.y
            obstacle_pose.pose.orientation.z = detected_objects.segmented_objects.objects[i].orientation.z
            obstacle_pose.pose.orientation.w = detected_objects.segmented_objects.objects[i].orientation.w

            obstacle_pose.pose.position.x = detected_objects.segmented_objects.objects[i].center.x
            obstacle_pose.pose.position.y = detected_objects.segmented_objects.objects[i].center.y
            obstacle_pose.pose.position.z = detected_objects.segmented_objects.objects[i].center.z
            self.scene.add_box("obstacle_" + str(i), obstacle_pose, size=(detected_objects.segmented_objects.objects[i].width, detected_objects.segmented_objects.objects[i].depth, detected_objects.segmented_objects.objects[i].height))
            while("obstacle_" + str(i) not in self.scene.get_known_object_names()):
                rospy.sleep(0.0001)

        ### 5. get the points of the table top, they may be the point to place the object.
        table_points = list(point_cloud2.read_points(table_info.point_cloud, field_names=("x", "y", "z"), skip_nans=True))

        max_lidar_range = 2.5
        disc_size = 0.01
        disc_factor = 1/disc_size
        image_size = int(max_lidar_range*2*disc_factor)
        env_image = np.ones((image_size,image_size,1),dtype=np.uint8) * 255

        for object_point in table_points:
            pt_x = object_point[0]
            pt_y = object_point[1]
            if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
                pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
                if (pix_x > image_size) or (pix_y > image_size):
                    continue
                else:
                    env_image[pix_y,pix_x] = [0]

        # # Taking a matrix of size 5 as the kernel
        kernel = np.ones((8, 8), np.uint8)
  
        env_image = cv2.dilate(env_image, kernel, iterations=1)

        has_place_solution = False
        object_pose_on_table[2, 3] += 0.005

        # try to place object
        for j in range(self.max_attempt_count * 2):

            # initialize the start state.
            self.move_group.set_start_state_to_current_state()

            rospy.sleep(1.0)

            # attach the target object in hand.
            target_object_pose = PoseStamped()
            target_object_pose.header.frame_id = "base_link"
            target_object_pose.pose = msgify(geometry_msgs.msg.Pose, numpify(self.move_group.get_current_pose().pose).dot(in_hand_pose))

            self.scene.add_box("target_object", target_object_pose, size=(target_object_width, target_object_depth, target_object_height))
            while("target_object" not in self.scene.get_known_object_names()):
                rospy.sleep(0.0001)
            # attach the target object to the hand
            self.move_group.attach_object("target_object", "wrist_roll_link", touch_links=["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"] )
            
            # randomly select a point on the table and consider it as the table origin.
            got_good_point_to_place = False
            sample_point_attempt = 0
            print "-- try to find a point to place "
            while not got_good_point_to_place and sample_point_attempt < 20:
                sample_point_attempt += 1

                place_point = random.choice(table_points)

                pix_x = int(math.floor((place_point[0] + max_lidar_range) * disc_factor))
                pix_y = int(math.floor((max_lidar_range - place_point[1]) * disc_factor))

                if env_image[pix_y,pix_x] == 255:
                    continue

                got_good_point_to_place = True
            
            if not got_good_point_to_place:
                continue

            print " -- find good point to place at ", place_point
            
            table_pose_mat[:3, 3] = place_point
            place_pose_on_table = table_pose_mat.dot(self.rotate_pose_z_random(object_pose_on_table))
            hand_pose_for_place = place_pose_on_table.dot(np.linalg.inv(in_hand_pose))
            hand_pose_for_pre_place = self.pick_shift.dot(hand_pose_for_place)
            hand_pose_for_release = hand_pose_for_place.dot(self.pre_grasp_shift)

            # First to compute the ik solution for checking the feasibility
            ik_target_pose = PoseStamped()
            ik_target_pose.header.stamp = rospy.Time.now()
            ik_target_pose.header.frame_id = 'base_link'
            ik_target_pose.pose = msgify(geometry_msgs.msg.Pose, hand_pose_for_pre_place)

            ik_req = GetPositionIKRequest()
            ik_req.ik_request.group_name = "arm"
            ik_req.ik_request.robot_state = self.robot.get_current_state()
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.pose_stamped = ik_target_pose
            ik_res = self.compute_ik_srv(ik_req)

            if not ik_res.error_code.val == 1:
                continue

            self.move_group.clear_pose_targets()

            ik_solution_list = [ik_res.solution.joint_state.position[ik_res.solution.joint_state.name.index(joint_name)] for joint_name in self.move_group.get_joints()]
            self.move_group.set_joint_value_target(ik_solution_list)

            plan_result = self.move_group.plan()
            if plan_result[0]:

                display_trajectory = moveit_msgs.msg.DisplayTrajectory()
                display_trajectory.trajectory_start = self.move_group.get_current_state()
                display_trajectory.trajectory.append(plan_result[1])

                # need to detach the object
                self.move_group.detach_object("target_object")
                self.scene.remove_world_object("target_object")
                while("target_object" in self.scene.get_known_object_names()):
                    rospy.sleep(0.0001)

                print " -- plan to place"
                # calculate the last state of the previous planned trajectory.
                moveit_robot_state = self.robot.get_current_state()
                start_position_list = list(moveit_robot_state.joint_state.position)
                for joint_name, joint_value in zip(plan_result[1].joint_trajectory.joint_names, plan_result[1].joint_trajectory.points[-1].positions):
                    start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                moveit_robot_state.joint_state.position = tuple(start_position_list)
                
                self.move_group.set_start_state(moveit_robot_state)
                (place_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_place)], 0.01, 0.0)
                
                # check whether you can place the object
                if fraction < 0.9:
                    continue

                display_trajectory.trajectory.append(place_plan)

                # plan to place down
                for joint_name, joint_value in zip(place_plan.joint_trajectory.joint_names, place_plan.joint_trajectory.points[-1].positions):
                    start_position_list[moveit_robot_state.joint_state.name.index(joint_name)] = joint_value
                moveit_robot_state.joint_state.position = tuple(start_position_list)

                self.move_group.set_start_state(moveit_robot_state)
                (release_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_release)], 0.01, 0.0)

                # check whether you can pick the object
                if fraction < 0.9:
                    continue

                display_trajectory.trajectory.append(release_plan)

                has_place_solution = True

                break

        self.move_group.clear_pose_targets()
        self.move_group.detach_object("target_object")
        self.move_group.detach_object(table_name)
        self.scene.clear()

        if not has_place_solution:
            self.return_failure("Can't find solution to place the object. ")
            return

        self.display_trajectory_publisher.publish(display_trajectory)

        ## execute the actual action.
        # # move to the pre-place pose
        # self.move_group.execute(plan_result[1])

        # # place the object
        # self.move_group.execute(place_plan)

        # # open gripper
        # self.openGripper()

        # # release object
        # self.move_group.execute(release_plan)

        self.in_hand_object_name = None

        place_as_result = PlaceResult()
        place_as_result.success = True
        rospy.loginfo("Placed object!")
        # self.move_back()
        self.place_as.set_succeeded(place_as_result)

    def project_3d_to_2d(self, point_3d, intrinsic_matrix):

        # Project the 3D point to 2D using the intrinsic matrix
        point_2d_homogeneous = np.dot(intrinsic_matrix, point_3d)

        # Normalize the homogeneous 2D point to obtain the Euclidean 2D point
        point_2d = point_2d_homogeneous / point_2d_homogeneous[2]

        return point_2d[:2]

    def is_in_box(self, point_3d, intrinsic_matrix, min_x, max_x, min_y, max_y):
        point_2d = self.project_3d_to_2d(point_3d, intrinsic_matrix)
        u = point_2d[0]
        v = point_2d[1]
        if(u < min_x or u > max_x or v < min_y or v > max_y):
            return False
        else:
            return True


def create_manipulation_clients():
    pickup_action_client = actionlib.SimpleActionClient(
        "pickup_server",
        PickupAction,
    )

    pickup_action_client.wait_for_server()

    place_action_client = actionlib.SimpleActionClient(
        "palce_server",
        PlaceAction,
    )

    place_action_client.wait_for_server()

    return pickup_action_client, place_action_client


if __name__ == '__main__':
    rospy.init_node('manipulation_node')
    action_server = Manipulation(isSim=False)
