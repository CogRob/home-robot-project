#! /usr/bin/env python

import rospy
import actionlib
from manipulation.msg import PickupAction, PlaceAction, PickupResult, PlaceActionResult
import random

# from coppeliasim_zmq.srv import AttachObjectToGripper, DetachObjectToGripper
from object_detector.srv import detect2DObject, detect2DObjectRequest

import tf2_ros
from ros_tensorflow_msgs.srv import *
from rail_segmentation.srv import *
from rail_manipulation_msgs.srv import *
from geometry_msgs.msg import TransformStamped, PoseStamped, TwistStamped, Pose
from manipulation_test.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive

from sensor_msgs import point_cloud2

from sensor_msgs.msg import CameraInfo, JointState
import tf
from ros_numpy import numpify, msgify
import numpy as np

import sys
import moveit_commander
import moveit_msgs.msg

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal,
)
# def get_zmq_clients():

#     rospy.wait_for_service("attach_object_to_gripper_service")
#     attach_object_to_gripper_service_client = rospy.ServiceProxy(
#         "attach_object_to_gripper_service", AttachObjectToGripper
#     )

#     rospy.wait_for_service("detach_object_to_gripper_service")
#     detach_object_to_gripper_service_client = rospy.ServiceProxy(
#         "detach_object_to_gripper_service", DetachObjectToGripper
#     )

#     return attach_object_to_gripper_service_client, detach_object_to_gripper_service_client


class Manipulation(object):
    def __init__(self, isSim = False):
    
    
        print "check object detector client"
        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        print "after check object detector client"
        self.object_detector_client.wait_for_service()

        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupAction, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        

        self.place_as = actionlib.SimpleActionServer("place_server", PlaceAction, execute_cb=self.place_cb, auto_start = False)
        self.place_as.start()

        self._sim = True 
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
        # if isSim:
        #     self.attach_client, self.detach_client = get_zmq_clients()

        # init a table searcher
        print "check before wait for service"
        rospy.wait_for_service('table_searcher/search_table')
        print "check after wait for service"
        self.table_searcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

        # init table-top object searcher
        rospy.wait_for_service('table_searcher/segment_objects')
        self.object_searcher = rospy.ServiceProxy('table_searcher/segment_objects', SegmentObjects)

        # init grasp predictor
        rospy.wait_for_service('grasp_predict')
        self.grasp_predictor = rospy.ServiceProxy('grasp_predict', Predict)
        
        # get camera transform from tf tree
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # init moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander("arm") 
        self.scene = moveit_commander.PlanningSceneInterface()

        rospy.spin()

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


    def pickup_cb(self, request):

        self.scene.clear()

        object_id = request.object_id

        print "object id = ", object_id
        self.openGripper()
        # call perception
        detections = self.object_detector_client()

        for detection in detections.detections.detections:
            if detection.object_id == object_id:
                break
        
        center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y

        print "receive bounding box"

        boundary = [center_x - size_x / 2, center_x + size_x / 2, center_y - size_y / 2, center_y + size_y / 2]

        table_info = self.table_searcher()
        detected_objects = self.object_searcher()

        # need to get the camera matrix
        camera_info = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
        camera_matrix = np.reshape(camera_info.K, (3, 3))
        
        # get the camera pose
        try:
            camera_trans = self.tfBuffer.lookup_transform('base_link', 'head_camera_rgb_optical_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("tf error")        

        select_object_id = 0
        has_selected_object = False
        for i in range(len(detected_objects.segmented_objects.objects)):
            object_center = np.array([detected_objects.segmented_objects.objects[i].center.x, 
                                      detected_objects.segmented_objects.objects[i].center.y, 
                                      detected_objects.segmented_objects.objects[i].center.z])
            object_center_in_cam = np.dot(np.linalg.inv(numpify(camera_trans.transform)), np.append(object_center, 1))[:3]
            
            if self.is_in_box(object_center_in_cam, camera_matrix, boundary[0], boundary[1], boundary[2], boundary[3]):
                select_object_id = i
                has_selected_object = True
                break

        # if there is no object selected.
        if not has_selected_object:
            # if object is not detected, then return failure
            pickup_as_result = PickupResult()
            pickup_as_result.success = False
            rospy.loginfo("Can't detect the target object!")
            self.pickup_as.set_succeeded(pickup_as_result)
            return

        try:
            predicted_grasp_result = self.grasp_predictor(table_info.full_point_cloud, detected_objects.segmented_objects.objects[select_object_id].point_cloud, camera_trans)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        if len(predicted_grasp_result.predicted_grasp_poses) == 0:
            # no way to grasp the object.
            pickup_as_result = PickupResult()
            pickup_as_result.success = False
            rospy.loginfo("No way to grasp the object!")
            self.pickup_as.set_succeeded(pickup_as_result)
            return

        # all information about the target object.
        target_object_pose = geometry_msgs.msg.PoseStamped()
        target_object_width = 0.0
        target_object_depth = 0.0
        target_object_height = 0.0

        # add all obstacle into the planniner scene
        for i in range(len(detected_objects.segmented_objects.objects)):
            if i != select_object_id:
                # add the object
                obstacle_pose = geometry_msgs.msg.PoseStamped()
                obstacle_pose.header.frame_id = "base_link"
                obstacle_pose.pose.orientation.x = detected_objects.segmented_objects.objects[i].orientation.x
                obstacle_pose.pose.orientation.y = detected_objects.segmented_objects.objects[i].orientation.y
                obstacle_pose.pose.orientation.z = detected_objects.segmented_objects.objects[i].orientation.z
                obstacle_pose.pose.orientation.w = detected_objects.segmented_objects.objects[i].orientation.w

                obstacle_pose.pose.position.x = detected_objects.segmented_objects.objects[i].center.x
                obstacle_pose.pose.position.y = detected_objects.segmented_objects.objects[i].center.y
                obstacle_pose.pose.position.z = detected_objects.segmented_objects.objects[i].center.z
                self.scene.add_box("obstacle_" + str(i), obstacle_pose, size=(detected_objects.segmented_objects.objects[i].width, detected_objects.segmented_objects.objects[i].depth, detected_objects.segmented_objects.objects[i].height))
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

        # add table into the planning scene
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.orientation.x = table_info.orientation.x
        table_pose.pose.orientation.y = table_info.orientation.y
        table_pose.pose.orientation.z = table_info.orientation.z
        table_pose.pose.orientation.w = table_info.orientation.w
        table_pose.pose.position.x = table_info.center.x
        table_pose.pose.position.y = table_info.center.y
        table_pose.pose.position.z = table_info.center.z / 2
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(table_info.width, table_info.depth, table_info.center.z))

        rospy.sleep(0.1)

        grasp_shift = np.array([[1,0,0,0.02],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        pre_grasp_shift = np.array([[1,0,0,-0.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        pick_shift = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.05],[0,0,0,1]])

        got_solution = False

        for i in range(len(predicted_grasp_result.predicted_grasp_poses)):
            # add the target object into the planning scene.
            self.scene.add_box("target_object", target_object_pose, size=(target_object_width, target_object_depth, target_object_height))

            # initialize the start state.
            self.move_group.set_start_state_to_current_state()
            
            # generate grasp pose
            grasp_pose = numpify(predicted_grasp_result.predicted_grasp_poses[i].pose).dot(grasp_shift)

            # calculate the pre-grasp pose
            pre_grasp_pose = grasp_pose.dot(pre_grasp_shift)
            
            # calculate the pick-up pose
            pick_up_pose = pick_shift.dot(grasp_pose)

            trans = tf.transformations.translation_from_matrix(pre_grasp_pose).tolist()
            quat = tf.transformations.quaternion_from_matrix(pre_grasp_pose).tolist()

            self.move_group.clear_pose_targets()
            self.move_group.set_pose_target(trans + quat)

            # plan the solution to the pre-grasp pose.
            plan_result = self.move_group.plan()

            if plan_result[0]:
                print "find way to pre-grasp pose"
                # need to remove the target object in the planning scene
                self.scene.remove_world_object("target_object")

                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = plan_result[1].joint_trajectory.joint_names
                joint_state.position = plan_result[1].joint_trajectory.points[-1].positions
                moveit_robot_state = RobotState()
                moveit_robot_state.joint_state = joint_state

                self.move_group.set_start_state(moveit_robot_state)
                (approach_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, grasp_pose)], 0.01, 0.0)
                print "approach fraction ", fraction
                # check whether you can approach the object
                if fraction < 0.9:
                    continue
                
                joint_state.header.stamp = rospy.Time.now()
                joint_state.position = approach_plan.joint_trajectory.points[-1].positions
                moveit_robot_state.joint_state = joint_state

                self.move_group.set_start_state(moveit_robot_state)
                (pick_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pick_up_pose)], 0.01, 0.0)
                # check whether you can pick the object
                if fraction < 0.9:
                    continue
                    
                print "got a way to pick up the object"
                got_solution = True
                
                break

        self.move_group.clear_pose_targets()
        self.scene.clear()

        if not got_solution:
            # can't find a way to approach the grasp pose.
            pickup_as_result = PickupResult()
            pickup_as_result.success = False
            rospy.loginfo("Can't find a way to approach and pick up the object!")
            self.pickup_as.set_succeeded(pickup_as_result)
            return

        ## need to execute the action actually.
        # move to pre-grasp
        self.move_group.execute(plan_result[1])
        # open gripper
        self.openGripper()
        # apporach the object
        self.move_group.set_start_state_to_current_state()
        (approach_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, grasp_pose)], 0.01, 0.0)
        self.move_group.execute(approach_plan)
        # grasp the object
        self.closeGripper(-0.01)
        # lift up object
        self.move_group.set_start_state_to_current_state()
        (pick_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, pick_up_pose)], 0.01, 0.0)
        self.move_group.execute(pick_plan)

        object_pose = Pose()
        object_pose.position.x = detected_objects.segmented_objects.objects[0].center.x
        object_pose.position.y = detected_objects.segmented_objects.objects[0].center.y
        object_pose.position.z = detected_objects.segmented_objects.objects[0].center.z
        object_pose.orientation.x = detected_objects.segmented_objects.objects[0].orientation.x
        object_pose.orientation.y = detected_objects.segmented_objects.objects[0].orientation.y
        object_pose.orientation.z = detected_objects.segmented_objects.objects[0].orientation.z
        object_pose.orientation.w = detected_objects.segmented_objects.objects[0].orientation.w
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

        # Change this
        pickup_as_result = PickupResult()
        pickup_as_result.success = True
        pickup_as_result.in_hand_pose = msgify(geometry_msgs.msg.Pose, in_hand_pose)
        pickup_as_result.object_pose_on_table = msgify(geometry_msgs.msg.Pose, object_pose_on_table)
        pickup_as_result.object_width = target_object_width
        pickup_as_result.object_depth = target_object_depth
        pickup_as_resultobject_height = target_object_height
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

        # call perception

        # perform manipulation
        rospy.loginfo("Placing object!")

        in_hand_pose = request.in_hand_pose
        object_pose_on_table = request.object_pose_on_table
        target_object_width = request.object_width
        target_object_depth = request.object_depth
        target_object_height = request.object_height

        # search a position to place
        # get the table info
        table_info = self.table_searcher()
        detected_objects = self.object_searcher()

        # add table into the planning scene
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.orientation.x = table_info.orientation.x
        table_pose.pose.orientation.y = table_info.orientation.y
        table_pose.pose.orientation.z = table_info.orientation.z
        table_pose.pose.orientation.w = table_info.orientation.w
        table_pose.pose.position.x = table_info.center.x
        table_pose.pose.position.y = table_info.center.y
        table_pose.pose.position.z = table_info.center.z / 2
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(table_info.width, table_info.depth, table_info.center.z))
        table_pose_mat = numpify(table_pose.pose)

        # need to add the table top object as obstacle into planning scene.
        for i in range(len(detected_objects.segmented_objects.objects)):
            # add the object
            obstacle_pose = geometry_msgs.msg.PoseStamped()
            obstacle_pose.header.frame_id = "base_link"
            obstacle_pose.pose.orientation.x = detected_objects.segmented_objects.objects[i].orientation.x
            obstacle_pose.pose.orientation.y = detected_objects.segmented_objects.objects[i].orientation.y
            obstacle_pose.pose.orientation.z = detected_objects.segmented_objects.objects[i].orientation.z
            obstacle_pose.pose.orientation.w = detected_objects.segmented_objects.objects[i].orientation.w

            obstacle_pose.pose.position.x = detected_objects.segmented_objects.objects[i].center.x
            obstacle_pose.pose.position.y = detected_objects.segmented_objects.objects[i].center.y
            obstacle_pose.pose.position.z = detected_objects.segmented_objects.objects[i].center.z
            self.scene.add_box("obstacle_" + str(i), obstacle_pose, size=(detected_objects.segmented_objects.objects[i].width, detected_objects.segmented_objects.objects[i].depth, detected_objects.segmented_objects.objects[i].height))

        # get the points of the table top
        points = list(point_cloud2.read_points(table_info.point_cloud, field_names=("x", "y", "z"), skip_nans=True))

        attached_object = AttachedCollisionObject()
        attached_object.link_name = "wrist_roll_link"

        # Create a CollisionObject
        collision_object = CollisionObject()
        collision_object.id = "object"
        collision_object.header.frame_id = "base_link"

        # Create a SolidPrimitive box
        box = SolidPrimitive()
        box.type = box.BOX
        box.dimensions = [target_object_width, target_object_depth, target_object_height]  # Size of the box

        collision_object.primitives = [box]
        collision_object.primitive_poses = [msgify(geometry_msgs.msg.Pose, numpify(self.move_group.get_current_pose().pose).dot(in_hand_pose))]

        # Add the collision object into the AttachedCollisionObject message
        attached_object.object = collision_object
        attached_object.object.operation = attached_object.object.ADD
        attached_object.touch_links = ["l_gripper_finger_link", "r_gripper_finger_link", "gripper_link"]

        has_place_solution = False

        # try to place object
        for j in range(15):
            self.move_group.set_start_state_to_current_state()
            
            # randomly select a point on the table and consider it as the table origin.
            table_pose_mat[:3, 3] = random.choice(points)
            
            place_pose_on_table = table_pose_mat.dot(rotate_pose_z_random(object_pose_on_table))
            
            hand_pose_for_place = place_pose_on_table.dot(np.linalg.inv(in_hand_pose))
            
            hand_pose_for_pre_place = pick_shift.dot(hand_pose_for_place)
            
            hand_pose_for_release = hand_pose_for_place.dot(pre_grasp_shift)

            trans = tf.transformations.translation_from_matrix(hand_pose_for_pre_place).tolist()
            quat = tf.transformations.quaternion_from_matrix(hand_pose_for_pre_place).tolist()

            self.move_group.clear_pose_targets()
            # need to attach the object on the end-effector
            moveit_robot_state = self.move_group.get_current_state()
            moveit_robot_state.attached_collision_objects = [attached_object]
            
            self.move_group.set_start_state(moveit_robot_state)
            
            self.move_group.set_pose_target(trans + quat)
            plan_result = self.move_group.plan()
            if plan_result[0]:
                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = plan_result[1].joint_trajectory.joint_names
                joint_state.position = plan_result[1].joint_trajectory.points[-1].positions
                moveit_robot_state = RobotState()
                moveit_robot_state.joint_state = joint_state
                self.move_group.set_start_state(moveit_robot_state)
                (place_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_place)], 0.01, 0.0)
                # check whether you can place the object
                if fraction < 0.9:
                    continue
                    
                joint_state.header.stamp = rospy.Time.now()
                joint_state.position = place_plan.joint_trajectory.points[-1].positions
                moveit_robot_state.joint_state = joint_state
                self.move_group.set_start_state(moveit_robot_state)
                (release_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_release)], 0.01, 0.0)
                # check whether you can pick the object
                if fraction < 0.9:
                    continue
                has_place_solution = True
                break

        self.move_group.clear_pose_targets()
        self.move_group.detach_object("object")
        self.scene.clear()

        if not has_place_solution:
            place_as_result = PlaceActionResult()
            place_as_result.success = False
            rospy.loginfo("Can't place object!")
            self.place_as.set_succeeded(place_as_result)
            return

        ## execute the actual action.
        # move to the pre-place pose
        self.move_group.execute(plan_result[1])
        # place the object
        self.move_group.set_start_state_to_current_state()
        (place_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_place)], 0.01, 0.0)
        self.move_group.execute(place_plan)
        # open gripper
        self.openGripper()
        # release object
        self.move_group.set_start_state_to_current_state()
        (release_plan, fraction) = self.move_group.compute_cartesian_path([msgify(geometry_msgs.msg.Pose, hand_pose_for_release)], 0.01, 0.0)
        self.move_group.execute(release_plan)

        place_as_result = PlaceActionResult()
        place_as_result.success = True
        rospy.loginfo("Placed object!")
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
    action_server = Manipulation()
