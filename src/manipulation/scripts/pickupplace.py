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
from geometry_msgs.msg import TransformStamped
from manipulation_test.srv import *

from sensor_msgs.msg import CameraInfo
import tf
from ros_numpy import numpify
import numpy as np

import sys
import moveit_commander
import moveit_msgs.msg

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
    
    
        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()

        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupAction, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        

        self.place_as = actionlib.SimpleActionServer("place_server", PlaceAction, execute_cb=self.pickup_cb, auto_start = False)
        self.place_as.start()

        # if isSim:
        #     self.attach_client, self.detach_client = get_zmq_clients()

        # init a table searcher
        rospy.wait_for_service('table_searcher/search_table')
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

    def pickup_cb(self, request):

        object_id = request.object_id

        # call perception
        detections = self.object_detector_client()

        for detection in detections:
            if detections.object_id == object_id:
                break
        
        center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y

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
            
            if is_in_box(object_center_in_cam, camera_matrix, boundary[0], boundary[1], boundary[2], boundary[3]):
                select_object_id = i
                has_selected_object = True
                break

        # if there is no object selected.
        if not has_selected_object:
            return

        try:
            predicted_grasp_result = self.grasp_predictor(table_info.full_point_cloud, detected_objects.segmented_objects.objects[select_object_id].point_cloud, camera_trans)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # add table into the planning scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.x = table_info.orientation.x
        box_pose.pose.orientation.y = table_info.orientation.y
        box_pose.pose.orientation.z = table_info.orientation.z
        box_pose.pose.orientation.w = table_info.orientation.w
        box_pose.pose.position.x = table_info.center.x
        box_pose.pose.position.y = table_info.center.y
        box_pose.pose.position.z = table_info.center.z
        box_name = "table"
        scene.add_box(box_name, box_pose, size=(0.5, 0.8, 0.001))

        # calculate the pre-grasp pose
        pre_grasp_shift = np.array([[1,0,0,-0.05],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        pre_grasp_pose = numpify(predicted_grasp_result.predicted_grasp_poses[3].pose).dot(pre_grasp_shift)

        trans = transformations.translation_from_matrix(pre_grasp_pose).tolist()
        quat = transformations.quaternion_from_matrix(pre_grasp_pose).tolist()

        self.move_group.set_pose_target(trans + quat)
        plan = self.move_group.plan()
        self.move_group.clear_pose_target()

        # perform manipulation
        rospy.loginfo("Picking up ")
        print(object_id)

        # Change this
        pickup_as_result = PickupResult()
        self.attach_client(object_id=object_id)
        pickup_as_result.success = True
        rospy.loginfo("Picked up!")
        self.pickup_as.set_succeeded(pickup_as_result)

    def place_cb(self, request):

        # call perception

        # perform manipulation
        rospy.loginfo("Placing object!")

        # Change this

        place_as_result = PlaceActionResult()
        place_as_result.success = random.choice([True, False])
        rospy.loginfo("Placed object!")
        self.place_as.set_succeeded(place_as_result)

    def project_3d_to_2d(self, point_3d, intrinsic_matrix):

        # Project the 3D point to 2D using the intrinsic matrix
        point_2d_homogeneous = np.dot(intrinsic_matrix, point_3d)

        # Normalize the homogeneous 2D point to obtain the Euclidean 2D point
        point_2d = point_2d_homogeneous / point_2d_homogeneous[2]

        return point_2d[:2]

    def is_in_box(self, point_3d, intrinsic_matrix, min_x, max_x, min_y, max_y):
        point_2d = project_3d_to_2d(point_3d, intrinsic_matrix)
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
