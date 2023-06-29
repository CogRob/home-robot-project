#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Point, TransformStamped, PoseStamped, TwistStamped, Pose, Twist, Point, Quaternion
from receptacle_navigator.srv import GetGoalPoseForReceptacle, GetGoalPoseForReceptacleResponse, GetGoalPoseForReceptacleRequest, GetReceptacleLocations, GetReceptacleLocationsRequest, GetReceptacleLocationsResponse
from receptacle_navigator.msg import NavigateToReceptaclesAction, NavigateToReceptaclesResult, NavigateToReceptacleAction, NavigateToReceptacleResult
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest



from local_path_planner.msg import moveRobotBaseGoal, moveRobotBaseAction, moveHeadAction, moveHeadGoal
from object_detector.srv import detect2DObject, detect2DObjectRequest
import tf2_ros
from ramsam_ros.srv import DetectReceptacle, DetectReceptacleRequest

from home_robot_msgs.msg import NamedLocation

from ros_numpy import numpify
import numpy as np



class ReceptacleNavigation(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer("receptacle_navigator", NavigateToReceptacleAction, execute_cb=self.receptacle_navigate_cb, auto_start = False)
        self._as.start()
        self.move_fetch_base_client = actionlib.SimpleActionClient("move_fetch_robot_base", moveRobotBaseAction)
        self.move_fetch_base_client.wait_for_server()

        self.move_fetch_head_client = actionlib.SimpleActionClient("move_fetch_head", moveHeadAction)
        self.move_fetch_head_client.wait_for_server()


        self.as_result = NavigateToReceptaclesResult()
        rospy.loginfo("Created receptacle navigator")

        self.receptor_approach_pose_service = rospy.Service(
            "receptor_approach_pose", GetGoalPoseForReceptacle, self.receptor_approach_pose_cb
        )
        rospy.loginfo("Created receptacle approach")

        rospy.wait_for_service("receptor_approach_pose")
        self.receptor_approach_pose_client = rospy.ServiceProxy(
            "receptor_approach_pose", GetGoalPoseForReceptacle
        )

        rospy.wait_for_service("receptacle_detector")
        self.receptacle_detector_client = rospy.ServiceProxy(
            "receptacle_detector", DetectReceptacle
        )

        self.present_receptacles = rospy.Service(
            "available_receptacles", GetReceptacleLocations, self.get_ordered_receptacle_locations
        )
        rospy.loginfo("Created available receptacles")
        rospy.wait_for_service("available_receptacles")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.current_room_client = rospy.ServiceProxy(
                        "/semantic_localize", SemanticLocalizer
                    )
        self.current_room_client.wait_for_service()
        
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1.0)

        rospy.spin()


    def move_back(self):
        move = Twist()
        move.linear.x = -0.5
        for _ in range(9):
            self.cmd_publisher.publish(move)
            rospy.sleep(0.5)

    def transform_point(self, tf_transform, target_points):
        # target points is of shape (n, 3)
        # returns (n, 3)
        target_points = target_points.copy().T
        transform_matrix = numpify(tf_transform)
        homog_point = np.vstack((target_points, np.ones((1, target_points.shape[1]))))
        transformed_point = np.dot(transform_matrix, homog_point)
        transformed_point /= transformed_point[3]
        return transformed_point.T



    def receptor_approach_pose_cb(self, request):
        receptacle = request.receptacle


        robot_location_map = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        robot_location_xy = (robot_location_map.transform.translation.x, robot_location_map.transform.translation.y)

        receptacle_location_xy = (receptacle.location.x,receptacle.location.y )

        angle = np.arctan2(receptacle_location_xy[1] - robot_location_xy[1], receptacle_location_xy[0] - robot_location_xy[0])
        goal_pose = Pose2D(receptacle_location_xy[0], receptacle_location_xy[1], angle)

        # # get 3d loc of receptacle.location in map frame by rec_loc_map = np.dot(numpify(robot_head_location_map.transform), receptacle.location)
        # # get angle by np.atan2(rec_loc_map[1] - robot_location_map.transform.y, rec_loc_map[0] - robot_location_map.transform.x)


        print("Goal pose is : ", goal_pose)
        # goal_pose = Pose2D(10.0226679908, -1.65996362634, -1.12498106991)
        response_object = GetGoalPoseForReceptacleResponse(goal_pose = goal_pose)
        return response_object


    def rotate_and_detect_receptacles(self, found_receptacles_dict, joint_values):
        # Turn left 45 degrees
        move_head_joints = moveHeadGoal()
        move_head_joints.joint_values = joint_values

        rospy.loginfo("Turning head")
        self.move_fetch_head_client.send_goal(move_head_joints)
        self.move_fetch_head_client.wait_for_result()
        rospy.sleep(2.0)

        cur_room = self.current_room_client().room

        # call object segmenter to get xyz.
        det_receptacles = self.receptacle_detector_client(DetectReceptacleRequest(room = cur_room))
        print("Detected receptacles : ", det_receptacles.receptacles)

        robot_head_location_map = self.tfBuffer.lookup_transform('map', 'head_camera_rgb_optical_frame', rospy.Time())


        for detected_receptacle in det_receptacles.receptacles:
            if detected_receptacle.name not in found_receptacles_dict:
                if detected_receptacle.location.z > 3.0:
                    continue
                receptacle_location_map_transform = self.transform_point(robot_head_location_map.transform, numpify(detected_receptacle.location).reshape((1,3)))
                # detected_receptacle.location = Point(receptacle_location_xy[0], receptacle_location_xy[1], 0)
                found_receptacles_dict[detected_receptacle.name] = Point(receptacle_location_map_transform.squeeze()[0], receptacle_location_map_transform.squeeze()[1], 0)

        # found_receptacles_dict["counter"] = Point(0.0, -2.5, 0.0)
        # found_receptacles_dict["shelf"] = Point(-0.978, -3.528, 1.064)
        print(found_receptacles_dict)
        return found_receptacles_dict

    def get_ordered_receptacle_locations(self,request):
        receptacles = request.receptacles


        found_receptacles_dict = {}

        # Turn 45 degrees left
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [0.75, 0.25])
        # Turn 90 degrees left
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [1.40, 0.25])
        # Turn 45 degrees right
        # found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [-0.75, 0.25])
        # Turn 90 degrees right
        # found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [-1.40, 0.25])
        # Turn center
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [0.0, 0.25])

        #scan the room and keep calling receptacles
        # rotate the base and call receptacle detector
        # store the 3D locations of the detectors found

        print(found_receptacles_dict)        

        receptacles_out = GetReceptacleLocationsResponse()
        for receptacle in receptacles:
            if receptacle in found_receptacles_dict:
                temp_receptacle = NamedLocation()
                temp_receptacle.name = receptacle
                temp_receptacle.location = found_receptacles_dict[receptacle]
                receptacles_out.receptacle_locations.append(temp_receptacle)
        
        print("Found receptacles : ", receptacles_out)
        return receptacles_out


    def receptacle_navigate_cb(self, request):

        """
        Navigate to a receptacle.
        Input: NamedLocation of Receptacle and its 2D position
        """


        receptacle_goal_point = self.receptor_approach_pose_client(GetGoalPoseForReceptacleRequest(receptacle = request.receptacle))
        move_goal = moveRobotBaseGoal(
            pose = receptacle_goal_point.goal_pose,
            use_carrot = True
            
        )
        rospy.loginfo("Created goal")
        self.move_fetch_base_client.send_goal_and_wait(move_goal)
        rospy.loginfo("Sent goal")

        rospy.sleep(3.0)

        if request.receptacle.name == "cabinet":
            print("-----------> Moving back")
            self.move_back()


        self.as_result.success = True
        self._as.set_succeeded(self.as_result)


def create_receptacle_navigation_action():
    receptacle_navigate_action_client = actionlib.SimpleActionClient(
        "receptacle_navigator",
        NavigateToReceptaclesAction,
    )

    receptacle_navigate_action_client.wait_for_server()
    return receptacle_navigate_action_client


if __name__ == '__main__':
    rospy.init_node('receptacle_navigation_server_node')
    _ = ReceptacleNavigation()