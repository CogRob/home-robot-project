#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Point
from receptacle_navigator.srv import GetGoalPoseForReceptacle, GetGoalPoseForReceptacleResponse, GetGoalPoseForReceptacleRequest, GetReceptacleLocations, GetReceptacleLocationsRequest, GetReceptacleLocationsResponse
from receptacle_navigator.msg import NavigateToReceptaclesAction, NavigateToReceptaclesResult, NavigateToReceptacleAction, NavigateToReceptacleResult

from local_path_planner.msg import moveRobotBaseGoal, moveRobotBaseAction, moveHeadAction, moveHeadGoal
from object_detector.srv import detect2DObject, detect2DObjectRequest


from home_robot_msgs.msg import NamedLocation

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


        self.present_receptacles = rospy.Service(
            "available_receptacles", GetReceptacleLocations, self.get_ordered_receptacle_locations
        )
        rospy.loginfo("Created available receptacles")
        rospy.wait_for_service("available_receptacles")


        rospy.spin()

    def receptor_approach_pose_cb(self, request):
        receptacle = request.receptacle

        

        # Perform the straight line free space drawing.
        if receptacle.name == "shelf":
            goal_pose = Pose2D(-2.75, -1.00, 0.0)
        if receptacle.name == "table":
            goal_pose = Pose2D(0.0, -3.1, 1.58)
        if receptacle.name == "counter":
            goal_pose = Pose2D(1.3, -2.9, -3.14)

        print("Goal pose is : ", goal_pose)

        # goal_pose = Pose2D(-1.124, 6.8556, 0.0)
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

        # call object segmenter to get xyz.
        # det_receptacles = self.receptacle_detector_client()
        # for detected_receptacle in det_receptacles.receptacles:
        #     if detected_receptacle.name not in found_receptacles_dict:
        #         found_receptacles_dict[detected_receptacle.name] = detected_receptacle.location

        found_receptacles_dict["counter"] = Point(0.0, -2.5, 0.0)

        return found_receptacles_dict

    def get_ordered_receptacle_locations(self,request):
        receptacles = request.receptacles


        found_receptacles_dict = {}

        # Turn 45 degrees right
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [-0.75, 0.25])
        # Turn 90 degrees right
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [-1.40, 0.25])
        # Turn 45 degrees left
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [0.75, 0.25])
        # Turn 90 degrees left
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [1.40, 0.25])
        # Turn center
        found_receptacles_dict = self.rotate_and_detect_receptacles(found_receptacles_dict, [0.0, 0.25])

        #scan the room and keep calling receptacles
        # rotate the base and call receptacle detector
        # store the 3D locations of the detectors found
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