#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D
from receptacle_navigator.srv import GetGoalPoseForReceptacle, GetGoalPoseForReceptacleResponse, GetGoalPoseForReceptacleRequest
from receptacle_navigator.msg import NavigateToReceptaclesAction, NavigateToReceptaclesResult

from local_path_planner.msg import moveRobotBaseGoal, moveRobotBaseAction

from object_detector.srv import detect2DObject, detect2DObjectRequest

class ReceptacleNavigation(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer("receptacle_navigator", NavigateToReceptaclesAction, execute_cb=self.receptacles_navigate_cb, auto_start = False)
        self._as.start()
        self.move_fetch_base_client = actionlib.SimpleActionClient("move_fetch_robot_base", moveRobotBaseAction)
        self.move_fetch_base_client.wait_for_server()
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
        rospy.spin()

    def receptor_approach_pose_cb(self, request):
        receptacle = request.receptacle

        # Perform the straight line free space drawing.
        goal_pose = Pose2D(-2.742, 5.542, 0.0)
        response_object = GetGoalPoseForReceptacleResponse(goal_pose = goal_pose)
        return response_object



    def receptacles_navigate_cb(self, request):

        receptacles = request.receptacles

        # call a server to keep IDing receptacles.

        # Call a server to get a goal point for receptacle
        receptacle_goal_point = self.receptor_approach_pose_client(receptacle = receptacles[0])
        print(receptacle_goal_point)

        move_goal = moveRobotBaseGoal(
            pose = receptacle_goal_point.goal_pose
        )
        # print("Object must go to room ", response, " at ", room_location)

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