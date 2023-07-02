#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D
from room_graph_navigator.msg import NavigateToRoomAction, NavigateToRoomResult
from room_graph_navigator.srv import ObjectRoomIdentifier, ObjectRoomIdentifierRequest
from semantic_localization.srv import SemanticLocationToPose, SemanticLocationToPoseRequest

from local_path_planner.msg import moveRobotBaseGoal, moveRobotBaseAction
import random

from object_detector.srv import detect2DObject, detect2DObjectRequest

class ObjectRoomNavigation(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer("object_room_navigator", NavigateToRoomAction, execute_cb=self.semantic_navigate_cb, auto_start = False)
        self._as.start()
        self.move_fetch_base_client = actionlib.SimpleActionClient("move_fetch_robot_base", moveRobotBaseAction)
        self.move_fetch_base_client.wait_for_server()
        self.as_result = NavigateToRoomResult()
        rospy.loginfo("Created navigate to room!")

        # self.object_room_identifier_client = rospy.ServiceProxy(
        #     "/object_room_identifier", ObjectRoomIdentifier
        # )
        # self.object_room_identifier_client.wait_for_service()

        self.semantic_location_to_pose2D_client = rospy.ServiceProxy(
            "/semantic_location_to_pose", SemanticLocationToPose
        )
        self.semantic_location_to_pose2D_client.wait_for_service()
        rospy.loginfo("Created!")
        rospy.spin()

    def semantic_navigate_cb(self, request):

        room_location = self.semantic_location_to_pose2D_client(SemanticLocationToPoseRequest(room = request.room))
        print(room_location)
        move_goal = moveRobotBaseGoal(
            pose = room_location.pose
        )
        rospy.loginfo("Created goal")
        self.move_fetch_base_client.send_goal_and_wait(move_goal)
        rospy.loginfo("Sent goal")
        self.as_result.success = True
        self._as.set_succeeded(self.as_result)


def create_object_room_navigation_action():
    object_room_navigate_action_client = actionlib.SimpleActionClient(
        "object_room_navigator",
        NavigateToRoomAction,
    )

    object_room_navigate_action_client.wait_for_server()
    return object_room_navigate_action_client


if __name__ == '__main__':
    rospy.init_node('object_room_navigation_server_node')
    action_server = ObjectRoomNavigation()