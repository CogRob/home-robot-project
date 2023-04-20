#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D
from object_goal_navigation.msg import objectNavigateAction, objectNavigateActionResult

from local_path_planner.msg import moveRobotBaseGoal, moveRobotBaseAction
import random

from object_detector.srv import detect2DObject, detect2DObjectRequest

class ObjectNavigation(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer("object_navigate_action_server", objectNavigateAction, execute_cb=self.semantic_navigate_cb, auto_start = False)
        self._as.start()
        self.goto_kp_client = actionlib.SimpleActionClient("move_fetch_robot_base", moveRobotBaseAction)
        self.goto_kp_client.wait_for_server()
        self.as_result = objectNavigateActionResult()


        self.object_detector_client = rospy.ServiceProxy(
            "/perception/detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()
        rospy.spin()


    def semantic_navigate_cb(self, request):

        rgb_image = request.rgbd_image.rgb
        depth_image = request.rgbd_image.depth
        # object_id = request.object_id

        goto_keypoint_goal = moveRobotBaseGoal()


        # call perception
        object_detections = self.object_detector_client(detect2DObjectRequest(rgb = rgb_image, depth = depth_image))

        # get current position using semantic_localization with "semantic_localize_callback" server

        # Change this
        # NOTE: anwesan. Make use of semanticlocalizer.srv defined in semantic_localization package.
        while True:
            goto_keypoint_goal.pose = Pose2D((random.random() * 5 + 8), -1 * random.random() * 5, random.random() * 3.14)
            print(goto_keypoint_goal.pose)
            self.goto_kp_client.send_goal(goto_keypoint_goal)
            self.goto_kp_client.wait_for_result()

            if random.choice([True, False]):
                self.as_result = random.choice([True, False])
                break
        
        self._as.set_succeeded(self.as_result)


def object_goal_navigation_action():
    semantic_navigate_action_client = actionlib.SimpleActionClient(
        "object_navigate_action_server",
        objectNavigateAction,
    )

    semantic_navigate_action_client.wait_for_server()
    return semantic_navigate_action_client


if __name__ == '__main__':
    rospy.init_node('object_navigation_server_node')
    action_server = ObjectNavigation()