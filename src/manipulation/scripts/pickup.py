#! /usr/bin/env python

import rospy
import actionlib
from manipulation.msg import PickupAction, PickupActionGoal, PickupActionResult
import random

class Manipulation(object):
    def __init__(self):
        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupActionGoal, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        self.pickup_as_result = PickupActionResult()

    def pickup_cb(self, request):

        rgb_image = request.rgb
        depth_image = request.depth
        object_id = request.object_id

        # call perception

        # perform manipulation
        rospy.loginfo("Picking up!")

        # Change this

        self.self.pickup_as_result = random.choice([True, False])
        rospy.loginfo("Picked up!")
        self.pickup_as.set_succeeded(self.pickup_as_result)


def create_semantic_navigation_action():
    pickup_action_client = actionlib.SimpleActionClient(
        "pickup_server",
        PickupAction,
    )

    pickup_action_client.wait_for_server()
    return pickup_action_client


if __name__ == '__main__':
    rospy.init_node('manipulation_node')
    action_server = Manipulation()