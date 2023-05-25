#! /usr/bin/env python

import rospy
import actionlib
from manipulation.msg import PickupAction, PlaceAction, PickupResult, PlaceActionResult
import random

from coppeliasim_zmq.srv import AttachObjectToGripper, DetachObjectToGripper
from object_detector.srv import detect2DObject, detect2DObjectRequest


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
    
    
        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()

        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupAction, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        

        self.place_as = actionlib.SimpleActionServer("place_server", PlaceAction, execute_cb=self.pickup_cb, auto_start = False)
        self.place_as.start()

        if isSim:
            self.attach_client, self.detach_client = get_zmq_clients()
        
        rospy.spin()

    def pickup_cb(self, request):

        object_id = request.object_id

        # call perception
        detections = self.object_detector_client()

        for detection in detections.detections.detections:
            if detection.object_id == object_id:
                break
        
        center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y


        # perform manipulation
        rospy.loginfo("Picking up ")
        print(object_id)

        # Change this
        pickup_as_result = PickupResult()
        # self.attach_client(object_id=object_id)
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