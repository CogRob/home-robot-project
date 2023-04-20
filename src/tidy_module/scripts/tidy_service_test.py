#! /usr/bin/env python

import rospy
from tidy_module.srv import GetCorrectPlacements, GetCorrectPlacementsRequest
from home_robot_msgs.msg import ObjectLocation

def correct_object_placement_service_client():
    rospy.wait_for_service("correct_object_placement_service")
    object_room_identifier_client = rospy.ServiceProxy(
        "correct_object_placement_service", GetCorrectPlacements
    )

    return object_room_identifier_client

if __name__ == '__main__':
    rospy.init_node('correct_object_placement_service_test_node')
    service_client = correct_object_placement_service_client()
    object_location = ObjectLocation(object_id = "sugarbox", room = "living", receptacle = "meeting_table")
    goal_req = GetCorrectPlacementsRequest(object_location = object_location)
    response = service_client(goal_req)
    print(response)
    rospy.spin()
    