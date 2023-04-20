#! /usr/bin/env python

import rospy
from room_graph_navigator.srv import ObjectRoomIdentifier, ObjectRoomIdentifierRequest


def create_object_room_identifier_client():
    rospy.wait_for_service("object_room_identifier")
    object_room_identifier_client = rospy.ServiceProxy(
        "object_room_identifier", ObjectRoomIdentifier
    )

    return object_room_identifier_client

if __name__ == '__main__':
    rospy.init_node('object_room_identifier_test_node')
    service_client = create_object_room_identifier_client()

    goal_req = ObjectRoomIdentifierRequest(object_id = "sugarbox")
    response = service_client(goal_req)
    print(response)
    rospy.spin()
    