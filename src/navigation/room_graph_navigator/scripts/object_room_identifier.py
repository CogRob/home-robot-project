#! /usr/bin/env python

import rospy
from room_graph_navigator.srv import ObjectRoomIdentifier, ObjectRoomIdentifierResponse
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest


class ObjectRoomIdentifierService(object):
    def __init__(self):
        self.object_room_identifier_service = rospy.Service(
            "object_room_identifier", ObjectRoomIdentifier, self.object_room_identifier_cb
        )
        rospy.loginfo("Created room identifier")
        self.pose_to_semantic_location_client = rospy.ServiceProxy(
            "/semantic_localize", SemanticLocalizer
        )
        self.pose_to_semantic_location_client.wait_for_service()

        rospy.spin()

    def object_room_identifier_cb(self, request):

        object_name = request.object_id
        cur_room = self.pose_to_semantic_location_client().room
        response_object = ObjectRoomIdentifierResponse()
        response_object.rooms.append(cur_room)
        return response_object

def create_object_room_identifier_client():
    rospy.wait_for_service("object_room_identifier")
    object_room_identifier_client = rospy.ServiceProxy(
        "object_room_identifier", ObjectRoomIdentifier
    )

    return object_room_identifier_client

if __name__ == '__main__':
    rospy.init_node('object_room_identifier_node')
    _ = ObjectRoomIdentifierService()
    
