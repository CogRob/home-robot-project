#! /usr/bin/env python

import rospy
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerResponse, SemanticLocationToPose, SemanticLocationToPoseResponse
from semantic_localization.msg import NavTuple
from geometry_msgs.msg import Pose2D
import tf2_ros
import random

class SemanticLocalizeService(object):
    def __init__(self):
        self.semantic_localize_service = rospy.Service(
            "semantic_localize", SemanticLocalizer, self.semantic_localize_callback
        )
        self.semantic_location_to_pose = rospy.Service(
            "semantic_location_to_pose", SemanticLocationToPose, self.semantic_location_to_pose_callback
        )
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def semantic_localize_callback(self, request):

        rgb_image = request.rgb
        depth_image = request.depth

        transform_se3 = self.tfBuffer.lookup_transform('/map', '/base_link', rospy.Time())

        response_object = SemanticLocalizerResponse()
        response_object.location = NavTuple()
        response_object.location.room = "room"
        response_object.location.place = "place"
        return response_object

    def semantic_location_to_pose_callback(self, request):

        location = request.location

        response_object = SemanticLocationToPoseResponse()
        response_object.pose = Pose2D(random.random() * 10, random.random() * 10, random.random() * 3.14)
        return response_object

def localizer_client():
    rospy.wait_for_service("semantic_localize")
    semantic_localize_client = rospy.ServiceProxy(
        "semantic_localize", SemanticLocalizer
    )
    semantic_location_to_pose_client = rospy.ServiceProxy(
        "semantic_location_to_pose", SemanticLocationToPose
    )

    return semantic_localize_client, semantic_location_to_pose_client

if __name__ == '__main__':
    rospy.init_node('semantic_localize_server_node')
    semantic_localize_server = SemanticLocalizeService()