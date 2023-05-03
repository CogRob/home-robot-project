#! /usr/bin/env python

import rospy
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerResponse, SemanticLocationToPose, SemanticLocationToPoseResponse, PoseToSemanticLocation, PoseToSemanticLocationResponse
from home_robot_msgs.msg import NavTuple
from geometry_msgs.msg import Pose2D
import tf2_ros
import random
import json

class SemanticLocalizeService(object):
    def __init__(self):
        self.semantic_localize_service = rospy.Service(
            "semantic_localize", SemanticLocalizer, self.semantic_localize_callback
        )

        self.semantic_location_to_pose = rospy.Service(
            "semantic_location_to_pose", SemanticLocationToPose, self.semantic_location_to_pose_callback
        )
        with open('/catkin_ws/src/navigation/semantic_localization/scripts/room_centers.json', 'r') as f:
            room_centers = json.load(f)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()

    def semantic_localize_callback(self, request):
        """
        Returns the current room from current image and current location.
        Request contains: image (maybe not needed)
        """
        rgb_image = request.rgbd_image.rgb
        depth_image = request.rgbd_image.depth
        transform_se3 = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        response_object = SemanticLocalizerResponse()
        response_object.room = "room"
        return response_object


    def semantic_location_to_pose_callback(self, request):
        """
        Returns the 2D position of the centroid of a room from the map.
        Request is made of: room
        """
        room_id = request.room
        response_object = SemanticLocationToPoseResponse()

        # change this!
        with open('/catkin_ws/src/navigation/semantic_localization/scripts/room_centers.json', 'r') as f:
            room_centers = json.load(f)
        room_center = room_centers[room_id]
        response_object.pose = Pose2D(room_center[0], room_center[1], room_center[2])
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