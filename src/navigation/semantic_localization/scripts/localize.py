#! /usr/bin/env python

import rospy
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerResponse, SemanticLocationToPose, SemanticLocationToPoseResponse, PoseToSemanticLocation, PoseToSemanticLocationResponse
from home_robot_msgs.msg import NavTuple
from geometry_msgs.msg import Pose2D
import tf2_ros
import random
import json
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SemanticLocalizeService(object):
    def __init__(self):
        self.map = rospy.wait_for_message("map", OccupancyGrid)
        print("Got map")

        self.semantic_localize_service = rospy.Service(
            "semantic_localize", SemanticLocalizer, self.semantic_localize_callback
        )

        self.semantic_location_to_pose = rospy.Service(
            "semantic_location_to_pose", SemanticLocationToPose, self.semantic_location_to_pose_callback
        )

        # self.pose_semantic_location_server = rospy.Service(
        #     "pose_to_semantic_location", PoseToSemanticLocation, self.pose_to_semantic_location_callback
        # )


        self.semantic_map = np.load("/catkin_ws/src/navigation/semantic_localization/semantic_map/%s.npy"%rospy.get_param("map_filename"))
        self.semantic_map_labels = open("/catkin_ws/src/navigation/semantic_localization/semantic_map/class_names.txt", "r").read().splitlines()

        with open('/catkin_ws/src/navigation/semantic_localization/scripts/room_centers.json', 'r') as f:
            room_centers = json.load(f)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


        rospy.spin()


    def pixel_to_metric(self, pix_x, pix_y):
        """
        Returns the 2D location in map frame from pixels.
        Can work for any shape!
        """
        real_x = pix_x * self.map.info.resolution + self.map.info.origin.position.x
        real_y = (self.map.info.height - pix_y) * self.map.info.resolution + self.map.info.origin.position.y

        return real_x, real_y

    def metric_to_pixel(self, x, y):
        """
        Returns the pixel location in map from 2D location
        """
        grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = self.map.info.height - int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        # grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return grid_x, grid_y



    def semantic_localize_callback(self, request):
        """
        Returns the current room from current image and current location.
        Request contains: image (maybe not needed)
        """
        # rgb_image = request.rgbd_image.rgb
        # depth_image = request.rgbd_image.depth
        transform_se3 = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())

        (x, y) = (transform_se3.transform.translation.x, transform_se3.transform.translation.y)
        grid_x, grid_y = self.metric_to_pixel(x, y)

        semantic_label_no = self.semantic_map[grid_y, grid_x]
        semantic_label = self.semantic_map_labels[semantic_label_no]



        response_object = SemanticLocalizerResponse()
        response_object.room = semantic_label
        return response_object

    # def map_pixel_to_coord(self, request):

    def cur_pose_in_map(self, request):
        transform_se3 = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        (x, y) = (transform_se3.transform.translation[0], transform_se3.transform.translation[1])
        
        
        map_data = rospy.wait_for_message("map", nav_msgs.msg.OccupancyGrid)


        grid_x = x - int(map_data.info.origin.position.x) / map_data.info.resolution
        grid_y = y - int(map_data.info.origin.position.y) / map_data.info.resolution
        print(grid_x, grid_y)

        map_data_point = map.data[grid_y * map.info.width + grid_x]
        print(map_data_point)


    # def pose_to_semantic_location_callback(self, request):



    def semantic_location_to_pose_callback(self, request):
        """
        Returns the 2D position of the centroid of a room from the map.
        Request is made of: room
        """
        room_id = request.room
        response_object = SemanticLocationToPoseResponse()

        # change this!
        # with open('/catkin_ws/src/navigation/semantic_localization/scripts/room_centers.json', 'r') as f:
        #     room_centers = json.load(f)
        print("Need to go to room : ", room_id)
        room_pixel_locations = np.argwhere(self.semantic_map == self.semantic_map_labels.index(room_id))
        room_center_pixel_location = np.mean(room_pixel_locations, axis=0)
        room_center_metric_location = self.pixel_to_metric(room_center_pixel_location[1], room_center_pixel_location[0])
        print("Navigating to pose : ", room_center_metric_location)

        response_object.pose = Pose2D(room_center_metric_location[0], room_center_metric_location[1], 4.1)
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