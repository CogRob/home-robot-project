#! /usr/bin/env python

import rospy
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, CameraInfo
from receptacle_detector.srv import DetectReceptacle, DetectReceptacleResponse
from rostopic import get_topic_type
import numpy as np
import cv2
from cv_bridge import CvBridge
from home_robot_msgs.msg import NamedLocation
from geometry_msgs.msg import Point


class ReceptacleDetector(object):
    def __init__(self):
        self.latest_detections = None
        rgb_image_type, rgb_image_topic, _ = get_topic_type(rospy.get_param("~rgb_image_topic"), blocking = False)
        self.rgbcompressed_input = rgb_image_type == "sensor_msgs/CompressedImage"

        depth_image_type, depth_image_topic, _ = get_topic_type(rospy.get_param("~depth_image_topic"), blocking = False)
        self.depthcompressed_input = depth_image_type == "sensor_msgs/CompressedImage"

        self.receptacle_detector = rospy.Service(
            "receptacle_detector", DetectReceptacle, self.receptacle_detector_cb
        )
                                                
        if self.depthcompressed_input:
            self.depth_image_sub = rospy.Subscriber(
                                    rospy.get_param("~depth_image_topic"), CompressedImage, self.depth_callback, queue_size=1
                                                )
        else:
            self.depth_image_sub = rospy.Subscriber(
                                    rospy.get_param("~depth_image_topic"), Image, self.depth_callback, queue_size=1
                                                )
        self.camera_info = rospy.Subscriber(
                            rospy.get_param("~camera_info_topic"), CameraInfo, self.info_callback, queue_size=1
                                    )
        detections_subscriber = rospy.Subscriber("/detic_detections/detections", Detection2DArray, self.yolo_callback)
        self.bridge = CvBridge()
        rospy.spin()

    def depth_callback(self, msg):
        self.depthdata = msg


    def info_callback(self, msg):
        self.camera_info = msg

        

    def yolo_callback(self, detections):
        
        self.latest_detections = detections


    def receptacle_detector_cb(self, request):


        camera_info = self.camera_info
        K = np.array(camera_info.K).reshape((3,3))
        depthdata = self.depthdata
        if self.depthcompressed_input:
            depth = self.bridge.compressed_imgmsg_to_cv2(depthdata, desired_encoding='passthrough')
        else:
            depth = self.bridge.imgmsg_to_cv2(depthdata, desired_encoding='passthrough')

        response_object = DetectReceptacleResponse()
        xyz_image = cv2.rgbd.depthTo3d(depth, np.array(K).reshape((3,3)))

        for detection in self.latest_detections.detections:
            det_receptacle = NamedLocation()
            detection_mask = self.bridge.imgmsg_to_cv2(detection.segmented_image, desired_encoding='passthrough').astype(np.bool)
            xyzs_of_obj = xyz_image[detection_mask]
            centroid = np.mean(xyzs_of_obj[xyzs_of_obj[...,0] == xyzs_of_obj[...,0]], axis = 0)
            det_receptacle.name = detection.object_id
            if det_receptacle.name == "kitchen_countertop":
                det_receptacle.name = "countertop"
            det_receptacle.location = Point(centroid[0], centroid[1], centroid[2])
            response_object.receptacles.append(det_receptacle)


        return response_object



def create_detector_client():
    rospy.wait_for_service("receptacle_detector")
    detector_2d_client = rospy.ServiceProxy(
        "receptacle_detector", DetectReceptacle
    )

    return detector_2d_client

if __name__ == '__main__':
    rospy.init_node('detector_2d_server_node')
    detector_2d_server = ReceptacleDetector()
    
