#! /usr/bin/env python

import rospy
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from object_detector.srv import detect2DObject, detect2DObjectResponse
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image


class Detector2DService(object):
    def __init__(self):
        self.detector_2d_service = rospy.Service(
            "detector_2d", detect2DObject, self.detector_2d_cb
        )

        detections_subscriber = rospy.Subscriber("/detic_detections/detections", Detection2DArray, self.yolo_callback)
        self.latest_detections = None
        
        rospy.spin()

    def yolo_callback(self, detections):
        
        self.latest_detections = Detection2DArray()
        for detection in detections.detections:
            if detection.object_id in ['table', 'countertop', 'dustbin', 'sofa', 'cabinet']:
                continue
            new_detection = detection
            new_detection.object_id = detection.object_id.replace(" ", "").replace("_","")
            new_detection.segmented_image = Image()
            self.latest_detections.detections.append(new_detection)
        


        # print(self.latest_detections)


    def detector_2d_cb(self, request):

        response_object = detect2DObjectResponse()
        response_object.detections = self.latest_detections
        # print("RESP OBJECT : ", response_object)
        return response_object

def create_detector_client():
    rospy.wait_for_service("detector_2d")
    detector_2d_client = rospy.ServiceProxy(
        "detector_2d", detect2DObject
    )

    return detector_2d_client

if __name__ == '__main__':
    rospy.init_node('detector_2d_server_node')
    detector_2d_server = Detector2DService()
    
