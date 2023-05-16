#! /usr/bin/env python

import rospy
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from object_detector.srv import detect2DObject, detect2DObjectResponse
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose2D


class Detector2DService(object):
    def __init__(self):
        self.detector_2d_service = rospy.Service(
            "detector_2d", detect2DObject, self.detector_2d_cb
        )

        detections_subscriber = rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.yolo_callback)
        self.latest_detections = None
        
        rospy.spin()

    def yolo_callback(self, bboxes):
        detections_2d = Detection2DArray()
        for bbox in bboxes.bounding_boxes:
            detection = Detection2D()
            center = ((bbox.xmax + bbox.xmin) / 2, (bbox.ymax + bbox.ymin) / 2) 
            center = Pose2D(center[0], center[1], 0.0)
            detection.bbox = BoundingBox2D(center, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin) # center, x and y go here
            detection.object_id = bbox.Class
            detection.confidence = bbox.probability
            detections_2d.detections.append(detection)
        
        self.latest_detections = detections_2d
        # print(self.latest_detections)


    def detector_2d_cb(self, request):

        response_object = detect2DObjectResponse()
        response_object.detections = self.latest_detections
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
    
