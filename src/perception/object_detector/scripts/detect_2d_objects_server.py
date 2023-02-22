#! /usr/bin/env python

import rospy
from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from object_detector.srv import detect2DObject, detect2DObjectResponse


class Detector2DService(object):
    def __init__(self):
        self.detector_2d_service = rospy.Service(
            "detector_2d", detect2DObject, self.detector_2d_cb
        )

    def detector_2d_cb(self, request):

        rgb_image = request.rgb
        depth_image = request.depth

        response_object = detect2DObjectResponse()
        num_objects_detected = 3
        for detected_obj in range(num_objects_detected):
            detection = Detection2D()
            detection.bbox = BoundingBox2D() # center, x and y go here
            detection.results = ObjectHypothesisWithPose() # id, score and name and 6D pose go here
            response_object.detections.append(detection)

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
