#! /usr/bin/env python

import rospy

from tidy_module.srv import IdentifyMisplacedObjects, IdentifyMisplacedObjectsResponse, GetCorrectPlacements, GetCorrectPlacementsResponse
from home_robot_msgs.msg import RoomReceptacle, ObjectLocation

from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from object_detector.srv import detect2DObject, detect2DObjectRequest
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest


class TidyModule(object):
    def __init__(self):
        self.objects_out_of_place_service = rospy.Service(
            "objects_out_of_place_service", IdentifyMisplacedObjects, self.id_misplaced_objects_cb
        )
        self.correct_object_placement_service = rospy.Service(
            "correct_object_placement_service", GetCorrectPlacements, self.get_object_receptacles_cb
        )

        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()

        rospy.loginfo("Created room identifier")
        self.semantic_localize_client = rospy.ServiceProxy(
            "/semantic_localize", SemanticLocalizer
        )
        self.semantic_localize_client.wait_for_service()

    def id_misplaced_objects_cb(self, request):
        # rgb_image = request.rgbd_image.rgb
        # depth_image = request.rgbd_image.depth
        object_detections = self.object_detector_client()
        for detection in object_detections:
            object_id = detection.object_id
        
        # center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y


        cur_room = self.semantic_localize_client().room

        response_object = IdentifyMisplacedObjectsResponse()
        for detected_object in object_detections.detections.detections:
            obj_loc = ObjectLocation(object_id = "sugar_box", room = cur_room, receptacle = "receptacle")
            print(response_object)
            response_object.object_locations.append(obj_loc)
        return response_object

    def get_object_receptacles_cb(self, request):
        object_id = request.object_location.object_id
        cur_room = request.object_location.room
        cur_receptacle = request.object_location.receptacle
        response_object = GetCorrectPlacementsResponse()
        response_object.placements.object_id = object_id
        rooms = ['kitchen', 'living_room']
        for room in rooms:
            room_receptacles = RoomReceptacle()
            room_receptacles.room = room
            receptacles = ["coffee table", "countertop"]
            room_receptacles.receptacles = receptacles
            response_object.placements.candidates.append(room_receptacles)
        return response_object


if __name__ == '__main__':
    rospy.init_node('tidy_module')
    objects_out_of_place_server = TidyModule()
    rospy.spin()
    
