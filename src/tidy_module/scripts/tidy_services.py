#! /usr/bin/env python

import rospy

from tidy_module.srv import IdentifyMisplacedObjects, IdentifyMisplacedObjectsResponse, GetCorrectPlacements, GetCorrectPlacementsResponse
from home_robot_msgs.msg import RoomReceptacle, ObjectLocation

from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from object_detector.srv import detect2DObject, detect2DObjectRequest
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest
import numpy as np



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

        self.kg_dict = self.get_kg_dict(data_path = "/catkin_ws/src/tidy_module/data/housekeep.npy")

    def get_kg_dict(self, data_path):
        # data_path = 'housekeep.npy'
        data_dict = np.load(data_path, allow_pickle=True, encoding='utf-8').item()

        objects = data_dict['objects']
        rooms = data_dict['rooms']
        room_receps = data_dict['room_receptacles']
        data = data_dict['data']

        our_object_list = sorted(["master_chef_can", "cracker_box", "sugar_box", "mustard_bottle", "tomato_soup_can", 
                                "mug", "potted_meat_can", "banana", "bleach_cleanser", "gelatin_box", "foam_brick"])

        our_room_list = sorted(["kitchen", "dining_room", "living_room", "corridor", "home_office"])
        mergeable_rooms = {"pantry_room": "kitchen", "lobby": "corridor", "storage_room": "kitchen"}

        our_recep_list = ['chair', 'coffee_machine', 'coffee_table', 'counter', 'shelf', 'sofa', 'table']
        mergeable_receps = {"sofa_chair": "sofa", "office_chair": "chair"}


        kg = {}
        for id in range(len(data)):
            row = data.loc[id]

            object_name = objects[row['object_idx']]
            if object_name not in our_object_list:
                continue
            elif object_name not in kg.keys():
                kg[object_name] = {}

            room_name = rooms[row['room_idx']]
            if room_name not in our_room_list:
                if room_name not in mergeable_rooms.keys():
                    continue
                else:
                    room_name = mergeable_rooms[room_name]

            correct_receps = [room_receps[r].split('|')[1] for r in row['correct']]
            if not correct_receps:
                continue

            if room_name not in kg[object_name].keys():
                kg[object_name][room_name] = {}

            for recep_rank, recep in enumerate(correct_receps):
                if recep not in our_recep_list:
                    if recep not in mergeable_receps.keys():
                        continue
                    else:
                        recep = mergeable_receps[recep]

                if recep not in kg[object_name][room_name].keys():
                    kg[object_name][room_name][recep] = len(correct_receps)-recep_rank
                else:
                    kg[object_name][room_name][recep]+= len(correct_receps)-recep_rank

        for object_name, room_dict in kg.items():
            for room_name, recep_dict in room_dict.items():
                kg[object_name][room_name] = {k: v for k, v in sorted(kg[object_name][room_name].items(), key=lambda item: item[1], reverse=True)}
                kg[object_name][room_name]["total"] = sum(kg[object_name][room_name].values())
            kg[object_name] = {k: v for k, v in sorted(kg[object_name].items(), key=lambda item: item[1]['total'], reverse=True)}
        
        return kg
    


    def id_misplaced_objects_cb(self, request):
        # rgb_image = request.rgbd_image.rgb
        # depth_image = request.rgbd_image.depth
        object_detections = self.object_detector_client()
        for detection in object_detections.detections.detections:
            object_id = detection.object_id
            print("Object detected : ", object_id)
        
        # print(object_detections)
        # center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y


        cur_room = self.semantic_localize_client().room

        response_object = IdentifyMisplacedObjectsResponse()
        for detected_object in object_detections.detections.detections:
            # obj_loc = ObjectLocation(object_id = "mug", room = cur_room, receptacle = "office_desk")
            obj_loc = ObjectLocation(object_id = detected_object.object_id, room = cur_room, receptacle = "office_desk")
            response_object.object_locations.append(obj_loc)
        return response_object

    def get_object_receptacles_cb(self, request):
        object_id = request.object_location.object_id
        cur_room = request.object_location.room
        cur_receptacle = request.object_location.receptacle
        response_object = GetCorrectPlacementsResponse()
        response_object.placements.object_id = object_id

        objkg = self.kg_dict[object_id]
        room_recep_list = []
        
        for room, recep_dict in objkg.items():
            recep_dict.pop('total')
            room_recep_list.append([room,list(recep_dict.keys())])

            room_receptacles = RoomReceptacle()
            room_receptacles.room = room
            room_receptacles.receptacles = list(recep_dict.keys())
            response_object.placements.candidates.append(room_receptacles)

        print("Candidates are : ", response_object)
        return response_object


if __name__ == '__main__':
    rospy.init_node('tidy_module')
    objects_out_of_place_server = TidyModule()
    rospy.spin()
    
