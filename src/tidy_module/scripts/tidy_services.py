#! /usr/bin/env python

import json
import rospy

from tidy_module.srv import IdentifyMisplacedObjects, IdentifyMisplacedObjectsResponse, GetCorrectPlacements, GetCorrectPlacementsResponse
from home_robot_msgs.msg import RoomReceptacle, ObjectLocation

from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from object_detector.srv import detect2DObject, detect2DObjectRequest
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest
import numpy as np
import pandas as pd
import collections
import operator
from ast import literal_eval

class TidyModule(object):
    def __init__(self):
        # Saving code 
        data_path = "/catkin_ws/src/tidy_module/data"
        objects = json.load(open("{}/objects.json".format(data_path),"r"))
        rooms = json.load(open("{}/rooms.json".format(data_path),"r"))
        room_receps = json.load(open("{}/room_receps.json".format(data_path),"r"))
        data = pd.read_csv("{}/housekeepdata.csv".format(data_path))
        usr_matrix =  pd.read_csv("{}/Usr_Matrix_HomeRobo.csv".format(data_path), index_col = 0, converters={'1':literal_eval})
        # usr_matrix =  pd.read_excel("{}/Usr_Matrix_HomeRobo.xlsx".format(data_path), engine="openpyxl", index_col = 0)
        usrs_idx = 26

        our_object_list = sorted(["masterchefcan", "crackerbox", "mustardbottle", "tomatosoupcan", 
                                "mug", "pottedmeatcan", "bleachcleanser", "gelatinbox"])

        our_room_list = sorted(["kitchen", "diningroom", "livingroom", "corridor", "homeoffice"])
        mergeable_rooms = {"pantryroom": "kitchen", "lobby": "corridor", "storageroom": "kitchen"}

        our_recep_list = ['chair', 'coffeemachine', 'coffeetable', 'counter', 'shelf', 'sofa', 'table']
        mergeable_receps = {"sofachair": "sofa", "officechair": "chair", "coffeetable" : "table", "counter" : "countertop"}

        self.kg_dict = self.get_kg_dict(objects, rooms, room_receps, data, our_object_list, our_room_list, mergeable_rooms, our_recep_list, mergeable_receps)

        self.global_misplaced_dict = self.get_usr_misplaced_dict(usr_matrix,usrs_idx, our_object_list,our_room_list, mergeable_rooms, our_recep_list, mergeable_receps)
        # self.global_misplaced_dict = self.get_global_misplaced_dict(objects, rooms, room_receps, data, our_object_list, our_room_list, mergeable_rooms, our_recep_list, mergeable_receps)

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
        print("All done!")


    def get_kg_dict(self, objects, rooms, room_receps, data, our_object_list, our_room_list, mergeable_rooms, our_recep_list, mergeable_receps):
        kg = {}
        for id in range(len(data)):
            row = data.loc[id]

            object_name = objects[row['object_idx']].encode("utf-8").replace("_", "")
            if object_name not in our_object_list:
                continue
            elif object_name not in kg.keys():
                kg[object_name] = {}

            room_name = rooms[row['room_idx']].encode("utf-8").replace("_", "")
            if room_name not in our_room_list:
                if room_name not in mergeable_rooms.keys():
                    continue
                else:
                    room_name = mergeable_rooms[room_name]

            correct_receps = [room_receps[r].split('|')[1].encode("utf-8").replace("_", "") for r in eval(row['correct'])]
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
                kg[object_name][room_name] = collections.OrderedDict(sorted(recep_dict.items(), key=operator.itemgetter(1), reverse=True))
                kg[object_name][room_name]["total"] = sum(kg[object_name][room_name].values())
            one = operator.itemgetter(1)
            special = operator.itemgetter('total')
            kg[object_name] = collections.OrderedDict(sorted(kg[object_name].items(), key=lambda x: special(one(x)), reverse=True))
        
        return kg

    def get_usr_misplaced_dict(self,usr_matrix, usrs_idx, our_object_list,our_room_list, mergeable_rooms, our_recep_list, mergeable_receps):

        usr_matrix = usr_matrix.loc[our_object_list,str(usrs_idx)]
        # usr_matrix = usr_matrix.loc[our_object_list][usrs_idx]
        # Filter usr_matrix by objects in our list and 
        for obj in usr_matrix.index:
            object_pref = usr_matrix.loc[obj]
            object_pref = eval(object_pref)
            # object_pref =  [ x.replace("_","") for x in object_pref]
            for i in range(len(object_pref)):
                object_pref[i] = object_pref[i].replace("_","")
                # obj,room,recp = object_pref[i].split('|')
                obj = object_pref[i].split('|')[0]
                room = object_pref[i].split('|')[1]
                recp = object_pref[i].split('|')[2]
                if room in mergeable_rooms.keys():
                    room = mergeable_rooms[room]
                if recp in mergeable_receps.keys():
                    recp = mergeable_receps[recp]
                object_pref[i] = obj + "|" + room + "|" + recp

            usr_matrix.loc[obj] = object_pref
        return usr_matrix.to_dict()

    def get_global_misplaced_dict(self, objects, rooms, room_receps, data, our_object_list, our_room_list, mergeable_rooms, our_recep_list, mergeable_receps):
        global_misplaced_dict = {}
        for id in range(len(data)):
            row = data.loc[id]

            object_name = objects[row['object_idx']].encode("utf-8").replace("_", "")
            if object_name not in our_object_list:
                continue

            room_name = rooms[row['room_idx']].encode("utf-8").replace("_", "")
            if room_name not in our_room_list:
                if room_name not in mergeable_rooms.keys():
                    continue
                else:
                    room_name = mergeable_rooms[room_name]

            misplaced_receps = [room_receps[r].split('|')[1].encode("utf-8").replace("_", "") for r in eval(row['misplaced'])]
            if not misplaced_receps:
                continue

            for recep_rank, recep in enumerate(misplaced_receps):
                if recep not in our_recep_list:
                    if recep not in mergeable_receps.keys():
                        continue
                    else:
                        recep = mergeable_receps[recep]
                
                key = "{}|{}|{}".format(object_name,room_name,recep)
                value = -(len(misplaced_receps)-recep_rank)

                if key not in global_misplaced_dict.keys():
                    global_misplaced_dict[key] = value
                else:
                    global_misplaced_dict[key]+=value

        return global_misplaced_dict


    def return_out_of_place_usr_pref(self,found_objects):
        out_of_place_list = []
        
        for obj_room_recep in found_objects:
            object_name, room_name, recep = obj_room_recep.object_id, obj_room_recep.room, obj_room_recep.receptacle
            key = "{}|{}|{}".format(object_name,room_name,recep)
            if key not in self.global_misplaced_dict[object_name]:
                out_of_place_list.append(tuple(key.split("|")))
        return out_of_place_list


    def return_out_of_place(self, found_objects):
        out_of_place_dict = {}
        
        for obj_room_recep in found_objects:

            object_name, room_name, recep = obj_room_recep.object_id, obj_room_recep.room, obj_room_recep.receptacle
            key = "{}|{}|{}".format(object_name,room_name,recep)
            
            if key in self.global_misplaced_dict.keys():
                out_of_place_dict[key] = self.global_misplaced_dict[key]
            else:
                out_of_place_dict[key] = 0

        out_of_place_dict = collections.OrderedDict(sorted(out_of_place_dict.items(), key=operator.itemgetter(1)))
        
        out_of_place_list = []
        for item in out_of_place_dict.keys():
            obj, room, rec = item.split("|")
            out_of_place_list.append(ObjectLocation(object_id = obj, room = room, receptacle = rec))
        
        return [tuple(item.split("|")) for item in out_of_place_dict.keys()]


    def id_misplaced_objects_cb(self, request):
        # rgb_image = request.rgbd_image.rgb
        # depth_image = request.rgbd_image.depth
        object_detections = self.object_detector_client()
        for detection in object_detections.detections.detections:
            object_id = detection.object_id
            print("Object detected : ", object_id)
        
        # print(object_detections)
        # center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y

        # TODO: Call the out of place module here as needed. Check example below.

        # homeoffice", "table
        cur_room = self.semantic_localize_client().room

        all_objects = []
        for detected_object in object_detections.detections.detections:
            obj_loc = ObjectLocation(object_id = detected_object.object_id, room = cur_room, receptacle = "table")
            all_objects.append(obj_loc)

        oop_objects = self.return_out_of_place_usr_pref(all_objects)
        # oop_objects = self.return_out_of_place(all_objects)

        response_object = IdentifyMisplacedObjectsResponse()
        for detected_object in oop_objects:
            # obj_loc = ObjectLocation(object_id = "mug", room = cur_room, receptacle = "office_desk")
            obj_loc = ObjectLocation(object_id = detected_object[0], room = detected_object[1], receptacle = detected_object[2])
            response_object.object_locations.append(obj_loc)

        print("Misplaced objects : ", response_object)
        return response_object

    def get_object_receptacles_cb(self, request):
        print("Requesting candidates for  : ", request)
        object_id = request.object_location.object_id
        cur_room = request.object_location.room
        cur_receptacle = request.object_location.receptacle
        response_object = GetCorrectPlacementsResponse()
        response_object.placements.object_id = object_id
        print(object_id)


        objkg = self.kg_dict[object_id]
        obj_usr_placement_list = self.global_misplaced_dict[object_id]

        room_recep_list = []

        for room, _ in  objkg.items():
            room_receptacles = RoomReceptacle()
            receptacal_l = []
            for placement in obj_usr_placement_list:
                if room in placement:
                    receptacal_l.append(placement.split("|")[2])
                
            room_receptacles.room = room
            room_receptacles.receptacles = receptacal_l
            response_object.placements.candidates.append(room_receptacles)

        print("Candidates are : ", response_object)

        return response_object

      
    def get_object_receptacles_cb_OLD(self, request):

        print("Requesting candidates for  : ", request)
        object_id = request.object_location.object_id
        cur_room = request.object_location.room
        cur_receptacle = request.object_location.receptacle
        response_object = GetCorrectPlacementsResponse()
        response_object.placements.object_id = object_id
        print(object_id)
        # object out of place "name_ID"

        objkg = self.kg_dict[object_id]
        room_recep_list = []
        
        for room, recep_dict in objkg.items():
            recep_dict.pop('total')
            # room_recep_list.append([room,list(recep_dict.keys())])

            room_receptacles = RoomReceptacle()
            room_receptacles.room = room
            room_receptacles.receptacles = list(recep_dict.keys())
            response_object.placements.candidates.append(room_receptacles)

        print("Candidates are : ", response_object)
        # [ ROOM, [CANDIDATES], ROOM, []]
        return response_object


if __name__ == '__main__':
    rospy.init_node('tidy_module')
    objects_out_of_place_server = TidyModule()
    rospy.spin()
    
