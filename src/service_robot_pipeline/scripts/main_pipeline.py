#! /usr/bin/env python

import rospy
import actionlib
from local_path_planner.msg import moveRobotBaseAction, moveRobotBaseActionGoal, moveRobotBaseActionResult, moveRobotBaseGoal
from tidy_module.srv import IdentifyMisplacedObjects, IdentifyMisplacedObjectsRequest, GetCorrectPlacements, GetCorrectPlacementsRequest


from manipulation.msg import PickupAction, PlaceAction, PickupGoal, PlaceGoal
from geometry_msgs.msg import Pose2D

from room_graph_navigator.msg import NavigateToRoomAction, NavigateToRoomGoal

from receptacle_navigator.msg import NavigateToReceptaclesAction, NavigateToReceptaclesGoal


class SRPipeline:
    def __init__(self):
        self.tidy_module_client = rospy.ServiceProxy(
            "objects_out_of_place_service", IdentifyMisplacedObjects
        )
        self.tidy_module_client.wait_for_service()

        self.move_fetch_base_client = actionlib.SimpleActionClient(
            "move_fetch_robot_base",
            moveRobotBaseAction,
        )
        self.move_fetch_base_client.wait_for_server()

        self.pickup_client = actionlib.SimpleActionClient(
            "pickup_server",
            PickupAction,
        )
        self.pickup_client.wait_for_server()
        print("Got pickup")

        self.place_client = actionlib.SimpleActionClient(
            "place_server",
            PlaceAction,
        )
        self.place_client.wait_for_server()
        print("Got manipulation")

        self.object_room_navigate_action_client = actionlib.SimpleActionClient(
            "object_room_navigator",
            NavigateToRoomAction,
        )
        self.object_room_navigate_action_client.wait_for_server()
        
        rospy.wait_for_service("correct_object_placement_service")
        self.correct_object_placement_client = rospy.ServiceProxy(
            "correct_object_placement_service", GetCorrectPlacements
        )
        print("Got tidy correct placement")

        self.receptacle_navigate_action_client = actionlib.SimpleActionClient(
            "receptacle_navigator",
            NavigateToReceptaclesAction,
        )
        self.receptacle_navigate_action_client.wait_for_server()
        print("Got room and receptacle")


    def runner(self):
        """
        
        """
        move_goal = moveRobotBaseGoal(
            pose = Pose2D(-0.01, -2.240, 0)
        )
        rospy.loginfo("Created goal")
        self.move_fetch_base_client.send_goal_and_wait(move_goal)
        rospy.loginfo("Sent goal")

        misplaced_objects_response = self.tidy_module_client(IdentifyMisplacedObjectsRequest()).object_locations
        print(misplaced_objects_response)

        for misplaced_object in misplaced_objects_response:
            print("> Misplaced object : ", misplaced_object)
            pickup_goal = PickupGoal(object_id = misplaced_object.object_id)
            self.pickup_client.send_goal_and_wait(pickup_goal)
            print("Picked up object ", misplaced_object)

            placement_candidates = self.correct_object_placement_client(GetCorrectPlacementsRequest(misplaced_object)).placements
            print("Candidates are : ", placement_candidates)

            for room_receptacles in placement_candidates.candidates:
                object_placed = False
                print("Navigating to goal room : ", room_receptacles.room)
                self.object_room_navigate_action_client.send_goal_and_wait(NavigateToRoomGoal(room = room_receptacles.room))
                print("Navigating to receptacles ", room_receptacles.receptacles)
                self.receptacle_navigate_action_client.send_goal_and_wait(NavigateToReceptaclesGoal(receptacles = room_receptacles.receptacles))
                break


if __name__ == '__main__':
    rospy.init_node('main_pipeline')
    instance = SRPipeline()
    instance.runner()

# Steps
'''
Steps for pipeline
1. local_planner(x_t, y_t) to go to table.
2. Call object out of place module to output (object) output. For now let us say 1
3. Call JH(object)
4. Call AP Object Room module: Input is (object), output is (x_room, y_room)
5. local_planner(x_room, y_room)
6. Call Sanmi's module to get (recepticles[]). Input is (object, room)
7. Call AP module with input(object, room, recepticles[]) and output is success. (Internally calls local_planner(x_rec, y_rec)). Returns success
8. JH drop module
'''