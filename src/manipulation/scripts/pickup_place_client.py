#! /usr/bin/env python

from manipulation.msg import PickupAction, PlaceAction, PickupGoal, PlaceGoal, PickupResult, SetJointsToActuateAction, SetJointsToActuateGoal
# from manipulation.msg import PickupAction, PlaceAction, PickupGoal, PlaceGoal, PickupResult

import actionlib
import rospy
import actionlib_msgs.msg as actionlib_msgs
from geometry_msgs.msg import Pose

class ManipulationClient:
    def __init__(self):
        self.pickup_client = actionlib.SimpleActionClient(
            "pickup_server",
            PickupAction
        )
        self.pickup_client.wait_for_server()

        self.place_client = actionlib.SimpleActionClient(
            "place_server",
            PlaceAction
        )
        self.place_client.wait_for_server()

        self.prepare_manip_client = actionlib.SimpleActionClient(
            "prepare_manipulation_joints",
            SetJointsToActuateAction
        )
        self.prepare_manip_client.wait_for_server()


    def run_pickup(self):
        object_id = "mug"
        self.pickup_client.send_goal(PickupGoal(object_id = object_id))
        self.pickup_client.wait_for_result()
        action_result = self.pickup_client.get_result()
        print("Pickup status : ", action_result)
        return action_result


    def run_place(self, pickup_result):
        place_goal = PlaceGoal()
        place_goal.in_hand_pose = pickup_result.in_hand_pose
        place_goal.object_pose_on_table = pickup_result.object_pose_on_table
        place_goal.object_width = pickup_result.object_width
        place_goal.object_depth = pickup_result.object_depth
        place_goal.object_height = pickup_result.object_height
        


        self.place_client.send_goal(place_goal)
        self.place_client.wait_for_result()
        action_result = self.place_client.get_result()
        print("Place status : ", action_result)
        return action_result


    def run_prepare(self):
        prepare_goal = SetJointsToActuateGoal()
        self.prepare_manip_client.send_goal(prepare_goal)
        self.prepare_manip_client.wait_for_result()
        action_result = self.prepare_manip_client.get_result()
        print("Prepare status : ", action_result)
        return action_result


if __name__ == '__main__':
    rospy.init_node('man_cl')
    mc = ManipulationClient()

    prepare = mc.run_prepare()
    pickup_res = mc.run_pickup()
    rospy.sleep(5.0)
    if not pickup_res.success:
        failed
    # pickup_res = PickupResult()
    # pickup_res.in_hand_pose = Pose()
    # pickup_res.in_hand_pose.position.x = 0.226730982629
    # pickup_res.in_hand_pose.position.y = -0.0185393874218
    # pickup_res.in_hand_pose.position.z = 0.0247130575363
    # pickup_res.in_hand_pose.orientation.x = 0.789566315608
    # pickup_res.in_hand_pose.orientation.y = -0.178046484257
    # pickup_res.in_hand_pose.orientation.z = -0.455373750769
    # pickup_res.in_hand_pose.orientation.w = 0.370835852921

    # pickup_res.object_pose_on_table = Pose()
    # pickup_res.object_pose_on_table.position.x = 0.0
    # pickup_res.object_pose_on_table.position.y = 0.0
    # pickup_res.object_pose_on_table.position.z = 0.0442141890526
    # pickup_res.object_pose_on_table.orientation.x = 0.0
    # pickup_res.object_pose_on_table.orientation.y = 0.0
    # pickup_res.object_pose_on_table.orientation.z = -0.0390930795187
    # pickup_res.object_pose_on_table.orientation.w = 0.999235573393

    # pickup_res.object_width = 0.116749048233
    # pickup_res.object_depth = 0.0923977047205
    # pickup_res.object_height = 0.074226140976

    place_res = mc.run_place(pickup_res)
    rospy.spin()