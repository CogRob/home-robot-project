#! /usr/bin/env python

import os, sys
sys.path.append("/catkin_ws/src/service_robot_pipeline/scripts")


import rospy
import py_trees
import py_trees_ros
from behaviors import (
    NavigateToRoomBehavior,
    NavigateToPose2DBehavior,
    NavigateToReceptacleBehavior,
    PickupBehavior,
    PlaceBehavior,
    GetObjectFromQueueBehavior,
    IDMisplacedObjectBehavior,
    GetPlacementCandidatesBehavior,
    GetReceptaclesLocationBehavior,
    RepeatUntilSuccessDecorator,
    OpenDrawerBehavior,
    CloseDrawerBehavior,
    NotPlaceInside

)

from home_robot_msgs.msg import ObjectLocation
from manipulation.msg import PickupResult

from geometry_msgs.msg import Pose

if __name__ == "__main__":

    rospy.init_node("testpickupplace")
    place_behavior_complex = py_trees.composites.Sequence(
        "ComplexPlaceAction", memory=True
    )
    place_inside_selector = py_trees.composites.Selector("Place Object Inside", memory=True)

   
    place_in_drawer_sequence = py_trees.composites.Sequence(
        "PlaceInDrawer", memory=True
    )
    place_in_drawer_sequence.add_child(OpenDrawerBehavior(blackboard_key = "open_result"))
    place_in_drawer_sequence.add_child(PickupBehavior(blackboard_key="misplaced_object", need_to_place=True, place_point_blackboard_key="open_result"))
    place_in_drawer_sequence.add_child(CloseDrawerBehavior(blackboard_key = "open_result"))

    place_inside_selector.add_children(
        [
            NotPlaceInside(
                name = "check_if_place",
                blackboard_key="target_receptacle"
            ),
            place_in_drawer_sequence
        ]
    )

    place_behavior_complex.add_children([PlaceBehavior(blackboard_key="pickup_result"), place_inside_selector])


    pickupplace = py_trees.composites.Sequence(
        "PickupPlace", memory=True
    )
    pickupplace.add_children([
        PickupBehavior(blackboard_key="misplaced_object", need_to_place=False),
        place_behavior_complex
    ])

    tree = py_trees_ros.trees.BehaviourTree(pickupplace, record_rosbag=False)

    pickup_res = PickupResult()
    pickup_res.in_hand_pose = Pose()
    pickup_res.in_hand_pose.position.x = 0.221754837975
    pickup_res.in_hand_pose.position.y = -0.0268449915206
    pickup_res.in_hand_pose.position.z = -0.0109799421452
    pickup_res.in_hand_pose.orientation.x = 0.58121279337
    pickup_res.in_hand_pose.orientation.y = -0.490851553033
    pickup_res.in_hand_pose.orientation.z = -0.324862856213
    pickup_res.in_hand_pose.orientation.w = 0.561890172864

    pickup_res.object_pose_on_table = Pose()
    pickup_res.object_pose_on_table.position.x = 0.0
    pickup_res.object_pose_on_table.position.y = 0.0
    pickup_res.object_pose_on_table.position.z = 0.0686509609222
    pickup_res.object_pose_on_table.orientation.x = 0.0
    pickup_res.object_pose_on_table.orientation.y = 0.0
    pickup_res.object_pose_on_table.orientation.z = -0.415919182444
    pickup_res.object_pose_on_table.orientation.w = 0.909401579983

    pickup_res.object_width = 0.09769260883331299
    pickup_res.object_depth = 0.1234748512506485
    pickup_res.object_height = 0.06997275352478027

    
        # pickuprs.object_pose_in_hand
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("misplaced_object", ObjectLocation(object_id = "mug"))
    blackboard.set("target_receptacle", "drawer")
    # blackboard.set("pickup_result", pickup_res)


    tree.visitors.append(py_trees.visitors.DebugVisitor())
    tree.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=False, display_activity_stream=True
        )
    )
    tree.setup(timeout=15.0)
    # py_trees.display.render_dot_tree(root)
    # tree.tick_tock(
    #     period_ms=2000.0,
    # )

    while True:
        r = tree.tick()
        if tree.root.status == py_trees.common.Status.FAILURE:
            break

        if tree.root.status == py_trees.common.Status.SUCCESS:
            break
