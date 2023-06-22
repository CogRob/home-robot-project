#! /usr/bin/env python

import rospy
import py_trees
import py_trees_ros
from behaviors import NavigateToRoomBehavior, NavigateToPose2DBehavior, NavigateToReceptacleBehavior, PickupBehavior, PlaceBehavior, GetObjectFromQueueBehavior, IDMisplacedObjectBehavior, GetPlacementCandidatesBehavior, GetReceptaclesLocationBehavior, RepeatUntilSuccessDecorator, PrepareToActuateBehavior
from py_trees.common import OneShotPolicy

from home_robot_msgs.msg import ObjectLocation, NamedLocation

def create_bt():

    place_obj_on_receptacle_behavior = py_trees.composites.Sequence("PlaceObjOnReceptacle", memory = True)
    place_obj_on_receptacle_behavior.add_children(
        [
            GetObjectFromQueueBehavior(
                name = "ReceptacleGetter",
                in_blackboard_key="available_receptacles",
                out_blackboard_key="target_receptacle"
            ),
            PrepareToActuateBehavior(
            ),
            NavigateToReceptacleBehavior(
                blackboard_key="target_receptacle"
            ),
            PlaceBehavior(
                blackboard_key="pickup_result"
            )
        ]
    )

    place_object_in_room_receptacle_behavior = py_trees.composites.Sequence("Put Away Object in Correct Location", memory = True)
    place_object_in_room_receptacle_behavior.add_children(
        [
            GetObjectFromQueueBehavior(
                name = "RoomGetter",
                in_blackboard_key = "placement_candidates",
                out_blackboard_key = "target_placement"
            ),
            NavigateToRoomBehavior(
                in_blackboard_key = "target_placement"
            ),
            GetReceptaclesLocationBehavior(
                in_blackboard_key = "target_placement",
                out_blackboard_key = "available_receptacles"
            ),
            RepeatUntilSuccessDecorator(
                name = "PlaceObjDecorator",
                child = place_obj_on_receptacle_behavior,
                policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
            )
        ]
    )

    tidy_seq = py_trees.composites.Sequence("PickAndTransportObject", memory = True)
    tidy_seq.add_children(
        [
            GetObjectFromQueueBehavior(
                name = "MisplacedObjectGetter",
                in_blackboard_key = "misplaced_objects",
                out_blackboard_key = "misplaced_object"
            ),

            PickupBehavior(
                blackboard_key = "misplaced_object"
            ),

            GetPlacementCandidatesBehavior(
                in_blackboard_key = "misplaced_object",
                out_blackboard_key = "placement_candidates",
            ),

            RepeatUntilSuccessDecorator(
                name = "PutAwayObjDec",
                child = place_object_in_room_receptacle_behavior,
                policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
            )
        ]
    )


    tidy_bt = py_trees.composites.Sequence("TidyModule", memory = True)
    tidy_bt.add_children(
        [
            # PrepareToActuateBehavior(),
            GetObjectFromQueueBehavior(
                name = "PotentialReceptaclesGetter",
                in_blackboard_key = "all_receptacles_available",
                out_blackboard_key = "potential_receptacle"
            ),

            # NavigateToReceptacleBehavior(
            #     blackboard_key="potential_receptacle"
            # ),

            IDMisplacedObjectBehavior(
                out_blackboard_key = "misplaced_objects"
            ),
            RepeatUntilSuccessDecorator(
                name="TidyRoomDec",
                child=tidy_seq,
                policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
            )

        ]
    )


    return tidy_bt


#     navigation = py_trees_ros.actions.ActionClient("gototable")
    



if __name__ == '__main__':
    rospy.init_node('main_pipeline_bt')

    root = create_bt()
    tree = py_trees_ros.trees.BehaviourTree(root, record_rosbag=False)
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("all_receptacles_available", [NamedLocation(name="table")])

    tree.visitors.append(py_trees.visitors.DebugVisitor())
    tree.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=False, display_activity_stream=True
        )
    )
    tree.setup(timeout=15.0)
    # py_trees.display.render_dot_tree(root)
    tree.tick_tock(
        period_ms=2000.0,
    )

    rospy.spin()
