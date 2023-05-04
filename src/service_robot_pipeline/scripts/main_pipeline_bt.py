#! /usr/bin/env python

import rospy
import py_trees
import py_trees_ros
from behaviors import NavigateToRoomBehavior, NavigateToPose2DBehavior, NavigateToReceptacleBehavior, PickupBehavior, PlaceBehavior, GetObjectFromQueueBehavior, IDMisplacedObjectBehavior, GetPlacementCandidatesBehavior, GetReceptaclesLocationBehavior, RepeatUntilSuccessDecorator
from py_trees.common import OneShotPolicy

from home_robot_msgs.msg import ObjectLocation

def create_bt():

    # place_obj_on_receptacle_behavior = py_trees.composites.Sequence("PlaceObjOnReceptacle")
    # place_obj_on_receptacle_behavior.add_children(
    #     [
    #         GetObjectFromQueueBehavior(
    #             name = "ReceptacleGetter",
    #             in_blackboard_key="available_receptacles",
    #             out_blackboard_key="target_receptacle"
    #         ),
    #         NavigateToReceptacleBehavior(
    #             blackboard_key="target_receptacle"
    #         ),
    #         PlaceBehavior(
    #             blackboard_key="target_receptacle"
    #         )
    #     ]
    # )

    # place_object_in_room_receptacle_behavior = py_trees.composites.Sequence("Put Away Object in Correct Location")
    # place_object_in_room_receptacle_behavior.add_children(
    #     [
    #         GetObjectFromQueueBehavior(
    #             name = "RoomGetter",
    #             in_blackboard_key = "placement_candidates",
    #             out_blackboard_key = "target_placement"
    #         ),
    #         NavigateToRoomBehavior(
    #             in_blackboard_key = "target_placement"
    #         ),
    #         GetReceptaclesLocationBehavior(
    #             in_blackboard_key = "target_placement",
    #             out_blackboard_key = "available_receptacles"
    #         ),
    #         RepeatUntilSuccessDecorator(
    #             name = "object_receptacle_place",
    #             child = place_obj_on_receptacle_behavior,
    #             policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    #         )
    #     ]
    # )

    # tidy_seq = py_trees.composites.Sequence("PickAndTransportObject", memory = True)
    # tidy_seq.add_children(
    #     [
    #         GetObjectFromQueueBehavior(
    #             name = "MisplacedObjectGetter",
    #             in_blackboard_key = "misplaced_objects",
    #             out_blackboard_key = "misplaced_object"
    #         ),

    #         PickupBehavior(
    #             blackboard_key = "misplaced_object"
    #         ),

    #         GetPlacementCandidatesBehavior(
    #             in_blackboard_key = "misplaced_object",
    #             out_blackboard_key = "placement_candidates",
    #         ),

    #         RepeatUntilSuccessDecorator(
    #             name = "put_away_object",
    #             child = place_object_in_room_receptacle_behavior,
    #             policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    #         )

    #     ]
    # )


    # tidy_bt = py_trees.composites.Sequence("TidyModule")
    # tidy_bt.add_children(
    #     [
    #         IDMisplacedObjectBehavior(
    #             out_blackboard_key = "misplaced_objects"
    #         ),
    #         RepeatUntilSuccessDecorator(
    #             name="tidy room",
    #             child=tidy_seq,
    #             policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    #         )


    #     ]
    # )


    place_obj_on_receptacle_behavior = py_trees.composites.Sequence("PlaceObjOnReceptacle", memory = True)
    place_obj_on_receptacle_behavior.add_children(
        [
            GetObjectFromQueueBehavior(
                name = "ReceptacleGetter",
                in_blackboard_key="available_receptacles",
                out_blackboard_key="target_receptacle"
            ),
            NavigateToReceptacleBehavior(
                blackboard_key="target_receptacle"
            ),
            PlaceBehavior(
                blackboard_key="target_receptacle"
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
                name = "object_receptacle_place",
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
                name = "put_away_object",
                child = place_object_in_room_receptacle_behavior,
                policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
            )
        ]
    )


    tidy_bt = py_trees.composites.Sequence("TidyModule")
    tidy_bt.add_children(
        [
            IDMisplacedObjectBehavior(
                out_blackboard_key = "misplaced_objects"
            ),
            RepeatUntilSuccessDecorator(
                name="tidy room",
                child=tidy_seq,
                policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
            )

        ]
    )


    return tidy_seq


#     navigation = py_trees_ros.actions.ActionClient("gototable")
    



if __name__ == '__main__':
    rospy.init_node('main_pipeline_bt')

    root = create_bt()
    tree = py_trees_ros.trees.BehaviourTree(root, record_rosbag=False)
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("misplaced_objects", [ObjectLocation("sugar_box", "room", "rec")])

    tree.visitors.append(py_trees.visitors.DebugVisitor())
    tree.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=False, display_activity_stream=True
        )
    )
    tree.setup(timeout=15.0)

    tree.tick_tock(
        period_ms=2000.0,
    )


    rospy.spin()
