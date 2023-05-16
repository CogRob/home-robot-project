#! /usr/bin/env python

import rospy
import py_trees
import py_trees_ros
from behaviors import NavigateToRoomBehavior, NavigateToPose2DBehavior, NavigateToReceptacleBehavior, PickupBehavior, PlaceBehavior, GetObjectFromQueueBehavior, IDMisplacedObjectBehavior, GetPlacementCandidatesBehavior, GetReceptaclesLocationBehavior, RepeatUntilSuccessDecorator


if __name__ == '__main__':
    root = PickupBehavior(
        blackboard_key = "misplaced_object"
    )

    tree = py_trees_ros.trees.BehaviourTree(root, record_rosbag=False)
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set("misplaced_object", ["025_mug"])

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
