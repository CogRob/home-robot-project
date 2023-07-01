#! /usr/bin/env python

import rospy
import py_trees
import py_trees_ros
from queue_maker import GetObjectFromQueue

import actionlib
from local_path_planner.msg import (
    moveRobotBaseAction,
    moveRobotBaseActionGoal,
    moveRobotBaseActionResult,
    moveRobotBaseGoal,
)
from tidy_module.srv import (
    IdentifyMisplacedObjects,
    IdentifyMisplacedObjectsRequest,
    GetCorrectPlacements,
    GetCorrectPlacementsRequest,
)


from manipulation.msg import (
    PickupAction,
    PlaceAction,
    OpenDrawerAction,
    CloseDrawerAction,
    PickupGoal,
    PlaceGoal,
    OpenDrawerGoal,
    CloseDrawerGoal,
    SetJointsToActuateAction,
    SetJointsToActuateGoal,
)
from geometry_msgs.msg import Pose2D

from room_graph_navigator.msg import NavigateToRoomAction, NavigateToRoomGoal
from receptacle_navigator.msg import (
    NavigateToReceptacleAction,
    NavigateToReceptacleGoal,
)
from receptacle_navigator.srv import (
    GetReceptacleLocations,
    GetReceptacleLocationsRequest,
)

from copy import deepcopy


class RepeatUntilSuccessDecorator(py_trees.decorators.OneShot):
    def update(self):
        """
        Bounce if the child has already successfully completed.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """

        for child in self.children:
            child.setup(timeout=5.0)

        if self.final_status:
            self.logger.debug("{}.update()[bouncing]".format(self.__class__.__name__))
            return self.final_status
        if self.decorated.status in self.policy.value:
            return self.decorated.status
        elif self.decorated.status == py_trees.common.Status.INVALID:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING


class NavigateToRoomBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, in_blackboard_key, out_blackboard_key=None):
        super(NavigateToRoomBehavior, self).__init__(
            name="RoomNavigator",
            action_namespace="object_room_navigator",
            action_spec=NavigateToRoomAction,
            blackboard_key=in_blackboard_key,
        )

    def initialise(self):
        self.action_goal = NavigateToRoomGoal()
        self.action_goal.room = self.blackboard.goal.room
        super(NavigateToRoomBehavior, self).initialise()


class NavigateToPose2DBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key):
        super(NavigateToPose2DBehavior, self).__init__(
            name="GotoPose",
            action_namespace="move_fetch_robot_base",
            action_spec=moveRobotBaseAction,
            blackboard_key=blackboard_key,
        )

    def initialise(self):
        self.action_goal = moveRobotBaseGoal()
        self.action_goal.pose = self.blackboard.goal.pose
        super(NavigateToPose2DBehavior, self).initialise()


class NavigateToReceptacleBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key):
        super(NavigateToReceptacleBehavior, self).__init__(
            name="ReceptacleNavigator",
            action_namespace="receptacle_navigator",
            action_spec=NavigateToReceptacleAction,
            blackboard_key=blackboard_key,
        )

    def initialise(self):
        self.action_goal = NavigateToReceptacleGoal()
        self.action_goal.receptacle = self.blackboard.goal
        super(NavigateToReceptacleBehavior, self).initialise()


class OpenDrawerBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key, out_blackboard_key="open_result"):
        super(OpenDrawerBehavior, self).__init__(
            name="OpenDrawer",
            action_namespace="open_drawer_server",
            action_spec=OpenDrawerAction,
            blackboard_key=blackboard_key,
            # out_blackboard_key = out_blackboard_key,
        )

        self.blackboard.register_key(
            key="out",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )

    def update(self):
        status = super(OpenDrawerBehavior, self).update()
        if status == py_trees.Status.SUCCESS:
            self.set_response_to_blackboard()
        return status

    def set_response_to_blackboard(self):
        opendrawer_res = {}
        for goal_attribute in self.action_result.__slots__:
            if goal_attribute != "success":
                opendrawer_res[goal_attribute] = getattr(
                    self.action_result, goal_attribute
                )
        print(opendrawer_res)
        self.blackboard.set("out", opendrawer_res)


class CloseDrawerBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key):
        """
        blackboard key must be from opendrawer
        """
        super(CloseDrawerBehavior, self).__init__(
            name="CloseDrawer",
            action_namespace="close_drawer_server",
            action_spec=CloseDrawerAction,
            blackboard_key=blackboard_key,
        )

    def initialise(self):
        self.action_goal = CloseDrawerGoal()

        for goal_attribute in self.action_goal.__slots__:
            setattr(
                self.action_goal, goal_attribute, self.blackboard.goal[goal_attribute]
            )

        super(CloseDrawerBehavior, self).initialise()


class NotPlaceInside(py_trees.behaviour.Behaviour):
    """Gets a location name from the queue"""

    def __init__(self, name, blackboard_key):
        super(NotPlaceInside, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="goal",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", blackboard_key
            ),
        )
    def update(self):
        """Checks for the status of the navigation action"""
        if "cabinet" in self.blackboard.goal.name or "drawer" in self.blackboard.goal.name:
            return py_trees.common.Status.FAILURE
        print("RETURNING SUCCESS")
        return py_trees.common.Status.SUCCESS


class PickupBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key, out_blackboard_key="pickup_result", need_to_place = False, place_point_blackboard_key = None):
        super(PickupBehavior, self).__init__(
            name="PickupObject",
            action_namespace="pickup_server",
            action_spec=PickupAction,
            blackboard_key=blackboard_key,
            # out_blackboard_key = out_blackboard_key,
        )

        self.blackboard.register_key(
            key="out",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )
        self.need_to_place = need_to_place
        if self.need_to_place:
            self.blackboard.register_key(
                key="place_point_key",
                access=py_trees.common.Access.READ,
                remap_to=py_trees.blackboard.Blackboard.absolute_name(
                    "/", place_point_blackboard_key
                ),
            )
            

    def update(self):
        status = super(PickupBehavior, self).update()
        if status == py_trees.Status.SUCCESS:
            self.set_response_to_blackboard()
        return status

    def initialise(self):
        self.action_goal = PickupGoal()
        self.action_goal.object_id = self.blackboard.goal.object_id
        if self.need_to_place:
            self.action_goal.need_to_place = True
            print("Result of opening : ", self.blackboard.place_point_key['place_point'])
            self.action_goal.place_point = self.blackboard.place_point_key['place_point']
        else:
            self.action_goal.need_to_place = False

        super(PickupBehavior, self).initialise()

    def set_response_to_blackboard(self):
        pickup_res = {}
        for goal_attribute in self.action_result.__slots__:
            if goal_attribute != "success":
                pickup_res[goal_attribute] = getattr(self.action_result, goal_attribute)
        print(pickup_res)
        self.blackboard.set("out", pickup_res)


class PlaceBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self, blackboard_key):
        super(PlaceBehavior, self).__init__(
            name="PlaceObject",
            action_namespace="place_server",
            action_spec=PlaceAction,
            blackboard_key=blackboard_key,
        )

    def initialise(self):
        self.action_goal = PlaceGoal()

        for goal_attribute in self.action_goal.__slots__:
            setattr(
                self.action_goal, goal_attribute, self.blackboard.goal[goal_attribute]
            )

        super(PlaceBehavior, self).initialise()


class PrepareToActuateBehavior(py_trees_ros.actions.FromBlackBoard):
    def __init__(self):
        super(PrepareToActuateBehavior, self).__init__(
            name="PrepareJointsForManip",
            action_namespace="prepare_manipulation_joints",
            action_spec=SetJointsToActuateAction,
            blackboard_key="prepare_joints_key",
        )

    def initialise(self):
        self.action_goal = SetJointsToActuateGoal()
        super(PrepareToActuateBehavior, self).initialise()


class GetObjectFromQueueBehavior(py_trees.behaviour.Behaviour):
    """Gets a location name from the queue"""

    def __init__(self, name, in_blackboard_key, out_blackboard_key):
        super(GetObjectFromQueueBehavior, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="object_list",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", in_blackboard_key
            ),
        )

        self.blackboard.register_key(
            key=out_blackboard_key,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )
        self.out_blackboard_key = out_blackboard_key

    def initialise(self):
        """
        Reset it
        """
        super(GetObjectFromQueueBehavior, self).initialise()
        self.sent_goal = False

    def update(self):
        # print("Calling update of get from queue : ", self.name)
        objects_list = self.blackboard.object_list
        if len(objects_list) == 0:
            self.logger.info("No more objects available available")
            return py_trees.common.Status.INVALID
        else:
            chosen_object = objects_list.pop(0)
            # self.logger.info("Object chosen is : %s"%chosen_object)
            self.blackboard.set(self.out_blackboard_key, chosen_object)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info("Terminated with status %s" % new_status)


class IDMisplacedObjectBehavior(py_trees_ros.services.FromBlackBoard):
    def __init__(self, out_blackboard_key, in_blackboard_key=None):
        super(IDMisplacedObjectBehavior, self).__init__(
            name="IDMisplacedObject",
            action_namespace="objects_out_of_place_service",
            action_spec=IdentifyMisplacedObjects,
            blackboard_key="",
        )

        self.blackboard.register_key(
            key="out",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )

    def initialise(self):
        self.action_goal = IdentifyMisplacedObjectsRequest()
        # action_goal.object_id = self.blackboard.goal
        super(IDMisplacedObjectBehavior, self).initialise()

    def update(self):
        status = super(IDMisplacedObjectBehavior, self).update()
        if status == py_trees.Status.SUCCESS:
            self.set_response_to_blackboard()
        return status

    def set_response_to_blackboard(self):
        self.blackboard.set("out", list(deepcopy(self.action_result.object_locations)))


class GetPlacementCandidatesBehavior(py_trees_ros.services.FromBlackBoard):
    def __init__(self, in_blackboard_key, out_blackboard_key):
        super(GetPlacementCandidatesBehavior, self).__init__(
            name="GetPlacementCandidates",
            action_namespace="correct_object_placement_service",
            action_spec=GetCorrectPlacements,
            blackboard_key=in_blackboard_key,
        )

        self.blackboard.register_key(
            key="out",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )

    def initialise(self):
        self.action_goal = GetCorrectPlacementsRequest()
        self.action_goal.object_location = self.blackboard.goal
        super(GetPlacementCandidatesBehavior, self).initialise()

    def set_response_to_blackboard(self):
        self.blackboard.set(
            "out", list(deepcopy(self.action_result.placements.candidates))
        )

    def update(self):
        status = super(GetPlacementCandidatesBehavior, self).update()
        if status == py_trees.Status.SUCCESS:
            self.set_response_to_blackboard()
        return status


class GetReceptaclesLocationBehavior(py_trees_ros.services.FromBlackBoard):
    def __init__(self, in_blackboard_key, out_blackboard_key):
        super(GetReceptaclesLocationBehavior, self).__init__(
            name="GetReceptaclesLocation",
            action_namespace="available_receptacles",
            action_spec=GetReceptacleLocations,
            blackboard_key=in_blackboard_key,
        )

        self.blackboard.register_key(
            key="out",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", out_blackboard_key
            ),
        )

    def initialise(self):
        self.action_goal = GetReceptacleLocationsRequest()
        self.action_goal.receptacles = self.blackboard.goal.receptacles
        super(GetReceptaclesLocationBehavior, self).initialise()

    def set_response_to_blackboard(self):
        self.blackboard.set(
            "out", list(deepcopy(self.action_result.receptacle_locations))
        )

    def update(self):
        status = super(GetReceptaclesLocationBehavior, self).update()
        if status == py_trees.Status.SUCCESS:
            self.set_response_to_blackboard()
        return status
