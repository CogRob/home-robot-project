#! /usr/bin/env python

import rospy
import actionlib
from manipulation.msg import PickupAction, PlaceAction, PickupResult, PlaceResult, SetJointsToActuateAction, SetJointsToActuateResult
import random
from local_path_planner.msg import moveHeadAction, moveHeadGoal


from coppeliasim_zmq.srv import AttachObjectToGripper, DetachObjectToGripper
from object_detector.srv import detect2DObject, detect2DObjectRequest
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandAction,
    GripperCommandGoal,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def get_zmq_clients():

    rospy.wait_for_service("attach_object_to_gripper_service")
    attach_object_to_gripper_service_client = rospy.ServiceProxy(
        "attach_object_to_gripper_service", AttachObjectToGripper
    )

    rospy.wait_for_service("detach_object_to_gripper_service")
    detach_object_to_gripper_service_client = rospy.ServiceProxy(
        "detach_object_to_gripper_service", DetachObjectToGripper
    )

    return attach_object_to_gripper_service_client, detach_object_to_gripper_service_client


class Manipulation(object):
    def __init__(self, isSim = True):
    
    
        self.object_detector_client = rospy.ServiceProxy(
            "detector_2d", detect2DObject
        )
        self.object_detector_client.wait_for_service()

        self.pickup_as = actionlib.SimpleActionServer("pickup_server", PickupAction, execute_cb=self.pickup_cb, auto_start = False)
        self.pickup_as.start()
        
        self.move_fetch_head_client = actionlib.SimpleActionClient("move_fetch_head", moveHeadAction)
        self.move_fetch_head_client.wait_for_server()


        self.place_as = actionlib.SimpleActionServer("place_server", PlaceAction, execute_cb=self.pickup_cb, auto_start = False)
        self.place_as.start()

        self.prepare_manip_as = actionlib.SimpleActionServer("prepare_manipulation_joints", SetJointsToActuateAction, execute_cb=self.prepare_manip_cb, auto_start = False)
        self.prepare_manip_as.start()


        if isSim:
            self.attach_client, self.detach_client = get_zmq_clients()
        self.torso_controller_client = actionlib.SimpleActionClient(
            "/torso_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        
        rospy.spin()

    def pickup_cb(self, request):

        object_id = request.object_id

        # call perception
        detections = self.object_detector_client()

        for detection in detections.detections.detections:
            if detection.object_id == object_id:
                break
        
        center_x, center_y, size_x, size_y = detection.bbox.center.x, detection.bbox.center.y, detection.bbox.size_x, detection.bbox.size_y


        # perform manipulation
        rospy.loginfo("Picking up ")
        print(object_id)

        # Change this
        pickup_as_result = PickupResult()

        move_head_joints = moveHeadGoal()
        move_head_joints.joint_values = [0.0, 0.25]

        rospy.loginfo("Turning head")
        self.move_fetch_head_client.send_goal(move_head_joints)
        self.move_fetch_head_client.wait_for_result()


        # self.attach_client(object_id=object_id)
        pickup_as_result.success = True
        rospy.loginfo("Picked up!")
        self.pickup_as.set_succeeded(pickup_as_result)


    def prepare_manip_cb(self, request):
        joint_names = ['shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'upperarm_roll_joint',
                        'elbow_flex_joint',
                        'forearm_roll_joint',
                        'wrist_flex_joint',
                        'wrist_roll_joint']
        joint_values = [-1.3089969389957472, -0.08726646259971647, -2.897246558310587, 1.3962634015954636, -1.8151424220741028, 1.8151424220741028, 1.1868238913561442]
        current_values = self.move_group.get_current_joint_values()
        self.move_group.set_joint_value_target(joint_values)
        plan = self.move_group.go()

        msg = FollowJointTrajectoryGoal()
        msg.trajectory.header.frame_id = ''
        msg.trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.35]
        point.time_from_start = rospy.Duration(1.0)

        msg.trajectory.header.stamp = rospy.Time.now()
        msg.trajectory.points = []
        msg.trajectory.points.append(point)

        self.torso_controller_client.send_goal_and_wait(
            msg, execute_timeout=rospy.Duration(2.0)
        )

        self.prepare_manip_as.set_succeeded(SetJointsToActuateResult(success=True))


    def place_cb(self, request):

        # call perception

        # perform manipulation
        rospy.loginfo("Placing object!")

        # Change this

        place_as_result = PlaceResult()
        place_as_result.success = random.choice([True, False])
        rospy.loginfo("Placed object!")
        self.place_as.set_succeeded(place_as_result)


def create_manipulation_clients():
    pickup_action_client = actionlib.SimpleActionClient(
        "pickup_server",
        PickupAction,
    )

    pickup_action_client.wait_for_server()

    place_action_client = actionlib.SimpleActionClient(
        "palce_server",
        PlaceAction,
    )

    place_action_client.wait_for_server()

    return pickup_action_client, place_action_client


if __name__ == '__main__':
    rospy.init_node('manipulation_node')
    action_server = Manipulation(isSim=False)