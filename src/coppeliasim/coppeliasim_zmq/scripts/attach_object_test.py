#!/usr/bin/env python
import rospy
from coppeliasim_zmq.srv import AttachObjectToGripper, DetachObjectToGripper

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


if __name__ == '__main__':
    rospy.init_node("zmq_test")

    a,d = get_zmq_clients()
    r1 = a(object_id = "sugarbox")
    # print(r1)
    # rospy.sleep(5.0)
    # r1 = d(object_id = "hook_respondable")
    # print(r1)
