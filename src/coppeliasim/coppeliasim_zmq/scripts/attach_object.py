#!/usr/bin/env python
import rospy
from zmqRemoteApi import RemoteAPIClient
from coppeliasim_zmq.srv import AttachObjectToGripper, AttachObjectToGripperResponse, DetachObjectToGripper, DetachObjectToGripperResponse

class ZMQServer:
    def __init__(self):
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

        self.attach_object_to_gripper_service = rospy.Service(
            "attach_object_to_gripper_service", AttachObjectToGripper, self.attach_object_to_gripper_service_cb
        )
        self.detach_object_to_gripper_service = rospy.Service(
            "detach_object_to_gripper_service", DetachObjectToGripper, self.detach_object_to_gripper_service_cb
        )

        rospy.spin()

    def attach_object_to_gripper_service_cb(self, request):
        obj_name = request.object_id
        obj_handle = self.sim.getObject("/" + obj_name)
        gripper_handle = self.sim.getObject("/gripper_attachment")
        val = self.sim.setObjectParent(obj_handle, gripper_handle, True)

        return AttachObjectToGripperResponse(success=True)

    def detach_object_to_gripper_service_cb(self, request):
        obj_name = request.object_id
        obj_handle = self.sim.getObject("/" + obj_name)
        val = self.sim.setObjectParent(obj_handle, -1, True)

        return DetachObjectToGripperResponse(success=True)


if __name__ == '__main__':
    rospy.init_node("zmq_server")
    rospy.loginfo("Inited!")
    zmq_srv = ZMQServer()
    rospy.spin()