#! /usr/bin/env python
import rospy
import actionlib
from room_graph_navigator.msg import NavigateToRoomAction, NavigateToRoomGoal


def create_object_room_navigation_action():
    object_room_navigate_action_client = actionlib.SimpleActionClient(
        "object_room_navigator",
        NavigateToRoomAction,
    )

    object_room_navigate_action_client.wait_for_server()
    return object_room_navigate_action_client


if __name__ == '__main__':
    rospy.init_node('init')
    rospy.loginfo("Inited")
    move_client = create_object_room_navigation_action()
    rospy.loginfo("Created client")
    object_goal = NavigateToRoomGoal(
        room = "office"
    )
    rospy.loginfo("Created goal")
    move_client.send_goal_and_wait(object_goal)
    rospy.loginfo("Sent goal")
    rospy.spin()
