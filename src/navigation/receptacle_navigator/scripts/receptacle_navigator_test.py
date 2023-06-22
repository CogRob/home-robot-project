#! /usr/bin/env python
import rospy
import actionlib
from receptacle_navigator.msg import NavigateToReceptacleAction, NavigateToReceptacleGoal
from home_robot_msgs.msg import NamedLocation
from geometry_msgs.msg import Point

def create_receptacle_navigation_action():
    receptacle_navigate_action_client = actionlib.SimpleActionClient(
        "receptacle_navigator",
        NavigateToReceptacleAction,
    )

    receptacle_navigate_action_client.wait_for_server()
    return receptacle_navigate_action_client


if __name__ == '__main__':
    rospy.init_node('init')
    rospy.loginfo("Inited")
    move_client = create_receptacle_navigation_action()
    rospy.loginfo("Created client")
    object_goal = NavigateToReceptacleGoal(
        receptacle = NamedLocation(name="table", location = Point(x=-2.73720573049, y = -3.02744987667, z = 0.0))
    )
    rospy.loginfo("Created goal")
    move_client.send_goal_and_wait(object_goal)
    rospy.loginfo("Sent goal")
    rospy.spin()
