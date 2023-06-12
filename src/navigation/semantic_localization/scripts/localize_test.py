#! /usr/bin/env python

import rospy
from semantic_localization.srv import SemanticLocalizer, SemanticLocalizerRequest
from home_robot_msgs.msg import NavTuple
from geometry_msgs.msg import Pose2D
import tf2_ros
import random
import json

if __name__ == '__main__':
    pose_to_semantic_location_client = rospy.ServiceProxy(
                "/semantic_localize", SemanticLocalizer
            )
    
    print(pose_to_semantic_location_client())