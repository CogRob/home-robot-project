#! /usr/bin/env python

import rospy
from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from object_detector.srv import detect2DObject, detect2DObjectResponse