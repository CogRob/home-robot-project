#! /usr/bin/python

import numpy as np
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import cv2

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

class ObjectSegmentation:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = "/head_camera/seg/image_rect"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.seg_topic = "/head_camera/seg/image_rect_color"
        self.image_pub = rospy.Publisher(self.seg_topic, Image, queue_size=10)

    def setObjectHandleIds(self, handles):
        self.object_handles = handles
        self.object_colors = [np.random.choice(range(256), size=3) for _ in range(len(handles))]

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            data = cv2_img.copy()
            data[:,:,0] *= (256*256)
            data[:,:,1] *= 256
            data = np.sum(data, axis=2)
            resultimg = np.zeros(cv2_img.shape, dtype=np.uint8)
            for j in range(len(self.object_handles)):
                resultimg[data==self.object_handles[j]] = self.object_colors[j]
            output_image = self.bridge.cv2_to_imgmsg(resultimg, "bgr8")
            output_image.header = msg.header
            self.image_pub.publish(output_image)

        except CvBridgeError, e:
            print(e)

object_names = ['hammer_visual', 'can_visual', 'bottle_visual', 'book_visual', 'Cuboid']


if __name__ == '__main__':
    # main()
    rospy.init_node('fake_object_segmentation')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ('Connected to remote API server')
        handleIds = []
        for object_name in object_names:
            res, handleId = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                handleIds.append(handleId)
            else:
                print('Failed to get object handle!')

        object_segementation = ObjectSegmentation()
        object_segementation.setObjectHandleIds(handleIds)

        rospy.spin()

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
