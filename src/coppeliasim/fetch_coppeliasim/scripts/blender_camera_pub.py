import bpy
import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf2_ros
import math
import mathutils

class ImagePublisher:
    def __init__(self, topic='blender_camera/image_raw', rate=10):
        # Set the resolution of the output image
        bpy.context.scene.render.resolution_x = 640
        bpy.context.scene.render.resolution_y = 480

        # Set the render settings
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        bpy.context.scene.render.image_settings.color_depth = '8'
        
        self.pub = rospy.Publisher(topic, Image, queue_size=10)
        self.bridge = CvBridge()
        bpy.context.scene.render.filepath = "rendered_image.png"
        self.rate = rospy.Rate(rate)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # set camera's field of view
        # prepare later for rotate the camera.s
        self.rotation_quaternion = mathutils.Quaternion([1,0,0], math.pi)
        self.load_robot_arm()

        self.manipulable_object_names = ['book']
        self.manipulable_object_ids = {}
        self.load_manipulable_objects()

    def load_camera(self):
        desired_fov_radians = math.radians(60)
        self.camera = bpy.data.objects['Camera']
        self.camera.data.lens = (0.5 * self.camera.data.sensor_width) / math.tan(0.5 * desired_fov_radians)
        # set camera's near and far clipping planes
        self.camera.data.clip_start = 0.16 # near clipping plane 
        self.camera.data.clip_end = 10 # far clipping plane
        self.camera.rotation_mode = 'QUATERNION'

    def load_robot_arm(self):
        # self.robot_arm_link_names = ['wrist_flex_link', 'wrist_roll_link']
        self.robot_arm_link_names = ['elbow_flex_link', 'forearm_roll_link', 'shoulder_lift_link', 'shoulder_pan_link', 'upperarm_roll_link', 'wrist_flex_link', 'wrist_roll_link', 'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link']
        self.robot_arm_link_bpy_objs = {}
        for link_name in self.robot_arm_link_names:
            if 'gripper' in link_name:
                bpy.ops.import_scene.obj(filepath='/catkin_ws/src/fetch/fetch_ros/fetch_description/meshes/%s.obj'%(link_name))
            else:
                bpy.ops.wm.collada_import(filepath='/catkin_ws/src/fetch/fetch_ros/fetch_description/meshes/%s.dae'%(link_name))
            self.robot_arm_link_bpy_objs[link_name] = bpy.context.selected_objects[0]
            self.robot_arm_link_bpy_objs[link_name].rotation_mode = 'QUATERNION'

    def load_manipulable_objects(self):
        for object_name in self.manipulable_object_names:
            bpy.ops.import_scene.obj(filepath='/catkin_ws/src/objects_description/%s/%s.obj'%(object_name, object_name))
            self.manipulable_object_ids[object_name] = bpy.context.selected_objects[0]
            self.manipulable_object_ids[object_name].rotation_mode = 'QUATERNION'

    def render_object(self, bpy_obj, object_name):
            trans = self.tfBuffer.lookup_transform('world', object_name, rospy.Time())
            bpy_obj.location = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            object_rotation = mathutils.Quaternion((trans.transform.rotation.w,
                                                    trans.transform.rotation.x,
                                                    trans.transform.rotation.y,
                                                    trans.transform.rotation.z))
            bpy_obj.rotation_quaternion = object_rotation @ self.rotation_quaternion

    def render_robot_arm(self):
        for object_name in self.robot_arm_link_bpy_objs:
            self.render_object(self.robot_arm_link_bpy_objs[object_name], object_name)


    def render_manipulable_objects(self):
        for object_name in self.manipulable_object_names:
            self.render_object(self.manipulable_object_ids[object_name], object_name)


    def publish_image(self):
        while not rospy.is_shutdown():
            # get the transform from world to camera
            try:
                self.render_object(self.camera, 'head_camera_rgb_optical_frame')
                self.render_robot_arm()
                self.render_manipulable_objects()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                print("------------> Excepted")
                continue
            try:
                # Render the scene
                bpy.ops.render.render(write_still=True)

                render_result = bpy.data.images['Render Result']

                image = cv2.imread(bpy.context.scene.render.filepath)

                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")

            except Exception as e:
                # print(e)
                break

            self.pub.publish(ros_image)


if __name__ == '__main__':
    rospy.init_node('blender_cam_publisher', anonymous=True)

    image_publisher = ImagePublisher()

    # redirect output to log file
    logfile = '/root/blender_render.log'
    open(logfile, 'a').close()
    old = os.dup(sys.stdout.fileno())
    sys.stdout.flush()
    os.close(sys.stdout.fileno())
    fd = os.open(logfile, os.O_WRONLY)

    try:
        image_publisher.publish_image()
    except rospy.ROSInterruptException:
        pass

    # disable output redirection
    os.close(fd)
    os.dup(old)
    os.close(old)
