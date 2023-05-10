import bpy
import numpy as np
import sys
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import tf2_ros
import tf
import math
import mathutils


def transform_stamped_to_array(transform):
    trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
    rot = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]

    return trans, rot

def array_to_transform_stamped(transform_array):

    trans, rot = transform_array
    transform = tf2_ros.TransformStamped()
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]

    transform.transform.rotation.x = rot[0]
    transform.transform.rotation.y = rot[1]
    transform.transform.rotation.z = rot[2]
    transform.transform.rotation.w = rot[3]

    return transform


def transformProduct(t1, t2):
    trans1, rot1 = transform_stamped_to_array(t1)
    # trans1[2] += 0.9
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)
    # print(mat1)

    trans2, rot2 = transform_stamped_to_array(t2)
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)


    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return array_to_transform_stamped([trans3, rot3])
    return array_to_transform_stamped([trans1, rot1])


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

        self.manipulable_object_names = ['mug', 'pitcher', 'banana', 'marker', 'mustard_bottle']
        self.manipulable_object_ids = {}
        self.load_manipulable_objects()
        self.load_camera()

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
            bpy.ops.import_scene.obj(filepath='/catkin_ws/src/objects_description/%s/textured.obj'%(object_name))
            self.manipulable_object_ids[object_name] = bpy.context.selected_objects[0]
            self.manipulable_object_ids[object_name].rotation_mode = 'QUATERNION'

    def render_object(self, bpy_obj, object_name, trans = None):
            if trans is None:
                trans = self.tfBuffer.lookup_transform('world', object_name, rospy.Time())
            bpy_obj.location = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            object_rotation = mathutils.Quaternion((trans.transform.rotation.w,
                                                    trans.transform.rotation.x,
                                                    trans.transform.rotation.y,
                                                    trans.transform.rotation.z))
            bpy_obj.rotation_quaternion = object_rotation @ self.rotation_quaternion

    def render_robot_arm(self):
        for object_name in self.robot_arm_link_bpy_objs:
            robot_base_trans = self.tfBuffer.lookup_transform('world', "fetch_robot_link_sim", rospy.Time())
            obj_trans_in_base_link = self.tfBuffer.lookup_transform("fetch_robot_link", object_name, rospy.Time())
            trans = transformProduct(robot_base_trans, obj_trans_in_base_link)

            self.render_object(self.robot_arm_link_bpy_objs[object_name], None, trans)


    def render_manipulable_objects(self):
        for object_name in self.manipulable_object_names:
            self.render_object(self.manipulable_object_ids[object_name], object_name)


    def publish_image(self):
        while not rospy.is_shutdown():
            # get the transform from world to camera
            try:
                robot_base_trans = self.tfBuffer.lookup_transform('world', "fetch_robot_link_sim", rospy.Time())
                camera_trans_in_base_link = self.tfBuffer.lookup_transform("fetch_robot_link", "head_camera_rgb_optical_frame", rospy.Time())
                trans = transformProduct(robot_base_trans, camera_trans_in_base_link)

                self.render_object(self.camera, None, trans)
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
                print(e)
                break

            self.pub.publish(ros_image)
            # print("PUBLISHED")


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
