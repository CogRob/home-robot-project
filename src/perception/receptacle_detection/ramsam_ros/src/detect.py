#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import supervision as sv
import torch
import torchvision
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type
import message_filters
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from segmentation_msgs.msg import SegmentationMask, SegmentationMasks
# import tf2_ros

from ramsam_ros.srv import DetectReceptacle, DetectReceptacleResponse
from geometry_msgs.msg import Point
from home_robot_msgs.msg import NamedLocation

# import pyrealsense2
# add gsa submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "Grounded-Segment-Anything"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from gsa submodules
from groundingdino.util.inference import Model
from segment_anything import sam_model_registry, SamPredictor

@torch.no_grad()
class RAMSAMDetector:
    def __init__(self):
        self.box_thres = rospy.get_param("~box_threshold")
        self.text_thres = rospy.get_param("~text_threshold")
        self.nms_thres = rospy.get_param("~nms_threshold")
        # Initialize config and weights
        GROUNDING_DINO_CONFIG_PATH = rospy.get_param("~config_path")
        gdino_weights = rospy.get_param("~gdino_weights")

        SAM_ENCODER_VERSION = rospy.get_param("~sam_encoder_version")
        sam_weights = rospy.get_param("~sam_weights")

        self.device = rospy.get_param("~device","")

        # Building GroundingDINO inference model
        self.grounding_dino_model = Model(model_config_path=GROUNDING_DINO_CONFIG_PATH, model_checkpoint_path=gdino_weights, device=self.device)

        # Building SAM Model and SAM Predictor
        sam = sam_model_registry[SAM_ENCODER_VERSION](checkpoint=sam_weights)
        self.sam_predictor = SamPredictor(sam)

        # Initialize subscriber to Image/CompressedImage topic
        print("RGB TOPIC : ", rospy.get_param("~rgb_image_topic"))
        rgb_image_type, rgb_image_topic, _ = get_topic_type(rospy.get_param("~rgb_image_topic"), blocking = False)
        self.rgbcompressed_input = rgb_image_type == "sensor_msgs/CompressedImage"

        depth_image_type, depth_image_topic, _ = get_topic_type(rospy.get_param("~depth_image_topic"), blocking = False)
        self.depthcompressed_input = depth_image_type == "sensor_msgs/CompressedImage"

        self.receptacle_detector = rospy.Service(
            "receptacle_detector", DetectReceptacle, self.service_callback
        )


        # if self.rgbcompressed_input:
        #     self.rgbimage_sub = message_filters.Subscriber(rgb_image_topic, CompressedImage)
        # else:
        #     self.rgbimage_sub = message_filters.Subscriber(rgb_image_topic, Image)

        # if self.depthcompressed_input:
        #     self.depthimage_sub = message_filters.Subscriber(depth_image_topic, CompressedImage)
        # else:
        #     self.depthimage_sub = message_filters.Subscriber(depth_image_topic, Image)
        
        # _, camera_info_topic, _ = get_topic_type(rospy.get_param("~camera_info_topic"), blocking = False)
        # self.camerainfo_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)

        # ts = message_filters.TimeSynchronizer([self.rgbimage_sub, self.depthimage_sub, self.camerainfo_sub], queue_size=10)
        # ts = message_filters.ApproximateTimeSynchronizer([self.rgbimage_sub, self.depthimage_sub], queue_size=10, slop = 0.5)
        # ts.registerCallback(self.callback)

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), SegmentationMasks, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()
        self.roomwise_receptacles = {"living_room": ["table", "sofa", "long cabinet", "window"], "home_office": ["table", "cabinet"], "kitchen": ["kitchen countertop"]}


        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()


    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def cv2_to_imgmsg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

    def service_callback(self, request):
        rgbdata = rospy.wait_for_message(rospy.get_param("~rgb_image_topic"), Image, timeout=5.0)
        depthdata = rospy.wait_for_message(rospy.get_param("~depth_image_topic"), Image, timeout=5.0)
        camera_info = rospy.wait_for_message(rospy.get_param("~camera_info_topic"), CameraInfo, timeout=5.0)
        K = np.array(camera_info.K).reshape((3,3))
        # K = np.array([527.3758609346917, 0.0, 326.6388366771264, 0.0, 523.6181455086474, 226.4866800158784, 0.0, 0.0, 1.0]).reshape((3,3))

        response_object = DetectReceptacleResponse()


        if self.rgbcompressed_input:
            rgb = self.bridge.compressed_imgmsg_to_cv2(rgbdata, desired_encoding="bgr8")
        else:
            # rgb = self.imgmsg_to_cv2(rgbdata)
            rgb = self.bridge.imgmsg_to_cv2(rgbdata, desired_encoding="bgr8")
            # rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        # if self.depthcompressed_input:
        #     depth = self.bridge.compressed_imgmsg_to_cv2(depthdata, desired_encoding='passthrough')
        # else:
        #     depth = self.bridge.imgmsg_to_cv2(depthdata, desired_encoding='passthrough')

        receptacle_categories = self.roomwise_receptacles[request.room]
        # receptacle_categories = ["table", "sofa", "cabinet", "shelf", "countertop"]
        # receptacle_categories = ["sofa", "countertop", "table", "cabinet", "shelf"]
        # receptacle_categories = ["sofa", "countertop", "table", "long cabinet", "shelf"]
        # cv2.imwrite("/root/ramsam_ws/src/ramsam_ros/src/grounded_sam_annotated_image_rgb.jpg", rgb)

        print("Asking ground dino")
        try:
            detections = self.grounding_dino_model.predict_with_classes(
                image=rgb,
                classes=receptacle_categories,
                box_threshold=self.box_thres,
                text_threshold=self.text_thres
            )
        except:
            return response_object


        # NMS post process
        nms_idx = torchvision.ops.nms(
            torch.from_numpy(detections.xyxy), 
            torch.from_numpy(detections.confidence), 
            self.nms_thres
        ).numpy().tolist()

        # print(detections)

        detections.xyxy = detections.xyxy[nms_idx]
        detections.confidence = detections.confidence[nms_idx]
        detections.class_id = detections.class_id[nms_idx]


        # Prompting SAM with detected boxes
        def segment(sam_predictor: SamPredictor, image: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
            sam_predictor.set_image(image)
            result_masks = []
            for box in xyxy:
                masks, scores, logits = sam_predictor.predict(
                    box=box,
                    multimask_output=True
                )
                index = np.argmax(scores)
                result_masks.append(masks[index])
            return np.array(result_masks)

        # convert detections to masks
        detections.mask = segment(
            sam_predictor=self.sam_predictor,
            image=cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB),
            xyxy=detections.xyxy
        )

        # annotate image with detections
        box_annotator = sv.BoxAnnotator()
        mask_annotator = sv.MaskAnnotator()
        labels = [
            f"{receptacle_categories[class_id]} {confidence:0.2f}" 
            for _, _, confidence, class_id, _ 
            in detections]
        annotated_image = mask_annotator.annotate(scene=rgb.copy(), detections=detections)
        annotated_image = box_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)

        # cv2.imwrite("/root/ramsam_ws/src/ramsam_ros/src/grounded_sam_annotated_image.jpg", annotated_image)

        segmentation_masks = SegmentationMasks()
        segmentation_masks.header = rgbdata.header
        segmentation_masks.image_header = rgbdata.header
        # Publish images
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "passthrough"))
        
        # *******SHOULD WORK UNTIL HERE*******    
        xyz_image = cv2.rgbd.depthTo3d(depth, np.array(K).reshape((3,3)))
        # xyz_image = cv2.rgbd.depthTo3d(depth, np.array(camera_info.K).reshape((3,3)))
        out_image = np.zeros((depth.shape[0], depth.shape[1]))
        

        if len(detections):
            # print(detections[0])

            for index, (xyxy, mask, confidence, class_id, _) in enumerate(detections):

                det_receptacle = NamedLocation()
                xyzs_of_obj = xyz_image[mask]
                centroid = np.mean(xyzs_of_obj[xyzs_of_obj[...,0] == xyzs_of_obj[...,0]], axis = 0)
                out_image[mask] = index
                if receptacle_categories[class_id] == "long cabinet":
                    receptacle_categories[class_id] = "shelf"
                det_receptacle.name = receptacle_categories[class_id]
                det_receptacle.location = Point(centroid[0], centroid[1], centroid[2])
                response_object.receptacles.append(det_receptacle)
        response_object.masked_image = self.bridge.cv2_to_imgmsg(out_image, "passthrough")
        return response_object
        

if __name__ == "__main__":

    print("Creating ramsam \n\n")
    print("\n")
    rospy.init_node("ramsam", anonymous=True)
    detector = RAMSAMDetector()
    
    rospy.spin()
