#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type


# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import multiprocessing as mp
import numpy as np
import tempfile
import time
import warnings

import tqdm
import mss

from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode, Visualizer
from detectron2.data import MetadataCatalog

detic_dir = "/Detic/"
sys.path.append(detic_dir)
sys.path.append('/Detic/third_party/CenterNet2/')
from centernet.config import add_centernet_config
from detic.config import add_detic_config
from detic.modeling.utils import reset_cls_test
from detic.modeling.text.text_encoder import build_text_encoder

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
from geometry_msgs.msg import Pose2D

# add yolov5 submodule to path


# import from yolov5 submodules

class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __getattr__(self, item):
        return None


@torch.no_grad()
class DeticDetector:
    def __init__(self):

        self.args = self.get_args()
        self.cfg = self.setup_cfg(self.args)
        # self.predictor = VisualizationDemo(self.cfg, self.args)
        # self.predictor = DefaultPredictor(self.cfg)
        self.initialise_predictor(self.args, self.cfg)
        
        rospy.loginfo("Predictor Loaded")


        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )


        # Initialize prediction publisher
        self.pred_2d_detector_pub = rospy.Publisher(
            rospy.get_param("~output_detection2d_topic"), Detection2DArray, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        
        im = self.preprocess(im)
        # predictions, output_viz = self.predictor.run_on_image(img)
        predictions, output_viz = self.predict(im)

        boxes = predictions.pred_boxes.tensor.detach().numpy() if predictions.has("pred_boxes") else None
        scores = predictions.scores.detach().numpy() if predictions.has("scores") else None
        classes = predictions.pred_classes.tolist() if predictions.has("pred_classes") else None
        class_names = self.metadata.get("thing_classes", None)
        # labels = _create_text_labels(classes, scores, self.metadata.get("thing_classes", None))
        labels = [class_names[i] for i in classes]
        # keypoints = predictions.pred_keypoints if predictions.has("pred_keypoints") else None
        pred_masks = predictions.pred_masks.detach().numpy() if predictions.has("pred_masks") else None

        detections_2d = Detection2DArray()
        for index, label in enumerate(labels):
            detection = Detection2D()
            x0, y0, x1, y1 = (boxes[index])
            center = (x0 + x1) / 2, (y0 + y1) / 2
            center = Pose2D(center[0], center[1], 0.0)
            detection.bbox = BoundingBox2D(center, x1 - x0, y1 - y0) # center, x and y go here
            detection.object_id = label.replace(" ", "")
            detection.confidence = scores[index]

            detection.segmented_image = self.bridge.cv2_to_imgmsg(pred_masks[index].astype(np.uint8), "passthrough")
            detections_2d.detections.append(detection)
            

        # Run inference
        # im = torch.from_numpy(im).to(self.device) 

        # Publish prediction
        self.pred_2d_detector_pub.publish(detections_2d)
        # cv2.imwrite("/catkin_ws/src/perception/output_image.png", output_viz)

        # Publish & visualize images
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_viz, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img0 = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB)
        img = np.ascontiguousarray(img0)

        return img


    def setup_cfg(self, args):
        cfg = get_cfg()
        if args.cpu:
            cfg.MODEL.DEVICE="cpu"
        add_centernet_config(cfg)
        add_detic_config(cfg)
        cfg.merge_from_file(args.config_file)
        # cfg.merge_from_list(args.opts)
        # Set score_threshold for builtin models
        cfg.MODEL.WEIGHTS = args.weights_file
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = args.confidence_threshold
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = args.confidence_threshold
        cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'rand' # load later
        cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = '/Detic/datasets/metadata/lvis_v1_train_cat_info.json'
        if not args.pred_all_class:
            cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

        self.BUILDIN_CLASSIFIER = {
            'lvis': '/Detic/datasets/metadata/lvis_v1_clip_a+cname.npy',
            'objects365': '/Detic/datasets/metadata/o365_clip_a+cnamefix.npy',
            'openimages': '/Detic/datasets/metadata/oid_clip_a+cname.npy',
            'coco': '/Detic/datasets/metadata/coco_clip_a+cname.npy',
        }
        self.BUILDIN_METADATA_PATH = {
            'lvis': 'lvis_v1_val',
            'objects365': 'objects365_v2_val',
            'openimages': 'oid_val_expanded',
            'coco': 'coco_2017_val',
        }

        cfg.freeze()
        return cfg


    def get_args(self):
        args = Namespace()
        args.config_file = rospy.get_param("~config_file")
        args.cpu = rospy.get_param("~cpu", False)
        args.vocabulary = rospy.get_param("~vocabulary")
        args.custom_vocabulary = rospy.get_param("~custom_vocabulary")
        args.confidence_threshold = rospy.get_param("~confidence_threshold")
        args.weights_file = rospy.get_param("~weights_file")
        return args


    def predict(self, image):

        outputs = self.predictor(image)["instances"].to(self.cpu_device)
        v = Visualizer(image[:, :, ::-1], self.metadata)
        out = v.draw_instance_predictions(outputs)
        return outputs, out.get_image()[:, :, ::-1]


    def get_clip_embeddings(self, vocabulary, prompt='a '):
        text_encoder = build_text_encoder(pretrain=True)
        text_encoder.eval()
        texts = [prompt + x for x in vocabulary]
        emb = text_encoder(texts).detach().permute(1, 0).contiguous().cpu()
        return emb


    def initialise_predictor(self, args, cfg, instance_mode=ColorMode.IMAGE, parallel=False):
        if args.vocabulary == 'custom':
            self.metadata = MetadataCatalog.get("__unused")
            self.metadata.thing_classes = args.custom_vocabulary.split(',')
            classifier = self.get_clip_embeddings(self.metadata.thing_classes)
        else:
            self.metadata = MetadataCatalog.get(
                self.BUILDIN_METADATA_PATH[args.vocabulary])
            classifier = self.BUILDIN_CLASSIFIER[args.vocabulary]

        num_classes = len(self.metadata.thing_classes)
        self.cpu_device = torch.device("cpu")
        self.instance_mode = instance_mode

        self.parallel = parallel
        if parallel:
            num_gpu = torch.cuda.device_count()
            self.predictor = AsyncPredictor(cfg, num_gpus=num_gpu)
        else:
            self.predictor = DefaultPredictor(cfg)
        reset_cls_test(self.predictor.model, classifier, num_classes)




if __name__ == "__main__":

    rospy.init_node("detic_detector", anonymous=True)
    detector = DeticDetector()
    
    rospy.spin()
