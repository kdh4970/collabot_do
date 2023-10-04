import rospy
import os
import sys
from pathlib import Path

import numpy as np
import torch

from models.common import DetectMultiBackend
from utils.general import (Profile, check_img_size, non_max_suppression, scale_boxes)
from utils.torch_utils import select_device
from utils.augmentations import letterbox


ROOT = '/home/kist/yolov5'  # YOLOv5 root dir

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


class DetectDrawer:
    def __init__(self,
                 weights='./weights/best.pt',
                 data=ROOT / 'data/collatbot.yaml',  # dataset.yaml path
                 imgsz=(640, 640),  # inference size (height, width)
                 device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 max_det=1000,  # maximum detections per image
                 ):
        
        self.weights = weights
        self.data = data
        self.imgsz = imgsz
        self.device = device
        self.conf_thres = conf_thres 
        self.iou_thres = iou_thres
        self.max_det = max_det

        self.device = select_device(self.device)
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=False, data=self.data, fp16=False)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(self.imgsz, s=self.stride)
        
        self.avg_roi = None
        self.bbox_coordinates = []
        

    def compute_avg_roi(self, bbox_coordinates):
        avg_roi = [] 
        for i in range(len(bbox_coordinates)):
            avg_roi.append(bbox_coordinates[i][0][:4])
        avg_roi = sum(avg_roi) // len(bbox_coordinates)
        return avg_roi.int()    
    
    def run(self, image):   
        rospy.loginfo('detect_drawer run')
        bs = 1
        
        im0 = image
        im = letterbox(im0, self.imgsz, stride=self.stride, auto=self.pt)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)         

        bbox_num = 1

        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else bs, 3, *self.imgsz))  # warmup
        seen, windows, dt = 0, [], (Profile(), Profile(), Profile())    
    
        if len(self.bbox_coordinates) < bbox_num:
            with dt[0]:
                im = torch.from_numpy(im).to(self.model.device)
                im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
                im /= 255  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim

            # Inference
            with dt[1]:
                pred = self.model(im, augment=False, visualize=False)

            # NMS
            with dt[2]:
                pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=None, agnostic=False, max_det=self.max_det)

            # Process predictions
            for i, det in enumerate(pred):  # per image

                seen += 1
                
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                    
                    self.bbox_coordinates.append(det.int())

            if len(self.bbox_coordinates) == bbox_num and self.avg_roi == None:
                self.avg_roi = self.compute_avg_roi(self.bbox_coordinates)
                self.bbox_coordinates = []
                return 
