# -*- coding: utf-8 -*-
# !/usr/bin/python3

import rospy
import cv2
import numpy as np
import argparse

from std_msgs.msg import String
from pathlib import Path
from detect_drawer import DetectDrawer
from optical_flow import OpticalFlow
import sys,signal,time
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]

print(ROOT)

def signal_handler(sig, frame):
    print('Killing Process...')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class DetectBook:
    def __init__(self):
        self.pub = rospy.Publisher('/detect', String, queue_size=5)
        self.sub = rospy.Subscriber('/set_bookcase', String, callback=self.callback)
        self.opt = self.parser_opt()
        self.detect_drawer = DetectDrawer(**vars(self.opt))
        self.optical_flow = OpticalFlow()
        self.find_drawer = True
        self.book_state = False
        self.roi = None
        self.trig = False
        self.start_detect=None
        print("Finished <<< Setting Up")

    def callback(self, msgs):
        # state: open
        # state: close
        print("callback")
        if msgs.data[7:] == 'open':
            self.find_drawer = True


    def publish(self):
        # state: diff
        msg = String()

        if self.book_state: 
            msg.data = 'detect'
        
        else:
            msg.data = 'no detect'

        self.pub.publish(msg)
        print("result published!")


    # yolo parameters
    def parser_opt(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default=ROOT / './weights/best.pt', help='model path or triton URL')
        parser.add_argument('--data', type=str, default=ROOT / 'data/drawer.yaml', help='(optional) dataset.yaml path')
        parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
        parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--conf-thres', type=float, default=0.6, help='confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.6, help='NMS IoU threshold')
        parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')

        opt = parser.parse_args()
        opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand

        return opt


    def run(self, source):
        source = str(source)

        cap = cv2.VideoCapture(int(source) if source.isnumeric() else str(source))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                rospy.logfatal("no data")
                break
            ## 콜백이 들어오면 실행.
            if self.find_drawer:
                self.detect_drawer.run(frame)

                if self.detect_drawer.avg_roi is not None:
                    self.roi = self.detect_drawer.avg_roi

                    self.find_drawer = False
                    
                    self.start_detect = rospy.get_rostime()

            # curr_time = rospy.get_rostime()
            # if self.start_detect is not None and self.start_detect.secs - curr_time.secs < 3:
            #     continue 

            if self.roi is not None and self.find_drawer is False:
                now = rospy.get_rostime()
                roi_img, ori_img, self.trig = self.optical_flow.run(frame, self.roi)
                
                if self.start_detect.secs - now.secs > 15:
                    self.publish()

                if self.trig:
                    print(f"running optical flow... : {str(self.trig)}")
                    self.book_state = True
                    self.publish()

                    self.trig = False

                    # 초기화
                    self.optical_flow = OpticalFlow()
                    self.roi = None
                    self.book_state = False
                    

                cv2.imshow('roi_img', roi_img)
            
            if cv2.waitKey(int(1000/fps)) & 0xFF == ord('q'):
                rospy.loginfo('quit')
                continue

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('detect_book_state')
    print("Starting...")
    # Using Camera
    source = 2
    
    # Using video
    #source = '/home/enbang/catkin_ws/src/Collabot/datasets/data_0822/video/57.mp4'

    detect_book = DetectBook()

    detect_book.run(source)   

