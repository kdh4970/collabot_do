# -*- coding: utf-8 -*-
# !/usr/bin/python3
import cv2
import numpy as np

from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]

class OpticalFlow:
    def __init__(self):
        self.termcriteria = (cv2.TERM_CRITERIA_EPS | cv2.TermCriteria_COUNT, 10, 0.03)
        self.color = np.random.randint(0, 255, (200, 3))
        self.prevImg = None
        self.nextImg = None
        self.lines = None
        self.roilines = None
        self.state_trig = False
        self.nextPt = None
        self.H = None


    def y_channel(self, img):
        yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        y_ch = yuv[:, :, 0]

        return y_ch
    
    

    def HSV_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        return h

    def calc_hiehgt(self, roi):
        self.H = abs(roi[3] - roi[1])


    def calc_state(self, mov_point, ref_point):
        if mov_point <= ref_point:
            print('over the line')
            trig = True
            return trig
        

    def run(self, image, roi):
        self.calc_hiehgt(roi)
        
        img_draw = image.copy()
        roi_height = roi[1].item() - self.H//2
        
        if roi_height < 0:
            roi_height = 0
            self.H = int(roi[3].item()*2/3)

        roi_img = image.copy()[int(roi_height):int(roi[3].item()), int(roi[0].item() + 10):int(roi[2].item()), ::1]
        y_ch = self.y_channel(roi_img)
        
        if self.prevImg is None:
            self.prevImg = y_ch
            
            self.lines = np.zeros_like(img_draw)
            self.roilines = np.zeros_like(roi_img)

            edge_mask = np.zeros((self.prevImg.shape[0], self.prevImg.shape[1]), dtype=np.uint8)
            cv2.rectangle(edge_mask, (0, int(self.H)), (int(edge_mask.shape[1] - 1), int(edge_mask.shape[0] - 1)), 255, cv2.FILLED)
            self.prevPt = cv2.goodFeaturesToTrack(self.prevImg, 20, 0.01, 5, mask=edge_mask)

        else:
            self.nextImg = y_ch

            self.nextPt, status, err = cv2.calcOpticalFlowPyrLK(self.prevImg, self.nextImg, self.prevPt, None, criteria=self.termcriteria, maxLevel=4, winSize=(11,11))

            prevMv = self.prevPt[status==1]
            nextMv = self.nextPt[status==1]
            

            for i, (p, n) in enumerate(zip(prevMv, nextMv)):
                px, py = p.ravel()
                nx, ny = n.ravel()

                if int(ny) <= int(self.H//2 + self.H//5):
                    print('over the line')
                    self.state_trig = True
                  
                cv2.line(self.roilines, (int(px), int(py)), (int(nx), int(ny)), self.color[i].tolist(), 2)
                cv2.circle(roi_img, (int(nx), int(ny)), 2, self.color[i].tolist(), -1)

                cv2.line(self.lines, ((int(px)+roi[0].item() + 10), (int(py+roi_height))), ((int(nx)+roi[0].item() + 10), (int(ny+roi_height))), self.color[i].tolist(), 2)
                cv2.circle(img_draw, ((int(nx)+roi[0].item() + 10), (int(ny+roi_height))), 2, self.color[i].tolist(), -1)
                        
                roi_img = cv2.add(roi_img, self.roilines)
                img_draw = cv2.add(img_draw, self.lines)
            
                self.prevImg = self.nextImg
                self.prevPt = nextMv.reshape(-1, 1, 2)


            roi_img = cv2.line(roi_img, (0, int(self.H//2 + self.H//5)), (int(roi_img.shape[1] - 1), int(self.H//2 + self.H//5)), (0,0,255), 2)

        return roi_img, img_draw, self.state_trig