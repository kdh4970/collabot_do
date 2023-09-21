#!/usr/bin/env python3
from skimage.metrics import structural_similarity as ssim
import numpy as np
import cv2
import json
import rospy,rospkg
import sys
from std_msgs.msg import Float32,String,Int32
from collabot_do.srv import call_ssim,call_ssimResponse
SSIM_THRESHOLD = 0.65
GRAD_THRESHOLD =15
FPS = 30
''' if you want to check port num
[bash]
ls -al /dev/video*
'''

#ros::Subscriber<std_msgs::String> close_flag("change", close_cb); // same/diff


class SSIM:
    def __init__(self,capture,ROI):
        self.data = None #전역변수로 선언을 해주고
        self.grad = 0
        self.state = "close"
        self.move = "same"
        
        self.publisher1 = rospy.Publisher('SSIM', Float32, queue_size=10)
        self.publisher2 = rospy.Publisher('GRAD', Float32, queue_size=10)
        # self.publisher3 = rospy.Publisher('change', String, queue_size=10)
        # self.subscriber2 = rospy.Subscriber(
        #     name="set_bookcase", data_class=String, callback=self.callbackFunction2)
        self.rate = rospy.Rate(30) # 0.5hz
        
        self.cap = capture
        self.ROI = ROI
        self.flag_diff = 0
        self.curr_cap =None
        self.past_cap = None
        self.past_bookcase_num = ""
        server = rospy.Service("ssim_server", call_ssim, self.handle_ssim)
        print("========== SSIM Service Ready! ==========")

    def ssim_publish(self):
        if self.state == "open":
            self.publisher1.publish(self.data)
            #rospy.loginfo(self.data)
        else:
            self.publisher1.publish(0)
        self.rate.sleep() #100hz가 될때 까지 쉬기
    
    def diff_publish(self,current_score,past_score):
        # FPS 30 : 1/30000s -> 1frame
        sec = 1/30000
        if self.state == "open":
            self.grad = (abs((current_score-past_score)/sec)/1000)**2
            print(f"grad : {self.grad}")
            self.publisher2.publish(self.grad)
        else:
            self.publisher2.publish(0)
        self.rate.sleep() #100hz가 될때 까지 쉬기

    def handle_ssim(self,req):
        bookcase_num = req.A
        while self.cap.isOpened() and self.move == "same":
            
            _,src = self.cap.read()
            self.curr_cap = src.copy()

            if self.past_bookcase_num == "" or self.past_bookcase_num != bookcase_num:
                print("ROI width : %d , height : %d " %( self.ROI[bookcase_num]['x2']-self.ROI[bookcase_num]['x1'] , self.ROI[bookcase_num]['y2']-self.ROI[bookcase_num]['y1']))
                self.past_bookcase_num = bookcase_num

            else:
                curr_crop = self.curr_cap[self.ROI[bookcase_num]['y1']: self.ROI[bookcase_num]['y2'], self.ROI[bookcase_num]['x1']:self.ROI[bookcase_num]['x2']]
                past_crop = self.past_cap[self.ROI[bookcase_num]['y1']: self.ROI[bookcase_num]['y2'], self.ROI[bookcase_num]['x1']:self.ROI[bookcase_num]['x2']]

                curr_gray = cv2.cvtColor(curr_crop, cv2.COLOR_BGR2GRAY)
                past_gray = cv2.cvtColor(past_crop, cv2.COLOR_BGR2GRAY)

                (score, diff) = ssim(curr_gray, past_gray, win_size= 63 ,full=True)
                diff = (diff * 255).astype("uint8")
                
                curr_score = score
                if self.flag_diff ==0:
                    self.flag_diff = 1 
                    pass
                else:
                    print(f"curr_score : {curr_score}, past_score :{past_score}")
                    self.diff_publish(curr_score,past_score)
                    if self.grad > GRAD_THRESHOLD:
                        print("diff occured!")
                        return call_ssimResponse(0)
                    else:
                        pass

                past_score = curr_score
                self.data = score
                self.ssim_publish()

                self.past_bookcase_num = bookcase_num
                cv2.imshow("VideoFrame", src)
                cv2.imshow("contour",curr_crop)
            self.past_cap = src
            if cv2.waitKey(int(1000/FPS)) & 0xFF == ord('q'): #FPS 30 =>  Time  = 1000 / FPS
                break
        return call_ssimResponse(-1)

if __name__ == '__main__':
    print("Starting SSIM Node...")
    rospy.init_node('ssim_pub_node')
    VideoNum = rospy.get_param("~video",0)
    print(f"TARGET_VIDEO_PATH : /dev/video{VideoNum}")
    rospack = rospkg.RosPack()
    json_path = rospack.get_path('collabot_do') + "/utils/ROI.json"
    print(f"ROI_JSON_PATH     : {json_path}")

    if len(sys.argv) == 2:
        VideoNum = int(sys.argv[1])
    
    cap = cv2.VideoCapture(VideoNum)
    print("camera width : %d, camera height : %d" %(cap.get(cv2.CAP_PROP_FRAME_WIDTH) , cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    with open(json_path, "r") as json_file:
        ROI = json.load(json_file)
    
    print("========== SSIM Camera Ready! ==========")
    s= SSIM(cap,ROI)
    rospy.spin()
    cv2.destroyAllWindows()