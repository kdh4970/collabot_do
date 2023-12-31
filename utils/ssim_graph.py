#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import count,cycle
import cv2
import rospy
from std_msgs.msg import Float32,String
import numpy as np
import sys
import argparse



score = None
ssim_score = None #Using For Global
grad = None
FPS = 30
fig=None
'''
        self.publisher1 = rospy.Publisher('SSIM', Float32, queue_size=10)
        self.publisher2 = rospy.Publisher('GRAD', Float32, queue_size=10)
        self.publisher3 = rospy.Publisher('change', String, queue_size=10)
'''

def kill_process():
    global fig
    print('Killing Process...')
    plt.close(fig=fig)
    cv2.destroyAllWindows()
    sys.exit(0)

class graph:
    def __init__(self):
        self.subscriber1 = rospy.Subscriber(
            name="SSIM", data_class=Float32, callback=self.callbackFunction1)
        #ros::Publisher bookcase_state("bookcase_state",  &state);
        # self.subscriber2 = rospy.Subscriber(
        #     name="bookcase_state", data_class=String, callback=self.callbackFunction2)
        self.subscriber = rospy.Subscriber(
            name="GRAD", data_class=Float32, callback=self.callbackFunction3)
        self.rate = rospy.Rate(30) # 0.5hz

    def callbackFunction1(self,msg): #기본 argument는 구독한 메세지 객체
        global ssim_score
        ssim_score = float(msg.data)
        self.rate.sleep() #100hz가 될때 까지 쉬기

    # def callbackFunction2(self,msg): #기본 argument는 구독한 메세지 객체 
    #     global state
    #     state = msg.data
    #     self.rate.sleep() #100hz가 될때 까지 쉬기

    def callbackFunction3(self,msg): #기본 argument는 구독한 메세지 객체 
        global grad
        grad = float(msg.data)
        self.rate.sleep() #100hz가 될때 까지 쉬기

SSIM_THRESHOLD = 0.65
GRAD_THRESHOLD = 15
x_max = 500

if __name__=="__main__":
    print("Starting SSIM_GRAPH Node...")
    rospy.init_node('ssim_sub_node')

    parser = argparse.ArgumentParser(description='Program for Graph (mode : SSIM/GRAD)')
    parser.add_argument('--mode',help= "Select GRAD or SSIM(default = 'GRAD')",default = 'GRAD')

    parser.add_argument('--ST',help= "SSIM Threshold(default = developer setting)",default = 'DEV')
    parser.add_argument('--GT',help= "GRAD Threshold(default = developer setting))",default = 'DEV')
    # args = parser.parse_args() #


    '''
    [Setting For SSIM Threshold]
    When User input the argument for threshold
    '''
    # if args.ST != 'DEV':
    #     SSIM_THRESHOLD = args.ST

    # if args.GT != 'DEV':
    #     GRAD_THRESHOLD = args.GT

    print("ST = ", SSIM_THRESHOLD)
    print("GT = ", GRAD_THRESHOLD)
    # print("mode  = ", args.mode)
    print("========== SSIM graph Ready! ==========")
    g = graph()

    # #그래프 정보 설정
    # plt.tight_layout()
    #plt.xlabel('time') #x 라벨
    #plt.ylabel('Score') #y 라벨
    #plt.title("SSIM") #그래프 이름


    ''' create the graph'''
    ggraph_x = np.array([])
    ggraph_y = np.array([])
    sgraph_x = np.array([])
    sgraph_y = np.array([])
    lst = [i for i in range(x_max)]
    #index = cycle(lst)

    gindex = count()
    sindex = count()

    fig = plt.figure() #figure(도표생성)

    ax1 = plt.subplot(211,xlim=(0, x_max), ylim=(0.3, 1)) #ssim
    plt.title("SSIM")

    ax2 = plt.subplot(212,xlim=(0, x_max), ylim=(0, 30)) #grad
    plt.title("Grad")


    line1, = ax1.plot([], [], lw=3)
    line2, = ax2.plot([], [], lw=3)

    def s_animate(i):
        if rospy.get_param('kill'):
            kill_process()
        global args
        global ssim_score
        global sgraph_x
        global sgraph_y
        global sindex
    
        score = ssim_score
        
        if next(sindex) >= x_max:
            sindex = count()
            sgraph_x,sgraph_y= [],[]
            ssim_anim.frame_seq = ssim_anim.new_frame_seq()
            ssim_anim.event_source.start()
        else:
            sgraph_x = np.append(sgraph_x,next(sindex))
            sgraph_y = np.append(sgraph_y,score)
        line1.set_data(sgraph_x, sgraph_y)
        return line1,


    def g_animate(i):
        global args
        global grad
        global ggraph_x
        global ggraph_y
        global gindex

        score = grad

        if next(gindex) >= x_max:
            gindex = count()
            ggraph_x,ggraph_y= [],[]
            grad_anim.frame_seq = grad_anim.new_frame_seq()
            grad_anim.event_source.start()
        else:

            ggraph_x = np.append(ggraph_x,next(gindex))
            ggraph_y = np.append(ggraph_y,score)
        line2.set_data(ggraph_x, ggraph_y)
        return line2,

    #plt.plot(graph_x,graph_y,color='blue',linestyle='-',marker='o')
    if (score == None):
        score = 0


    ssim_anim = FuncAnimation(fig, s_animate,interval=100,repeat = True)
    grad_anim = FuncAnimation(fig, g_animate,interval=100,repeat = True)
    #frames=200


    ax1.hlines(SSIM_THRESHOLD, 0, x_max, color='green', linestyle='solid', linewidth=3)
    ax2.hlines(GRAD_THRESHOLD, 0, x_max, color='green', linestyle='solid', linewidth=3)
    # args.mode == 'SSIM':

    plt.tight_layout(h_pad=3)#, w_pad=8)
    plt.show()
    cv2.destroyAllWindows()