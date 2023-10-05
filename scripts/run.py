#!/usr/bin/env python3
import rospy
from std_msgs.msg import String,Bool,Int16
from azbt_msgs.msg import Elem, bt_data
from collections import deque
import threading
import signal
import os,sys,time
from dynamic_reconfigure.server import Server
from collabot_do.cfg import collabot_doConfig
from collabot_do.srv import call_ssim


def signal_handler(sig, frame):
    print('Killing Process...')
    rospy.set_param('kill', True)
    rospy.sleep(1.5)
    kill_command = "rosnode kill -a"
    os.system(kill_command)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
ac_threshold = None

def config_callback(config, level):
    global ac_threshold
    print(f"threshold changed! from {ac_threshold} to {config['ac_threshold']}")
    rospy.set_param('ac_threshold', config['ac_threshold'])
    ac_threshold = config['ac_threshold']
    return config

class MainNode():
    def __init__(self):
        global ac_threshold
        self.detection_method = rospy.get_param('~detection_method')
        print(f"Detection Method : {self.detection_method}")
        self.count = 0
        self.taskque = deque()
        self.ac_info = None
        self.taskflag = False
        self.turtlebot_moved = False
        self.bodytracker_sub = rospy.Subscriber('bt_result', bt_data, self.bt_callback)
        self.bookcase_num_sub = rospy.Subscriber('bluetooth_input', String, self.bluetooth_callback)
        
        ac_threshold = rospy.get_param('ac_threshold', 150.)

        self.set_bookcase_pub = rospy.Publisher('set_bookcase', String, queue_size=10)
        # move_turtlebot contained : 0,1,2.3 (1,2,3 means bookcase number, 0 means reset)
        self.move_turtlebot_pub = rospy.Publisher('/move_turtlebot', String, queue_size=10)

        # this for optical flow
        if self.detection_method == 'optical_flow':
            self.optical_flow_pub = rospy.Publisher('of_call',Int16,queue_size=2)
            self.optical_flow_sub = rospy.Subscriber('of_respond',Bool,self.of_callback)
            self.of_signal = False

        # this for ssim
        elif self.detection_method == 'ssim':
            srv = Server(collabot_doConfig, config_callback)
        
    def node_spin(self):
        rospy.loginfo("Main Node Ready.")
        rospy.loginfo("Press Ctrl+C to exit.")
        rospy.loginfo("Waiting Bluetooth Input...")
        rospy.spin()

    def of_callback(self,msg):
        if msg.data is not None:
            print(f"optical flow returns {msg.data}")
            self.of_signal = True

    def bluetooth_callback(self, msg):
        input_cmd = msg.data
        print("\n>>> Received bluetooth input : {}".format(input_cmd))
        self.taskque.append(input_cmd)
        print(f"<<< Task Queue : {list(map(str,self.taskque))} <<< new task added : {input_cmd}\n")

    def bt_callback(self, msg):
        global ac_threshold
        # get body tracking data
        if not self.taskflag:
            detected_user=[]
            for _ in range(len(msg.data)):
                elem = Elem()
                elem.body_id = msg.data[_].body_id
                elem.length = msg.data[_].length # mm
                elem.location_x = msg.data[_].location_x
                elem.location_y = msg.data[_].location_y
                detected_user.append(elem)
            
            length = detected_user[0].length // 10
            # if only one person is detected
            if len(detected_user) == 1:
                if length == 0:
                    self.ac_info = "None"
                elif length >= ac_threshold:
                    self.ac_info = "adult"
                else:
                    self.ac_info = "child"
                pass
    
    def subtask_open(self):
        self.set_bookcase_pub.publish(self.taskque[0]+" open")
        print(f"execute : Open bookcase {self.taskque[0][4]}")

    def subtask_close(self):
        self.set_bookcase_pub.publish(self.taskque[0]+" close")
        print(f"execute : Close bookcase {self.taskque[0][4]}")

    def subtask_reset(self):
        self.set_bookcase_pub.publish("reset")
        print(f"execute : Reset     count {self.count} >>> 0")
        self.count = 0
        self.taskflag = False

    def subtask_turtlebot_move(self,bookcasenum):
        if (self.ac_info == "adult" and self.count>=3):
            self.move_turtlebot_pub.publish("move 0")
            print("execute : Move turtlebot <<< support adult 0")
            self.turtlebot_moved = True
        elif (self.ac_info == "child" and self.taskque[0][4] in ["1","2","3"]):
            self.move_turtlebot_pub.publish("move " + bookcasenum)
            print(f"execute : Move turtlebot <<< support child {bookcasenum}")
            self.turtlebot_moved = True

    def subtask_turtlebot_reset(self):
        if (self.ac_info == "adult" and self.turtlebot_moved):
            self.move_turtlebot_pub.publish("reset a")
            print("execute : Move turtlebot <<< reset a")
            self.turtlebot_moved = False
        elif (self.ac_info == "child" and self.turtlebot_moved):
            self.move_turtlebot_pub.publish("reset c")
            print("execute : Move turtlebot <<< reset c")
            self.turtlebot_moved = False

    def wait_motor_open(self,open_time):
        print("execute : Waiting motor open...")
        while True:
            curr_time = rospy.Time.now().secs
            if (curr_time-open_time) > 3: break

    def subtask_ssim(self,bookcase_num):
        rospy.wait_for_service('ssim_server')
        print("execute : Calling SSIM server...")
        try:
            ssim_server = rospy.ServiceProxy('ssim_server', call_ssim)
            print("info    : SSIM attached!")
            result = ssim_server(bookcase_num)
            print(f"respond : SSIM respond {result}")
        except rospy.ServiceException as e:
            print("error : Service call failed: %s"%e)

    def subtask_of(self,bookcase_num):
        print("execute : call optical flow detector")
        self.optical_flow_pub.publish(bookcase_num)
        while not self.of_signal:
            time.sleep(0.1)
        self.of_signal=False
        print("respond : optical flow finished")


    # ssim 끝나면 led키고, 몇초 기다린다음 닫기  #348 5초 나머지 1.5초
    def ssim_light(self):
        on_time = rospy.Time.now().secs
        self.set_bookcase_pub.publish("led on")
        print("execute : LED on")
        if self.taskque[0][4] in ["3","4","8"]:
            while True:
                curr_time = rospy.Time.now().secs
                if (curr_time-on_time) > 4: break
        else:
            while True:
                curr_time = rospy.Time.now().secs
                if (curr_time-on_time) > 2: break
        self.set_bookcase_pub.publish("led off")
        print("execute : LED off")

    def run(self):
        while True:
            if (len(self.taskque) is not 0) and (self.ac_info is not "None"): # running task
                turtlebot_condition = (self.ac_info == "adult" and self.count>=3) or (self.ac_info == "child" and self.taskque[0][4] in ["1","2","3"])
                print("+------------------- Task Info -------------------+")
                print(f"Task Queue   : {list(map(str,self.taskque))}")
                print(f"Current Task : {self.taskque[0]}")
                print(f"User         : {self.ac_info}")
                self.taskflag = True
                if self.taskque[0][:4] == "book":
                    self.count += 1
                    
                    print(f"Book count   : {self.count}")
                    print("+------------------- Exec Info -------------------+")
                    self.subtask_turtlebot_move(self.taskque[0][4])
                    self.subtask_open()
                    self.wait_motor_open(rospy.Time.now().secs)
                    if self.detection_method == 'optical_flow':
                        if self.taskque[0][4] in ["0","1","2","5","6","7","9"]:
                            self.subtask_of(int(self.taskque[0][4]))
                        elif self.taskque[0][4] in ["3","4","8"]:
                            print("[Warning] Optical Flow does not support for bookcase 3 or 4 or 8 ")
                        else:
                            pass
                    elif self.detection_method == 'ssim':
                        self.subtask_ssim(int(self.taskque[0][4]))
                    self.ssim_light()
                    self.subtask_close()
                    
                elif self.taskque[0] == "reset":
                    print("+------------------- Exec Info -------------------+")
                    self.subtask_turtlebot_reset()
                    self.subtask_reset()
                else:pass
                
                self.taskque.popleft()
                # self.ac_info = None
                print("+-------------------------------------------------+\n\n")
            else: # waiting task
                pass
                # self.taskflag = False

def main():
    rospy.init_node("Collabot_main")
    print("#############################################################################")
    print("##                                                                         ##")
    print("##       ####    ####     ##  ##     ##     #####     ####    ########     ##")
    print("##     ##      ##    ##   ##  ##   ##  ##   ##  ##  ##    ##     ##        ##")
    print("##    ##      ##      ##  ##  ##  ########  #####  ##      ##    ##        ##")
    print("##     ##      ##    ##   ##  ##  ##    ##  ##  ##  ##    ##     ##        ##")
    print("##       ####    ####     ##  ##  ##    ##  #####     ####       ##        ##")
    print("##                                                                         ##")
    print("############################################################################# \n") 
    print("Waiting...")
    rospy.set_param('kill', False)
    time.sleep(5)
    node = MainNode()
    rospy.loginfo("Starting Main Node thread...")
    exec_thread = threading.Thread(target=node.run)
    exec_thread.daemon=True
    exec_thread.start()
    rospy.loginfo("Main Node Ready.")
    rospy.loginfo("Press Ctrl+C to exit.")
    rospy.loginfo("Waiting Bluetooth Input...")
    rospy.spin()

if __name__=="__main__":
    main()