#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from azbt_msgs.msg import Elem, bt_data
from collections import deque
import threading
import signal
import os,sys
from dynamic_reconfigure.server import Server
from collabot_do.cfg import collabot_doConfig
from collabot_do.srv import call_ssim,call_ssimResponse

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
        self.count = 0
        self.taskque = deque()
        self.ac_info = None
        self.taskflag = False
        
        self.bodytracker_sub = rospy.Subscriber("bt_result", bt_data, self.bt_callback)
        self.bookcase_num_sub = rospy.Subscriber("bluetooth_input", String, self.bluetooth_callback)
        rospy.set_param('kill', False)
        ac_threshold = rospy.get_param('ac_threshold', 150.)

        self.set_bookcase_pub = rospy.Publisher("set_bookcase", String, queue_size=10)
        # move_turtlebot contained : 0,1,2.3 (1,2,3 means bookcase number, 0 means reset)
        self.cmd_turtlebot_pub = rospy.Publisher("move_turtlebot", String, queue_size=10)
        srv = Server(collabot_doConfig, config_callback)

    def node_spin(self):
        rospy.loginfo("Main Node Ready.")
        rospy.loginfo("Press Ctrl+C to exit.")
        rospy.loginfo("Waiting Bluetooth Input...")
        rospy.spin()


    def bluetooth_callback(self, msg):
        input_cmd = msg.data
        self.taskque.append(input_cmd)
        print("Received bluetooth input: {}".format(input_cmd))
        print(f"Task added   : {input_cmd}")

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
        print("execute : Reset")

    def subtask_turtlebot_move(self,bookcasenum):
        self.cmd_turtlebot_pub.publish(bookcasenum)
        print("execute : Move turtlebot")

    def subtask_turtlebot_reset(self):
        self.cmd_turtlebot_pub.publish(0)
        print("execute : Reset turtlebot")

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
            print("execute : SSIM attached!")
            result = ssim_server(bookcase_num)
            print(f"execute : SSIM respond {result}")
        except rospy.ServiceException as e:
            print("error : Service call failed: %s"%e)
        
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
                print(f"Task Queue   : {self.taskque}")
                print(f"Current Task : {self.taskque[0]}")
                print(f"User         : {self.ac_info}")
                self.taskflag = True
                if self.taskque[0][:4] == "book":
                    self.count += 1
                    print(f"Book count   : {self.count}")
                    print("+------------------- Exec Info -------------------+")
                    if turtlebot_condition:
                        self.subtask_turtlebot_move(int(self.taskque[0][4]))
                    self.subtask_open()
                    self.wait_motor_open(rospy.Time.now().secs)
                    self.subtask_ssim(int(self.taskque[0][4]))
                    self.ssim_light()
                    self.subtask_close()
                    
                elif self.taskque[0] == "reset":
                    self.count = 0
                    
                    print(f"Book count   : {self.count}")
                    print("+------------------- Exec Info -------------------+")
                    if turtlebot_condition:
                        self.subtask_turtlebot_reset()
                    self.subtask_reset()
                else:pass
                
                self.taskque.popleft()
                # self.ac_info = None
                print("+-------------------------------------------------+\n\n")
            else: # waiting task
                self.taskflag = False



#
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
    node = MainNode()
    rospy.loginfo("Starting Main Node thread...")
    # t1 = threading.Thread(target=node.node_spin)
    t2 = threading.Thread(target=node.run)
    # t1.daemon=True
    t2.daemon=True
    # t1.start()
    t2.start()
    rospy.loginfo("Main Node Ready.")
    rospy.loginfo("Press Ctrl+C to exit.")
    rospy.loginfo("Waiting Bluetooth Input...")
    rospy.spin()
    # t1.join()
    # t2.join()




if __name__=="__main__":
    main()