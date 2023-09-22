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

scenario = None
count = 0
taskque = deque()
ac_info = None
taskflag = False
ac_threshold = None

def config_callback(config, level):
    global ac_threshold
    print(f"threshold changed! from {ac_threshold} to {config['ac_threshold']}")
    rospy.set_param('ac_threshold', config['ac_threshold'])
    ac_threshold = config['ac_threshold']
    return config

class Sub():
    def __init__(self):
        global ac_threshold
        self.bodytracker_sub = rospy.Subscriber("bt_result", bt_data, self.bt_callback)
        self.bookcase_num_sub = rospy.Subscriber("bluetooth_input", String, self.bluetooth_callback)
        self.scenario_sub = rospy.Subscriber("scenario", Int32, self.scenario_callback)
        rospy.set_param('kill', False)
        ac_threshold = rospy.get_param('ac_threshold', 150.)

    def run(self):
        rospy.loginfo("Main Node Ready.")
        rospy.loginfo("Press Ctrl+C to exit.")
        rospy.loginfo("Waiting Bluetooth Input...")
        rospy.spin()

    def scenario_callback(self, msg):
        global scenario
        scenario = msg.data

    def bluetooth_callback(self, msg):
        global taskque, taskflag
        input_cmd = msg.data
        taskque.append(input_cmd)
        rospy.loginfo("Received bluetooth input: {}".format(input_cmd))

    def bt_callback(self, msg):
        # get body tracking data
        global ac_info, taskflag, ac_threshold
        if not taskflag:
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
                    ac_info = "None"
                elif length >= ac_threshold:
                    ac_info = "adult"
                else:
                    ac_info = "child"
                pass



class TaskExecutor:
    def __init__(self):
        # set_bookcase contained : "bookN open, bookN close, reset"
        self.set_bookcase_pub = rospy.Publisher("set_bookcase", String, queue_size=10)
        # move_turtlebot contained : 0,1,2.3 (1,2,3 means bookcase number, 0 means reset)
        self.cmd_turtlebot_pub = rospy.Publisher("move_turtlebot", String, queue_size=10)
        srv = Server(collabot_doConfig, config_callback)

    def subtask_open(self):
        global taskque
        self.set_bookcase_pub.publish(taskque[0]+" open")
        rospy.loginfo("open")

    def subtask_close(self):
        global taskque
        self.set_bookcase_pub.publish(taskque[0]+" close")
        rospy.loginfo("close")

    def subtask_reset(self):
        self.set_bookcase_pub.publish("reset")
        rospy.loginfo("reset")

    def subtask_turtlebot_move(self,bookcasenum):
        self.cmd_turtlebot_pub.publish(bookcasenum)
        rospy.loginfo("move turtlebot")

    def subtask_turtlebot_reset(self):
        self.cmd_turtlebot_pub.publish(0)
        rospy.loginfo("reset turtlebot")

    def wait_motor_open(self,open_time):
        while True:
            curr_time = rospy.Time.now().secs
            if (curr_time-open_time) > 3: break

    def subtask_ssim(self,bookcase_num):
        rospy.wait_for_service('ssim_server')
        print("Calling SSIM server...")
        try:
            ssim_server = rospy.ServiceProxy('ssim_server', call_ssim)
            print("SSIM attached!")
            result = ssim_server(bookcase_num)
            print(f"SSIM {result}")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def run(self):
        global taskque, ac_info, count, taskflag
        while True:
            if len(taskque) != 0: # running task
                turtlebot_condition = (ac_info == "adult" and count>=3) or (ac_info == "child" and taskque[0][4] in ["1","2","3"])
                print("+-------------- Task Info --------------+")
                print(f"Task Queue   : {taskque}")
                print(f"Current Task : {taskque[0]}")
                print(f"User         : {ac_info}")
                taskflag = True
                if taskque[0][:4] == "book":
                    count += 1
                    print(f"Book Count   : {count}")
                    if turtlebot_condition:
                        self.subtask_turtlebot_move(int(taskque[0][4]))
                    self.subtask_open()
                    self.wait_motor_open(rospy.Time.now().secs)
                    self.subtask_ssim(int(taskque[0][4]))
                    self.subtask_close()
                    
                elif taskque[0] == "reset":
                    count = 0
                    
                    print(f"Book Count   : {count}")
                    if turtlebot_condition:
                        self.subtask_turtlebot_reset()
                    self.subtask_reset()
                else:pass
                
                taskque.popleft()
                ac_info = None
            else: # waiting task
                taskflag = False

            
            


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
    rospy.loginfo("Starting Subscriber thread...")
    sub = Sub()
    rospy.loginfo("Starting TaskExecutor thread...")
    Te = TaskExecutor()
    t1 = threading.Thread(target=sub.run)
    t2 = threading.Thread(target=Te.run)
    t1.daemon=True
    t2.daemon=True
    t1.start()
    t2.start()
    t1.join()
    t2.join()




if __name__=="__main__":
    main()