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
change = False
ac_info = None
taskflag = False

class Sub():
    def __init__(self):
        # self.publisher3 = rospy.Publisher('change', String, queue_size=10)
        # self.subscriber1 = rospy.Subscriber(
        #     name='bookcase_state', data_class=String, callback=self.callbackFunction1)
        self.bodytracker_sub = rospy.Subscriber("bt_result", bt_data, self.bt_callback)
        self.bookcase_num_sub = rospy.Subscriber("bluetooth_input", String, self.bluetooth_callback)
        self.change_sub = rospy.Subscriber("close_sig", String, self.change_callback)
        self.scenario_sub = rospy.Subscriber("scenario", Int32, self.scenario_callback)
        rospy.set_param('kill', False)
        self.ac_threshold = rospy.get_param('ac_threshold', 150.)
        
        srv = Server(collabot_doConfig, self.config_callback)

    def config_callback(self, config, level):
        rospy.set_param('ac_threshold', config['ac_threshold'])
        self.ac_threshold = config['ac_threshold']
        return config
    
    def ssim_client(bookcase_num):
        rospy.wait_for_service('ssim_server')
        try:
            ssim_server = rospy.ServiceProxy('ssim_server', call_ssim)
            result = ssim_server(bookcase_num)
            if result == 0:
                return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


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
        global ac_info, taskflag
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
                elif length >= self.ac_threshold:
                    ac_info = "adult"
                else:
                    ac_info = "child"
                pass

    def change_callback(self, msg):
        global change
        if msg.data == "diff":
            change = True
        else:
            pass


class TaskExecutor:
    def __init__(self):
        self.set_bookcase_pub = rospy.Publisher("set_bookcase", String, queue_size=10)
        self.cmd_turtlebot_pub = rospy.Publisher("cmd_turtlebot", String, queue_size=10)

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

    def subtask_turtlebot_move(self):
        self.cmd_turtlebot_pub.publish("go")
        rospy.loginfo("move turtlebot")

    def wait_close_flag(self):
        global change
        while True:
            if change:
                rospy.loginfo("close flag")
                change = False
                break
            else:
                pass

    def run(self):
        global taskque, change, ac_info, count, taskflag
        while True:
            if len(taskque) != 0: # running task
                print("+-------------- Task Info --------------+")
                print(f"Task Queue   : {taskque}")
                print(f"Current Task : {taskque[0]}")
                print(f"User         : {ac_info}")
                taskflag = True
                if taskque[0][:4] == "book":
                    count += 1
                    print(f"Book Count   : {count}")
                    if (ac_info == "adult" and count>=3) or (ac_info == "child" and taskque[0][4] in ["1","2","3"]):
                        self.subtask_turtlebot_move()
                    self.subtask_open()
                    self.ssim_client(int(taskque[0][4]))
                    # self.wait_close_flag()
                    self.subtask_close()
                    
                elif taskque[0] == "reset":
                    count = 0
                    print(f"Book Count   : {count}")
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