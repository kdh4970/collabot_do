#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from azbt_msgs.msg import Elem, bt_data
from collections import deque
import threading


scenario = None
count = 0
taskque = deque()
change = False
ac_info = None
taskflag = False

class MainNode():
    def __init__(self):
        # self.publisher3 = rospy.Publisher('change', String, queue_size=10)
        # self.subscriber1 = rospy.Subscriber(
        #     name='bookcase_state', data_class=String, callback=self.callbackFunction1)
        self.bodytracker_sub = rospy.Subscriber("bt_result", bt_data, self.bt_callback)
        self.bookcase_num_sub = rospy.Subscriber("bluetooth_input", String, self.bluetooth_callback)
        self.change_sub = rospy.Subscriber("change", String, self.change_callback)
        self.scenario_sub = rospy.Subscriber("scenario", Int32, self.scenario_callback)

        self.set_bookcase_pub = rospy.Publisher("set_bookcase", String, queue_size=10)
        self.cmd_turtlebot_pub = rospy.Publisher("cmd_turtlebot", String, queue_size=10)
        self.height_threshold = rospy.get_param('height_threshold', 150)
        
    def run(self):
        rospy.spin()

    def scenario_callback(self, msg):
        global scenario
        scenario = msg.data

    def bluetooth_callback(self, msg):
        global taskque, taskflag
        input_cmd = msg.data
        taskque.append(input_cmd)

    def bt_callback(self, msg):
        # get body tracking data
        global ac_info, taskflag
        if not taskflag:
            detected_user=[]
            for _ in range(len(msg.data)):
                elem = Elem()
                elem.id = msg.data[_].id
                elem.length = msg.data[_].length # mm
                elem.x = msg.data[_].x
                elem.y = msg.data[_].y
                detected_user.append(elem)
            
            length = detected_user[0].length // 10
            # if only one person is detected
            if len(detected_user) == 1:
                if length == 0:
                    rospy.loginfo("[az_body_tracker] Not detected.")
                    ac_info = "None"
                elif length >= self.height_threshold:
                    rospy.loginfo("[az_body_tracker] Adult")
                    ac_info = "adult"
                else:
                    rospy.loginfo("[az_body_tracker] Child")
                    ac_info = "child"
                pass

    def change_callback(self, msg):
        global change
        if msg.data == "diff":
            change = True
        else:
            pass


class TaskExecutor:
    def __init__(self, cmd_turtlebot_pub, set_bookcase_pub):
        self.cmd_turtlebot_pub = cmd_turtlebot_pub
        self.set_bookcase_pub = set_bookcase_pub

    def subtask_open(self):
        global taskque
        self.set_bookcase_pub.publish(taskque[0]+"open")
        rospy.loginfo("open")

    def subtask_close(self):
        global taskque
        self.set_bookcase_pub.publish(taskque[0]+"close")
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
                break
            else:
                pass

    def run(self):
        global taskque, change, ac_info, count, taskflag
        while True:
            if len(taskque) != 0: # running task
                taskflag = True
                if taskque[0][:4] == "book":
                    if (ac_info == "adult" & count>=3) or ac_info == "child":
                        self.subtask_turtlebot_move()
                    self.subtask_open()
                    self.wait_close_flag()
                    self.subtask_close()
                    count += 1
                elif taskque[0] == "reset":
                    self.subtask_reset()
                    count = 0
                else:pass
                taskque.popleft()
                change = False
                ac_info = None
            else: # waiting task
                taskflag = False

            
            


def main():
    rospy.init_node("Collabot_main")
    rospy.loginfo("#############################################################################")
    rospy.loginfo("##                                                                         ##")
    rospy.loginfo("##       ####    ####     ##  ##     ##     #####     ####    ########     ##")
    rospy.loginfo("##     ##      ##    ##   ##  ##   ##  ##   ##  ##  ##    ##     ##        ##")
    rospy.loginfo("##    ##      ##      ##  ##  ##  ########  #####  ##      ##    ##        ##")
    rospy.loginfo("##     ##      ##    ##   ##  ##  ##    ##  ##  ##  ##    ##     ##        ##")
    rospy.loginfo("##       ####    ####     ##  ##  ##    ##  #####     ####       ##        ##")
    rospy.loginfo("##                                                                         ##")
    rospy.loginfo("############################################################################# \n\n") 
    node = MainNode()
    te = TaskExecutor(node.cmd_turtlebot_pub, node.set_bookcase_pub)
    t1 = threading.Thread(target=node.run)
    t2 = threading.Thread(target=te.run)
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    while not rospy.is_shutdown():
        rospy.spinOnce()
        
        
    




if __name__=="__main__":
    main()
