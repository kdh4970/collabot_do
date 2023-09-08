#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from rospy.node import Node
from azbt_msgs.msg import Elem, bt_data

class App(Node):
    def __init__(self):
        super().__init__("Collabot_main")
        self.bodytracker_sub = self.create_subscriber(bt_data, "bt_result", self.bt_callback)
        self.bookcase_num_sub = self.create_subscriber(String, "bookcase_num", self.bookcase_callback)
        self.bookcase_state_sub = self.create_subscriber(String, "bookcase_state", self.bookcase_state_callback)
        self.count_sub = self.create_subscriber(Int32, "count", self.count_callback)
        self.ssim_sub = self.create_subscriber(Float32, "SSIM", self.ssim_callback)
        self.grad_sub = self.create_subscriber(Float32, "GRAD", self.grad_callback)
        self.change_sub = self.create_subscriber(String, "change", self.change_callback)
        # set ros parameter
        self.command_pub = self.create_publisher(String, "cmd", 10)

        self.height_threshold = rospy.get_param('~height_threshold')
        self.OnTask = False
        self.ac = None

    def bt_callback(self, msg):
        # get body tracking data
        self.OnTask = True
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
                rospy.loginfo("사람이 인식되지 않음")
                self.ac = "None"
            elif length >= self.height_threshold:
                rospy.loginfo("adult")
                self.ac = "adult"
            else:
                rospy.loginfo("child")
                self.ac = "child"

            pass
            # self.ac = "adult"
            # self.bookcase = "bookcase1"
            # self.sceinaro = "sceinaro1"
            # self.count = 1
            # self.flag = 1
    def bookcase_callback(self, msg):
        self.bookcase = msg.data
        if self.bookcase == "reset":
            self.flag = -1
        else:
            self.flag = 1

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
    app = App()
    while not rospy.is_shutdown():
        if not app.OnTask: rospy.spinOnce()
        




if __name__=="__main__":
    main()