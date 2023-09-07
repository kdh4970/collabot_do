#!/usr/bin/env python
# -*- coding: utf-8 -*-

#reference: turtlebot3/turtlebot3_example/nodes/turtlebot3_pointop_key

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

class stool:

    turtlebot3_move=False

    def __init__(self):

        self.bookcase = None

        self.subscriber1 = rospy.Subscriber(
            name="sceinaro_num", data_class=String, callback=self.callback1)
        
        self.subscriber2 = rospy.Subscriber(
            name="bookcase_num", data_class=String, callback=self.callback2)


    def callback1(self, msg):

        if msg.data == "sc 1": 
            stool.turtlebot3_move=True   
            self.movebase_client(1, 2.1, 0.55, 0) # 책장 옆 트롤리 위치
            stool.turtlebot3_move=False

        elif msg.data == "sc 2":
            stool.turtlebot3_move=True   
            self.movebase_client(1, 1.4, 1, 0)
            stool.turtlebot3_move=False

        elif msg.data == "sc 3":
            stool.turtlebot3_move=True   
            self.movebase_client(1, 2.1, 1.1, 0)
            stool.turtlebot3_move=False

	elif msg.data == "sc 4":
            stool.turtlebot3_move=True   
            self.movebase_client(0, 0.3, 0, 0)
            stool.turtlebot3_move=False

        else:
            pass
            #rospy.loginfo("Something is error with subscribing sceinaro")


    def callback2(self,msg):
        self.bookcase = msg.data
        #rospy.loginfo(self.bookcase)

    def movebase_client(self,n,x,y,z):

        tf_prefix = 'tb3_' + str(n)
        cmd_vel = tf_prefix + '/cmd_vel'
        odom = tf_prefix + '/odom'
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher(cmd_vel, Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = odom
        base_footprint = tf_prefix + '/base_footprint'
        base_link = tf_prefix + '/base_link'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, base_footprint, rospy.Time(), rospy.Duration(1.0))
            self.base_frame = base_footprint
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, base_link, rospy.Time(), rospy.Duration(1.0))
                self.base_frame = base_link
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (tf_prefix, goal_x, goal_y, goal_z) = [float(n), float(x), float(y), float(z)]
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
       
        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)
            
            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle

            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)*1.4
            #move_cmd.linear.x = 0.1
            
            
            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.1:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.3
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.3
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.3
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.3
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        self.cmd_vel.publish(Twist())


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)    

if __name__ == '__main__':
	
        rospy.init_node('chair_stool', anonymous=False)
        stool()
        rospy.spin()
