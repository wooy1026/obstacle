#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class joy_drive:
    def __init__(self):
		
        rospy.init_node('joy_drive')
        rospy.Subscriber("/joy", Joy, self.send_joy)
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)
        self.auto_manaul = 0
        self.speed = 0
        self.steering = 0
        self.flag = 0

    def send_joy(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        drive = Twist()

        manaul = self.joy_btn[5]
        # print(manaul)
        if manaul == 1:
            self.speed = self.joy_axes[4]
            # Twist에 int만 넣을 수 있어서 *10을 곱해서 보냄
            # 아두이노 에서는 값을 10 나눠서 제어함	
            self.steering = self.joy_axes[0]*30
            drive.linear.x = float(self.speed)
            drive.angular.z = float(self.steering)
        else:
            drive.linear.x = 0
            drive.angular.z = 0
            
        print(drive.linear.x, drive.angular.z)
        self.drive_pub.publish(drive)
        
    def detect_obstacle(self, data):
        self.obstacle = data.data

if __name__ == '__main__':
    ManualCar = joy_drive()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")
