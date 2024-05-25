#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class obstacle_move:
    def __init__(self):
        rospy.init_node('joy_drive')

        #구독파트
        rospy.Subscriber("/joy", Joy, self.send_joy_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        #발행파트
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)

        #변수초기화
        self.auto_manaul = 0
        self.speed = 0
        self.steering = 0
        self.flag = 0

    def send_joy_callback(self, data): #joy_callback함수
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]

    def drive_control(self, event): #실제 주행명령함수
        drive = Twist()
        
        if self.manual == 1:
            self.speed = self.joy_axes[4]
            self.steering = self.joy_axes[0]
            drive.linear.x = self.speed*1
            drive.angular.z = int(self.steering * 30)

            
        elif self.auto == 1:
            stanley_steer_angle = self.stanley_control_angle()
            if stanley_steer_angle is None: # 큰경우 정지
                drive.linear.x = 0
                drive.angular.z = 0
            else:
                drive.linear.x = 0.7
                drive.angular.z = stanley_steer_angle

        else:
            drive.linear.x = 0
            drive.angular.z = 0
        self.drive_pub.publish(drive)
        
    def detect_obstacle(self, data):
        self.obstacle = data.data















if __name__ == '__main__':
    try:
        MoveCar = obstacle_move()
        rospy.spin()
    except KeyboardInterrupt:
        print("Program terminated")
