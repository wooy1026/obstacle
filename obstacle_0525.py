#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from math import pi, floor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String, Bool

class ObstacleMove:
    def __init__(self):
        rospy.init_node('joy_drive')

        # 구독 파트
        rospy.Subscriber("/joy", Joy, self.send_joy_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # 발행 파트
        self.drive_pub = rospy.Publisher("/cmd_vel_steer", Twist, queue_size=1)

        # 변수 초기화
        self.speed = 0
        self.angle = 0
        self.scaning = False

        # joy 변수
        self.auto = 0
        self.manual = 0
        self.manual_speed = 0
        self.manual_steering = 0

        # lidar 변수
        self.obstacle_ranges = []
        self.center_list_left = []
        self.center_list_right = []

        # 장애물 인식 시간 변수
        self.current_time = rospy.Time.now()
        self.obstacle_exit = False

        # parameter 변수
        self.min_dist = 0.4
        self.range_obs = 1
        self.obs_perception_boundary = 10
        self.obs_range = 40

        self.obs_speed = 0.4
        self.obs_backward_speed = -0.2
        self.obs_angle = 0

        self.lidar_flag = False
        self.degrees = []
        self.ranges_length = 0
        self.dist_data = 0
        self.direction = "front"

    def send_joy_callback(self, data): 
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
    
    def laser_callback(self, msg):
        self.msg = msg
        self.scaning = True
        self.obstacle_ranges = []
        if len(self.obstacle_ranges) > self.obs_perception_boundary:
            obs_detected_time = rospy.Time.now()
            elapsed_time = (obs_detected_time - self.current_time).to_sec()

            if elapsed_time < 5:
                drive = Twist()
                drive.linear.x = 0
                drive.angular.z = 0
                self.drive_pub.publish(drive)
            else:
                self.obstacle_exit = True
        else:
            self.obstacle_exit = False

    def LiDAR_scan(self):
        if not self.lidar_flag:
            self.degrees = [(self.msg.angle_min + i * self.msg.angle_increment) * 180 / pi for i, _ in enumerate(self.msg.ranges)]
            self.ranges_length = len(self.msg.ranges)
            self.lidar_flag = True
            
        obstacle_indices = [i for i, data in enumerate(self.msg.ranges) if 0 < data < self.range_obs and 190 - self.obs_range < self.degrees[i] < 190 + self.obs_range]
        if obstacle_indices:
            self.dist_data = min([self.msg.ranges[i] for i in obstacle_indices])
            first = obstacle_indices[0]
            last = obstacle_indices[-1]
            first_dst = first
            last_dst = self.ranges_length - last
            self.obstacle_ranges = self.msg.ranges[first:last + 1]
            return first, first_dst, last, last_dst
        return 0, 0, 0, 0

    def move_direction(self, last, first):
        if self.direction == "right":
            Lcenter = self.center_list_left[floor(first / 2)]
            self.angle = -self.msg.angle_increment * Lcenter
            self.speed = self.obs_speed
        elif self.direction == "left":
            Rcenter = self.center_list_right[floor(last + (self.ranges_length - last) / 2)]
            self.angle = self.msg.angle_increment * Rcenter / 2.5
            self.speed = self.obs_speed
        elif self.direction == "back":
            self.angle = self.obs_angle
            self.speed = self.obs_speed
        else:
            self.angle = self.obs_angle
            self.speed = self.obs_backward_speed

    def compare_space(self, first_dst, last_dst):
        if self.obstacle_exit:
            if first_dst > last_dst and self.dist_data > self.min_dist:
                self.direction = "right"
            else:
                self.direction = "left"
        else:
            self.direction = "front"

    def drive_control(self, event): 
        drive = Twist()
        if self.manual:
            self.manual_speed = self.joy_axes[4]
            self.manual_steering = self.joy_axes[0]
            drive.linear.x = self.manual_speed * 1
            drive.angular.z = self.manual_steering * 30
        elif self.auto:
            if self.scaning:
                first, first_dst, last, last_dst = self.LiDAR_scan()
                self.compare_space(first_dst, last_dst)
                self.move_direction(last, first)
                drive.linear.x = self.speed
                drive.angular.z = self.angle
        else:
            drive.linear.x = 0
            drive.angular.z = 0
        self.drive_pub.publish(drive)

if __name__ == '__main__':
    try:
        move_car = ObstacleMove()
        rospy.spin()
    except KeyboardInterrupt:
        print("Program terminated")
