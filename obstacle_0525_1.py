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
        rospy.init_node('joy_drive', anonymous=True)

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
        self.manual_speed = 0.2
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

        self.obs_speed = 0.2
        self.obs_backward_speed = -0.2
        self.obs_angle = 0

        self.lidar_flag = False
        self.degrees = []
        self.ranges_length = 0
        self.dist_data = 0
        self.direction = "front"

        # 타이머 설정
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    def send_joy_callback(self, data): 
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
    
    def laser_callback(self, msg):
        self.msg = msg
        self.scaning = True

        self.degrees = [(msg.angle_min + i * msg.angle_increment) * 180 / pi for i, _ in enumerate(msg.ranges)]
        self.ranges_length = len(msg.ranges)

        # 0도에서 45도 및 315도에서 360도 범위 내의 데이터를 필터링
        self.obstacle_ranges = [
            r for i, r in enumerate(msg.ranges) 
            if r > 0 and r < self.range_obs 
            and (135 <= self.degrees[i] % 360 <= 225)
            ]
        
        if len(self.obstacle_ranges) > self.obs_perception_boundary:
            print("obs_true")
            obs_detected_time = rospy.Time.now()
            elapsed_time = (obs_detected_time - self.current_time).to_sec()
            print(elapsed_time)
            if elapsed_time < 5:
                drive = Twist()
                drive.linear.x = 0
                drive.angular.z = 0
                self.drive_pub.publish(drive)
            else:
                self.obstacle_exit = True
                print("flag = true")
        else:
            print("obs_none")
            self.current_time = rospy.Time.now()
            self.obstacle_exit = False

    def LiDAR_scan(self):
        if not self.lidar_flag:
            self.degrees = [(self.msg.angle_min + i * self.msg.angle_increment) * 180 / pi for i, _ in enumerate(self.msg.ranges)]
            self.ranges_length = len(self.msg.ranges)
            self.lidar_flag = True
            
        # 0도에서 45도 및 315도에서 360도 범위 내의 장애물 인덱스 필터링
        obstacle_indices = [
            i for i, data in enumerate(self.msg.ranges) 
            if 0 < data < self.range_obs 
            and (135 <= self.degrees[i] % 360 <= 225)
            ]
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
                Lcenter_index = int(floor(first / 2))
                if 0 <= Lcenter_index < len(self.center_list_left):
                    Lcenter = self.center_list_left[Lcenter_index]
                    self.angle = -self.msg.angle_increment * Lcenter
                    self.speed = self.obs_speed
                else:
                    print("Lcenter index out of range")
            elif self.direction == "left":
                Rcenter_index = int(floor(last + (self.ranges_length - last) / 2))
                if 0 <= Rcenter_index < len(self.center_list_right):
                    Rcenter = self.center_list_right[Rcenter_index]
                    self.angle = self.msg.angle_increment * Rcenter / 2.5
                    self.speed = self.obs_speed
                else:
                    print("Rcenter index out of range")
            elif self.direction == "back":
                self.angle = self.obs_angle
                self.speed = self.obs_backward_speed
            else:
                self.angle = self.obs_angle
                self.speed = self.obs_speed


    def compare_space(self, first_dst, last_dst):
        if self.obstacle_exit:
            if first_dst > last_dst and self.dist_data > self.min_dist:
                self.direction = "right"
            else:
                self.direction = "left"
        else:
            self.direction = "front"

    def drive_control(self): 
        drive = Twist()
        if self.manual == 1:
            self.manual_speed = self.joy_axes[4]
            self.manual_steering = self.joy_axes[0]
            drive.linear.x = self.manual_speed * 1
            drive.angular.z = self.manual_steering * 30
        elif self.auto == 1:
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

    def control_loop(self, event):
        self.drive_control()

if __name__ == '__main__':
    try:
        Move_car = ObstacleMove()
        print("running")
        rospy.spin()  # 노드가 종료될 때까지 대기
    except KeyboardInterrupt:
        print("Program terminated")
