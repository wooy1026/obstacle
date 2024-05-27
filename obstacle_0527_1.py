#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from math import pi,floor
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

class ObstacleMove:
    def __init__(self):
        rospy.init_node("obstacle_drive") #노드명 설정

        #구독파트
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/joy", Joy, self.send_joy_callback)
        
        #발행파트
        self.drive_pub = rospy.Publisher("/cmd_vel_steer", Twist, queue_size=3)

        #변수 초기화
        self.rate = rospy.Rate(30) # 반복주기설정 (30hz)
        self.msg = None # 라이다센서 리스트 초기화
        self.lidar_flag = False
        self.dist_data = 0
        self.direction = None # 회피주행 방향 명령
        self.is_scan = False  # 라이다 작동상태 초기화

        self.current_time = rospy.Time.now() #현재 시간 기록

        self.obstacle_ranges = [] # 라이다 장애물 인식 범위초기화
        self.center_list_left = ([]) # 라이다 왼쪽 범위 초기화
        self.center_list_right = ([])

        self.speed = 0 # 회피주행 명령 속도 초기화
        self.angle = 0

        # 파라미터 파트
        self.min_dist = 0.2  # 너무가까워서 후진해야 하는 거리 기준
        self.range_obs = 1 #장애물 인식 기준거리
        self.obs_range = 40 #라이다 인식각도(180도 기준 +-)

        self.default_speed = 0.2  # 기본속도 (주행 및 장애물 회피)
        self.default_angle = 0.0  # 기본조향각 (주행 및 장애물 회피)
        self.backward_speed = -0.1 # 후진 속도

        self.OBSTACLE_PERCEPTION_BOUNDARY = 10 # 장애물 인식 점 개수
        self.ranges_length = None

        self.stop_flag = False # 정지여부 초기화
        self.obstacle_exit_flag = False  # 장애물 존재 여부 초기화


    def send_joy_callback(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]


    def laser_callback(self, msg):
        self.msg = msg
        self.is_scan = True
        self.obstacle_ranges = []
        if len(self.obstacle_ranges) > self.OBSTACLE_PERCEPTION_BOUNDARY:
            obstacle_detected_time = rospy.Time.now()
            elapsed_time = (obstacle_detected_time-self.current_time).to_sec()
            if elapsed_time < 5:
                self.stop_flag = True
            else:
                self.obstacle_exit_flag = True
                self.stop_flag = False
        else:
            self.obstacle_exit_flag = False
            self.current_time = rospy.Time.now()


    def LiDAR_scan(self):
        obstacle = []
        if self.lidar_flag == False:
            self.degrees = [(self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi for i, data in enumerate(self.msg.ranges)]
            self.ranges_length = len(self.msg.ranges)
            self.lidar_flag = True
            
        if len(self.degrees) > 0:
            for i, data in enumerate(self.msg.ranges):
                if 0 < data < self.range_obs and 180 - self.obs_range < self.degrees[i] < 180 + self.obs_range:
                    print(data)
                    obstacle.append(i)
                if len(obstacle) > 0:
                    self.dist_data = min(obstacle)
            if obstacle:
                first = obstacle[0]
                first_dst = first
                last = obstacle[-1]
                last_dst = self.ranges_length - last
                self.obstacle_ranges = self.msg.ranges[first : last + 1]
            else:
                first, first_dst, last, last_dst = 0, 0, 0, 0
            return first, first_dst, last, last_dst


    def move_direction(self, last, first):
        if self.direction == "right":
            for i in range(first):
                self.center_list_left.append(i)
            Lcenter = self.center_list_left[floor(first / 2)]
            center_angle_left = -self.msg.angle_increment * Lcenter
            self.angle = center_angle_left
            self.speed = self.default_speed

        elif self.direction == "left":
            for i in range(len(self.msg.ranges)):
                self.center_list_right.append(last + i)
            Rcenter = self.center_list_right[floor(last + (self.ranges_length - last) / 2)]
            center_angle_right = self.msg.angle_increment * Rcenter
            self.angle = center_angle_right / 2.5
            self.speed = self.default_speed
        elif self.direction == "back":
            self.angle = self.default_angle
            self.speed = self.backward_speed
        else:
            self.angle = self.default_angle
            self.speed = self.default_speed

    def compare_space(self, first_dst, last_dst):
        if self.obstacle_exit_flag == True:
            if first_dst > last_dst and self.dist_data > self.min_dist:
                self.direction = "right"
            elif first_dst <= last_dst and self.dist_data > self.min_dist:
                self.direction = "left"
            else:
                self.direction = "back"
        else:
            self.direction = "front"

    def main(self):
        drive = Twist()
        
        if self.manual == 1:
            self.manual_speed = self.joy_axes[4]
            self.manual_steering = self.joy_axes[0]
            drive.linear.x = self.manual_speed * 0.5
            drive.angular.z = self.manual_steering * 30
        
        elif self.auto == 1:
            if self.is_scan == True:
                first, first_dst, last, last_dst = self.LiDAR_scan()
                self.compare_space(first_dst, last_dst)
                self.move_direction(last, first)
                
                if self.stop_flag == True:
                    print("Stop")
                    drive.linear.x = 0
                    drive.angular.z = 0
                
                else :
                    if self.obstacle_exit_flag == False:
                        print("Auto: foward")
                        drive.linear.x = 0.2
                        drive.angular.z = 0
                    else:  
                        print("Auto: Obs_Move")
                        drive.linear.x = self.speed
                        drive.angular.z = self.angle
        else :
            drive.linear.x = 0
            drive.angular.z = 0

        self.drive_pub.publish(drive)
        self.rate.sleep()

if __name__ == "__main__":
    limo_obstacle_avoidence = ObstacleMove()
    try:
        print("start")
        while not rospy.is_shutdown():
            limo_obstacle_avoidence.main()
    except rospy.ROSInterruptException:
        pass

