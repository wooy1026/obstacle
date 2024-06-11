#!/usr/bin/env python3 
# -*- coding:utf-8 -*-  
import rospy
import numpy as np
import math
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class move_limo:
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        #obstacle_init
        self.dist_obstacle = 3
        self.x_list = []
        self.y_list = []

        # Topic_pub_lidar
        rospy.Subscriber("scan", LaserScan, self.scan_callback)

        rospy.Timer(rospy.Duration(0.03), self.obstacle)

    def scan_callback(self, data):
        self.x_list = []
        self.y_list = []

        for i, n in enumerate(data.ranges):
            angle = data.angle_min + data.angle_increment * i
            angle_degree = angle * 180 / math.pi
            x = -n * math.cos(angle)
            y = n * math.sin(angle)
            if y > -0.25 and y < 0.25 and x > 0:
                self.x_list.append(x)
                self.y_list.append(y)
        if self.x_list:
            self.dist_obstacle = min(self.x_list)
        else:
            self.dist_obstacle = 3  # 기본값으로 설정하거나 적절한 값을 설정하십시오
        return self.dist_obstacle
    
    def obstacle(self, event):
        if self.dist_obstacle < 0.5:
            print("장애물 존재")
        else:
            print("장애물 없음")

if __name__ == '__main__':
    MoveCar = move_limo()
    try:
        rospy.spin()
        print("#############3")
    except KeyboardInterrupt:
        print("program down")
