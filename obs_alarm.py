#!/usr/bin/env python3 
# -*- coding:utf-8 -*-  
import rospy
import numpy as np
import math
import requests
import json
from PyKakao import Message
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class move_limo:
    def __init__(self):
        rospy.init_node('control', anonymous=True)

        #obstacle_init
        self.dist_obstacle = 3
        self.x_list = []
        self.y_list = []
        self.last_alarm_time = 0  # Timestamp of the last alarm

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
    
    def alarm(self):
        API = Message(service_key = "d7fbc8a68b786c91e039530cc7d04832") # API키 (성환) # API키 (우영)
        url = "https://kapi.kakao.com/v2/api/talk/memo/default/send"

        token = "IKFEaXHofDXdE1VwJ2zVCx_B6pgQJkkZAAAAAQopyV4AAAGQB0gi5Fv0-avl6D9k" # 변경된 토큰 대입

        headers = {"Authorization": "Bearer " + token}

        data = {
            "template_object": json.dumps({
                "object_type": "feed",
                "content": {
                    "title": "!!차량!!",
                    "description": "차량의 앞에 장애물 있습니다.",
                    "image_url": "https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTh4p-cl_IRGnmdxj5hIzneRzzO617LWM8ekw&s", # 이미지 URL
                    "link": {}
                },
                "buttons": [{"title": "확인하세요","link": {}}]
            })
        }

        response = requests.post(url, headers=headers, data=data)
        print(response.status_code, ", 알림 발송")

    def obstacle(self, event):
        current_time = time.time()
        if self.dist_obstacle < 1:
            if current_time - self.last_alarm_time >= 10:
                self.alarm()
                self.last_alarm_time = current_time
            print("장애물 있음")
        else:
            print("장애물 없음")

if __name__ == '__main__':
    MoveCar = move_limo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")
