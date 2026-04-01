#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import threading
import cv2
import os
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CAMERA_FOV_RAD = 2.0 
MIN_OVERLAP = 0.1

class SurveyDrone:
    def __init__(self, id, height, dist_between, area_length, step_y):
        self.id = id
        self.height = height
        self.dist_between = dist_between
        self.area_length = area_length
        self.step_y = step_y
        self.mavros_ns = f"uav{id}"
        self.camera_ns = f"iris{id}"
        
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.leader_pose = PoseStamped()
        self.latest_image = None
        self.bridge = CvBridge()

        self.save_path = f"survey_results/drone_{id}"
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)

        rospy.Subscriber(f'{self.mavros_ns}/mavros/state', State, self.state_cb)
        rospy.Subscriber(f'{self.mavros_ns}/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber(f'{self.camera_ns}/down_camera_link/down_raw_image', Image, self.image_cb)
        
        if self.id > 0:
            rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.leader_cb)

        self.pos_pub = rospy.Publisher(f'{self.mavros_ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arm_srv = rospy.ServiceProxy(f'{self.mavros_ns}/mavros/cmd/arming', CommandBool)
        self.mode_srv = rospy.ServiceProxy(f'{self.mavros_ns}/mavros/set_mode', SetMode)

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg
    def image_cb(self, msg):
        try: self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError: pass
    def leader_cb(self, msg): self.leader_pose = msg

    def navigate_to(self, x, y, z):
        target = PoseStamped()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        rate = rospy.Rate(5) 
        
        while not rospy.is_shutdown():
            curr = self.current_pose.pose.position
            dist = math.sqrt((curr.x - x)**2 + (curr.y - y)**2 + (curr.z - z)**2)
            
            if dist < 0.2:
                time.sleep(3.0) 
                return True
                
            self.pos_pub.publish(target)
            rate.sleep()

    def follow_leader(self):
        rate = rospy.Rate(10)
        target = PoseStamped()
        offset_x = self.id * self.dist_between
        while not rospy.is_shutdown():
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = self.leader_pose.pose.position.x + offset_x
            target.pose.position.y = self.leader_pose.pose.position.y
            target.pose.position.z = self.height
            self.pos_pub.publish(target)
            rate.sleep()

    def run_mission(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            time.sleep(0.1)
        for _ in range(50):
            self.pos_pub.publish(self.current_pose)
            time.sleep(0.05)

        self.mode_srv(custom_mode="OFFBOARD")
        self.arm_srv(True)

        if self.id == 0:
            y_pos = 0.0
            photo_idx = 0
            while y_pos <= self.area_length:
                self.navigate_to(0, y_pos, self.height)
                if self.latest_image is not None:
                    cv2.imwrite(f"{self.save_path}/img_{photo_idx:03d}.jpg", self.latest_image)
                    photo_idx += 1
                y_pos += self.step_y
            
            self.navigate_to(0, 0, self.height)
            self.mode_srv(custom_mode="AUTO.LAND")
        else:
            def follower_photo_logic():
                p_idx = 0
                last_y = -10.0
                while not rospy.is_shutdown():
                    curr_y = self.current_pose.pose.position.y
                    if abs(curr_y - last_y) >= (self.step_y * 0.95):
                        time.sleep(2.5) 
                        if self.latest_image is not None:
                            cv2.imwrite(f"{self.save_path}/img_{p_idx:03d}.jpg", self.latest_image)
                            p_idx += 1
                            last_y = curr_y
                    time.sleep(0.5)
            
            threading.Thread(target=follower_photo_logic).start()
            self.follow_leader()

def main():
    rospy.init_node('swarm_pro_survey')
    try:
        h = float(input("Высота: "))
        l = float(input("Длина: "))
        w = float(input("Ширина: "))
    except:
        h, l, w = 5.0, 30.0, 15.0

    footprint_width = 2 * h * math.tan(CAMERA_FOV_RAD / 2)
    dist_between = footprint_width * (1 - MIN_OVERLAP)
    step_y = footprint_width * 0.8 

    threads = []
    for i in range(3):
        drone = SurveyDrone(i, h, dist_between, l, step_y)
        t = threading.Thread(target=drone.run_mission)
        threads.append(t)
        t.start()
    for t in threads: t.join()

if __name__ == '__main__':
    main()