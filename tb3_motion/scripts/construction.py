#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os, cv2
import numpy as np
from enum import Enum
from std_msgs.msg import UInt8, Float64, Bool, String
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist

class DetectSign():
    def __init__(self):
        # Subscriber
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------        
        # Motion Control
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_lane_toggle = rospy.Publisher('/detect/lane_toggle', Bool, queue_size=1)
        self.pub_yellow_lane_toggle = rospy.Publisher('/detect/yellow_toggle', Bool, queue_size=1)
        self.pub_white_lane_toggle = rospy.Publisher('/detect/white_toggle', Bool, queue_size=1)
        # publishes traffic sign image in compressed type 
        # self.pub_image_traffic_sign = rospy.Publisher('/detect/sign/compressed', CompressedImage, queue_size = 1)
        
        # -----------------------------------------------------------------------------------------------------------
        # Image Detection
        self.sub_sign = rospy.Subscriber('/yolo/sign_detection', String, self.cbFindConstruction, queue_size=1)
        
        # -----------------------------------------------------------------------------------------------------------
        # Task Manage
        self.task_complete = rospy.Publisher('/task/complete', String, queue_size=1)
        self.intersection_complete = rospy.Subscriber('/task/complete', String, self.cbStartConstruction,queue_size=1)
        # self.task_check_complete = rospy.Subscriber('/task/complete', String, self.cbTaskComplete, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------
        # Variables
        self.cvBridge = CvBridge()
        self.white_detect_counter = 0
        self.avoid_obstacle_count = 0
        
        self.start_task = False
        self.construction = False

        self.is_obstacle_detected = False

        # -----------------------------------------------------------------------------------------------------------
        # Get Param
        # Motion Control Param
        self.turn1 = int(rospy.get_param("~turn1", 35))
        self.move1 = float(rospy.get_param("~move1", 0.5))
        self.turn2 = int(rospy.get_param("~turn2", 70))
        self.move2 = float(rospy.get_param("~move2", 0.59))
        # Optional
        self.turn3 = int(rospy.get_param("~turn3", 50))
        self.move3 = float(rospy.get_param("~move3", 0.1))

        # Lidar Param
        self.threshold_distance = float(rospy.get_param("~threshold_distance", 0.3))
        self.angle_scan = int(rospy.get_param("~angle_scan", 5))

    # def cbTaskComplete(self, complete_task):
    #     if complete_task.data.strip() == "Intersection_Complete":
    #         self.start_task = True

    def cbStartConstruction(self, msg):
        if msg.data.strip() == 'IntersectionComplete':
            rospy.loginfo("[Construction] Start!")
            self.start_task = True
    def cbScanObstacle(self, scan):
        # angle_scan = 25
        scan_start = 0 - self.angle_scan
        scan_end = 0 + self.angle_scan

        is_obstacle_detected = False
        
        if self.construction:
            for i in range(scan_start, scan_end):
                if scan.ranges[i] < self.threshold_distance and scan.ranges[i] > 0.15:
                    is_obstacle_detected = True
            
            self.is_obstacle_detected = is_obstacle_detected
            rospy.loginfo("[Construction] lsdgmplserp:")
            rospy.loginfo(self.is_obstacle_detected)
            rospy.sleep(0.5)
            rospy.loginfo("[Construction] Callback Started!")
            self.pub_white_lane_toggle.publish(True)
            self.pub_yellow_lane_toggle.publish(False)
            rospy.sleep(6)
            self.avoidObstacle()

    def avoidObstacle(self):
        rospy.loginfo('[Construction] Avoid Obstacle!')
        # Close All Lane Detection
        self.pub_lane_toggle.publish(False)
        
        # Avoid Obstacle
        self.turn_fn(self.turn1, "left")
        self.move_fn(self.move1, "Forward")
        self.turn_fn(self.turn2, "right")
        self.move_fn(self.move2, "Forward")
        self.turn_fn(self.turn3, "left")
        self.move_fn(self.move3, "Forward")
        rospy.loginfo('[Construction] Construction Completed!')
        # Re-Activate Lane Detection
        self.pub_white_lane_toggle.publish(True)
        self.pub_yellow_lane_toggle.publish(True)
        self.pub_lane_toggle.publish(True)

        # Move on to the next task
        self.task_complete.publish(String(data="ConstructionComplete"))

    def cbFindConstruction(self, sign):
        rospy.loginfo("[Construction] YOLO: %r", sign.data)
        if sign.data.strip() == "Construction":
            self.construction = True
            rospy.loginfo("[Construction] Sign Detected")
            self.cbConstruction()

    def cbConstruction(self):
        # if not self.start_task:
        #     rospy.loginfo(self.start_task)
        #     return
        if self.start_task:
            if self.construction == True and self.white_detect_counter == 0:
                rospy.sleep(0.5)
                rospy.loginfo("[Construction] Callback Started!")
                self.pub_white_lane_toggle.publish(True)
                self.pub_yellow_lane_toggle.publish(False)

                self.white_detect_counter += 1

    # Turn Function
    def turn_fn(self, turn_angle, dir):
        omega = 0.5
        rate_hz = 20
        target_angle = math.radians(turn_angle)
        angle_turned = 0.0
        # rospy.loginfo(turn_angle)

        if dir == "right":
            count = -1
        elif dir == "left":
            count = 1

        # rospy.loginfo(dir)
        # rospy.loginfo(count)

        rate = rospy.Rate(rate_hz)
        twist = Twist()
        twist.angular.z = count * abs(omega)

        # timestamp of previous iteration
        last_t = rospy.Time.now().to_sec()

        while angle_turned < target_angle and not rospy.is_shutdown():
            # publish spin
            self.pub_cmd_vel.publish(twist)

            # compute dt
            now = rospy.Time.now().to_sec()
            dt = now - last_t
            last_t = now

            # integrate
            angle_turned += abs(omega) * dt

            rate.sleep()

        # stop
        self.pub_cmd_vel.publish(Twist())

    # Move Fucntion
    def move_fn(self, distance, move):
        speed = 0.1
        # if move == 'forward':
        #     movetype = 1
        # if move == 'backward':
        #     movetype = -1
        rate_hz = 20
        target_dist = abs(distance)
        dist_moved = 0.0

        # set up the Twist message: negative x for backwards
        twist = Twist()
        twist.linear.x = abs(speed)

        rate = rospy.Rate(rate_hz)
        last_t = rospy.Time.now().to_sec()

        while dist_moved < target_dist and not rospy.is_shutdown():
            # send the backward command
            self.pub_cmd_vel.publish(twist)

            # compute elapsed time
            now = rospy.Time.now().to_sec()
            dt = now - last_t
            last_t = now

            # integrate distance
            dist_moved += abs(speed) * dt

            rate.sleep()
            
        self.pub_cmd_vel.publish(Twist())


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    # rospy.loginfo('[Construction] Node Started')
    rospy.init_node('detect_construction')
    node = DetectSign()
    node.main()