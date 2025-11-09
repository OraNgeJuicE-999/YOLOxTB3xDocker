#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python and os packages 
import rospy, os, cv2
import numpy as np
from enum import Enum
import math
# Ros packages 
from std_msgs.msg import UInt8, Float64, Bool, String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Moving robot in-place (param)
# from turtlebot3_autorace_msgs.msg import MovingParam

class DetectSign():
    def __init__(self):
        # Odom messages
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------
        # Control & Motor contorls
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_lane_toggle = rospy.Publisher('/detect/lane_toggle', Bool, queue_size=1)
        self.pub_white_lane_toggle = rospy.Publisher('/detect/white_toggle', Bool, queue_size=1)
        self.pub_yellow_lane_toggle = rospy.Publisher('/detect/yellow_toggle', Bool, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------
        # Changes the angle of the ROI
        self.pub_bot_x = rospy.Publisher('/detect/lane_bot_x', UInt8, queue_size=1)
        self.pub_top_y = rospy.Publisher('/detect/lane_top_y', UInt8, queue_size=1)
        self.pub_top_x = rospy.Publisher('/detect/lane_top_x', UInt8, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------
        # Image Detection
        self.sub_sign = rospy.Subscriber('/yolo/sign_detection', String, self.cbCheckSign, queue_size=1)
        
        # -----------------------------------------------------------------------------------------------------------
        # Task Manage
        self.task_complete = rospy.Publisher('/task/complete', String, queue_size=1)
        self.task_busy_toggle = rospy.Publisher('/task/busy', Bool, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------
        # Variable Initialization
        self.cvBridge = CvBridge()
        self.counter = 1

        self.current_orientation_w = 0.0
        self.current_orientation = 0.0

        # -----------------------------------------------------------------------------------------------------------
        # Flag (After detecting the signs)
        self.intersection = False
        self.right_turn = False
        self.left_turn = False

        self.intersection_count = 0 
        self.white_lane_counter = 0
        self.right_count = 0
        self.left_count = 0

        # -----------------------------------------------------------------------------------------------------------
        # Motion Control (Move: m, Turn: omega)
        self.turn_in_time = int(rospy.get_param("~turn_in_time", 10))
        self.turn_in = int(rospy.get_param("~turn_in", 25))
        self.move_in = float(rospy.get_param("~move_in", 0.152))
        self.right_turn_in = int(rospy.get_param("~right_turn_in", 76))
        self.left_turn_in = int(rospy.get_param("~left_turn_in", 78))
        self.move_right = float(rospy.get_param("~move_right", 0.15))
        self.move_left = float(rospy.get_param("~move_left", 0.15))
        self.right_out_time = int(rospy.get_param("~right_out_time", 15))
        self.move_right_out = float(rospy.get_param("~move_right_out", 0.3))
        self.turn_right_out = int(rospy.get_param("~turn_right_out", 90))
        self.move_intersection_out = float(rospy.get_param("~move_intersection_out", 0.12))
        
        # Optional
        # self.construction_start = int(rospy.get_param("~construction_start", 15))
        # self.turn1 = int(rospy.get_param("~turn1", 35))
        # self.move1 = float(rospy.get_param("~move1", 0.5))
        # self.turn2 = int(rospy.get_param("~turn2", 70))
        # self.move2 = float(rospy.get_param("~move2", 0.59))
        # self.turn3 = int(rospy.get_param("~turn3", 50))
        # self.move3 = float(rospy.get_param("~move3", 0.1))

    def cbOdom(self, odom_msg):
        self.current_orientation_w = odom_msg.pose.pose.orientation.w

    def cbCheckSign(self, sign):
        if sign.data.strip() == "Intersection" and not self.intersection:
            self.intersection = True
            self.cbIntersection()
        elif sign.data.strip() == "Right" and not self.right_turn:
            self.right_turn = True
            self.cbIntersection()
        elif sign.data.strip() == "Left" and not self.left_turn:
            self.left_turn = True
            self.cbIntersection()      

    def cbIntersection(self):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 2 != 0:
        #     self.counter += 1
        #     return
        # else:
        #     self.counter = 1

        # Check if the intersection is seen and only operate it once
        if self.intersection == True and self.intersection_count == 0:
            for _ in range(self.turn_in_time):
                rospy.sleep(1)

            rospy.loginfo('[Intersection] INTERSECTION')
            self.pub_lane_toggle.publish(False)

            # Adjust to the right angle

            # Turn in
            self.turn_fn(self.turn_in, "left")
            # Move in
            self.move_fn(self.move_in, 'forward')

            rospy.sleep(1)

            # Finish Intersection, start yolo
            self.task_busy_toggle.publish(False)
            # Operate intersection once
            self.intersection_count += 1

        # Check for left and right turn
        if (self.right_turn == True or self.left_turn == True):

            if self.right_turn == True and self.right_count == 0:
                # 90 , 40, 150, 120
                # self.pub_top_x.publish(90)
                # self.pub_bot_x.publish(140)
                rospy.sleep(1)
                rospy.loginfo('[Intersection] RIGHT')

                # Right turn in
                self.turn_fn(self.right_turn_in, "right")
                
                rospy.sleep(1)

                # Move right
                self.move_fn(self.move_right, 'forward')
                
                self.pub_yellow_lane_toggle.publish(False)
                # rospy.loginfo("[Intersection] Lane Follow !")
                self.pub_lane_toggle.publish(True)

                for _ in range(self.right_out_time):
                    rospy.sleep(1)
                
                self.pub_lane_toggle.publish(False)
                
                
                # Move right out
                self.move_fn(self.move_right_out, 'forward')

                # Turn right out
                self.turn_fn(self.turn_right_out, "right")

                # Move intersection out
                self.move_fn(self.move_intersection_out, 'forward')
                

                self.pub_yellow_lane_toggle.publish(True)
                self.pub_lane_toggle.publish(True)
                
                self.right_count += 1

                # Finish Right, start yolo
                
            elif self.left_turn == True and self.left_count == 0:
                # 90 , 40, 100, 120
                # self.pub_top_x.publish(90)
                # self.pub_bot_x.publish(100)
                # self.pub_bot_x.publish(155)
                rospy.sleep(1)
                rospy.loginfo('[Intersection] LEFT')
                # Left turn in
                self.turn_fn(self.left_turn_in, "left")

                rospy.sleep(1)

                # Move left
                self.move_fn(self.move_left, 'forward')
                
                # Turning left
                self.pub_white_lane_toggle.publish(False)
                self.pub_lane_toggle.publish(True)
                rospy.sleep(10)
                
                # self.pub_white_lane_toggle.publish(True)
                # self.pub_lane_toggle.publish(True)
                self.left_count += 1

                # Finish Left, start yolo
            self.task_busy_toggle.publish(False)    
            self.task_complete.publish(String(data="IntersectionComplete"))

            # go to next mission
            if self.white_lane_counter == 0:
                self.white_lane_counter += 1
                self.pub_white_lane_toggle.publish(False)
            # self.task_complete.publish(String(data="Intersection_Complete"))
            rospy.loginfo("[Intersection] Finished Intersection :)")

            
            # ----------------------------------
            # rospy.sleep(14)
            # self.pub_white_lane_toggle.publish(True)
            # self.pub_yellow_lane_toggle.publish(False)
            # rospy.sleep(self.construction_start)

            # self.turn_fn(self.turn1, "left")
            # self.move_fn(self.move1, "Forward")
            # self.turn_fn(self.turn2, "right")
            # self.move_fn(self.move2, "Forward")
            # self.turn_fn(self.turn3, "left")
            # self.move_fn(self.move3, "Forward")
            
            
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

    # Move Function
    def move_fn(self, distance, move):
        speed = 0.1
        if move == 'forward':
            count = 1
        if move == 'backward':
            count = -1
        rate_hz = 20
        target_dist = abs(distance)
        dist_moved = 0.0

        # set up the Twist message: negative x for backwards
        twist = Twist()
        twist.linear.x = count * abs(speed)

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
    rospy.init_node('detect_intersection')
    node = DetectSign()
    node.main()