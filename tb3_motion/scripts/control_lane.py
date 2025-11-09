#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class ControlLane():
    def __init__(self):
        # Sub to the output of detect lane
        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        self.lastError = 0

        # Max Velocity
        self.MAX_VEL = float(rospy.get_param("~max_vel", 0.1))
        # PID Gains
        self.Kp = float(rospy.get_param("~kp", 0.0072))
        self.Kd = float(rospy.get_param("~kd", 0.047))

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 500

        # default value
        # Kp = 0.0013
        # Kd = 0.009

        # gazebo value
        # Kp = 0.009
        # Kd = 0.03

        # Angular Calculation (PID control)
        angular_z = self.Kp * error + self.Kd * (error - self.lastError)
        
        self.lastError = error
        
        # Move 
        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 1200) ** 2.2), 0.2)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -1.0) if angular_z < 0 else -min(angular_z, 1.0)
        
        # rospy.loginfo("linearx")
        # rospy.loginfo(min(self.MAX_VEL * ((1 - abs(error) / 1200) ** 2.2), 0.2))
        # rospy.loginfo("angularz")
        # rospy.loginfo(-max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0))
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()