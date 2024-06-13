#! /usr/bin/env python
#-*- coding: utf-8 -*-

# Basic Import
import rospy
import math

# Enum Import 
from enums import DriveModeNum

# Publisher msg Import 
from ackermann_msgs.msg import AckermannDriveStamped

# Subscriber msg Import
from lane_detection.msg import detected_msg
from yumicart.msg import center_msgs


# Class Control
class Control():
    def __init__(self):
        # Create Log
        rospy.loginfo('Control is Created')

        # Publish Declaration
        self.control_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)

        # Subscriber Declaration
        rospy.Subscriber("/lane_pub",   detected_msg,   self.lane_pub_callback)
        rospy.Subscriber('/center',     center_msgs,    self.center_callback)

        # Variables Declaration and Initialization
        # lane_pub variables
        self.lane_x             = 0.0
        self.lane_y             = 0.0
        self.steering_angle     = 0.0
        self.speed              = 2.5
        # center variables
        self.drive_mode         = 0
        
        # ROS
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()
 
    # Callback Functions
    # /lane_pub Callback Function
    def lane_pub_callback(self, msg):
        self.lane_x = msg.xdetected - 160.0
        self.lane_y = 240.0 - msg.ydetected
        self.steering_angle = -1 * math.atan(self.lane_x / self.lane_y) / math.pi * 2 * 0.34 * 4
        # rospy.loginfo(f'{self.lane_x}')
        # rospy.loginfo(f'{self.lane_y}')
    # /center Callback Function
    def center_callback(self, msg):
        self.drive_mode = msg.drive_mode

    # Process Function
    def process(self):
        publish_data = AckermannDriveStamped()
        publish_data.header.stamp     = rospy.Time.now()
        publish_data.header.frame_id  = 'base_link'

        if self.drive_mode == DriveModeNu.FOLLOWING:
            publish_data.drive.steering_angle  = self.steering_angle
            publish_data.drive.speed           = self.speed * 0.25
            if self.drive_mode == DriveModeNu.STOP:
                publish_data.drive.speed = 0.0

        self.control_pub.publish(publish_data)

def run():
    rospy.init_node('control_node')
    control = Control()
    # rospy.spin()

if __name__=='__main__':
    run()