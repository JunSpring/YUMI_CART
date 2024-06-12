#! /usr/bin/env python
#-*- coding: utf-8 -*-

# Basic Import
import rospy
import math

from functions import calc_distance

# Enum Import 
from mode_number import ModeNum

# Publisher msg Import 
from yumicart.msg import center_msgs

# Subscriber msg Import
from obstacle_detector.msg  import Obstacles
from scale_car_yolov5.msg   import Objects, Yolo_Objects
from fiducial_msgs.msg      import FiducialTransformArray, FiducialTransform


# Class Center
class Center():
    def __init__(self):
        # Create Log
        rospy.loginfo('Center is Created')

        # Publish Declaration
        self.center_pub = rospy.Publisher('/center', center_msgs, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber('/raw_obstacles',          Obstacles,              self.raw_obstacles_callback)
        rospy.Subscriber('/yolo',                   Yolo_Objects,           self.yolo_callback)
        rospy.Subscriber("/fiducial_transforms",    FiducialTransformArray, self.fiducial_transforms_callback)
        # rospy.Subscriber("/ui",                     detected_msg,           self.ui_callback)

        # Variables Declaration and Initialization
        # raw_obstacles variables
        self.min_dist_obstacle  = 0.0
        # yolo variables
        # fiducial variables
        self.fiducial_id        = -1
        self.fiducial_z         = 0.0
        # ui variables
        # process variables
        self.drive_mode         = 0
        self.steering_angle     = 0.0
        self.speed              = 2.5
        
        # ROS
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()
 
    # Callback Functions
    # /raw_obstacles Callback Function
    def raw_obstacles_callback(self, msg):
        if msg.circles:
            min_dist = float('inf')
            for circle in msg.circles:
                x = circle.center.x
                y = circle.center.y
                dist =  calc_distance(0.0, 0.0, x, y)
                if dist < min_dist:
                    min_dist = dist
            self.min_dist_obstacle = min_dist
        # else:
        #     self.min_dist_obstacle = 0.0
        # rospy.loginfo(f'{self.min_dist_obstacle}')

    # /yolo Callback Function
    def yolo_callback(self, msg):
        pass

    # /fiducial_transforms Callback Function
    def fiducial_transforms_callback(self, msg):
        if msg.transforms:
            self.fiducial_id    = msg.transforms[0].fiducial_id
            self.fiducial_z     = msg.transforms[0].transform.translation.z
        else:
            self.fiducial_id    = -1
            self.fiducial_z     = 0.0
        rospy.loginfo(f'{self.fiducial_id}')
        rospy.loginfo(f'{self.fiducial_z}')

    # /ui Callback Function
    def ui_callback(self, msg):
        pass

    # Process Function
    def process(self):
        temp = center_msgs()
        if self.fiducial_id == 4 and self.fiducial_z < 1.0 or self.min_dist_obstacle < 1.0:
            temp.drive_mode = ModeNum.STOP
        else:
            temp.drive_mode = ModeNum.FOLLOWING
        self.center_pub.publish(temp)

def run():
    rospy.init_node('center_node')
    center = Center()
    # rospy.spin()

if __name__=='__main__':
    run()