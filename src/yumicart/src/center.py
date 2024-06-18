#! /usr/bin/env python
#-*- coding: utf-8 -*-

# Basic Import
import rospy
import math

from functions import calc_distance

# Enum Import 
from enums import DriveModeNum, ProductNum

# Publisher msg Import 
from yumicart.msg import center_msgs

# Subscriber msg Import
from obstacle_detector.msg  import Obstacles
from scale_car_yolov5.msg   import Objects, Yolo_Objects
from fiducial_msgs.msg      import FiducialTransformArray, FiducialTransform
from yumicart.msg           import ui_msgs


# Class Center
class Center():
    def __init__(self):
        # Create Log
        rospy.loginfo('Center is Created')

        # Publish Declaration
        self.center_pub = rospy.Publisher('/center', center_msgs, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber('/raw_obstacles',          Obstacles,              self.raw_obstacles_callback)
        rospy.Subscriber('/yolov5_pub',             Yolo_Objects,           self.yolo_callback)
        rospy.Subscriber("/fiducial_transforms",    FiducialTransformArray, self.fiducial_transforms_callback)
        rospy.Subscriber("/ui",                     ui_msgs,                self.ui_callback)

        # Variables Declaration and Initialization
        # raw_obstacles variables
        self.min_dist_obstacle  = 0.0
        # yolo variables
        self.products = []
        # fiducial variables
        self.fiducial_id        = -1
        self.fiducial_z         = 0.0
        # ui variables
        self.drive_mode         = 0
        self.product_num        = 0
        # process variables
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
        self.min_dist_obstacle = 10.0
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
        rospy.loginfo(f'{self.min_dist_obstacle}')

    # /yolo Callback Function
    def yolo_callback(self, msg):
        self.products.clear()
        for yolo_object in msg.yolo_objects:
            self.products.append(yolo_object.c)
        # print(self.products)

    # /fiducial_transforms Callback Function
    def fiducial_transforms_callback(self, msg):
        if msg.transforms:
            self.fiducial_id    = msg.transforms[0].fiducial_id
            self.fiducial_z     = msg.transforms[0].transform.translation.z
        # else:
        #     self.fiducial_id    = -1
        #     self.fiducial_z     = 0.0
        # rospy.loginfo(f'{self.fiducial_id}')
        # rospy.loginfo(f'{self.fiducial_z}')

    # /ui Callback Function
    def ui_callback(self, msg):
        self.drive_mode = msg.drive_mode
        self.product_num = msg.product_number
        # rospy.loginfo(f'{self.drive_mode}')
        # rospy.loginfo(f'{self.product_num}')

    # Process Function
    def process(self):
        temp = center_msgs()
        temp.is_payment = False

        if self.drive_mode == DriveModeNum.FOLLOWING:
            self.fiducial_id    = -1
            self.fiducial_z     = 0.0
            
            if self.min_dist_obstacle < 1.0:
                temp.drive_mode = DriveModeNum.STOP
            else:
                temp.drive_mode = DriveModeNum.FOLLOWING
            # temp.drive_mode = DriveModeNum.FOLLOWING

        elif self.drive_mode == DriveModeNum.STOP:
            temp.drive_mode = DriveModeNum.STOP

        elif self.drive_mode == DriveModeNum.SEARCHING:
            if self.fiducial_id == self.product_num and self.fiducial_z < 1.0 or self.min_dist_obstacle < 1.0:
                temp.drive_mode = DriveModeNum.STOP
            else:
                temp.drive_mode = DriveModeNum.SEARCHING

        elif self.drive_mode == DriveModeNum.PAYMENT:
            if self.fiducial_id == 4 and self.fiducial_z < 1.0 or self.min_dist_obstacle < 1.0:
                temp.drive_mode = DriveModeNum.STOP
                temp.is_payment = True
            else:
                temp.drive_mode = DriveModeNum.FOLLOWING
                
        self.center_pub.publish(temp)

def run():
    rospy.init_node('center_node')
    center = Center()
    # rospy.spin()

if __name__=='__main__':
    run()