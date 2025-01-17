#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class kalmanFilter(Node):
    def __init__(self):
        super().__init__("anand_kalman_filter")
        self.odom_sub_ = self.create_subscription(Odometry,"bumperbot_controller/odom_noisy",self.odom_callback,10)
        self.imu_sub_ = self.create_subscription(Imu,"imu/data",self.imu_callback,10)

        self.odom_pub_ = self.create_publisher(Odometry,"bumperbot_controller/odom_kalman",10)
        self.mean_ = 0.0
        self.variance_ = 1000.0
        
        self.imu_angular_z_ = 0.0 #filtering only ang vel about z!!
        self.is_first_odom_ = True  #to identify first wheel odometry msg
        self.last_angular_z_ = 0.0  #store value of ang vel coming from encoder sensor(odometry)

        self.motion_ = 0.0
        self.kalman_odom_ = Odometry() #contains result of kalman filter
        self.motion_variance = 4.0
        self.measurement_variance  = 0.5  #to configure kalman filter
    
    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance*self.mean_ + self.variance_*self.imu_angular_z_)/(self.variance_ + self.measurement_variance)

    def imu_callback(self,imu):
        self.imu_angular_z_ = imu.angular_velocity.z

    def odom_callback(self, odom):
        self.kalman_odom_ = odom
# computing the 1st ieteration of odometry function!
        if self.is_first_odom_:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = self.mean_

            self.is_first_odom_ = False
            return
        self.statePrediction()
        self.measurementUpdate()

