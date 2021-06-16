#!/usr/bin/env python3

import imp
from tkinter import W
import rclpy
from rclpy.node import MsgType, Node
import csv
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Point
import time


class my_node(Node):
    count = 0
    ext_count=0
    def __init__(self):
        super().__init__("pub_node")

        self.create_timer(1/30,self.timer_call)

        self.odom_pub=self.create_publisher(Odometry,"zed2_imu",10)
        #setting the time 
        self.current_time =  5
        self.last_time =  3
        
        self.get_logger().info("pub_node is started")

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


    def timer_call(self):
        odom = Odometry()
        pose = Pose()
        twist= Twist()

        self.current_time = 4
        dt = (self.current_time - self.last_time)
        #Required_Data
        x = 0.0 
        y = 0.0 
        th= 0.0
        #given
        v_right = 5. 
        v_left  = 5.
        WHEEL_BASE = .5
        #calculations 
        vx = (v_right + v_left) / 2.0
        vy = 0.0
        vth = (v_right - v_left) / WHEEL_BASE
        # update the new position
        delta_x = (vx * cos(th) - vy * sin(th)) * float(dt)
        delta_y = (vx * sin(th) + vy * cos(th)) * float(dt)
        delta_th = vth * dt
        x += delta_x
        y += delta_y 
        th += delta_th
        Quat_val = self.quaternion_from_euler(0, 0, th)
        #>>>time
        #loading odom msg
        #--------------Orientation----------------
        odom.pose.pose.orientation.x = Quat_val.x
        odom.pose.pose.orientation.y = Quat_val.y
        odom.pose.pose.orientation.z = Quat_val.z
        odom.pose.pose.orientation.w = Quat_val.w
        #--------------Position----------------
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.covariance = [0.01 , 0.0  , 0.0  ,0.0  , 0.0  ,0.0,
                                0.0  , 0.01 , 0.0  ,0.0  , 0.0  ,0.0,
                                0.0  , 0.0  , 0.01 ,0.0  , 0.0  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.1 , 0.0  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.0 , 0.1  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.0 , 0.0  ,0.1]
        #--------------Velocity----------------
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        odom.twist.covariance = [0.01 , 0.0  , 0.0  ,0.0  , 0.0  ,0.0,
                                0.0  , 0.01 , 0.0  ,0.0  , 0.0  ,0.0,
                                0.0  , 0.0  , 0.01 ,0.0  , 0.0  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.1 , 0.0  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.0 , 0.1  ,0.0,
                                0.0  , 0.0  , 0.0  , 0.0 , 0.0  ,0.1]
        #--------------Frame&time---------------
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(odom)


            
       #self.get_logger().warn("Imu_msg.linear_acceleration.x = {} , Imu_msg.angular_velocity.z= {}".format(Imu_msg.linear_acceleration.x,Imu_msg.angular_velocity.z))
      #  self.get_logger().info("Hello")  # Call Back

    



def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()    
