#!/usr/bin/env python3

import rclpy
import time
import serial
from math import sin,cos
from functions import quaternion_from_euler
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np 
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi



x = 0.0
y = 0.0
theta = 0.0 
dt = 1.0
WHEEL_BASE = .5 
v_l=0
v_r=0
vr_err_present=0
vr_err_accum=0
vr_err_last=0
vr_err_diff=0
vr_P=200 #250
vr_I=30 #25 #20
vr_d=15 #15 #20

vl_err_present=0
vl_err_accum=0
vl_err_last=0
vl_err_diff=0
vl_P=200 #250
vl_I=30 #25
vl_d=15 #15 #20


class my_node(Node):
    def __init__(self):
        super().__init__("odom_node")

        
        self.serialcomm = serial.Serial('/dev/ttyACM0')
        self.serialcomm .flushInput()
        self.pub = self.create_publisher(Odometry,"odometry/wheel",rclpy.qos.qos_profile_sensor_data)
        self.pose_sub=self.create_subscription(Twist,"/cmd_vel",self.get_robot_pose,10) 
        self.create_timer(1/10,self.callback)
        

    def get_robot_pose(self,msg):
        global x,y,theta,dt,WHEEL_BASE ,v_r,v_l

        Linear_vel = msg.linear.x
        Angular_vel = msg.angular.z

        v_r=((Angular_vel*WHEEL_BASE)+2*Linear_vel)/2
        v_l=(2*Linear_vel-Angular_vel*WHEEL_BASE)/2

    def callback(self):
        global vr_err_present,vr_err_accum, vr_err_last, vr_P,  vr_I,vr_d,vl_err_present,vl_err_accum, vl_err_last, vl_P,  vl_I,vr_d ,vl_err_diff,vr_err_diff
        global x,y,theta,dt,WHEEL_BASE ,v_r,v_l
        
        Odometry_data = Odometry()
        l = self.serialcomm.readline().decode('ascii').split(",")

        vl = float(l[1])
        vr = float(l[2])
        
        v_linear = (vr+vl)/2
        v_angular=(vr-vl)/.58   #note .55 is wheel distance 
        Odometry_data.twist.twist.linear.x =v_linear
        Odometry_data.twist.twist.angular.z =v_angular
        Odometry_data.header.frame_id="odom"
        Odometry_data.child_frame_id="base_link"
        Odometry_data.header.stamp=self.get_clock().now().to_msg()


        vl_err_present=v_l-vl
        vl_err_accum+=vl_err_present
        vl_err_diff=vl_err_present-vl_err_last
        vl_err_last=vl_err_present

        ul=vl_err_present*vl_P + vl_err_accum * vl_I + vl_err_diff *vl_d
        if (ul>255):
            ul=255
        elif ul<-255:
            ul=-255

        vr_err_present=v_r-vr
        vr_err_accum+=vr_err_present
        vr_err_diff=vr_err_present-vr_err_last
        vr_err_last=vr_err_present

        if vr_I*vr_err_accum > 255 :
            vr_err_accum = 255/vr_I
        
        if vl_I*vl_err_accum > 255 :
            vl_err_accum = 255/vl_I


        ur=vr_err_present*vr_P + vr_err_accum * vr_I + vr_err_diff *vr_d
        if (ur>255):
            ur=255
        elif ur<-255:
            ur=-255
            
        #self.serialcomm.write(("{},{}".format(ul,ur).encode()))  #------------------------------------uncomment 
        print ("vl_feed back = {} , vr_feed back ={}".format(vl_err_present,vr_err_present))
        print ("vl_dESIRED = {} , vl_actual ={}".format(v_l,vl))
        print ("vr_dESIRED = {} , vr_actual ={}".format(v_r,vr))
        print ("vR_PWM = {} , vL_PWM ={}".format(ur,ul))
        self.pub.publish(Odometry_data)

        

def main(args = None):
    rclpy.init(args=args)

    node = my_node()

    rclpy.spin(node)

    rclpy.shutdown()
    

if __name__ == "__main__":
    main()

  

