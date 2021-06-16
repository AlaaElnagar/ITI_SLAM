#!/usr/bin/env python3

from odom import WHEEL_BASE
import rclpy
import time 
import serial
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


# linear and angular velocities 
v = 0.0
w = 0.0 

## left error , left error diffrence , left error integeral 
e_l = 0.0
E_l = 0.0 
e_old_l = 0.0
## right error , right error diffrence , right error integeral 
e_r = 0.0          
E_r = 0.0          
e_old_r = 0.0
##________________________control variables____________________##
# linear pid control variables
kp_l = 200 
ki_l = 0
kd_l = 0
# linear pid control variables
kp_r = 200
ki_r = 0
kd_r = 0



class my_node(Node):
    def __init__(self):
        super().__init__("node1")

        self.serialcomm = serial.Serial('/dev/ttyACM0')
        self.serialcomm .flushInput()
        # create sub object to pose topic to get current state of the robot
        self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10) 

        #create timer 
        self.create_timer(1/10, self.callback )

    def callback(self):

        global v,w
        global e_l,E_l,e_old_l
        global e_r,E_r,e_old_r
        global kp_l,ki_l,kd_l
        global kp_r,ki_r,kd_r


        # 1 ) Get data from arduino 
        self.serialcomm.timeout = 1
        #time.sleep(0.5)

        #read data from sensor
        l = self.serialcomm.readline().decode('ascii').split(","); 

        # actual left and right velocities (actual)
        vl_a = float(l[1])
        vr_a = float(l[2])

        # desired left and right velocities (goal)

        vl_d = (w * WHEEL_BASE + 2*v) / 2.0
        vr_d = (2*v - w * WHEEL_BASE) / 2.0

        # calculate PID for left velocity
	    # 1- current error in left velocity 
        e_l = vl_d - vl_a
        # 2- error diffrence
        e_dot_l = e_l - e_old_l
        # 3- error sum
        E_l += e_l 
        # 4- controller output
        u_l = kp_l*e_l + ki_l*E_l + kd_l*e_dot_l

        # send to arduino serial 
        # set old error to current error
        e_old_l = e_l 

        
        # calculate PID for right velocity
        # 1- current error in angels 
        e_r = vr_d - vr_a
        # 2- error diffrence
        e_dot_r = e_r - e_old_r
        # 3- error sum
        E_r += e_r 
        # 4- controller output
        u_r = kp_r*e_r + ki_r*E_r + kd_r*e_dot_r
        # send to arduino serial 
        self.serialcomm.write(("{},{}".format(u_l,u_r).encode()))
        # set old error to current error
        e_old_r = e_r



        

    def vel_callback(self,msg):

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
	
    
def main(args = None):

    rclpy.init(args=args)

    node = my_node()

    rclpy.spin(node)

    rclpy.shutdown()
    

if __name__ == "__main__":

    main()