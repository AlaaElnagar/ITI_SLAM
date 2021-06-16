#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped



class my_node(Node):
    count=5
    degree = pi / 180.0      #rad
    angle = -1.57 
    flag=0
    def __init__(self):
        super().__init__("transformation_publisher")
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'servo'
        self.odom_trans.child_frame_id = 'lidar'

        self.create_timer(1/128,self.timer_call)
        self.get_logger().info("int is started")

    def timer_call(self):
        now = self.get_clock().now()  
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = float(0)
        self.odom_trans.transform.translation.y = float(0)
        self.odom_trans.transform.translation.z = float(0)
        self.odom_trans.transform.rotation = \
            self.euler_to_quaternion(0,  self.angle+pi/2, 0) # roll,pitch,yaw

                # send the joint state and transform
        
        self.broadcaster.sendTransform(self.odom_trans)

                # Create new robot state

        if self.flag ==0:
            self.angle += (self.degree/4)
        else :
            self.angle -= (self.degree/4)	
                
        if self.angle>-1.1:
            self.flag =1
        elif self.angle <-1.57:
            self.flag =0
        self.get_logger().info("{} started".format(self.angle))	       
#  self.get_logger().info("Hello")  # Call Back

#self.get_logger().info("{} is publishing {}".format( msg.data, msg.num))

    def euler_to_quaternion(self,roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()