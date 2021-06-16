#!/usr/bin/env python3

from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, TransformStamped
class my_node (Node):
    def __init__(self):
        super().__init__("sub_node")
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Odometry,"odometry/wheel",self.odometry_call,10)
        self.create_subscription(LaserScan,"scan_bag",self.scan_bag_call,10)    
        self.get_logger().info("subscriber is started")
        #self.create_timer(1/13,self.scan_call)
        self.scan_pub=self.create_publisher(LaserScan,"scan",rclpy.qos.qos_profile_sensor_data)
        #self.create_timer(1/10,self.odom_call)
        self.odom_pub=self.create_publisher(Odometry,"odom",rclpy.qos.qos_profile_sensor_data)
        

    def odometry_call(self,msg):
        msg.header.stamp=self.get_clock().now().to_msg()
      
       # self.odom_pub.publish(msg)
  


    def scan_bag_call(self,msg):
        msg.header.stamp=self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)
'''
    def odom_call(self):
        self.get_logger().info("Heloo ziad 1234")
        msg=Odometry()
        msg.
        self.obj_pub.publish(msg)      
'''
def main (args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__=="__main__":
    main()


