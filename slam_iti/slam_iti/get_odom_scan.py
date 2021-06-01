#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class my_node(Node):
    
    def __init__(self):

        super().__init__("sub_node")
        self.lidar_sub=self.create_subscription(LaserScan,"/scan",self.get_lidar_topic,10) 
        self.odom_sub=self.create_subscription(Odometry,"/odometry/wheel",self.get_odom_topic,10) 


        self.lidar_pub=self.create_publisher(LaserScan,"slam/scan",10)
        self.odom_pub=self.create_publisher(Odometry,"slam/Odometry",10)

        self.get_logger().info("lidar & odom  subscriper and publisher is started")
#publish lidar data which we get from Bag file  with fresh time stamp
    def get_lidar_topic(self,msg):
        lidar_data = LaserScan()
        lidar_data.header.frame_id = "lazer"
        lidar_data.header.stamp=self.get_clock().now().to_msg()
        lidar_data.angle_min=msg.angle_min
        lidar_data.angle_max=msg.angle_max
        lidar_data.angle_increment=msg.angle_increment
        lidar_data.time_increment=msg.time_increment
        lidar_data.scan_time=msg.scan_time
        lidar_data.range_min=msg.range_min
        lidar_data.range_max=msg.range_max
        lidar_data.ranges=msg.ranges
        lidar_data.intensities=msg.intensities
        #msg.data="AlaaElnagar heard {} {} times".format( x , self.count)
        self.lidar_pub.publish(lidar_data)
        #self.get_logger().info(msg.data)

    def get_odom_topic(self,odom_msg):
        odom=Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        #--------------Orientation----------------
        odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z 
        odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w
        #--------------Position----------------
        odom.pose.pose.position.x = odom_msg.pose.pose.position.x 
        odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        odom.pose.pose.position.z = odom_msg.pose.pose.position.z

        odom.pose.covariance = odom_msg.pose.covariance

        #--------------Velocity----------------
       
        odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x
        odom.twist.twist.linear.y = odom_msg.twist.twist.linear.y 
        odom.twist.twist.angular.z = odom_msg.twist.twist.angular.z
        odom.twist.covariance = odom_msg.twist.covariance

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()