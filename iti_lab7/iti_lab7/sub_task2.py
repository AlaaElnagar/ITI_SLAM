#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from turtlesim.msg import Pose



class my_node(Node):
    count =0
    def __init__(self):

        super().__init__("sub_node")
        self.obj_pub=self.create_subscription(Pose,"/turtle1/custom_pose",self.x,rclpy.qos.qos_profile_sensor_data) 
        #self.obj_pub=self.create_publisher(String,"qos_test_topic",10)
        self.get_logger().info("subscriber is started")

    def x(self,msg):

        print("{} , {}".format(msg.x,msg.y))
        #msg.data="AlaaElnagar heard {} {}".format( self.count,msg.data)
        #self.obj_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()