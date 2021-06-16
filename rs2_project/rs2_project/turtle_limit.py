#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.msg import Pose

class my_node(Node):
    flag = 0
    x_turt_c =0.
    y_turt_c =0.
    theta_turt_c=0.
    regen_spawn_x = 0.
    regen_spawn_y = 0.
    def __init__(self):
        super().__init__("limitation_node")
        self.sub_sub=self.create_subscription(Pose,"/turtle1/pose",self.func_call,10)

        self.get_logger().info("limitation_node  node is started")
       


    def func_call(self , msg_):
        done = 0
        self.regen_spawn_x = msg_.x
        self.regen_spawn_x = msg_.y
        if not( msg_.x > 2 and  msg_.x < 8) :
            done = 1
 
        if not (msg_.y > 2 and  msg_.y < 8) :
            done = 1
        
        if done:
            client=self.create_client(Empty,"/reset")
            while client.wait_for_service(1)==False:
                self.get_logger().warn("wating for turtlesim")
            req_reset = Empty.Request()
            futur_obj2=client.call_async(req_reset)
            done = 0
                  

        #self.get_logger().info("regx {} -  regy {}".format ( self. response.linrx,self.response.linry))



def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()


