#!/usr/bin/env python3

from re import X
import rclpy
import csv
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from turtlesim.msg import Pose
from geometry_msgs.msg import Quaternion


class my_node(Node):

    def __init__(self):
        super().__init__("pub_node")
        self.csv_file_path = "pose.csv"
        self.lines = []
        self.count =1
        with open(self.csv_file_path, newline='\n') as csvfile:       
          self.readCSV = csv.reader(csvfile, delimiter = ',')
          for row in self.readCSV:
              self.lines.append(row)

       # self.create_timer(1/30,self.timer_call)

        self.odom_pub=self.create_publisher(Odometry,"odom",10)
        self.pose_sub=self.create_subscription(Pose,"/turtle1/pose",self.func_call,10)
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


    def func_call(self,msg):
       # self.get_logger().info("msg.x".format(msg.x))
        
        row = self.lines[self.count]
        row[0] = float(row[0])
        row[1] = float(row[1])
        row[2] = float(row[2])
        #convert to degree
        msg.theta *= 57.2957795 
        
        #------------------------------------------
        if ( ((row[0] < (msg.x + 2)) and (row[0] > ( msg.x - 2))  ) and  ( (row[1] < (msg.y + 2)) and (row[1] > ( msg.y -2))  )    ):  #and ( (row[2] < (msg.theta +5)) and (row[2] > (msg.theta-5)) )
            if (self.count ==4):
                self.count = 1
                self.get_logger().info("I reached final step !")
                msg.theta /= 57.2957795
                self.odom_call(msg.theta, row[0],row[1],row[2])

            else:
                self.count+=1
                self.get_logger().info("Nice ...goto next step !")
                msg.theta /= 57.2957795
                self.odom_call(msg.theta, row[0],row[1],row[2])                
        else :
            self.get_logger().info("going to point ({})x def = {} , y_def = {} , theta_def {}!".format(self.count,(row[0] - msg.x) , (row[1] - msg.y) ,(row[2] - (msg.theta )) ) )
           # self.get_logger().info("msg.x".format(msg.x))
        

        #---------------------------------------------------
        

    def odom_call(self,theta , x,y,z):
        odom = Odometry()
        pose = Pose()


        #Required_Data
 
        Quat_val = self.quaternion_from_euler(0, 0, theta)
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

        #--------------Frame&time---------------
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(odom)


def main (args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__=="__main__":
    main()
