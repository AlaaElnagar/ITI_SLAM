from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
    
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0      #rad
        loop_rate = self.create_rate(50)
        angle = -1.57 
        flag=0
        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'servo'
        odom_trans.child_frame_id = 'lidar'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()


                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = float(0)
                odom_trans.transform.translation.y = float(0)
                odom_trans.transform.translation.z = float(0)
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, angle+ pi/2 , 0) # roll,pitch,yaw

                # send the joint state and transform
                
                self.broadcaster.sendTransform(odom_trans)

                        # Create new robot state

                if flag ==0:
                    angle += (degree/4)
                else :
                    angle -= (degree/4)	
                        
                if angle>-1.1:
                    flag =1
                elif angle <-1.57:
                    flag =0
                self.get_logger().info("{} started".format(angle))	
                

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()

