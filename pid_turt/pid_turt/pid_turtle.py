import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math



class my_node(Node):
    flag =0
    _err =0.
    _theta =0.
    err_theta =0.

    theta_err = 0.

    main_x= 0.
    main_y= 0.
    main_theta= 0.
    
    sub_x = 0.
    sub_y = 0.
    sub_theta = 0.
    parallel = 0.
    adj = 0.
    regen_spawn_x = 9.    #target
    regen_spawn_y = 9.    #target
    start_flag = 0
    theta_flag = 0
    s_flag = 0
    dist_acumerr=0
    last_err=0
    flag_x =0

    def __init__(self):
        super().__init__("controll_node")
         
        self.sub_sub=self.create_subscription(Pose,"/turtle1/pose",self.func_call,10)
        self.turtle_pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.get_logger().info("controll_node is started")
        #------start with new turtile 
        self._dist_ki=0
        self._dist_kp=1
        self._dist_kd=0
        self._dist_err=0
        self._theta_err=0
        self.theta_ki=0
        self.theta_kp=1
        self.theta_kd=0
        self.theta_acumerr=0
        self.last_thetaerr=0

    

    def func_call(self ,msg): 


        #get my turtal pose     
        self.main_x =msg.x
        self.main_y = msg.y
        self.main_theta=msg.theta

 
        self._err = (math.sqrt( ((self.main_x -  self.regen_spawn_x)**2)+ ((self.main_y -self.regen_spawn_y)**2) ) ) 
        self.dist_acumerr+=self._err 

        u_ =  self._err  * self._dist_kp +  self.dist_acumerr * self._dist_ki + (self._err-self.last_err ) * self._dist_kd
        self.last_err= self._err


        self._theta=math.atan2((self.regen_spawn_y - self.main_y ) , (self.regen_spawn_x -self.main_x  )  )
        self.theta_acumerr+=self._theta

        u_theta = self._theta *self.theta_kp + self.theta_acumerr *self.theta_ki+(self._theta-self.last_thetaerr) *self.theta_kd
        self.last_thetaerr=self._theta
        T_msg = Twist()
        self.get_logger().info("Theat____________ {} - ".format (self._err ))     

     
        if (self._err >1):
            T_msg.angular.z = u_theta 
                    
            T_msg._linear.x = u_
        else :
       
            T_msg.angular.z = 0
            T_msg._linear.x = 0
            self.flag_x=1


             
 
        self.turtle_pub.publish(T_msg)

        

     




def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()