import rclpy
from rclpy.node import Node
from ros2projsrv.srv import Boolsrv
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
    regen_spawn_x = 0.
    regen_spawn_y = 0.
    start_flag = 0
    theta_flag = 0
    s_flag = 0
    def __init__(self):
        super().__init__("controll_node")
         
        self.sub_sub=self.create_subscription(Pose,"/turtle1/pose",self.func_call,10)
        self.turtle_pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.get_logger().info("controll_node is started")
        #------start with new turtile 
        self.serv_client(True,False)

    def func_call(self ,msg): 

        if (self.start_flag ==1):
            self.serv_client(True,False)
            self.start_flag = 0

        #get my turtal pose     
        self.main_x =msg.x
        self.main_y = msg.y
        self.main_theta=msg.theta

        #self.get_logger().info("mainx {} -  main {}".format ( self.main_x,self.main_y))
        #self.get_logger().info("regx {} -  regy {}".format ( self.regen_spawn_x,self.regen_spawn_y ))

        #-------------------------------------------Make some cool calculations 
        #self.regen_spawn_x ---> pose of regenerated turtle from service 
        self._err = (math.sqrt( ((self.main_x -  self.regen_spawn_x)**2)+ ((self.main_y -self.regen_spawn_y)**2) ) ) 
           
        self._theta=math.atan2((self.regen_spawn_y - self.main_y ) , (self.regen_spawn_x -self.main_x  )  )

        self.err_theta = (self._theta - self.main_theta ) *.7
        T_msg = Twist()
       # self.get_logger().info(" Out optimizw linear err {} - angerr {}".format ( self._err,self.err_theta))     

#ang movement 
        if abs(self.err_theta) >.2 :   #.01
            #T_msg._linear.x = 0.                   #self._err
            T_msg._linear.x = 0.

            T_msg.angular.z = abs(self.err_theta)
            self.turtle_pub.publish(T_msg)

        self.get_logger().info(" theta optimizw linear err {} - angerr {}".format ( self._err,abs(self.err_theta)))     
            
 #linear movement 

        if abs(self.err_theta) < .2 :
            #self.s_flag = 1
            T_msg._linear.x = self._err
 
            T_msg.angular.z = 0.
            self.turtle_pub.publish(T_msg)

        

     


#killing 
        if (self._err <.5):
                                       #kill
            self.start_flag =1  
            self.s_flag = 0 
            self.serv_client(False,True)


            #self.get_logger().info("kill  {} -  Err t {}".format ( self._err,self.err_theta))
       # self.get_logger().info("_err  {} -  theta{}".format ( self._err,self.err_theta))





    def serv_client(self , a,b):
        client=self.create_client(Boolsrv,"my_server")
        
        while client.wait_for_service(1)==False:
            self.get_logger().warn("wating for my_server server")
        request=Boolsrv.Request()
        request.newturtle=a
        request.arrivedornot=b
        futur_obj=client.call_async(request)
        futur_obj.add_done_callback(self.future_call)

    def future_call(self,res_msg):
        #self.get_logger().info(str(res_msg.result().state))
        self.regen_spawn_x= res_msg.result().linrx
        self.regen_spawn_y=res_msg.result().linry



def main(args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=="__main__":
    main()
