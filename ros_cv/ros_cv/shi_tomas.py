#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge as cv_bridge
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import cv2
bridge = CvBridge()

class sub_node(Node):
    
    def __init__(self):

        super().__init__("sub_node")
        self.create_subscription(Image,"/intel_realsense_d435_depth/image_raw",self.img_cb, rclpy.qos.qos_profile_sensor_data)
        self.get_logger().info("subscriber is started")

    def img_cb(self, message):

        colored_image = bridge.imgmsg_to_cv2(message, "bgr8")
        black_image = np.zeros((240,320,1), np.uint8)
        gray_image = cv2.cvtColor(colored_image,cv2.COLOR_BGR2GRAY)

        first_timer = time.time()
        image_corners = cv2.goodFeaturesToTrack(gray_image,25,0.01,10)
        sec_timer = time.time() - first_timer
        print(sec_timer)

        image_corners = np.int0(image_corners)

        if image_corners is not None :

            for i in image_corners:  
                x, y = i.ravel()
                cv2.circle(black_image,(x, y),3,255,-1)

            cv2.imshow("My_Image", colored_image)
            cv2.imshow("My_Corners", black_image)
        
        if (cv2.waitKey(1) & 0xff) == ord('q'):
            cv2.destroyAllWindows()  

def main (args=None):
    rclpy.init(args=args)
    node = sub_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()