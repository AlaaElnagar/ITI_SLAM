#!/usr/bin/env python3
from contextlib import nullcontext
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


image1 = nullcontext
image2 = nullcontext
matches=nullcontext
class sub_node(Node):
    
    def __init__(self):

        super().__init__("sub_node")
        self.create_subscription(Image,"/intel_realsense_d435_depth/image_raw",self.img_cb, rclpy.qos.qos_profile_sensor_data)
        self.get_logger().info("subscriber is started")
        self.flag = 0 
        self.start = 0 

    def img_cb(self, message):
        global image2,image1,matches
        colored_image = bridge.imgmsg_to_cv2(message, "bgr8")
        black_image = np.zeros((240,320,1), np.uint8)
        gray_image = cv2.cvtColor(colored_image,cv2.COLOR_BGR2GRAY)

        if (self.flag ==0 and self.start==0):
            image1 = np.copy(gray_image)
            self.flag =1
        elif (self.flag ==1 and self.start==0):
            image2 = np.copy(gray_image)
            self.flag =0
            self.start =1

  

        #start ORB     
        if self.start :
            #image1 = cv2.imread('start_frame.png')
            #image2 = cv2.imread('end_frame.png')
            orb = cv2.ORB_create()  # Initiate ORB object
            BFMatcher = cv2.BFMatcher(normType = cv2.NORM_HAMMING,crossCheck = True)

            keypoints, descriptors = orb.detectAndCompute(image1, None)
            keypoints_2, descriptors_2 = orb.detectAndCompute(image2, None)

            if descriptors is not None and descriptors_2 is not None:
                matches = BFMatcher.match(queryDescriptors = descriptors,
                            trainDescriptors = descriptors_2)

                matches = sorted(matches, key = lambda x: x.distance)

            # Draw first 15 matches

                list_kp1 = [keypoints[mat.queryIdx].pt for mat in matches] 
                list_kp2 = [keypoints_2[mat.trainIdx].pt for mat in matches]



            #get average
                x1= 0
                y1=0
                x2= 0
                y2=0
                for i in range(len(list_kp1)):
                    x2+=(int (list_kp1[i][0]) -int (list_kp2[i][0]))
                    y2+=(int (list_kp1[i][1]) -int (list_kp2[i][1]))

                if len(list_kp1) :    
                    x2/=len(list_kp1)
                    y2/=len(list_kp1)


                #Drow arrow 
                color = (0, 255, 0) 
                thickness = 8
                input_image_height, input_image_width = gray_image.shape

                start_point = (int (input_image_width/2),int (input_image_height/2)  )              
                # End coordinate
                end_point = (int (input_image_width/2)+int (x2*5),int (input_image_height/2)+int (y2*5))    

                image = cv2.arrowedLine(colored_image, start_point, end_point,
                                        color, thickness)


                if gray_image is not None :

        
                    cv2.imshow("My_Image", image)
                image1=np.copy(image2) 
                image2=np.copy(gray_image) 
            
          
 
        if (cv2.waitKey(1) & 0xff) == ord('q'):
            #status = cv2.imwrite('frame3.png',gray_image)
            cv2.destroyAllWindows()  

def main (args=None):
    rclpy.init(args=args)
    node = sub_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
