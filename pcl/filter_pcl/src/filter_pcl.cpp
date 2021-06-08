#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
// Ransac includes
#include "filter_pcl/processPointClouds.h"
#include "processPointClouds.cpp"

 
using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cityBlock( ProcessPointClouds<pcl::PointXYZ> pointProcessor, pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<float> boundaries)
{
    
    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.02, Eigen::Vector4f(boundaries[0], boundaries[2], boundaries[4], 1), Eigen::Vector4f(boundaries[1], boundaries[3], boundaries[5], 1));

    return inputCloud;
}


class PointcloudFilter : public rclcpp::Node
{
  public:
    std::string pcl_boundaries = "-1 1 -1 0.1 0 1"  ; 
    std::string ip_Topic_name = "" ;
    std::string op_Topic_name_cropped = "" ;
    
    // a b c d e f
    // c, d up & down
    // e f forward (e = 0 is camera f = x as x is max required distance)

    PointcloudFilter()
    : Node("pcl_filter_node")
    {
      this->declare_parameter<std::string>("pcl_boundaries", "-1 1 -1 0.1 0 1");
      this->declare_parameter<std::string>("ip_Topic_name", "/camera/depth/color/points");
      this->declare_parameter<std::string>("op_Topic_name", "/intel/cropped");

      this->get_parameter("pcl_boundaries", pcl_boundaries);
      this->get_parameter("ip_Topic_name", ip_Topic_name);
      this->get_parameter("op_Topic_name", op_Topic_name_cropped);
         
            
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "create sub");
      subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(ip_Topic_name, 5, std::bind(&PointcloudFilter::subscribe_message, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("intel/original", 10);
      publisher_cropped = this->create_publisher<sensor_msgs::msg::PointCloud2>(op_Topic_name_cropped, 10);

    }

  private:

    void subscribe_message(const sensor_msgs::msg::PointCloud2::SharedPtr message) const
    {
        std::string frame_id = (message->header.frame_id).c_str ();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "param:  %s  frame: %s", pcl_boundaries.c_str (), frame_id.c_str ());
        // Conversion to PCL
	      pcl::PCLPointCloud2 pcl_pc2;
	      pcl_conversions::toPCL(*message,pcl_pc2);
	      pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZ>);
	      pcl::fromPCLPointCloud2(pcl_pc2,*inputCloudI);
      

        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudII (new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < inputCloudI->points.size(); i++){
            float x = inputCloudI->points[i].x;
            float y = inputCloudI->points[i].y;
            float z = inputCloudI->points[i].z;
           
            // x is right and left , + is left
            // z is forward and back , + is forward (all data should be + , close obst should be 0:1m)
            // y is down and up , +ve is down, ground obst should be > 0.1
            //if (z < 1 &&  y < 0.1)    // Another way to filter without using processpointclouf file        
              inputCloudII->push_back (pcl::PointXYZ (x, y, z));
        }
        

	      pcl::PCLPointCloud2 pcl_pcl2;
	      pcl::toPCLPointCloud2(*inputCloudII, pcl_pcl2);
              
	      sensor_msgs::msg::PointCloud2::SharedPtr outmsg;
	      outmsg = message;
              outmsg->header.stamp =  now();
              outmsg->header.frame_id = frame_id;
        

	      pcl_conversions::fromPCL (pcl_pcl2, *outmsg);
        
        outmsg->header.stamp =  now();
        outmsg->header.frame_id = frame_id; 
        publisher_->publish(*outmsg);

              
        // Processing
        ProcessPointClouds<pcl::PointXYZ> pointProcessorI;
        std::string comma = "," ;
        vector<float> boundaries ;  
        std::stringstream ss(pcl_boundaries);
        float temp ;        
        while (ss >> temp)
            boundaries.push_back(temp);
      
        pcl::PointCloud<pcl::PointXYZ>::Ptr out = cityBlock(pointProcessorI, inputCloudII, boundaries);    

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size before: %d after: %d ", inputCloudI->points.size(), out->points.size());
            
	      pcl::PCLPointCloud2 pcl_pcl2_cropped;
	      pcl::toPCLPointCloud2(*out, pcl_pcl2_cropped);
	      sensor_msgs::msg::PointCloud2::SharedPtr outmsg_cropped;
	      outmsg_cropped = message;
              outmsg_cropped->header.stamp =  now();
              outmsg_cropped->header.frame_id = frame_id;
        
             
	      pcl_conversions::fromPCL (pcl_pcl2_cropped, *outmsg_cropped);	
              outmsg_cropped->header.stamp =  now();
              outmsg_cropped->header.frame_id = frame_id;
              publisher_cropped->publish(*outmsg_cropped);      


    }
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cropped;
        
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudFilter>());
  rclcpp::shutdown();
  return 0;
}
