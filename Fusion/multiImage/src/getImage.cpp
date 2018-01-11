#include<ros/ros.h> //ros标准库头文件
#include<iostream> //C++标准输入输出库
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

 
void chatterCallback(const sensor_msgs::Image & msg)  
{  
  ROS_INFO("I heard: 111");  
  cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例

     try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
     catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
       cv::waitKey(15);
       cv::imshow("image",cv_ptr->image);

}  
void chatterCallback1(const sensor_msgs::Image & msg)  
{  
  ROS_INFO("I heard: 222");  
  cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例

     try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
     catch(cv_bridge::Exception& e)  //异常处理
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
       cv::waitKey(15);
       cv::imshow("depth",cv_ptr->image);

} 
  
int main(int argc, char **argv)  
{  
 
  ros::init(argc, argv, "listener");  
  
  ros::NodeHandle n;  
  
  ros::Subscriber sub1 = n.subscribe("/kinect2/qhd/image_color_rect", 1, chatterCallback);  
  ros::Subscriber sub2 = n.subscribe("/kinect2/qhd/image_depth_rect", 1, chatterCallback1);  
 
  ros::spin();  
  
  return 0;  
} 
