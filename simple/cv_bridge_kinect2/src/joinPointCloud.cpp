#include <ros/ros.h> 
#include <iostream> 
#include <sensor_msgs/image_encodings.h> 
#include <image_transport/image_transport.h>

#include "slamBase.h"
#include <opencv2/core/eigen.hpp>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cv_bridge/cv_bridge.h> 
#include <boost/timer.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;



void run(void);
void chatterCallback(const sensor_msgs::Image & msg) ;
void chatterCallback1(const sensor_msgs::Image & msg) ;

FRAME frame_now, frame_last;
char count_initial=0;
char flag=0;
pcl::visualization::CloudViewer viewer( "viewer" );
PointCloud::Ptr output (new PointCloud());


int main( int argc, char** argv )
{
    
    ros::init(argc, argv, "listener"); 
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/kinect2/sd/image_color_rect", 2, chatterCallback);
    ros::Subscriber sub2 = n.subscribe("/kinect2/sd/image_depth_rect", 2, chatterCallback1);

    ros::spinOnce();
    ros::Rate loop_rate(20);

    while (ros::ok())
    {

      // frame_now.rgb.copyTo(frame_last.rgb);
     //  frame_now.depth.copyTo(frame_last.depth);

       ros::spinOnce();
      if( flag  || count_initial>10)

      {
          run();
          flag=1;
      }

       loop_rate.sleep();
    }

    return 0;
}




void chatterCallback(const sensor_msgs::Image & msg)
{

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

       frame_now.rgb = cv_ptr->image;

       count_initial++;

}

void chatterCallback1(const sensor_msgs::Image & msg)
{
  //ROS_INFO("I heard: 222");
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

        frame_now.depth = cv_ptr->image;

}


void run(void)
{

  ParameterReader pd;
  // 提取特征并计算描述子
  /*cout<<"extracting features"<<endl;
  string detecter = pd.getData( "detector" );
  string descriptor = pd.getData( "descriptor" );


  computeKeyPointsAndDesp( frame_now, detecter, descriptor );
  computeKeyPointsAndDesp( frame_last, detecter, descriptor );*/

  // 相机内参
 CAMERA_INTRINSIC_PARAMETERS camera;
  camera.fx = atof( pd.getData( "camera.fx" ).c_str());
  camera.fy = atof( pd.getData( "camera.fy" ).c_str());
  camera.cx = atof( pd.getData( "camera.cx" ).c_str());
  camera.cy = atof( pd.getData( "camera.cy" ).c_str());
  camera.scale = atof( pd.getData( "camera.scale" ).c_str() );

   /*cout<<"solving pnp"<<endl;
   // 求解pnp
  RESULT_OF_PNP result = estimateMotion( frame_now, frame_last, camera );

  cout<<result.rvec<<endl<<result.tvec<<endl;

  // 处理result
  // 将旋转向量转化为旋转矩阵
   cv::Mat R;
  cv::Rodrigues( result.rvec, R );
  Eigen::Matrix3d r;
  cv::cv2eigen(R, r);

  // 将平移向量和旋转矩阵转换成变换矩阵
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd angle(r);
  cout<<"translation"<<endl;
  Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
  T = angle;
  T(0,3) = result.tvec.at<double>(0,0);
  T(1,3) = result.tvec.at<double>(0,1);
  T(2,3) = result.tvec.at<double>(0,2);  */

  // 转换点云
  cout<<"converting image to clouds"<<endl;

  PointCloud::Ptr cloud1 = image2PointCloud( frame_now.rgb, frame_now.depth, camera );

  //PointCloud::Ptr cloud2 = image2PointCloud( frame_last.rgb, frame_last.depth, camera );

  // 合并点云
  /*cout<<"combining clouds"<<endl;
  boost::timer timer1,timer2;
  pcl::transformPointCloud( *cloud1, *cloud1_temp, T.matrix() );
  *output += *cloud1_temp;*/

  //pcl::io::savePCDFile("data/result.pcd", *output);
  //cout<<"Final result saved."<<endl;

  viewer.showCloud( cloud1 );


}










