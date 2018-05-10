#include <ros/ros.h> 
#include <iostream> 
#include <sensor_msgs/image_encodings.h> 
#include <image_transport/image_transport.h>

#include "slamBase.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cv_bridge/cv_bridge.h> 
#include <boost/timer.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen !
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

//#include<map>
#include<vector>

//using namespace std;

using namespace dlib;



void run(void);
void chatterCallback(const sensor_msgs::Image & msg) ;
void chatterCallback1(const sensor_msgs::Image & msg) ;

FRAME frame_now;

cv::Mat RGB_image;

pcl::visualization::CloudViewer viewer( "viewer" );



int main( int argc, char** argv )
{
    
    ros::init(argc, argv, "listener"); 
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/kinect2/sd/image_color_rect", 2, chatterCallback);
    ros::Subscriber sub2 = n.subscribe("/kinect2/sd/image_depth_rect", 2, chatterCallback1);

    std::vector<cv::Point> feature_points;


    /**********************人脸特征点定位的程序初始化**************************/
    frontal_face_detector detector = get_frontal_face_detector();
    shape_predictor predictor;
    deserialize("/home/zhang/shape_predictor_68_face_landmarks.dat") >> predictor;
    image_window win;
    std::vector<rectangle> faces;
    /*********************************************************************/


    // 相机内参
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );

    ros::spinOnce();
    ros::Rate loop_rate(40);


    while (ros::ok())
    {
      ros::spinOnce();

      /*****************检测并显示人脸人脸特征点*****************************/
     /* cv_image<bgr_pixel> cimg(frame_now.rgb);
      faces = detector(cimg);
      feature_points.clear();
      if (faces.size() > 0)
      {
          full_object_detection shape = predictor(cimg, faces[0]);
          for (unsigned int i = 0; i < 68; ++i)
          {
              //cv::Point temp(shape.part(i).x(), shape.part(i).y());
              //feature_points.push_back(temp);
              cv::circle(frame_now.rgb, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
          }
        cv::imshow("demo", frame_now.rgb);
        cv::waitKey(1);
       }
       /******************************************************************/
      // 转换点云
      cout<<"converting image to clouds"<<endl;
      PointCloud::Ptr cloud = image2PointCloud( frame_now.rgb, frame_now.depth, camera );
      viewer.showCloud(cloud);
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
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
       frame_now.rgb = cv_ptr->image;

       //RGB_image=cv_ptr->image;

       //count_initial++;
}

void chatterCallback1(const sensor_msgs::Image & msg)
{
    //ROS_INFO("I heard: 222");
    cv_bridge::CvImagePtr cv_ptr;
     try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
        }
     catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        frame_now.depth = cv_ptr->image;
}












