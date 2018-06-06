#ifndef TRACK_HEAD_H_
#define TRACK_HEAD_H_


#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Geometry>
#include <eigen3/Eigen/Geometry>



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Track_head
{
   public:
       Track_head()
       {
          have_detect_face=false;
       }

       Eigen::Isometry3d cam_to_armbase;
       Eigen::Isometry3d armbase_to_cam;

   private:

       cv::Mat lookupX, lookupY;

       size_t width, height;

       cv::Mat cameraRGBinfor,cameraDepthinfor;

       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;


       /**********在RGB图片上点击获得三维坐标时用的变量,   ***********/
       /***      只在show_clicked_3d_infor函数中用       *********/
       int  img_x, img_y ,img_x_last, img_y_last;

       std::ostringstream ossXYZ;   //用来在图片窗口显示的

       pcl::PointXYZRGB pt_now;

       pcl::PointXYZRGB pt_last;

       const cv::Scalar colorText = CV_RGB(255, 0, 0);

       const int font = cv::FONT_HERSHEY_SIMPLEX;

       /***************人脸特征点定位************************/
       dlib::image_window win;

       std::vector<dlib::rectangle> faces;

       dlib::frontal_face_detector detector;

       dlib::shape_predictor predictor;

       bool have_detect_face;

      /*******************追踪头部**************************/
      Eigen::Isometry3d T_end_to_base;    //机械臂末端相对于基座,只在一开始获取位姿的时候用

      Eigen::Isometry3d T_head_to_cam;    //头部坐标系相对于相机

      Eigen::Isometry3d T_illpoint_to_head;    //病灶点相对头部坐标系

      Eigen::Isometry3d T_illpoint_to_base;    //病灶点相对机械臂基座

      Eigen::Vector3d head_X_inCam,head_Y_inCam,head_Z_inCam,head_ori;

      std::vector<cv::Point> coordinate_point_2d;

      std::vector<Eigen::Vector3d> coordinate_point_3d;

      Eigen::Vector4d differ1,differ2;

      double Rx_target,Ry_target,Rz_target;


    public:
       void init_track_head(size_t width_in,size_t height_in,cv::Mat cameraRGBinfor_in,cv::Mat cameraDepthinfor_in);

       void init_cloud(void);

       void createLookup(void);

       void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const;

       void Init_Dlib(void);

       bool detect_face(cv::Mat& color_in,cv::Mat& depth_in);

       void show_clicked_3d_infor(cv::Mat &color, cv::Mat depth);

       void Detect_facial_points(cv::Mat & color);

       void get_Head_Coordinate(void);

       void track_head(double *target, int type);

       void get_end_to_base(double ur5_Px,double ur5_Py,double ur5_Pz,double ur5_Rx,double ur5_Ry,double ur5_Rz);

       void get_illpoint_to_head(double *differ111, double *differ222);

};


#endif

