#ifndef POINTS_3D_SHOW_H_
#define POINTS_3D_SHOW_H_


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



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Points_3d_show
{
   public:
       Points_3d_show()
       {
          have_detect_face=false;
       }
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
       /****************************************************/

       /***************人脸特征点定位************************/
       dlib::image_window win;
       std::vector<dlib::rectangle> faces;
       dlib::frontal_face_detector detector;
       dlib::shape_predictor predictor;
       std::vector<cv::Point> nose_position;
       bool have_detect_face;

      /****************************************************/

    public:
       void init_Point_3d_show(size_t width_in,size_t height_in,cv::Mat cameraRGBinfor_in,cv::Mat cameraDepthinfor_in);

       void init_cloud(void);

       void createLookup(void);

       void Init_Dlib(void);

       void cloudViewer(cv::Mat& color_in,cv::Mat& depth_in);

       void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const;

       void show_clicked_3d_infor(cv::Mat &color, cv::Mat depth);

       void Detect_facial_points(cv::Mat & color);

       void get_Nose_Position(Eigen::Vector4d &nose1, Eigen::Vector4d &nose2);

};


#endif

