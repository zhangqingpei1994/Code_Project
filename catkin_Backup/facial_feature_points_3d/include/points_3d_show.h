#ifndef POINTS_3D_SHOW_H_
#define POINTS_3D_SHOW_H_

#include"grab_image.h"

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Points_3d_show
{
   public:
       Points_3d_show(Receiver & tmp):receive_data(tmp),viewer("3D Viewer")//viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
       {
           width=receive_data.color.cols;
           height=receive_data.color.rows;
           detected_face=false;
       }

   private:
       cv::Mat lookupX, lookupY;

       cv::Mat color,depth;

       size_t width, height;

       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

       Receiver & receive_data;

       pcl::visualization::CloudViewer viewer;

       //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

       std::vector<cv::Point> feature_points_vce;
       ofstream outfile;
       float Point1_x,Point1_y,Point1_z,Point2_x,Point2_y,Point2_z;


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
       bool detected_face;

      /****************************************************/

    public:
       void Inital(void);

       void init_cloud(void);

       void createLookup(void);

       void Init_Dlib(void);

       void cloudViewer();

       void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

       void show_clicked_3d_infor(void);

       void Detect_facial_points(void);

};


#endif
