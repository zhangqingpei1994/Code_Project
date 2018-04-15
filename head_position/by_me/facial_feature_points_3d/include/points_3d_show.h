#ifndef POINTS_3D_SHOW_H_
#define POINTS_3D_SHOW_H_

#include"grab_image.h"

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Points_3d_show
{
   public:
       Points_3d_show(Receiver & tmp):receive_data(tmp),viewer("viewer")
       {
           width=receive_data.color.cols;
           height=receive_data.color.rows;
       }

   private:
       cv::Mat lookupX, lookupY;

       cv::Mat color,depth;

       size_t width, height;

       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

       Receiver & receive_data;

       pcl::visualization::CloudViewer viewer;

       /**********在RGB图片上点击获得三维坐标时用的变量,   ***********/
       /***      只在show_clicked_3d_infor函数中用       *********/
       int  img_x, img_y ,img_x_last, img_y_last;
       std::ostringstream ossXYZ;   //用来在图片窗口显示的

       pcl::PointXYZRGBA pt_now;
       pcl::PointXYZRGBA pt_last;

       const cv::Scalar colorText = CV_RGB(255, 0, 0);
       const int font = cv::FONT_HERSHEY_SIMPLEX;
       /****************************************************/

       /***************人脸特征点定位************************/
       dlib::image_window win;
       std::vector<dlib::rectangle> faces;
       dlib::frontal_face_detector detector;
       dlib::shape_predictor predictor;

      /****************************************************/

    public:
       void init_cloud(void);

       void createLookup(void);

       void Init_Dlib(void);

       void cloudViewer();

       void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;

       void show_clicked_3d_infor(void);

       void Detect_facial_points(void);

};


#endif
