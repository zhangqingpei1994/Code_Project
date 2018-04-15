#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>


#include <grab_image.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>

// C++标准库
#include <fstream>
#include <vector>
#include <map>

#include <fstream>



using namespace dlib;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


pcl::visualization::CloudViewer viewer( "viewer" );

ofstream outfile;


PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth)
{
    PointCloud::Ptr cloud ( new PointCloud );
    //double fx=551.9 ,fy=552.9 ,cx=472.9 ,cy =277.6 , scale=1000.0 ;

    double fx=367.646,fy=367.646 ,cx=259.123 ,cy =205.083 , scale=1000.0 ;



    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            cv::Point temp(m,n);

            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / scale;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];
            // 把p加入到点云中
            cloud->points.push_back(p);

            /*std::vector<cv::Point>::iterator result = find( feature_point_2D.begin( ), feature_point_2D.end( ), temp );
            if ( result != feature_point_2D.end( ) )      //找到
            {
                outfile<<p.x<<" "<<p.y<<" "<<p.z<<endl;

            }*/

        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}




int main(void)
{
   cv::Mat rgbd,depth;

   grab_image temp;

   temp.Initial_KinectV2_driver();

   cv::Mat temp1,temp2;

   /**********************人脸特征点定位的程序初始化**************************/
   image_window win;
   std::vector<rectangle> faces;
   frontal_face_detector detector = get_frontal_face_detector();
   shape_predictor predictor;
   deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;

   /*********************************************************************/

   outfile.open("data.txt");
   if(!outfile) cout<<"error"<<endl;

   while(1)
   {

       temp.Grab_image_KinectV2(rgbd,depth,temp1,temp2);

       cv::imshow("rgbd",rgbd);

       cv::waitKey(2);

       /*****************检测并显示人脸人脸特征点*****************************/
      /* cv_image<bgr_pixel> cimg(rgbd);
       faces = detector(cimg);
       if (faces.size() > 0)
       {
           full_object_detection shape = predictor(cimg, faces[0]);
           for (unsigned int i = 0; i < 68; ++i)       //鼻尖是29和30
           {
               cv::circle(rgbd, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
           }
         cv::imshow("viewer",rgbd);
         cv::waitKey(1);

         //feature_point_2D.push_back(cv::Point(shape.part(29).x(), shape.part(29).y()));
         feature_point_2D.push_back(cv::Point(shape.part(30).x(), shape.part(30).y()));
       }

       /******************************************************************/


       // 转换点云
       std::cout<<"converting image to clouds"<<endl;
       PointCloud::Ptr cloud = image2PointCloud(rgbd, depth);
       viewer.showCloud(cloud);

   }

   outfile.close();

   return 0;

}
