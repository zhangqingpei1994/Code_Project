
#include "slamBase.h"
#include <iostream>


#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

void int2str(const int &int_temp,string &string_temp)
{
        stringstream stream;
        stream<<int_temp;
        string_temp=stream.str();   //此处也可以用 stream>>string_temp
}

int main( int argc, char** argv )
{
    
    joinPointCloud joinPointCloud_example;
    FRAME frame_now;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(0, 0, -1, 0, -1, 0, 0);
    int i=0;

    string string_temp;
    string data_color="/home/zhang/image/000";
    string data_depth="/home/zhang/image/000";

    while (i<2)
    {


      int2str(i,string_temp) ;
      frame_now.rgb=cv::imread(data_color+ string_temp+"_color.jpg");
      frame_now.depth=cv::imread(data_depth+ string_temp+"_depth.png",-1);

      joinPointCloud_example.joinPointCloud_run(frame_now);

       i++;
    }

    viewer->removePointCloud("sample cloud");
    viewer->addPointCloud(output, "sample cloud");
   // pcl::io::savePCDFile("/home/zhang/image/data/result.pcd", *output);
    cout<<"Final result saved."<<endl;


    while(1)
    {

        viewer->spinOnce (1);      //调用内部渲染函数一次，重新渲染输出时间最大不超过time 单位是ms
    }


    return 0;
}
















/* FRAME test1,test2;
 test1.rgb=imread("/home/zhang/catkin_ws/src/kinect2_test/build-cv_bridge_kinect2-Desktop_Qt_5_6_1_GCC_64bit-Default/opencv-logo.png");
 test2=test1;
 test1.rgb=imread("/home/zhang/catkin_ws/src/kinect2_test/build-cv_bridge_kinect2-Desktop_Qt_5_6_1_GCC_64bit-Default/opencv.jpg");
 //Mat image=test2.rgb;
 imshow("test2",test2.rgb );
 imshow("test3", test1.rgb);
 waitKey();*/














