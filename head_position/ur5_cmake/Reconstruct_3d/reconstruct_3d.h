#ifndef Reconstruct_3D_H_
#define Reconstruct_3D_H_

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>

#include<iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include"../kinect2_grab/grab_kinect2.h"


struct callback_args
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


class KinFuApp
{
public:
    KinFuApp(void) : exit_ (false),  iteractive_mode_(false), pause_(false),
    viewer(new pcl::visualization::PCLVisualizer("viewer")),cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>())
    {
         kfusion::KinFuParams params = kfusion::KinFuParams::default_params();  //返回一个KinFuParams p,里面主要是存了相机分辨率,体素大小,双边滤波参数,截断误差等.
         kinfu_ = kfusion::KinFu::Ptr( new kfusion::KinFu(params) );

         construction_flag=false;
         initial_callback=false;

         viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
         viewer->addCoordinateSystem (1.0);
    }

    bool execute(void);

    void take_cloud(void);

    static void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* pthis);

    void show_depth(const cv::Mat& depth);

    void show_raycasted(kfusion::KinFu& kinfu);

    bool iteractive_mode_;

    kfusion::KinFu::Ptr kinfu_;

    bool construction_flag;

private:

    bool pause_ ;
    bool exit_;
    bool initial_callback;

    cv::Mat view_host_;
    kfusion::cuda::Image view_device_;
    kfusion::cuda::Depth depth_device_;
    kfusion::cuda::DeviceArray<kfusion::Point> cloud_buffer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    struct callback_args cb_args;

    Grab_image grab_kinect2;

};


#endif
