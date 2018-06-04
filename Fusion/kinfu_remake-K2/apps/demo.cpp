#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <io/capture.hpp>
#include<iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include"grab_kinect2.h"



using namespace kfusion;


/**************************************************************/
/**********                                          **********/
/**********            PCL play by qingpei           **********/
/**********                                          **********/
/**********                                          **********/
/**************************************************************/

typedef pcl::PointXYZ PointT;

struct callback_args
{
    // structure used to pass arguments to the callback function
    pcl::PointCloud<PointT>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

/**************************************************************/
/**************************************************************/


struct KinFuApp
{
    static void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* pthis)
    {
        KinFuApp& kinfu = *static_cast<KinFuApp*>(pthis);

        if(event.action != cv::viz::KeyboardEvent::KEY_DOWN)
            return;

        if(event.code == 't' || event.code == 'T')
            kinfu.take_cloud(*kinfu.kinfu_);

        if(event.code == 'i' || event.code == 'I')
            kinfu.iteractive_mode_ = !kinfu.iteractive_mode_;
    }


    //KinFuApp(OpenNISource& source) : exit_ (false),  iteractive_mode_(false), capture_ (source), pause_(false)
    KinFuApp(void) : exit_ (false),  iteractive_mode_(false), pause_(false)
      ,viewer(new pcl::visualization::PCLVisualizer("viewer")),cloud_pcl (new pcl::PointCloud<pcl::PointXYZ> ())   //PCL play by qingpei*******************************************
    {
        KinFuParams params = KinFuParams::default_params();  //返回一个KinFuParams p,里面主要是存了相机分辨率,体素大小,双边滤波参数,截断误差等.
        kinfu_ = KinFu::Ptr( new KinFu(params) );

        //capture_.setRegistration(true);

        cv::viz::WCube cube(cv::Vec3d::all(0), cv::Vec3d(params.volume_size), true, cv::viz::Color::apricot());
        viz.showWidget("cube", cube, params.volume_pose);
        viz.showWidget("coor", cv::viz::WCoordinateSystem(0.1));
        viz.registerKeyboardCallback(KeyboardCallback, this);

        viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);                                                        //PCL play by qingpei*************************************************
        viewer->addCoordinateSystem (1.0);                                                                       //PCL play by qingpei*************************************************

    }

    void show_depth(const cv::Mat& depth)
    {
        cv::Mat display;
        //cv::normalize(depth, display, 0, 255, cv::NORM_MINMAX, CV_8U);
        depth.convertTo(display, CV_8U, 255.0/4000);
        cv::imshow("Depth", display);
    }

    void show_raycasted(KinFu& kinfu)
    {
        const int mode = 3;
        if (iteractive_mode_)
            kinfu.renderImage(view_device_, viz.getViewerPose(), mode);
        else
        {
            kinfu.renderImage(view_device_, mode);   //这句话在执行

        }

        view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);  //view_host_是Mat类型
        view_device_.download(view_host_.ptr<void>(), view_host_.step);        //从GPU传数据到CPU
        cv::imshow("Scene", view_host_);
    }

    void take_cloud(KinFu& kinfu)
    {
        cuda::DeviceArray<Point> cloud = kinfu.tsdf().fetchCloud(cloud_buffer);
        cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);   //1行
        cloud.download1(cloud_host.ptr<Point>());
        viz.showWidget("cloud", cv::viz::WCloud(cloud_host));
        std::cout<<"take cloud"<<std::endl;

        /**************************************************************/
        /**********                                          **********/
        /**********            PCL play by qingpei           **********/
        /**********                                          **********/
        /**********                                          **********/
        /**************************************************************/
        pcl::PointXYZ p_temp;
        for(int i=0;i<cloud_host.cols;i++)
        {
            p_temp.x=cloud_host.at<Point>(0,i).x;
            p_temp.y=cloud_host.at<Point>(0,i).y;
            p_temp.z=cloud_host.at<Point>(0,i).z;
            cloud_pcl->points.push_back(p_temp);
        }
        cloud_pcl->width=cloud_pcl->points.size();
        cloud_pcl->height=1;
        cloud_pcl->is_dense=false;
        cloud_pcl->points.resize(cloud_pcl->width*cloud_pcl->height);
        viewer->addPointCloud(cloud_pcl, "test_pcl");

        pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
        viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
        /**************************************************************/
        /**************************************************************/


    }

    bool execute()
    {

        KinFu& kinfu = *kinfu_;              //kinfu_在构造函数中进行了初始化，就是创建了一个类，并以默认的KinFuParams作为参数创建的
        cv::Mat depth, image,depth_ori,image_ori;
        double time_ms = 0;
        bool has_image = true;

        grab_kinect2.Initial_KinectV2_driver();

        for (int i = 0; !exit_ && !viz.wasStopped(); ++i)
        {
            //bool has_frame = capture_.grab(depth, image);    //grab获得深度图像和彩色图像 --

            grab_kinect2.Grab_image_KinectV2(image,depth,image_ori,depth_ori);

            //cv::imshow("kinect2",image);
            //cv::waitKey(2);

            //这个函数最后goto到device_memory.cpp的upload函数,主要功能是从CPU拷贝二维数组到GPU上,存到了DeviceMemory2D的×data中 --
            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);   //depth_device_：DeviceArray2D<unsigned short> -- depth.step:一行的字节数

            //这个括号加上之后运行时间会变少
            {
                SampledScopeTime fps(time_ms); (void)fps;   // 为了消除编译器报的错误
                has_image = kinfu(depth_device_);
            }

            if (has_image)
                show_raycasted(kinfu);

            //show_depth(depth);
            //cv::imshow("Image", image);

            if (!iteractive_mode_)
                viz.setViewerPose(kinfu.getCameraPose());
                //vis.setWidgetPose ( "Camera", kinfu.getCameraPose() );


            int key = cv::waitKey(pause_ ? 0 : 3);
            switch(key)
            {
            case 't': case 'T' : take_cloud(kinfu); break;
            case 'i': case 'I' : iteractive_mode_ = !iteractive_mode_; break;
            case 27: exit_ = true; break;
            case 32: pause_ = !pause_; break;
            }

            viz.spinOnce(3, true);
            viewer->spinOnce(10);
        }

        return true;
    }

    bool pause_ /*= false*/;
    bool exit_, iteractive_mode_;
    //OpenNISource& capture_;
    KinFu::Ptr kinfu_;
    cv::viz::Viz3d viz;

    //cv::viz::Viz3d vis ;

    cv::Mat view_host_;
    cuda::Image view_device_;           //Image：cuda::DeviceArray2D<RGB>
    cuda::Depth depth_device_;          //Depth：cuda::DeviceArray2D<unsigned short>
    cuda::DeviceArray<Point> cloud_buffer;

   /********************************************************************/
   /***************      by qingpei    *********************************/

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl;
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
     struct callback_args cb_args;

     Grab_image grab_kinect2;

};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[])
{
    int device = 0;

    //这四句貌似没有本质上的作用
    cuda::setDevice (device);
    cuda::printShortCudaDeviceInfo (device);
    if(cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    //OpenNISource capture;                 //初始化了 depth_focal_length_VGA (0.f), baseline (0.f),shadow_value (0), no_sample_value (0), pixelSize (0.0), max_depth (0)
    //capture.open (0);                     //主要是对类OpenNISource中私有变量impl_的一些操作，里面有getParams函数得到相机的参数并赋给了OpenNISource里面的一些私有变量
    //KinFuApp app (capture);               //初始化了volume_和 icp_
    KinFuApp app;

    try { app.execute (); }
    catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
    catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }



    return 0;
}
