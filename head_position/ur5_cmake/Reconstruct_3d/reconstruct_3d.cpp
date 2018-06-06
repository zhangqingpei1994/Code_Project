

#include"reconstruct_3d.h"


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZ current_point;

    event.getPoint(current_point.x, current_point.y, current_point.z);

    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(data->clicked_points_3d, 255, 0, 0);

    data->viewerPtr->removePointCloud("clicked_points");

    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");

    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


void KinFuApp::show_depth(const cv::Mat& depth)
{
     cv::Mat display;
     //cv::normalize(depth, display, 0, 255, cv::NORM_MINMAX, CV_8U);
     depth.convertTo(display, CV_8U, 255.0/4000);
     cv::imshow("Depth", display);
}

void KinFuApp::show_raycasted(kfusion::KinFu& kinfu)
{
     const int mode = 3;
     if (iteractive_mode_)
     {
         kinfu.renderImage(view_device_, kinfu.getCameraPose(), mode);
     }
     else
     {
         kinfu.renderImage(view_device_, mode);   //这句话在执行

     }
     view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);  //view_host_是Mat类型
     view_device_.download(view_host_.ptr<void>(), view_host_.step);        //从GPU传数据到CPU
     cv::imshow("Scene", view_host_);
}

void KinFuApp::take_cloud(void)
{
     if(initial_callback==false)
     {
         pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
         cb_args.clicked_points_3d = clicked_points_3d;
         cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
         viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);

         initial_callback=true;
     }
     kfusion::cuda::DeviceArray<kfusion::Point> cloud = kinfu_->tsdf().fetchCloud(cloud_buffer);
     cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);
     cloud.download1(cloud_host.ptr<kfusion::Point>());

     std::cout<<"take cloud"<<std::endl;

     pcl::PointXYZ p_temp;
     for(int i=0;i<cloud_host.cols;i++)
     {
        p_temp.x=cloud_host.at<kfusion::Point>(0,i).x;
        p_temp.y=cloud_host.at<kfusion::Point>(0,i).y;
        p_temp.z=cloud_host.at<kfusion::Point>(0,i).z;
        cloud_pcl->points.push_back(p_temp);
     }
     cloud_pcl->width=cloud_pcl->points.size();
     cloud_pcl->height=1;
     cloud_pcl->is_dense=false;
     cloud_pcl->points.resize(cloud_pcl->width*cloud_pcl->height);
     viewer->removePointCloud("test_pcl");
     viewer->addPointCloud(cloud_pcl, "test_pcl");
}

bool KinFuApp::execute(void)
{
    kfusion::KinFu& kinfu = *kinfu_;
    double time_ms = 0;
    bool has_image = true;

    cv::Mat image,depth,image_ori,depth_ori;

    grab_kinect2.Initial_KinectV2_driver();

    while(construction_flag && !exit_)
    {
        grab_kinect2.Grab_image_KinectV2(image,depth,image_ori,depth_ori);
        depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);   //depth_device_：DeviceArray2D<unsigned short> -- depth.step:一行的字节数
        //这个括号加上之后运行时间会变少
        {
           kfusion::SampledScopeTime fps(time_ms); (void)fps;   // 为了消除编译器报的错误,显示循环时间用的
           has_image = kinfu(depth_device_);
        }

        show_raycasted(kinfu);

        cv::waitKey(1);

        viewer->spinOnce(10);
    }
        return true;
}


