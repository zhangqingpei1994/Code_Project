#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <kfusion/kinfu.hpp>
#include <io/capture.hpp>
#include<iostream>

using namespace kfusion;

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

    KinFuApp() : exit_ (false),  iteractive_mode_(false),  pause_(false)
    //KinFuApp(OpenNISource& source) : exit_ (false),  iteractive_mode_(false), capture_ (source), pause_(false)
    {

        kinect2_driver.Initial_KinectV2_driver();

        KinFuParams params = KinFuParams::default_params();  //返回一个KinFuParams p
        params.intr=kinect2_driver.Kinect2_Intr;

        kinfu_ = KinFu::Ptr( new KinFu(params) );

        //capture_.setRegistration(true);

        cv::viz::WCube cube(cv::Vec3d::all(0), cv::Vec3d(params.volume_size), true, cv::viz::Color::apricot());
        viz.showWidget("cube", cube, params.volume_pose);
        viz.showWidget("coor", cv::viz::WCoordinateSystem(0.1));
        viz.registerKeyboardCallback(KeyboardCallback, this);


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
            kinfu.renderImage(view_device_, mode);

        view_host_.create(view_device_.rows(), view_device_.cols(), CV_8UC4);
        view_device_.download(view_host_.ptr<void>(), view_host_.step);
        cv::imshow("Scene", view_host_);
    }

    void take_cloud(KinFu& kinfu)
    {
        cuda::DeviceArray<Point> cloud = kinfu.tsdf().fetchCloud(cloud_buffer);
        cv::Mat cloud_host(1, (int)cloud.size(), CV_32FC4);
        cloud.download(cloud_host.ptr<Point>());
        viz.showWidget("cloud", cv::viz::WCloud(cloud_host));
        //viz.showWidget("cloud", cv::viz::WPaintedCloud(cloud_host));
    }

    bool execute()
    {

        KinFu& kinfu = *kinfu_;              //kinfu_在构造函数中进行了初始化，就是创建了一个类，并以默认的KinFuParams作为参数创建的
        cv::Mat depth, image;
        double time_ms = 0;
        bool has_image = false;

        for (int i = 0; !exit_ && !viz.wasStopped(); ++i)

        {
            double start_time_total = (double)cv::getTickCount();
            //capture_是 OpenNISource capture;capture.open (0);定义后传进来的参量 --
            //bool has_frame = capture_.grab(depth, image);    //grab获得深度图像和彩色图像 --
            bool has_frame=kinect2_driver.Grab_image_KinectV2(depth,image);
            if (!has_frame)
                return std::cout << "Can't grab" << std::endl, false;

            cv::imshow("depth_kinect2",depth);
            cv::imshow("image_kinect2",image);
            cv::waitKey(3);


            //这个函数最后goto到device_memory.cpp的upload函数,主要功能是从CPU拷贝二维数组到GPU上,存到了DeviceMemory2D的×data中 --
            //double start_time = (double)cv::getTickCount();

            depth_device_.upload(depth.data, depth.step, depth.rows, depth.cols);   //depth_device_：DeviceArray2D<unsigned short> -- 512*424

            //double time=((double)cv::getTickCount() - start_time)*1000.0/cv::getTickFrequency();
            //std::cout<<"the time of data from CPU to GPU is: "<<time<<"ms"<<std::endl;
           // std::cout<< depth.step<<std::endl;

            //这个括号加上之后运行时间会变少
            {
                SampledScopeTime fps(time_ms); (void)fps;   // 为了消除编译器报的错误
                has_image = kinfu(depth_device_);
            }

            if (has_image)
                show_raycasted(kinfu);

            show_depth(depth);
            //cv::imshow("Image", image);

            if (!iteractive_mode_)
                viz.setViewerPose(kinfu.getCameraPose());

            int key = cv::waitKey(pause_ ? 0 : 3);

            switch(key)
            {
            case 't': case 'T' : take_cloud(kinfu); break;
            case 'i': case 'I' : iteractive_mode_ = !iteractive_mode_; break;
            case 27: exit_ = true; break;
            case 32: pause_ = !pause_; break;
            }

            //exit_ = exit_ || i > 100;
            viz.spinOnce(3, true);
            double time_total=((double)cv::getTickCount() - start_time_total)*1000.0/cv::getTickFrequency();
            std::cout<<"the total_time of every circle is : "<<time_total<<"ms"<<std::endl;
        }

        return true;
    }

    bool pause_ /*= false*/;
    bool exit_, iteractive_mode_;
    //OpenNISource& capture_;
    KinFu::Ptr kinfu_;
    cv::viz::Viz3d viz;

    cv::Mat view_host_;
    cuda::Image view_device_;   //Image：DeviceArray2D<RGB>
    cuda::Depth depth_device_;  //Depth：DeviceArray2D<unsigned short>
    cuda::DeviceArray<Point> cloud_buffer;

    libfreenect2_driver kinect2_driver;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char* argv[])
{
    int device = 0;
    int zhang_qt=0;

    //这四句貌似没有本质上的作用
    cuda::setDevice (device);
    cuda::printShortCudaDeviceInfo (device);
    if(cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    //OpenNISource 外面能用的是相机的参数
    OpenNISource capture;                 //初始化了 depth_focal_length_VGA (0.f), baseline (0.f),shadow_value (0), no_sample_value (0), pixelSize (0.0), max_depth (0)
    //capture.open (0);                     //主要是对类OpenNISource中私有变量impl_的一些操作，里面有getParams函数得到相机的参数并赋给了OpenNISource里面的一些私有变量

    KinFuApp app ;               //初始化了volume_和 icp_

//     executing
    try { app.execute (); }
    catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
    catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

    zhang_qt++;

    return 0;
}
