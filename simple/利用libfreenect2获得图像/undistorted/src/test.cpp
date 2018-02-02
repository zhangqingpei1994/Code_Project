#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;


class  libfreenect2_driver
{
public:

    libfreenect2_driver();

    int Initial_KinectV2_driver(void);

    void Grab_image_KinectV2(void);

    enum
    {
        Processor_cl,
        Processor_gl,
        Processor_cpu
    };
private:
    bool protonect_shutdown = false;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;
    libfreenect2::SyncMultiFrameListener listener;

};


libfreenect2_driver::libfreenect2_driver():listener(libfreenect2::Frame::Color |libfreenect2::Frame::Depth |libfreenect2::Frame::Ir)
{
;
}

int libfreenect2_driver::Initial_KinectV2_driver(void)
{
    if(freenect2.enumerateDevices() == 0)      //Number of devices, 0 if none
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "SERIAL: " << serial << std::endl;

    int depthProcessor = Processor_gl;
    switch(depthProcessor)
   {
    case Processor_cpu:
        if(!pipeline)
            pipeline = new libfreenect2::CpuPacketPipeline();
        break;

    case Processor_gl:
        #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
        {
            pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
        #else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
        #endif
        break;

    case Processor_cl:
        #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
        #else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
        #endif
        break;
   }


    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    //signal(SIGINT, sigint_handler);
    protonect_shutdown = false;


    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = true;
    config.EnableEdgeAwareFilter = true;
    config.MinDepth = 0.1;
    config.MaxDepth = 12.0;
    dev->setConfiguration(config);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;


    return 0;

}

void libfreenect2_driver::Grab_image_KinectV2(void)
{

    libfreenect2::FrameMap frames;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);

    Mat rgbmat, irmat,depthmat, depthmatUndistorted , rgbd, rgbd2;

    cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
    //cv::namedWindow("ir", WND_PROP_ASPECT_RATIO);
    //cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);
    //cv::namedWindow("undistorted", WND_PROP_ASPECT_RATIO);
   // cv::namedWindow("registered", WND_PROP_ASPECT_RATIO);

    if(!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

        cv::imshow("rgb", rgbmat);
        cv::imshow("depth", depthmat / 4500.0f);

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
       // cv::imshow("undistorted", depthmatUndistorted);
        cv::imshow("registered", rgbd);

        //cv::waitKey(0);

        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener.release(frames);
    }


}

int main(void)
{
   libfreenect2_driver temp;
   temp.Initial_KinectV2_driver();
   while(1)
   {
       temp.Grab_image_KinectV2();
   }

}
