#pragma once

#include <kfusion/kinfu.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>




namespace kfusion
{
    class KF_EXPORTS OpenNISource
    {
    public:
        typedef kfusion::PixelRGB RGB24;

        enum { PROP_OPENNI_REGISTRATION_ON  = 104 };

        OpenNISource();
        OpenNISource(int device);
        OpenNISource(const std::string& oni_filename, bool repeat = false);

        void open(int device);
        void open(const std::string& oni_filename, bool repeat = false);
        void release();

        ~OpenNISource();

        bool grab(cv::Mat &depth, cv::Mat &image);

        //parameters taken from camera/oni
        int shadow_value, no_sample_value;
        float depth_focal_length_VGA;
        float baseline;               // mm
        double pixelSize;             // mm
        unsigned short max_depth;     // mm

        bool setRegistration (bool value = false);
    private:
        struct Impl;
        cv::Ptr<Impl> impl_;
        void getParams ();

    };



    class KF_EXPORTS libfreenect2_driver
    {
    public:
        libfreenect2_driver();
        ~libfreenect2_driver();

        int Initial_KinectV2_driver(void);
        bool Grab_image_KinectV2(cv::Mat &Depth, cv::Mat &image);

        Intr Kinect2_Intr;

        enum
        {
            Processor_cl,
            Processor_gl,
            Processor_cpu
        };
    private:
        bool protonect_shutdown;
         int depthProcessor;
         libfreenect2::Freenect2Device::Config config;


        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = NULL;
        libfreenect2::PacketPipeline  *pipeline = NULL;
        libfreenect2::SyncMultiFrameListener listener;




    };
}
