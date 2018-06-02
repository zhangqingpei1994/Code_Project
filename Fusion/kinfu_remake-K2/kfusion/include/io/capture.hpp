#pragma once

#include <kfusion/kinfu.hpp>
#include <opencv2/core/core.hpp>
#include <string>

namespace kfusion
{
    class KF_EXPORTS OpenNISource
    {
    public:
        typedef kfusion::PixelRGB RGB24;

        enum { PROP_OPENNI_REGISTRATION_ON  = 104 };

        OpenNISource();
        OpenNISource(int device);   //这个函数没啥用,里面实现是调用的 open(int device)
        OpenNISource(const std::string& oni_filename, bool repeat = false);  //这个函数可能是在使用 视频文件 进行重建的时候用

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
        struct Impl;                //里面是定义了硬件的东西
        cv::Ptr<Impl> impl_;
        void getParams ();          // 在open()和 setRegistration()中调用,其他地方没用

    };
}
