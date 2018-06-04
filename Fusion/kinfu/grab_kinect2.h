#ifndef GRAB_KINECT2_H_
#define GRAB_KINECT2_H_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>



class  Grab_image
{
  public:

    Grab_image(): listener(libfreenect2::Frame::Color |libfreenect2::Frame::Depth |libfreenect2::Frame::Ir),
     undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4),method("opengl")
    {

    }

    cv::Mat cameraMatrixColor, distortionColor,cameraMatrixDepth, distortionDepth;

    bool Initial_KinectV2_driver(void);

    bool initPipeline(const std::string &method);

    bool initDevice(void);

    void Grab_image_KinectV2(cv::Mat& image_reg, cv::Mat& depth_undis, cv::Mat& rgb_ori, cv::Mat& depth_ori);

    bool Close_KinectV2(void);

  private:

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;

    libfreenect2::SyncMultiFrameListener listener;
    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted, registered, depth2rgb;

    /*******************用来设置类型的参数********************/
    const std::string method="opengl";      //设置初始化pipeline的方法
    const std::string image_type="QHD";     //设置获得的图像的分辨率

};



#endif
