
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

class  grab_image
{
  public:

    grab_image(): listener(libfreenect2::Frame::Color |libfreenect2::Frame::Depth |libfreenect2::Frame::Ir),
     undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4),method("opengl")
    {

    }

    enum
    {
        Processor_cl,
        Processor_gl,
        Processor_cpu
    };

    bool Initial_KinectV2_driver(void);

    bool initPipeline(const std::string &method);

    bool initDevice(void);

    void Grab_image_KinectV2(cv::Mat& image_reg,cv::Mat& depth_undis,cv::Mat& image_ori,cv::Mat& depth_ori);




  private:

    bool protonect_shutdown = false;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;

    libfreenect2::SyncMultiFrameListener listener;

    libfreenect2::FrameMap frames;

    libfreenect2::Frame undistorted, registered, depth2rgb;

    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;

    cv::Mat cameraMatrixColor, distortionColor,cameraMatrixDepth, distortionDepth;
    const std::string method;

};
