#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>

#include <pcl/pcl_config.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <map>

using namespace cv;

/**
 * Get the value of a specified key from a std::map.
 * @param m the map
 * @param key the key
 * @param defaultValue the default value used if the key is not found
 * @return the value
 */
template<class K, class V>
inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
{
    V v = defaultValue;
    typename std::map<K, V>::const_iterator i = m.find(key);
    if(i != m.end())
    {
        v = i->second;
    }
    return v;
}

/********** global variable begin ************/

int deviceId_;
libfreenect2::Freenect2* freenect2_;
libfreenect2::SyncMultiFrameListener* listener_;
libfreenect2::Freenect2Device* dev_;
libfreenect2::Registration * reg_;

/********** global variable end ************/

bool init()
{
    libfreenect2::PacketPipeline* pipeline;
    pipeline = new libfreenect2::CpuPacketPipeline();

    std::cout << ">>> opening default device ..." << std::endl;
    dev_ = freenect2_->openDefaultDevice(pipeline);
    pipeline = 0;

    if(dev_)
    {
        std::cout << ">>> Had got default device " << std::endl;
        libfreenect2::Freenect2Device::Config config;
        config.EnableBilateralFilter = true;
        config.EnableEdgeAwareFilter = true;
        config.MinDepth = 0.3f;
        config.MaxDepth = 12.0f;
        dev_->setConfiguration(config);

        dev_->setColorFrameListener(listener_);
        dev_->setIrAndDepthFrameListener(listener_);

        dev_->start();

        std::cout << ">>> CameraFreenect2: device serial: " << dev_->getSerialNumber() << std::endl;
        std::cout << ">>> CameraFreenect2: device firmware: " << dev_->getFirmwareVersion() << std::endl;

        //default registration params
        libfreenect2::Freenect2Device::IrCameraParams depthParams = dev_->getIrCameraParams();
        libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev_->getColorCameraParams();
        reg_ = new libfreenect2::Registration(depthParams, colorParams);

    }

    return true;
}

void captureImage()
{
    if(dev_ && listener_)
    {
        libfreenect2::FrameMap frames;

        if(!listener_->waitForNewFrame(frames,1000))
        {
            std::cout << "*** CameraFreenect2: Failed to get frames!" << std::endl;
        }
        else
        {
            libfreenect2::Frame* rgbFrame = 0;
            libfreenect2::Frame* irFrame = 0;
            libfreenect2::Frame* depthFrame = 0;

            irFrame = uValue(frames, libfreenect2::Frame::Ir, (libfreenect2::Frame*)0);
            depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
            rgbFrame=uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);

            cv::Mat ir,depth;
            float fx = 0, fy = 0, cx = 0, cy = 0;

            if(irFrame && depthFrame && rgbFrame)
            {
                cv::Mat irMat((int)irFrame->height, (int)irFrame->width, CV_32FC1, irFrame->data);
                //convert to gray scaled
                float maxIr_ = 0x7FFF;
                float minIr_ = 0x0;
                const float factor = 255.0f / float((maxIr_ - minIr_));
                ir = cv::Mat(irMat.rows, irMat.cols, CV_8UC1);
                for(int i=0; i<irMat.rows; ++i)
                {
                    for(int j=0; j<irMat.cols; ++j)
                    {
                        ir.at<unsigned char>(i, j) = (unsigned char)std::min(float(std::max(irMat.at<float>(i,j) - minIr_, 0.0f)) * factor, 255.0f);
                    }
                }

                cv::Mat((int)depthFrame->height, (int)depthFrame->width, CV_32FC1, depthFrame->data).convertTo(depth, CV_16U, 1);

                cv::Mat color = cv::Mat(rgbFrame->height, rgbFrame->width, CV_8UC4, rgbFrame->data);

                if(rgbFrame->format == libfreenect2::Frame::BGRX)
                {
                  cv::cvtColor(color, color, CV_BGRA2BGR);
                }
                else
                {
                  cv::cvtColor(color, color, CV_RGBA2BGR);
                }

                //void flip(InputArray src, OutputArray dst, int flipCode); 参数fipCode: 整数，水平发转；0垂直反转；负数，水平垂直均反转。
                cv::flip(color, color, 1);
                cv::flip(ir, ir, 1);
                cv::flip(depth, depth, 1);
                imshow("depth",depth);
                imshow("ir",ir);
                imshow("rgb",color);

                waitKey(10);
                libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
                fx = params.fx;
                fy = params.fy;
                cx = params.cx;
                cy = params.cy;
               
            }

            listener_->release(frames);
        }
    }
}

int main(int argc, char const *argv[])
{


    freenect2_ = new libfreenect2::Freenect2();
    //listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir);
    listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth | libfreenect2::Frame::Color);

    /********** initial begin **********/
    init();
    /********* initial end *************/

    while(1)
    captureImage();


    /********* destroy begin *************/
    delete freenect2_;
    delete listener_;
    /********* destroy end ***************/
    return 0;
}
