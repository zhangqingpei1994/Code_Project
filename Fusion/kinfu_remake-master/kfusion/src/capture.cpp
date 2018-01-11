#pragma warning (disable :4996)
#undef _CRT_SECURE_NO_DEPRECATE
#include "XnCppWrapper.h"
#include <io/capture.hpp>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <map>

//#include <pcl/pcl_config.h>

using namespace std;
using namespace xn;
using namespace cv;



#define REPORT_ERROR(msg) kfusion::cuda::error ((msg), __FILE__, __LINE__)

struct kfusion::OpenNISource::Impl
{
    Context context;
    ScriptNode scriptNode;
    DepthGenerator depth;
    ImageGenerator image;
    ProductionNode node;
    DepthMetaData depthMD;
    ImageMetaData imageMD;
    XnChar strError[1024];
    Player player_;

    bool has_depth;
    bool has_image;
};

kfusion::OpenNISource::OpenNISource() : depth_focal_length_VGA (0.f), baseline (0.f),
    shadow_value (0), no_sample_value (0), pixelSize (0.0), max_depth (0) {}

kfusion::OpenNISource::OpenNISource(int device) {open (device); }
kfusion::OpenNISource::OpenNISource(const string& filename, bool repeat /*= false*/) {open (filename, repeat); }
kfusion::OpenNISource::~OpenNISource() { release (); }

void kfusion::OpenNISource::open (int device)
{
    impl_ = cv::Ptr<Impl>( new Impl () );

    XnMapOutputMode mode;         //定义图片大小和帧率的一个结构体
    mode.nXRes = XN_VGA_X_RES;    //640
    mode.nYRes = XN_VGA_Y_RES;    //480
    mode.nFPS = 30;

    XnStatus rc;                 //unsigned int
    rc = impl_->context.Init ();
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    xn::NodeInfoList devicesList;
    rc = impl_->context.EnumerateProductionTrees ( XN_NODE_TYPE_DEVICE, NULL, devicesList, 0 );
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    xn::NodeInfoList::Iterator it = devicesList.Begin ();
    for (int i = 0; i < device; ++i)
        it++;

    NodeInfo node = *it;
    rc = impl_->context.CreateProductionTree ( node, impl_->node );
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    XnLicense license;
    const char* vendor = "PrimeSense";
    const char* key = "0KOIk2JeIBYClPWVnMoRKn5cdY4=";
    sprintf (license.strKey, key);
    sprintf (license.strVendor, vendor);

    rc = impl_->context.AddLicense (license);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "licence failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    rc = impl_->depth.Create (impl_->context);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Depth generator  failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }
    //rc = impl_->depth.SetIntProperty("HoleFilter", 1);
    rc = impl_->depth.SetMapOutputMode (mode);
    impl_->has_depth = true;
    rc = impl_->image.Create (impl_->context);
    if (rc != XN_STATUS_OK)
    {
        printf ("Image generator creation failed: %s\n", xnGetStatusString (rc));
        impl_->has_image = false;
    }
    else
    {
        impl_->has_image = true;
        rc = impl_->image.SetMapOutputMode (mode);
    }


    getParams ();

    rc = impl_->context.StartGeneratingAll ();
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Start failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }
}

void kfusion::OpenNISource::open(const std::string& filename, bool repeat /*= false*/)
{
    impl_ = cv::Ptr<Impl> ( new Impl () );

    XnStatus rc;

    rc = impl_->context.Init ();
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Init failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    //rc = impl_->context.OpenFileRecording (filename.c_str (), impl_->node);
    rc = impl_->context.OpenFileRecording (filename.c_str (), impl_->player_);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "Open failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }
    
    impl_->player_.SetRepeat(repeat);

    rc = impl_->context.FindExistingNode (XN_NODE_TYPE_DEPTH, impl_->depth);
    impl_->has_depth = (rc == XN_STATUS_OK);

    rc = impl_->context.FindExistingNode (XN_NODE_TYPE_IMAGE, impl_->image);
    impl_->has_image = (rc == XN_STATUS_OK);

    if (!impl_->has_depth)
        REPORT_ERROR ("No depth nodes. Check your configuration");

    if (impl_->has_depth)
        impl_->depth.GetMetaData (impl_->depthMD);

    if (impl_->has_image)
        impl_->image.GetMetaData (impl_->imageMD);

    // RGB is the only image format supported.
    if (impl_->imageMD.PixelFormat () != XN_PIXEL_FORMAT_RGB24)
        REPORT_ERROR ("Image format must be RGB24\n");

    getParams ();
}

void kfusion::OpenNISource::release ()
{
    if (impl_)
    {
        impl_->context.StopGeneratingAll ();
        impl_->context.Release ();
    }

    impl_.release();;
    depth_focal_length_VGA = 0;
    baseline = 0.f;
    shadow_value = 0;
    no_sample_value = 0;
    pixelSize = 0.0;
}

bool kfusion::OpenNISource::grab(cv::Mat& depth, cv::Mat& image)
{
    XnStatus rc = XN_STATUS_OK;

    rc = impl_->context.WaitAndUpdateAll ();
    if (rc != XN_STATUS_OK)
        return printf ("Read failed: %s\n", xnGetStatusString (rc)), false;

    if (impl_->has_depth)
    {
        impl_->depth.GetMetaData (impl_->depthMD);
        const XnDepthPixel* pDepth = impl_->depthMD.Data ();
        int x = impl_->depthMD.FullXRes ();
        int y = impl_->depthMD.FullYRes ();
        cv::Mat(y, x, CV_16U, (void*)pDepth).copyTo(depth);
    }
    else
    {
        depth.release();
        printf ("no depth\n");
    }

    if (impl_->has_image)
    {
        impl_->image.GetMetaData (impl_->imageMD);
        const XnRGB24Pixel* pImage = impl_->imageMD.RGB24Data ();
        int x = impl_->imageMD.FullXRes ();
        int y = impl_->imageMD.FullYRes ();
        image.create(y, x, CV_8UC3);

        cv::Vec3b *dptr = image.ptr<cv::Vec3b>();
        for(size_t i = 0; i < image.total(); ++i)
            dptr[i] = cv::Vec3b(pImage[i].nBlue, pImage[i].nGreen, pImage[i].nRed);
    }
    else
    {
        image.release();
        printf ("no image\n");
    }

    return impl_->has_image || impl_->has_depth;
}

void kfusion::OpenNISource::getParams ()
{
    XnStatus rc = XN_STATUS_OK;

    max_depth = impl_->depth.GetDeviceMaxDepth ();

    rc = impl_->depth.GetRealProperty ( "ZPPS", pixelSize );  // in mm
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "ZPPS failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    XnUInt64 depth_focal_length_SXGA_mm;   //in mm
    rc = impl_->depth.GetIntProperty ("ZPD", depth_focal_length_SXGA_mm);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "ZPD failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    XnDouble baseline_local;
    rc = impl_->depth.GetRealProperty ("LDDIS", baseline_local);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "ZPD failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }

    XnUInt64 shadow_value_local;
    rc = impl_->depth.GetIntProperty ("ShadowValue", shadow_value_local);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "ShadowValue failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }
    shadow_value = (int)shadow_value_local;

    XnUInt64 no_sample_value_local;
    rc = impl_->depth.GetIntProperty ("NoSampleValue", no_sample_value_local);
    if (rc != XN_STATUS_OK)
    {
        sprintf (impl_->strError, "NoSampleValue failed: %s\n", xnGetStatusString (rc));
        REPORT_ERROR (impl_->strError);
    }
    no_sample_value = (int)no_sample_value_local;


    // baseline from cm -> mm
    baseline = (float)(baseline_local * 10);

    //focal length from mm -> pixels (valid for 1280x1024)
    float depth_focal_length_SXGA = static_cast<float>(depth_focal_length_SXGA_mm / pixelSize);
    depth_focal_length_VGA = depth_focal_length_SXGA / 2;
}

bool kfusion::OpenNISource::setRegistration (bool value)
{
    XnStatus rc = XN_STATUS_OK;

    if (value)
    {
        if (!impl_->has_image)
            return false;

        if (impl_->depth.GetAlternativeViewPointCap ().IsViewPointAs (impl_->image) )
            return true;

        if (!impl_->depth.GetAlternativeViewPointCap ().IsViewPointSupported (impl_->image) )
        {
            printf ("SetRegistration failed: Unsupported viewpoint.\n");
            return false;
        }

        rc = impl_->depth.GetAlternativeViewPointCap ().SetViewPoint (impl_->image);
        if (rc != XN_STATUS_OK)
            printf ("SetRegistration failed: %s\n", xnGetStatusString (rc));

    }
    else   // "off"
    {
        rc = impl_->depth.GetAlternativeViewPointCap ().ResetViewPoint ();
        if (rc != XN_STATUS_OK)
            printf ("SetRegistration failed: %s\n", xnGetStatusString (rc));
    }

    getParams ();
    return rc == XN_STATUS_OK;
}



/*************************************************************************/
/*************************************************************************/
/********************************Kinect V2 Driver*************************/
/*************************************************************************/
/*************************************************************************/





kfusion::libfreenect2_driver::libfreenect2_driver():listener(libfreenect2::Frame::Color |libfreenect2::Frame::Depth |libfreenect2::Frame::Ir)
{
   depthProcessor= Processor_gl;
   protonect_shutdown = false;
   config.EnableBilateralFilter = true;
   config.EnableEdgeAwareFilter = true;
   config.MinDepth = 0.1;
   config.MaxDepth = 12.0;

}

kfusion::libfreenect2_driver::~libfreenect2_driver()
{


}

int kfusion::libfreenect2_driver::Initial_KinectV2_driver(void)
{

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "SERIAL: " << serial << std::endl;
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

    dev->setConfiguration(config);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    Kinect2_Intr.fx=dev->getIrCameraParams().fx;
    Kinect2_Intr.fy=dev->getIrCameraParams().fy;
    Kinect2_Intr.cx=dev->getIrCameraParams().cx;
    Kinect2_Intr.cy=dev->getIrCameraParams().cy;

    //cout<<Kinect2_Intr.fx<<"  "<<Kinect2_Intr.fy<<" "<<Kinect2_Intr.cx<<" "<<Kinect2_Intr.cy<<endl;


    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    return 0;

}

bool kfusion::libfreenect2_driver::Grab_image_KinectV2(Mat & Depth,Mat &image)
{

    libfreenect2::FrameMap frames;
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
    Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;

    //cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
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
        //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        //cv::Mat(depth->height, depth->width, CV_16U, depth->data).copyTo(depthmat);


        //registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        registration->apply(rgb, depth, &undistorted, &registered);

        //cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
        //cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
        //cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

        Mat depth_tmp;

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(Depth);


        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(image);

       // cv::imshow("undistorted", Depth / 4500.0f);
        // cv::imshow("undistorted", depthmatUndistorted);
        //cv::imshow("registered", image);
        //cv::waitKey(3);

        listener.release(frames);

        return true;
    }
    else
        return false;


}




