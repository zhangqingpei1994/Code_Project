

#include <grab_image.h>


/************************************************************
 * ***     Function Name: Initial_KinectV2_driver  **********
 * ***     Return:                                 **********
 * ***     true: 初始化设备成功                       **********
 * ***     false: 失败                              *********
 * ***     说明:获得相机serial,使用相机自带内参初始化    *********
 *             Mat类型的相机内参矩阵                  *********
 * *********************************************************/
bool grab_image::Initial_KinectV2_driver(void)
{

     if(!initPipeline(method))
     {
         std::cout<<"initPipeline failed"<<std::endl;
         return false;
     }

     if(!initDevice())
     {
        return false;
     }

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

      return true;

}

/************************************************************
 * ***     Function Name: initPipeline               **********
 * ***     Return:                                 **********
 * ***     true: 初始化设备成功                       **********
 * ***     false: 失败                              *********
 * ***     说明:获得相机serial,使用相机自带内参初始化    *********
 *             Mat类型的相机内参矩阵                  *********
 * *********************************************************/
bool grab_image::initPipeline(const std::string &method )
{
   if(method == "default")
    {
     #ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
       pipeline = new libfreenect2::CudaPacketPipeline();
    #endif

    }
   else if(method == "cpu")
    {
      pipeline = new libfreenect2::CpuPacketPipeline();

    }
   else if(method == "cuda")
    {
     #ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
     pipeline = new libfreenect2::CudaPacketPipeline();
     #else
     std::cout<<"Cuda depth processing is not available!"<<std::endl;
      return false;
     #endif
    }
   else if(method == "opencl")
    {
      #ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      pipeline = new libfreenect2::OpenCLPacketPipeline(device);
      #else
      std::cout<<"OpenCL depth processing is not available!"<<std::endl;
      return false;
      #endif
    }
   else if(method == "opengl")
    {
      #ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      pipeline = new libfreenect2::OpenGLPacketPipeline();
      #else
     std::cout<<"OpenGL depth processing is not available!"<<std::endl;
      return false;
      #endif
    }

  else
   {

    std::cout<<"Unknown depth processing method: " << method<<std::endl;
    return false;

   }

    return true;
}

/************************************************************
 * ***     Function Name: initDevice               **********
 * ***     Return:                                 **********
 * ***     true: 初始化设备成功                       **********
 * ***     false: 失败                              *********
 * ***     说明:获得相机serial,使用相机自带内参初始化    *********
 * ***         Mat类型的相机内参矩阵                  *********
 * *********************************************************/

 bool grab_image::initDevice(void)
 {
     if(freenect2.enumerateDevices() == 0)
      {
        std::cout << "no device connected!" << std::endl;
        return false;
      }
     std::string serial = freenect2.getDefaultDeviceSerialNumber();
     std::cout << "SERIAL: " << serial << std::endl;

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
        return false;
      }

      dev->setColorFrameListener(&listener);
      dev->setIrAndDepthFrameListener(&listener);
      if(!dev->start())
      {
          std::cout<<"could not start device!"<<std::endl;

          return false;
      }

      colorParams = dev->getColorCameraParams();
      irParams = dev->getIrCameraParams();

      if(!dev->stop())
      {
        std::cout<<"could not stop device!"<<std::endl;

        return false;
      }

      cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
      distortionColor = cv::Mat::zeros(1, 5, CV_64F);

      cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
      cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
      cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
      cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
      cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
      cameraMatrixColor.at<double>(2, 2) = 1;

      cameraMatrixDepth = cv::Mat::eye(3, 3, CV_64F);
      distortionDepth = cv::Mat::zeros(1, 5, CV_64F);

      cameraMatrixDepth.at<double>(0, 0) = irParams.fx;
      cameraMatrixDepth.at<double>(1, 1) = irParams.fy;
      cameraMatrixDepth.at<double>(0, 2) = irParams.cx;
      cameraMatrixDepth.at<double>(1, 2) = irParams.cy;
      cameraMatrixDepth.at<double>(2, 2) = 1;

      distortionDepth.at<double>(0, 0) = irParams.k1;
      distortionDepth.at<double>(0, 1) = irParams.k2;
      distortionDepth.at<double>(0, 2) = irParams.p1;
      distortionDepth.at<double>(0, 3) = irParams.p2;
      distortionDepth.at<double>(0, 4) = irParams.k3;

      //std::cout<<cameraMatrixColor<<std::endl;
      //std::cout<<cameraMatrixDepth<<std::endl;
      //std::cout<<distortionDepth<<std::endl;

      return true;



 }

 /*void grab_image::initCalibration(const std::string &calib_path, const std::string &sensor)
 {
   std::string calibPath = calib_path + sensor + '/';

   struct stat fileStat;
   bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
   if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
   {
     OUT_WARN("using sensor defaults for color intrinsic parameters.");
   }

   if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
   {
     OUT_WARN("using sensor defaults for ir intrinsic parameters.");
   }

   if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
   {
     OUT_WARN("using defaults for depth shift.");
     depthShift = 0.0;
   }

   const int mapType = CV_16SC2;
   //注意这句话:单目相机的输出矩阵和输入矩阵一般相同
   cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
   cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
   cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);
 }

 bool grab_image::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
 {
   cv::FileStorage fs;
   if(fs.open(filename, cv::FileStorage::READ))
   {
     fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
     fs[K2_CALIB_DISTORTION] >> distortion;
     fs.release();
   }
   else
   {
     OUT_ERROR("can't open calibration file: " << filename);
     return false;
   }
   return true;
 }*/

void grab_image::Grab_image_KinectV2(cv::Mat& rgb_reg,cv::Mat& depth_undistor,cv::Mat& image_ori,cv::Mat& depth_ori)
{

     libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

     listener.waitForNewFrame(frames);
     libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
     libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
     libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

     registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

     cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depth_undistor);
     cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgb_reg);
     // cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

     cv::flip(rgb_reg, rgb_reg, 1);
     if(rgb->format == libfreenect2::Frame::BGRX)
      {
         cv::cvtColor(rgb_reg, rgb_reg, CV_BGRA2BGR);
      }
     else
      {
         cv::cvtColor(rgb_reg, rgb_reg, CV_RGBA2BGR);
      }

      depth_undistor.convertTo(depth_undistor, CV_16U, 1);
      cv::flip(depth_undistor,depth_undistor, 1);

      listener.release(frames);


}






