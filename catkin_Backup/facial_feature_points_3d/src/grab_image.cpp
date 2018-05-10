
#include"grab_image.h"



void Receiver::start()
 {
   //不同分辨率的图片对应不同的相机内参信息   topicColor: /kinect2/hd/image_color_rect
   std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";    //rfind: Find last position of a character.
   std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

   image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");      //设置传输图片的格式
   subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
   subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
   subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
   subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

   if(useExact)
   {
     syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
     syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));  //bind函数参考: https://www.cnblogs.com/blueoverflow/p/4740093.html
   }
   else
   {
     syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
     syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
   }

   spinner.start();

   std::chrono::milliseconds duration(1);
   while(!updateImage || !updateCloud)              //一开始这俩都是false,当调用了一次callback后,这俩都变成了true,所以只要一调用callback这个循环就不执行了,就到下部分了
   {
     if(!ros::ok())
     {
       return;
     }
     std::this_thread::sleep_for(duration);
   }



}

void Receiver::stop()
{
  spinner.stop();

  if(useExact)
  {
    delete syncExact;
  }
  else
  {
    delete syncApproximate;
  }
  delete subImageColor;
  delete subImageDepth;
  delete subCameraInfoColor;
  delete subCameraInfoDepth;

}

void Receiver::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
  cv::Mat color, depth;

  readCameraInfo(cameraInfoColor, cameraMatrixColor);
  readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
  readImage(imageColor, color);
  readImage(imageDepth, depth);

  lock.lock();
  this->color = color;
  this->depth = depth;
  updateImage = true;
  updateCloud = true;
  lock.unlock();
}

void Receiver::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
{
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
  pCvImage->image.copyTo(image);
}


void Receiver::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
{
  double *itC = cameraMatrix.ptr<double>(0, 0);
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
}



