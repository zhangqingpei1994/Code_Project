#ifndef GRAB_IMAGE_H_
#define GRAB_IMAGE_H_

#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>



#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <geometry_msgs/PointStamped.h>


class Receiver
{
private:

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  const size_t queueSize;


  //用来做消息同步用的 具体参考某个博客 https://blog.csdn.net/Start_From_Scratch/article/details/52337689?locationNum=10&fps=1
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;           //多线程的时候要用,但是在这个cpp中只用了单线程
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

public:

  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), queueSize(5),
      nh("~"), spinner(0), it(nh)
     {
         cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
         cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
     }

  Receiver();


  bool updateImage, updateCloud;
  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  std::mutex lock;

  void start();

  void stop();

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;

};









#endif
