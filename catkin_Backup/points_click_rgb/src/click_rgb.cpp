 /**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

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

int mouseX;
int mouseY;
int mouseBtnType;

void onMouse(int event, int x, int y, int flags, void* ustc) 
{
    mouseX  = x;
    mouseY  = y;
    mouseBtnType = event;
}


class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  //用来做消息同步用的 具体参考某个博客 https://blog.csdn.net/Start_From_Scratch/article/details/52337689?locationNum=10&fps=1
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;   //多线程的时候要用,但是在这个cpp中只用了单线程
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  //std::vector<int> params;
  ros::Publisher leftBtnPointPub =nh.advertise<geometry_msgs::PointStamped>("/kinect2/click_point/left", 1);
  ros::Publisher rightBtnPointPub = nh.advertise<geometry_msgs::PointStamped>("/kinect2/click_point/right", 1);

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
   /* params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);*/


  }

  ~Receiver()
  {
  }

  void run(const Mode mode)    //mode: CLOUD
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

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

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
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

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)   //这个if不会执行的
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }


  void imageViewer()
  {

  }


  void cloudViewer()
  {
  cv::Mat color, depth;
  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
  double fps = 0;
  size_t frameCount = 0;
  std::ostringstream oss;
  std::ostringstream ossXYZ; // 新增一个string流
  const cv::Point pos(5, 15);
  const cv::Scalar colorText = CV_RGB(255, 0, 0);
  const double sizeText = 0.5;
  const int lineText = 1;
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  // 从全局变量获取当前鼠标坐标
  int img_x = mouseX;
  int img_y = mouseY;

  //自己添加的     =======================================================================
  int img_x_click,img_y_click,img_x_click_last,img_y_click_last;
  geometry_msgs::PointStamped ptMsg_temp;
  geometry_msgs::PointStamped ptMsg_temp_last;
  //=====================================================================================


  geometry_msgs::PointStamped ptMsg;
  ptMsg.header.frame_id = "kinect_link";

  lock.lock();
  color = this->color;
  depth = this->depth;
  updateCloud = false;
  lock.unlock();

  const std::string window_name = "color viewer";
  cv::namedWindow(window_name,2);      //2是用来设置窗口的属性
  // 注册鼠标回调函数, 第三个参数是C++11中的关键字, 若不支持C++11, 替换成NULL
  cv::setMouseCallback(window_name, onMouse, nullptr);
  createCloud(depth, color, cloud);

  for(; running && ros::ok();)
  {
    if(updateCloud)
    {
      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock();

      createCloud(depth, color, cloud);
      img_x = mouseX;
      img_y = mouseY;
      const pcl::PointXYZRGBA& pt = cloud->points[img_y * depth.cols + img_x];
      ptMsg.point.x = pt.x;
      ptMsg.point.y = pt.y;
      ptMsg.point.z = pt.z;

      // 根据鼠标左键压下或右键压下, 分别发布三维坐标到不同的话题上去,下面只是循环的执行的程序并不是回调函数的内容
      switch (mouseBtnType)
      {
      case cv::EVENT_LBUTTONUP:
          ptMsg.header.stamp = ros::Time::now();
          leftBtnPointPub.publish(ptMsg);
          ros::spinOnce();

         //自己加的===========================================================================
          ptMsg_temp_last=ptMsg_temp;
          img_x_click_last=img_x_click;
          img_y_click_last=img_y_click;
          img_x_click=img_x;
          img_y_click=img_y;
          ptMsg_temp=ptMsg;
         //==================================================================================
          break;

      case cv::EVENT_RBUTTONUP:
          ptMsg.header.stamp = ros::Time::now();
          rightBtnPointPub.publish(ptMsg);
          ros::spinOnce();
          break;
      default:
          break;
      }

      mouseBtnType = cv::EVENT_MOUSEMOVE;



      //这段程序用来显示帧率的,原来就有的
     /* ++frameCount;
      now = std::chrono::high_resolution_clock::now();
      double elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
      if(elapsed >= 1.0)
      {
        fps = frameCount / elapsed;
        oss.str("");
        oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
        start = now;
        frameCount = 0;
      }
      cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);*/

      //自己加的程序 =================================================================================================
      ossXYZ.str("");

      ossXYZ << "( " << ptMsg_temp.point.x << ", " << ptMsg_temp.point.y<< ", " << ptMsg_temp.point.z << " )";

      cv::putText(color, ossXYZ.str(), cv::Point(img_x_click+10, img_y_click), font, 1, colorText, 3, CV_AA);

      cv::circle(color, cv::Point(img_x_click, img_y_click), 5, cv::Scalar(0, 255, 0), -1);


      ossXYZ.str("");

      ossXYZ << "( " << ptMsg_temp_last.point.x << ", " << ptMsg_temp_last.point.y << ", " << ptMsg_temp_last.point.z << " )";

      cv::putText(color, ossXYZ.str(), cv::Point(img_x_click_last+10, img_y_click_last), font, 1, colorText, 3, CV_AA);

      cv::circle(color, cv::Point(img_x_click_last, img_y_click_last), 5, cv::Scalar(0, 255, 0), -1);
      //============================================================================================================

      cv::imshow(window_name, color);
      cv::waitKey(1);
    }

  }

  cv::destroyAllWindows();

  cv::waitKey(10);
}



  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }



  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;

        if(*itD == 0)
        {
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }



  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};




int main(int argc, char **argv) 
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "kinect2_viewer222", ros::init_options::AnonymousName);  //   /kinect2_viewer_1523418347917552750


  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;   //"kinect2"

  std::string topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;    //这种赋值不太理解，把HD改成QHD可以变分辨率，两个都改一下
  std::string topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;    //"hd"


  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  // topicColor: /kinect2/hd/image_color_rect
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;


  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);  //在终端输出信息，FG_CYAN和NO_COLOR是控制字体和背景颜色用的
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  OUT_INFO("starting receiver...");
  OUT_INFO("Please click mouse in color viewer...");
  receiver.run(mode);

  ros::shutdown();
  return 0;

}
