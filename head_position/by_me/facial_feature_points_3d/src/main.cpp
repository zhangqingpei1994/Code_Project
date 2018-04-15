

#include"points_3d_show.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);  //   /kinect2_viewer_1523418347917552750
  if(!ros::ok())
  {
    return 0;
  }

  std::thread pointcloud_viewer;    //点云显示和人脸特征点定位线程

  std::string ns = K2_DEFAULT_NS;   //"kinect2"
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;    //这种赋值不太理解，把HD改成QHD可以变分辨率，两个都改一下
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  bool useExact = true;
  bool useCompressed = false;


  topicColor = "/" + ns + topicColor;       // topicColor: /kinect2/hd/image_color_rect
  topicDepth = "/" + ns + topicDepth;

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

  receiver.start();

  Points_3d_show points_3d_show(receiver);

  points_3d_show.init_cloud();

  points_3d_show.createLookup();

  points_3d_show.Init_Dlib();

  pointcloud_viewer = std::thread(&Points_3d_show::cloudViewer, &points_3d_show);

  while(ros::ok());

  receiver.stop();

  ros::shutdown();
  return 0;

}
