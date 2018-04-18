#ifndef CAMERAPOSITION
#define CAMERAPOSITION
#include <ARToolKitPlus/TrackerSingleMarker.h>
#include <ARToolKitPlus/TrackerMultiMarker.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include <stdio.h>
#include<math.h>
#include <QImage>
//#define pi 3.1415926
using namespace cv;
//using namespace std;//快捷键是ctrl + shift + f
using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
extern double tool_Position[16];
class Positing
{
   public:
  int oldCalcuate(Mat src,Mat &dst,TrackerSingleMarker * tracker,double *outputMatrix);
  int tryAutoThreshold(cv::Mat src, TrackerSingleMarker* tracker,ARMarkerInfo* & markInfo,double*outputMatrix,int tryNum);
  void drawMarkerInfo(cv::Mat &image, ARToolKitPlus::ARMarkerInfo*  markInfo);
  private:

};
void init_traker(TrackerSingleMarker *tracker, int maker_width );
void get_visual_pose(Mat frame_src,Mat* frame_dst,TrackerSingleMarker *tracker);
void rotToEular_rad(double *rot, double *eular);
void rotToEular_angle(double *rot, double *eular);
QImage  Mat2QImage(cv::Mat cvImg);//图片格式转换，opencv转为QT
#endif // CAMERAPOSITION

