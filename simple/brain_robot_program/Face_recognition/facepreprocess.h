#ifndef FACEPREPROCESS
#define FACEPREPROCESS
#include<opencv2/opencv.hpp>
#include<iostream>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include<opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include "opencv2/face.hpp"



using namespace cv;
using namespace std;
void detectLargestObject(const Mat &img, CascadeClassifier &cascade, Rect &largestObject, int scalewidth = 320);
Mat getPreprocessedFace(Mat &srcImg, int desiredFaceWidth, CascadeClassifier &faceCascade, CascadeClassifier &eyeCascade1, CascadeClassifier &eyeCascade2, bool doLeftAndRightSeparately, Rect *storeFaceRect, Point *storeLeftEye, Point *storeRightEye, Rect *searchedLeftEye, Rect *searchedRightEye);
void equalizeLeftAndRightHalves(Mat &faceImg);
#endif // FACEPREPROCESS

