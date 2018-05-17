#ifndef FACEPREPROCESS_H
#define FACEPREPROCESS_H
#include<opencv2/opencv.hpp>
#include<iostream>
#include<stdio.h>
using namespace cv;
using namespace std;
void detectLargestObject(const Mat &img, CascadeClassifier &cascade, Rect &largestObject, int scalewidth = 320);
Mat getPreprocessedFace(Mat &srcImg, int desiredFaceWidth, CascadeClassifier &faceCascade, CascadeClassifier &eyeCascade1, CascadeClassifier &eyeCascade2, bool doLeftAndRightSeparately, Rect *storeFaceRect, Point *storeLeftEye, Point *storeRightEye, Rect *searchedLeftEye, Rect *searchedRightEye);
void equalizeLeftAndRightHalves(Mat &faceImg);
#endif // FACEPREPROCESS

