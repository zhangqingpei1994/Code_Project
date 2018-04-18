#ifndef FACEMAIN
#define FACEMAIN
#include<opencv2/opencv.hpp>
#include<objdetect/objdetect.hpp>
#include<iostream>
#include<stdio.h>
#include <vector>
#include <fstream>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include<algorithm>
#include<opencv2/features2d.hpp>
#include<opencv2/face/facerec.hpp>
#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include"facepreprocess.h"
#include"facerecognition.h"
using namespace std;
using namespace cv;
using namespace cv::face;
void facebegin();
int recognized();
//#include"dataprocess.h"
#endif // FACEMAIN

