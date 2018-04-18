#ifndef FACERECOGNITION
#define FACERECOGNITION
#include<opencv2/opencv.hpp>
#include<opencv2/face.hpp>
#include<algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include<vector>
#include<opencv2/features2d.hpp>
#include "opencv2/core.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include<opencv2/face/facerec.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio/videoio_c.h"
#include "opencv2/highgui/highgui_c.h"
using namespace cv;
using namespace std;
using namespace cv::face;
double getSimilarity(const Mat A, const Mat B);
void learnCollectedFaces(const Ptr<BasicFaceRecognizer> &model,const vector<Mat> preprocessedFaces, const vector<int> faceLabels, const string facerecAlgorithm = "FaceRecognizer.Eigenfaces");
void showTrainingDebugData( Ptr<BasicFaceRecognizer> model, const int faceWidth, const int faceHeight);
Mat reconstructFace( const Ptr<BasicFaceRecognizer> &model, const Mat &preprocessedFace);
#endif // FACERECOGNITION

