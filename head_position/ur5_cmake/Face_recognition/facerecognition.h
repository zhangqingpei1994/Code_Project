#ifndef FACERECOGNITION_H
#define FACERECOGNITION_H
#include<opencv2/opencv.hpp>
#include<algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include<vector>
using namespace cv;
using namespace std;
double getSimilarity(const Mat A, const Mat B);
void learnCollectedFaces(Ptr<FaceRecognizer> &model,const vector<Mat> preprocessedFaces, const vector<int> faceLabels, const string facerecAlgorithm = "FaceRecognizer.Eigenfaces");
void showTrainingDebugData(const Ptr<FaceRecognizer> model, const int faceWidth, const int faceHeight);
Mat reconstructFace(const Ptr<FaceRecognizer> &model, const Mat &preprocessedFace);
#endif // FACERECOGNITION

