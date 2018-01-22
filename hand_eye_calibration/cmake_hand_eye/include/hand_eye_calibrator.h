/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#ifndef HAND_EYE_CALIBRATOR_H
#define HAND_EYE_CALIBRATOR_H   

#include <vector>
#include <iostream>
#include <termios.h>


#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


#include <eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>





class hand_eye_Calibrator
{

private:
    //std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<Eigen::Vector4d>> objectPoints_eigen;

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<Eigen::Vector2d>> imagePoints_eigen;

    std::vector<std::vector<Eigen::Vector2d>> Project_points;

    std::vector< Eigen::Affine3d> armEnd_Pose_eigen;
    Eigen::Affine3d end_to_checker;
    Eigen::Affine3d cam_to_base;


  public:
	
     hand_eye_Calibrator();

     // Open the chessboard images and extract corner points
     int addChessboardPoints(const std::vector<std::string>& filelist, cv::Size & boardSize);

     int readTransformPairsFromFile(std::string filename);

     void get_size_of_data(void);

     void  calculate(void);





};

struct ReprojectionError

{

  ReprojectionError(double observed_x, double observed_y): observed_x(observed_x), observed_y(observed_y)
  {

  }

  template <typename T>
  bool operator()(const T* const camera,  const T* const point,const T* const base_to_end, T* residuals) const
  {
     cv::Mat base_to_end_mat = (cv::Mat_<T>(4,4) << base_to_end[0], base_to_end[1],base_to_end[2],base_to_end[3],
                                                     base_to_end[4], base_to_end[5],base_to_end[6],base_to_end[7],
                                                     base_to_end[8], base_to_end[9],base_to_end[10],base_to_end[11],
                                                     base_to_end[12], base_to_end[13],base_to_end[14],base_to_end[15]);
     Eigen::Affine3d base_to_end_eigen;
     cv::cv2eigen(base_to_end_mat,base_to_end_eigen.matrix());


     Eigen::Matrix<double,4,1> point_eigen;
     Eigen::Vector4d project_temp_4x1;
     cv::Mat point_mat = (cv::Mat_<T>(3,1) << point[0], point[1],point[2]);
     point_eigen<<point_mat.at<double>(0,0),point_mat.at<double>(1,0),point_mat.at<double>(2,0),1.0;


     cv::Mat end_to_checker = (cv::Mat_<T>(3,1) << camera[0], camera[1],camera[2]);
     cv::Mat cam_to_base = (cv::Mat_<T>(3,1) << camera[6], camera[7],camera[8]);
     cv::Mat end_to_checker_t = (cv::Mat_<T>(3,1) << camera[3], camera[4],camera[5]);
     cv::Mat cam_to_base_t = (cv::Mat_<T>(3,1) << camera[9], camera[10],camera[11]);
     cv::Mat end_to_checker_R,cam_to_base_R;
     Rodrigues(end_to_checker,end_to_checker_R);
     Rodrigues(cam_to_base,cam_to_base_R);
     cv::Mat_<double> end_to_checker_T = cv::Mat_<double>::eye(4,4);
     cv::Mat_<double> cam_to_base_T = cv::Mat_<double>::eye(4,4);
     for(int j=0;j<3;j++)
       for(int jj=0;jj<3;jj++)
         {
           end_to_checker_T.at<double>(j,jj)=end_to_checker_R.at<double>(j,jj);
           cam_to_base_T.at<double>(j,jj)=cam_to_base_R.at<double>(j,jj);

         }
      end_to_checker_T.at<double>(0,3)=end_to_checker_t.at<double>(0,0);
      end_to_checker_T.at<double>(1,3)=end_to_checker_t.at<double>(1,0);
      end_to_checker_T.at<double>(2,3)=end_to_checker_t.at<double>(2,0);
      cam_to_base_T.at<double>(0,3)=cam_to_base_t.at<double>(0,0);
      cam_to_base_T.at<double>(1,3)=cam_to_base_t.at<double>(1,0);
      cam_to_base_T.at<double>(2,3)=cam_to_base_t.at<double>(2,0);

      Eigen::Affine3d end_to_checker_Eigen,cam_to_base_Eigen;
      cv::cv2eigen(end_to_checker_T,end_to_checker_Eigen.matrix());
      cv::cv2eigen(cam_to_base_T,cam_to_base_Eigen.matrix());


      project_temp_4x1=cam_to_base_Eigen*base_to_end_eigen*end_to_checker_Eigen*point_eigen;

      std::cout<<"aaa"<<std::endl;


//     // camera[0,1,2] are the angle-axis rotation.
//    T p[3];
//    ceres::AngleAxisRotatePoint(camera, point, p);

//    // camera[3,4,5] are the translation.
//    p[0] += camera[3];
//    p[1] += camera[4];
//    p[2] += camera[5];

//    // Compute the center of distortion. The sign change comes from
//    // the camera model that Noah Snavely's Bundler assumes, whereby
//    // the camera coordinate system has a negative z axis.
//    T xp = - p[0] / p[2];
//    T yp = - p[1] / p[2];

//    // Apply second and fourth order radial distortion.
//    const T& l1 = camera[7];
//    const T& l2 = camera[8];
//    T r2 = xp*xp + yp*yp;
//    T distortion = 1.0 + r2  * (l1 + l2  * r2);

//    // Compute final projected point position.
//    const T& focal = camera[6];
//    T predicted_x = focal * distortion * xp;
//    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = project_temp_4x1(1,0)-T(0.3);
    residuals[1] = project_temp_4x1(2,0)-T(0.3);

    return true;
  }


  static ceres::CostFunction* Create(const double observed_x,const double observed_y)
  {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 1, 1,1>( new ReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};

#endif


