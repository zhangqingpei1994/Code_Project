

#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <iostream>
#include <QTcpSocket>
#include <stddef.h>
#include <stdlib.h>
#include <QAbstractSocket>
#include <unistd.h>
#include <arpa/inet.h>

#include<QFile>
#include<string.h>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>


#define pi 3.141592653





 class Robot_arm
 {
   public:

       Robot_arm()
       {
         tcpClient = new QTcpSocket();
       }

       QTcpSocket *tcpClient;
       Eigen::Affine3d handeye_data_Affin3d;
       double Rx,Ry,Rz;

       void movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v);
       void robotstate(double*q_actual);
       void track_head(Eigen::Vector4d &nose1, Eigen::Vector4d &nose2);

    private:
       QString commondqueue;
       Eigen::Vector4d target_nose1,target_nose2;
       Eigen::Vector4d target;

 };




#endif
