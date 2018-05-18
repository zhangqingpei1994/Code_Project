

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


#include <iostream>
#include <stdio.h>
#include <iostream>
#include <fstream>




#define pi 3.141592653


 class Robot_arm
 {
   public:

       Robot_arm()// :filename_position("/home/zhang/calibrate_data/TCP_position.txt")
       {
         tcpClient = new QTcpSocket();
//         outFile=ofstream (filename_position.c_str(), ios_base::out);    //按新建或覆盖方式写入
//         if (!outFile.is_open())
//          {
//                std::cout << "打开文件失败" << endl;
//          }
       }

       QTcpSocket *tcpClient;
       Eigen::Affine3d handeye_data_Affin3d;
       Eigen::Vector4d differ1,differ2;
       double Rx,Ry,Rz;


       void movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v);
       void robotstate(double*q_actual);
       void track_head(Eigen::Vector4d &nose1, Eigen::Vector4d &nose2, double speed);
       void get_position(double t_x,double t_y,double t_z);
       //void  Write_data_to_txt(double data1,double data2,double data3,double data4,double data5,double data6);

    private:
       QString commondqueue;
       Eigen::Vector4d target_nose1,target_nose2;
       Eigen::Vector4d target;
       double arm_X,arm_Y,arm_Z;

//       std::ofstream outFile;     //
//       std::string filename_position;



 };




#endif
