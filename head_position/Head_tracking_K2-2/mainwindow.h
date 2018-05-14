#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QAbstractSocket>
#include <QTcpSocket>
#include<QFile>
#include<QTimer>
#include<string.h>
#include<iostream>
#include<vector>

#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>


#include "grab_kinect2.h"
#include "points_3d_show.h"
#include "robot_arm.h"



namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void display_picture(cv::Mat image_dis,int type, int page);

private:
    Ui::MainWindow *ui;
    QTimer *timer;                    //实时刷新状态
    std::string filename_txt;
    ofstream outFile;
    /*******************************************************************/
    /*****            机械臂模块 变量                            *********/
    /*******************************************************************/
    bool if_ur5_connect;
    Robot_arm robot_ur5;
    double data[24]={0};

    /******************************************************************/



    /*******************************************************************/
    /*****            相机获取取照片模块                          *********/
    /*******************************************************************/
    bool initial_kinect2;                 //采集图像模块
    Grab_image grab_kinect2;
    Points_3d_show points_3d_show;

    cv::Mat rgb_undis,rgb_ori,depth_undis,depth_ori;

    int capture_count;                     //标定模块
    std::string pic_savePath;
    cv::Size  boardSize;                   //标定板的尺寸,角点，小的

    /*******************************************************************/

    bool Kinect2_tracking_show;
    bool track_head;
    int cyc_count;


    Eigen::Vector4d  nose1;
    Eigen::Vector4d  nose2;


 private slots:

    void connectok();
    void on_pushButton_Connect_clicked();

    void on_pushButton_sendPosition_clicked();
    void on_pushButton_stop_clicked();

    void on_pushButton_take_picture_clicked();

    void handleTimeout(void);

    void on_pushButton_START_Image_clicked();

    void on_pushButton_close_Image_clicked();
    void on_pushButton_Connect_2_clicked();
    void on_pushButton_stop_2_clicked();
    void on_pushButton_startKinect2_2_clicked();
    void on_pushButton_stopKinect2_2_clicked();
    void on_pushButton_get_weizi_clicked();

    void on_pushButton_trackHead_clicked();
    void on_pushButton_stoptrack_clicked();
};

#endif
