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

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "capture.h"
//#include "CameraCalibrator.h"

using namespace cv;



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




    int Write_data_to_txt(void);


private:
    Ui::MainWindow *ui;

    QTimer *timer;                    //实时刷新状态

    std::string filename_txt;


    /*******************************************************************/
    /*****            机械臂模块程序                            *********/
    /*****           包括定义的变量和函数                       ************/
    /*****           wrote by  qingpei                       ***********/
    /*******************************************************************/
    QTcpSocket *tcpClient;
    QString commondqueue;

    bool if_ur5_connect=false;
    double targetposition1[6];
    double speed;
    double acc;  

    std::vector<cv::Mat> TCP_Rotation_vector;
    std::vector<cv::Mat> TCP_t_vector;

    double data[24]={0};

    void robotstate(double*q_actual);

    void save_TCP_Position(void);

    void movelpos(double x,double y,double z,double Rx,double Ry,double Rz,double a,double v);




    /*******************************************************************/
    /*****            相机拍摄模块程序                            *********/
    /*****           包括定义的变量和函数                       ************/
    /*****           wrote by  qingpei                       ***********/
    /*******************************************************************/
    bool Camera_connected;
    OpenNISource capture;
    cv::Mat image;
    cv::Mat depth;                        //相机采集到的深度图和彩色图
    int capture_count;
    std::string pic_savePath;
    cv::Size  boardSize;                  //标定板的尺寸,角点，小的

    void display_picture(cv::Mat image_dis, int type);

    /*******************************************************************/



 private slots:
    void connectok();
    void on_pushButton_Connect_clicked();

    void on_pushButton_sendPosition_clicked();
    void on_pushButton_stop_clicked();


    void on_pushButton_take_picture_clicked();

    void handleTimeout(void);
    void on_pushButton_cali_clicked();


    void on_pushButton_START_Image_clicked();
};

#endif
