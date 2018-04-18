#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QPushButton>
#include <QMainWindow>
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QThread>
#include"pose_detect/cameraposition.h"
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <string>
#include <iostream>
#include <fstream>
#include<uart.h>
#include <QTcpSocket>
#include<QDebug>
#include<QDateTimeEdit>
#include<Eigen>
#include<stdio.h>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include"force_sensor/ftusb.h"
#include "Face_recognition/facemain.h"
#include"UR5_kin/ur5_kin.h"
#include"PD/force_control.h"
#include"bp_nn/bp.h"
#include"robotState/UR5_state.h"
#include"mythread/mythread.h"
using namespace std;
using namespace cv;
using namespace Eigen;
QImage  Mat2QImage(cv::Mat cvImg);
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Ptr<ANN_MLP> ann;


public slots:
//void robotstate(double*q_actual);

private slots:
void on_Open_Video_clicked();
void readcamera_Frame();
//void on_Robot_star_clicked();
void on_robot_connect_clicked();
void connectok();
void on_Pos_control_clicked();
void robotstate_change();
void force();
void on_Joint_control_clicked();
//void force_conl();
void on_pushButton_clicked();

void on_openforce_clicked();

void on_closeforce_clicked();

void on_pushButton_2_clicked();

void on_pushButton_3_clicked();
//void DisplayMsg(double*);
//void robotstate1(double*q_actual);
void on_level_clicked();

signals:
  void tcp2(int);
private:
    Ui::MainWindow *ui;
    //QTimer    *camera_timer;
    double q_actual[24];
    bool ifopen;
    double *forcevalue1;
    QTimer *video_timer;
    QTimer *robot_timer;
    QTimer *forcecontrol_timer;
    QTimer *plot_timer;
    Mat frame;
    QImage image;
    VideoCapture capture;
   forceThread thread;
  QTcpSocket *tcpClient;
};

#endif // MAINWINDOW_H
