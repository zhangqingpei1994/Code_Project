#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QPushButton>
#include <QMainWindow>
#include <QWidget>
#include <QImage>
#include <QTimer>
#include"pose_detect/cameraposition.h"
#include <QInputDialog>
#include <QLineEdit>
#include <string>
#include<QDateTimeEdit>
#include "Face_recognition/facemain.h"
#include"UR5_kin/ur5_kin.h"
#include"bp_nn/bp.h"
#include"robotState/UR5_state.h"
#include"mythread/mythread.h"
#include"PD/force_control.h"

#include"kinect2_grab/grab_kinect2.h"
#include"head_track/track_head.h"


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
    CvANN_MLP ann;

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
    void automation_track(double *track_positon);


    void on_pushButton_startKinect2_clicked();
    void Kinect2_cycle(void);
    void on_pushButton_get_weizi_clicked();
    void on_pushButton_Stop_Trackhead_clicked();
    void on_pushButton_Start_Trackhead_clicked();

    void on_pushButton_stopKinect2_clicked();

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
    VideoCapture capture;
    Mat frame;
    QImage image;
    forceThread thread;
    QTcpSocket *tcpClient;


    QTimer *Kinect2_Timer;
    Grab_image grab_kinect2;
    cv::Mat rgb_corrected,depth_corrected,rgb_ori,depth_ori;
    bool track_head;
    bool connect_kinect2;
    bool connect_ur5;
    Track_head trackhead;
};

#endif
