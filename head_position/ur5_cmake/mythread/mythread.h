#ifndef MYTHREAD_H
#define MYTHREAD_H
#include<QThread>
#include<iostream>
#include<opencv2/opencv.hpp>
#include <fstream>
#include <iterator>
#include <vector>
#include<opencv2/ml/ml.hpp>
#include <ml.h>
#include <string.h>
#include <cvaux.h>
#include <QTcpSocket>
#include<QDebug>
#include<Eigen>
#include<arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>
#include<QMutex>
#include <QFileDialog>
#include"force_sensor/ftusb.h"
#include"PD/force_control.h"
#include"tcpip_robot.h"
using namespace Eigen;
using namespace std;
using namespace cv;
class forceThread: public QThread
{ //Q_OBJECT
    Q_OBJECT
signals:
    void sandsignal(double*);
    void signal(QString);

public:
   explicit forceThread (QObject *parent = 0);
    ~forceThread();
     QTcpSocket *tcpClient2;
      QMutex mutex;
      QByteArray dataread;
      double q_actual[24];
      QString strInstruct1;
      bool ifok;
      bool ifopen;
protected:
  void run();

private slots:
//void tcp2connect(int);

void robotstate(double*q_actual);
};
#endif
