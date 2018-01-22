#ifndef UR5_H
#define UR5_H
#include <QMainWindow>
#include <QAbstractSocket>
#include <QTcpSocket>
#include<QFile>
#include<QTimer>
#include "optoports.h"
using namespace std;

namespace Ui 
{
  class TCPIP;
}

class UR5 : public QMainWindow
{
    Q_OBJECT

public:
    explicit UR5(QWidget *parent = 0);
    ~TCPIP();

private slots:
    void on_pushButton_2_clicked();
    void on_pushButton_clicked();
    void connectok();
    void handleTimeout();
    void on_pushButton_3_clicked();
    void movelpos(double x,double y,double z,double Rx,double Ry,double Rz,double a,double v);
    void changevalue();
    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:
    Ui::TCPIP *ui;
     QTimer *timer;//自动跟踪定时器
     QTimer *timer1;//实时刷新机械臂状态
    QTcpSocket *tcpClient;
   // QByteArray outBlock; //数据缓冲区，即存放每次要发送的数据块
    QByteArray outBlock;//数据缓冲区，即存放每次要发送的数据块
    QByteArray dataread;//接收TCP发送过来的数据
    QString commondqueue;
    double targetposition1[6];
    char *autonumber;
    bool ifconnect=false;
};

bool open (OPort port, bool modeSetup=false, int baudRate=1000000);
#endif 
