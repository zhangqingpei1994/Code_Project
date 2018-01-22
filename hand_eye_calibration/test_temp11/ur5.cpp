#include "ur5.h"
#include "ui_tcpip.h"
#include<arpa/inet.h>
#include <iostream>
#include<optodaq.h>
#include <unistd.h>
#include "opto.h"
//#include<SDKDDKVer.h>
#define TIMER_TIMEOUT   (5*1000)
unsigned long int unpack_to;
unsigned short int offset = 0;
char *buf;
int len;
unsigned long int q;
double q_actual[6];
double tool_actual[6];
qint64 bytes_read;
#include <dlfcn.h>
//extern "C"{
//#include "opto.h"}

typedef unsigned long long mytime_t;
UR5::UR5(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TCPIP)
{     // daq=new OptoDAQ;
//     ports=new OptoPorts;
    // portlist=new OPort;
//     pack3D=new OptoPackage;
//     pack6D=new OptoPackage6D;
    ui->setupUi(this);
    tcpClient = new QTcpSocket(this);
    timer = new QTimer(this);//开启定时器实现自动跟踪
    timer1=new QTimer(this);
    //当连接服务器成功后，发出connecct()信号，开始传送文件
     ui->lineEdit->setText(QString("192.168.1.100"));
     ui->lineEdit_2->setText(QString("30003"));
     //ui->joint_actual1->setText(QString::number(q1, 10, 5));
     ui->pushButton->setEnabled(false);//发送位姿按钮
    // ui->pushButton_3->setEnabled(false);//自动跟踪按钮
   connect(tcpClient,SIGNAL(connected()),this,SLOT(connectok()));
   connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout())); //定时器到时响应
   connect(timer1,SIGNAL(timeout()),this,SLOT(changevalue()));
}

void UR5::connectok() //当连接服务器成功后，发出connecct()信号，开始传送文件
{
    ifconnect=true;
    ui->label_3->setText(tr("连接成功，准备发送数据"));
    ui->pushButton->setEnabled(true);//发送位姿按钮
    ui->pushButton_3->setEnabled(true);//自动跟踪按钮
    timer1->start (500);//500ms刷新一次
}

UR5::~UR5()
{
    delete ui;
}

void UR5::on_pushButton_2_clicked()
{
    ui->pushButton_2->setEnabled(false);
    ui->label_3->setText(tr("连接中。。"));
    //使用conectToHost函数来链接到服务器.(QTcpServer类自带的函数)
    //void QAbstractSocket::connectToHost(const QString &hostName,
    //  quint16 port, OpenMode openMode = ReadWrite, NetworkLayerProtocol protocol = AnyIPProtocol)
    tcpClient->connectToHost(ui->lineEdit->text(),ui->lineEdit_2->text().toInt());
//    VideoCapture capture("1.avi");//¶ÁÈëÊÓÆµ
//    Mat frame;
//    while (1)
//    {
//        capture>>frame;//¶ÁÈ¡µ±Ç°Ö¡
//        imshow("1", frame);
//        waitKey(30);
//    }

}

void UR5::on_pushButton_clicked()
{
        targetposition1[0]=ui->lineEdit_4->text().toDouble();
        targetposition1[1]=ui->lineEdit_5->text().toDouble();
        targetposition1[2]=ui->lineEdit_6->text().toDouble();
        targetposition1[3]=ui->lineEdit_7->text().toDouble();
        targetposition1[4]=ui->lineEdit_8->text().toDouble();
        targetposition1[5]=ui->lineEdit_9->text().toDouble();
        movelpos(targetposition1[0],targetposition1[1],targetposition1[2],targetposition1[3],targetposition1[4],targetposition1[5],0.3,1.0);
        QString strInstruct=commondqueue;
        int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
            qDebug()<<strInstruct<<";"<<write_byte;

     //QByteArray ba = ui->lineEdit_3->text().toLatin1(); // 将QString转为char
   //  autonumber=ba.data();
//      if(ifconnect)
//      {tcpClient->write(autonumber);
//      ui->label_3->setText(tr("发送成功"));}
//      else
//       ui->label_3->setText(tr("没有设备连接"));

}
void MySleep(unsigned long p_uMillisecs)
{
    usleep(p_uMillisecs * 1000);
}
bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex)
{
    //MySleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList
    OPort * portList = p_Ports.listPorts(true);
    int iLastSize = p_Ports.getLastSize();
    if (p_iIndex >= iLastSize) {
        // index overflow
        return false;
    }
    bool bSuccess = p_optoDAQ.open(portList[p_iIndex]);
    //bool bSuccess = open(portList[p_iIndex]);
//    if (bSuccess) {
//        ShowInformation(p_optoDAQ, portList[p_iIndex]);
//    }
    return bSuccess;
}

void UR5::on_pushButton_3_clicked()//自动跟踪按钮相应函数
{
    MySleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList
    OptoDAQ daq;
    OptoPorts ports;
    OPort *portlist;
    OptoPackage pack3D;
    OptoPackage6D pack6D;

     portlist=ports.listPorts(true);

     if (OpenPort(daq, ports, 1) == false) {
             std::cout<<"Could not open port"<<std::endl;
            // return 0;
         }
    if (ports.getLastSize()>0)
    {
        daq.open(portlist[0]);

        if (daq.getVersion()!=_95 && daq.getVersion() != _64) // It is a 3D sensor
        {

            int size=daq.read(pack3D,false);	// Reading Sensor #0 (up to 16 Sensors)
            std::cout<<"x: "<<pack3D.x<<" y: "<<pack3D.y<<" z: "<<pack3D.z<<std::endl;

        }
        else					  // It is a 6D sensor
        {

            int size=daq.read6D(pack6D,false);
            std::cout<<"Fx: "<<pack6D.Fx<<" Fy: "<<pack6D.Fy<<" Fz: "<<pack6D.Fz<<" ";
            std::cout<<"Tx: "<<pack6D.Tx<<" Ty: "<<pack6D.Ty<<" Tz: "<<pack6D.Tz<<std::endl;
        }
        daq.close();
    }
    else
    {
        std::cout<<"No sensor available"<<std::endl;
    }
}
void UR5::handleTimeout()
{   ui->label_3->setText(tr("自动跟踪进行中"));
    //*autonumber+=char(a1)+','+char(a2)+','+char(a3)+char(a4)+','+char(a5)+','+char(a6);
//    QDataStream sendOut(&outBlock,QIODevice::WriteOnly);//固定格式
//    sendOut.setVersion(QDataStream::Qt_5_4);
//    //QString.right(n) 返回的是一个substring包含了n个最右边的元素,QString.lastIndexOf返回的是字符串里面最后一个/的位置
//    sendOut<<qint64(0)<<qint64(0)<<autonumber;
//    sendOut.device()->seek(0);
//    //减去totalBytes和文件名大小所占用的两个qint64(0)的大小
//    sendOut<<qint64(outBlock.size() - sizeof(qint64)*2);
    tcpClient->write(autonumber);

}
void UR5::changevalue()
{

    dataread.resize(tcpClient->bytesAvailable());
    bytes_read=this->tcpClient->read(dataread.data(),dataread.size());
    //dataread=tcpClient->readAll();
    if(bytes_read%1060==0)
    {
     buf=dataread.data();
    memcpy(&len, &buf[0], sizeof(len));
    len=ntohl(len);//是将一个无符号长整形数从网络字节顺序转换为主机字节顺序， ntohl()返回一个以主机字节顺序表达的数。
    offset += sizeof(len);
    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
    offset += sizeof(double)+sizeof(double)*30;
    for(int i=0;i<6;i++)
    {
    memcpy(&q, &buf[offset], sizeof(double));
    q=be64toh(q);//字节序转换
    memcpy(&q_actual[i], &q, sizeof(q));
    offset += sizeof(double);
    }
    ui->joint_actual1->setText(QString::number(q_actual[0], 10, 4));
    ui->joint_actual2->setText(QString::number(q_actual[1], 10, 4));
    ui->joint_actual3->setText(QString::number(q_actual[2], 10, 4));
    ui->joint_actual4->setText(QString::number(q_actual[3], 10, 4));
    ui->joint_actual5->setText(QString::number(q_actual[4], 10, 4));
    ui->joint_actual6->setText(QString::number(q_actual[5], 10, 4));
    offset+=sizeof(double)*18;
    for(int i=0;i<6;i++)
    {
        memcpy(&q, &buf[offset], sizeof(double));
        q=be64toh(q);//字节序转换
        memcpy(&tool_actual[i], &q, sizeof(q));
        offset += sizeof(double);
    }
    ui->tool_actual1->setText(QString::number(tool_actual[0], 10, 4));
    ui->tool_actual2->setText(QString::number(tool_actual[1], 10, 4));
    ui->tool_actual3->setText(QString::number(tool_actual[2], 10, 4));
    ui->tool_actual4->setText(QString::number(tool_actual[3], 10, 4));
    ui->tool_actual5->setText(QString::number(tool_actual[4], 10, 4));
    ui->tool_actual6->setText(QString::number(tool_actual[5], 10, 4));
    }
    qDebug()<<"bytes_read="<<bytes_read;
   // qDebug()<<"dataread="<<dataread.data();
    qDebug()<<"a="<<q;
    qDebug()<<"len="<<len;
}

void UR5::on_pushButton_4_clicked()//停机按钮
{
tcpClient->disconnectFromHost();
 ui->label_3->setText(tr("已断开连接"));
 timer->stop();
 ui->pushButton_3->setEnabled(true);
 ui->pushButton_2->setEnabled(true);
}
void UR5::movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v)
{
    commondqueue="def f():\n\t";
    QString strInstruct=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t").arg(x).arg(y).arg(z).arg(Rx).arg(Ry).arg(Rz);
       strInstruct+=QString("movel(wp1,a=%1,v=%2)\n\t").arg(a).arg(v);
       if(strInstruct.right(1).toLatin1() != "\n")
               strInstruct.append('\n');
           commondqueue+=strInstruct;
           commondqueue+="end\n";

}








