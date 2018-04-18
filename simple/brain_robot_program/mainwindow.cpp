#include "mainwindow.h"
#include "ui_mainwindow.h"
#include"tcpip_robot.h"
#include<arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace ARToolKitPlus;
double targetpos[6];//存储控制姿态
QString strInstruct;//存储控制的姿态字符串
double targetjoint[6];
ARToolKitPlus::TrackerSingleMarker tracker(640,480,8, 6, 6, 6, 0);
int j=0;
Ptr<ANN_MLP> bp1= ANN_MLP::create();

/*.............手眼矩阵操作*/
//double tbase[16]={0.84344,-0.5372,-0.0041088,0.0033039, 0.012684,0.027559,-0.99954,0.68627,0.53707,0.843,0.030059 ,0.75893, 0,0,0,1};
double tbase[16]={0.93878,-0.34408,0.017128,0.23035, 0.0017775,-0.04488,-0.99899,0.62915,0.3445,0.93787,-0.041521 ,0.92921, 0,0,0,1};
//double tend[16]={-0.73426,-0.67776,0.038842,0.065031,-0.67843,0.73464,-0.0060975,0.16156,-0.024402 ,-0.030829,-0.99923,0.085408,0,0,0,1};
double tend[16]={-0.68576,0.72783,0.00094168,0.17907,0.72738,0.68538,-0.034326,-0.06548,-0.025629 ,-0.022854,-0.99941,0.085436,0,0,0,1};
double k[16]={-0.799009,0.465878,0.380186,0.0268994,0.462642 ,0.880157,-0.106239,0.0538056,-0.384118 ,0.0910042, -0.918788,0.454948,0,0,0,1};
double ft[6];
float kin[6];

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //thread=new forceThread(this);
    video_timer= new QTimer(this);//开启视频检测定时器
    robot_timer=new QTimer(this);
    forcecontrol_timer=new QTimer(this);
    connect(video_timer,SIGNAL(timeout()),this,SLOT(readcamera_Frame()));
    connect(robot_timer,SIGNAL(timeout()),this,SLOT(robotstate_change()));
    connect(forcecontrol_timer,SIGNAL(timeout()),this,SLOT(force()));

    ui->label_11->setAlignment(Qt::AlignCenter);//设置图标字体显示位置
/*------------------------TCPIP连接------------------------*/
    tcpClient= new QTcpSocket(this);

   connect(tcpClient,SIGNAL(connected()),this,SLOT(connectok()));
   // connect(&thread, SIGNAL(sandsignal(double *)), this, SLOT(DisplayMsg(double*)),Qt::DirectConnection);
   // connect(&thread, SIGNAL(signal(QString)), this, SLOT(Move(QString)));
    //connect( this,SIGNAL(tcp2(int)), &thread, SLOT(tcp2connect(int)));
    ui->line_Host->setText(QString("192.168.1.100"));
    ui->line_port->setText(QString("30003"));
    ui->Pos_control->setEnabled(false);
    ui->Joint_control->setEnabled(false);
   // ui->patien_jpg->setPixmap( QPixmap(/home/linzc/QT/2-4/untitled_test/facexml/12/12.jpg));
}

MainWindow::~MainWindow()
{
   // capture.release();
    delete ui;
}
void MainWindow::on_Open_Video_clicked()
{
        //capture->open(0);
    if (capture.isOpened())
       capture.release();     //decide if capture is already opened; if so,close it
       capture.open(0);           //open the default camera
        video_timer->start(100);
           if (capture.isOpened())
           {

               capture >> frame;
              if (!frame.empty())
               {

                   image = Mat2QImage(frame);
                   ui->vedio_frame->setPixmap(QPixmap::fromImage(image));

              }

       }
}
/*------------ 实时显示并检测二维码-----------------*/
void MainWindow::readcamera_Frame()
{
    ui->Open_Video->setEnabled(false);
    Map<MatrixXd>Tbase_arm(tbase,4,4);
    Map<MatrixXd>Tend_arm(tend,4,4);
    Matrix4d T;
    Matrix3d T1;
    VectorXd pos1(3);
    AngleAxisd  V2;
    VectorXd v(3);
        if (capture.isOpened())
        {
            capture >> frame;
           if (!frame.empty())
            {
              init_traker(&tracker,80);//二维码检测
              get_visual_pose(frame,&frame,&tracker);
              image = Mat2QImage(frame);
             Map<MatrixXd> dymMat(tool_Position,4,4);//按列排布
//             for (int i = 0; i <  4; i++)
//                {

//                 for(int j = 0; j <  4; j++)
//                   { qDebug()<<dymMat(i,j);
//                 }
//             }
           // Map<MatrixXd> dymMat(k,4,4);
            //  MatrixXd dymMat=MatrixXd::Identity(4,4);
            // Tbase_arm=Tbase_arm.transpose();
             //Tend_arm=Tend_arm.transpose();
             T=Tend_arm*(dymMat)*Tbase_arm;//转置等于逆
             for (int i = 0; i <  3; i++)
                {
                 pos1(i)=T(i,3);
                //qDebug()<< pos1(i);
                 for(int j = 0; j <  3; j++)
                 {T1(i,j)=T(i,j);
                    //qDebug()<< T1(i,j);
                 }
             }
          //cout<<T1;
           // cout<<T1;
             //T=Tend_arm*(dymMat)*Tbase_arm;
             V2.fromRotationMatrix(T1);//旋转向量（轴角）到旋转矩阵

//              for (int i = 0; i < T1.size(); i++)
//                      std::cout << *(T1.data() + i) << " ";
//              std::cout << std::endl << std::endl;
              v=V2.axis()*V2.angle();
              ui->poscontrol_x->setText((QString::number(pos1(0), 10, 4)));
              ui->poscontrol_y->setText((QString::number(pos1(1)+0.15, 10, 4)));
              ui->poscontrol_z->setText((QString::number(pos1(2), 10, 4)));
              ui->poscontrol_rx->setText((QString::number(v(0), 10, 4)));
              ui->poscontrol_ry->setText((QString::number(v(1)-3.14, 10, 4)));
              ui->poscontrol_rz->setText((QString::number(v(2), 10, 4)));

              ui->vedio_frame->setPixmap(QPixmap::fromImage(image));
        }

    }


}
/*------------------------TCPIP连接------------------------*/
void MainWindow::on_robot_connect_clicked()
{
    ui->robot_connect->setEnabled(false);
    ui->connect_info->setText(tr("连接中。。"));
    tcpClient->connectToHost(ui->line_Host->text(),ui->line_port->text().toInt());
}
void MainWindow::connectok()
{
     ui->connect_info->setText(tr("连接成功，准备发送数据"));
     robot_timer->start(200);
     ui->Pos_control->setEnabled(true);
     ui->Joint_control->setEnabled(true);
     //ann=ANN_MLP::load<ANN_MLP>("/home/zhang/program_hebing/brain_robot_program/bp_nn/bp_param.xml");
     PDinitial();

}
/*----------------------机械臂状态及控制--------------------------*/
void MainWindow::on_Pos_control_clicked()
{
    targetpos[0]=ui->poscontrol_x->text().toDouble();
    targetpos[1]=ui->poscontrol_y->text().toDouble();
    targetpos[2]=ui->poscontrol_z->text().toDouble();
    targetpos[3]=ui->poscontrol_rx->text().toDouble();
    targetpos[4]=ui->poscontrol_ry->text().toDouble();
    targetpos[5]=ui->poscontrol_rz->text().toDouble();
   strInstruct=movelpos(targetpos[0],targetpos[1],targetpos[2],targetpos[3],targetpos[4],targetpos[5],0.1,0.05);
    int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
    qDebug()<<strInstruct<<";"<<write_byte;
}

void MainWindow::on_Joint_control_clicked()
{
    targetjoint[0]=ui->joint1_control->text().toDouble();
    targetjoint[1]=ui->joint2_control->text().toDouble();
    targetjoint[2]=ui->joint3_control->text().toDouble();
    targetjoint[3]=ui->joint4_control->text().toDouble();
    targetjoint[4]=ui->joint5_control->text().toDouble();
    targetjoint[5]=ui->joint6_control->text().toDouble();
   strInstruct=movelpos(targetjoint[0],targetjoint[1],targetjoint[2],targetjoint[3],targetjoint[4],targetjoint[5],0.7,1.5);
    int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
    qDebug()<<strInstruct<<";"<<write_byte;
}
/*-------------------------------机械臂运动状态显示--------------------------*/
void MainWindow::robotstate_change()
{

    UR5_state(q_actual,tcpClient);
    ui->joint_actual1->setText(QString::number(q_actual[0], 5, 2));
    ui->joint_actual2->setText(QString::number(q_actual[1], 5, 2));
    ui->joint_actual3->setText(QString::number(q_actual[2], 5, 2));
    ui->joint_actual4->setText(QString::number(q_actual[3], 5, 2));
    ui->joint_actual5->setText(QString::number(q_actual[4], 5, 2));
    ui->joint_actual6->setText(QString::number(q_actual[5], 5, 2));

    ui->joint_speed1->setText(QString::number(q_actual[6], 6, 3));
    ui->joint_speed2->setText(QString::number(q_actual[7], 6, 3));
    ui->joint_speed3->setText(QString::number(q_actual[8], 6, 3));
    ui->joint_speed4->setText(QString::number(q_actual[9], 6, 3));
    ui->joint_speed5->setText(QString::number(q_actual[10], 6, 3));
    ui->joint_speed6->setText(QString::number(q_actual[11], 6, 3));

    ui->tool_actual1->setText(QString::number(q_actual[12]*1000, 4, 1));
    ui->tool_actual2->setText(QString::number(q_actual[13]*1000, 4, 1));
    ui->tool_actual3->setText(QString::number(q_actual[14]*1000, 4, 1));
    ui->tool_actual4->setText(QString::number(q_actual[15], 4, 3));
    ui->tool_actual5->setText(QString::number(q_actual[16], 4, 3));
    ui->tool_actual6->setText(QString::number(q_actual[17], 4, 3));


    ui->tool_speed1->setText(QString::number(q_actual[18], 6, 3));
    ui->tool_speed2->setText(QString::number(q_actual[19], 6, 3));
    ui->tool_speed3->setText(QString::number(q_actual[20], 6, 3));
    ui->tool_speed4->setText(QString::number(q_actual[21], 6, 4));
    ui->tool_speed5->setText(QString::number(q_actual[22], 6, 4));
    ui->tool_speed6->setText(QString::number(q_actual[23], 6, 4));

}
void MainWindow::force()
{

    double Fx=0,Fy=0,Fz=0;
    double Tx=0,Ty=0,Tz=0;
    Vector3d v;
    Vector3d v1;
    Vector3d v2;
    Vector3d Tr;//重力补偿力矩
    VectorXd Vd(6);//末端坐标系
    Vector3d mgMat(3);
    Vector3d TmgMat(3);
    mgMat<< 0,0,-19.4;
    TmgMat<<0.0459,0.0023,0.02896;
    // robotstate1(q_actual);
    double a = q_actual[0], b = q_actual[1], c =q_actual[2], d = q_actual[3], e = q_actual[4], f = q_actual[5];
    double q[6] = { a*3.14 / 180, b*3.14 / 180, c*3.14 / 180, d*3.14 / 180, e*3.14 / 180, f*3.14 / 180 };
    double* T = new double[16];
    forward(q, T);
    double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
    double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
    double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;
    double translate[9];
    translate[0]=T00;translate[1]=T01;translate[2]=T02;translate[3]=T10;
    translate[4]=T11;translate[5]=T12;translate[6]=T20;translate[7]=T21;
    translate[8]=T22;
    Map<MatrixXd> dymMat(translate,3,3);//以列存储
    //qDebug()<<T00<<" "<<T01<<" "<<T02<<" "<<T10<<" "<<T11<<" "<<T12<<" "<<T20<<" "<<T21<<" "<<T22<<" \n";
//    for (int i = 0; i <  dymMat.size(); i++)
//            std::cout << *(dymMat.data() + i) << " ";
    MatrixXd m(3,1);//重力补偿力
    m=dymMat*mgMat;
//    for (int i = 0; i <  3; i++)
//            std::cout << m(i,0) << " ";
    v=dymMat.transpose()*TmgMat;
    Tr=v.cross(mgMat);
    Tr=dymMat*Tr;

    //cross((dymMat*TmgMat),mgMat);
//    for (int i = 0; i <  3; i++)
//            std::cout << Tr(i) << " ";
/*...........力控制专用..............*/
//     Tx=-Tr(0);Ty=-Tr(1);Tz=-Tr(2);
//     Fx=-m(0,0);Fy=-m(1,0);Fz=-m(2,0)+10;
//    strInstruct=forcecontrol(Fx,Fy,Fz,Tx,Ty,Tz);
//     int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
//         qDebug()<<strInstruct<<";"<<write_byte;
/*...........采集数据专用..............*/
    Tx=Tr(0);Ty=Tr(1);Tz=Tr(2);
    Fx=m(0,0);Fy=m(1,0);Fz=m(2,0);
qDebug()<<Fx<<" "<<Fy<<" "<<Fz<<" "<<Tx<<" "<<Ty<<" "<<Tz<<" \n";
kin[0]=Fx;kin[1]=Fy;kin[2]=Fz;
kin[3]=Tx;kin[4]=Ty;kin[5]=Tz;
if(ifopen)
{
 forcevalue1=force_measure();
//qDebug()<<*forcevalue1/10<<" "<<*(forcevalue1+1)/10<<" "<<*(forcevalue1+2)/10<<" "<<*(forcevalue1+3)/1000<<" "<<*(forcevalue1+4)/1000<<" "<<*(forcevalue1+5)/1000<<" \n";
ft[0]=*forcevalue1/10;ft[1]=*(forcevalue1+1)/10;ft[2]=*(forcevalue1+2)/10;
ft[3]=*(forcevalue1+3)/1000;ft[4]=*(forcevalue1+4)/1000;ft[5]=*(forcevalue1+5)/1000;
}

/*...........自己力控算法..............*/
    if(ifopen)
     {
//      forcevalue1=force_measure();
//     ft[0]=(*forcevalue1)/10-1.7;ft[1]=(*(forcevalue1+1))/10+0.5;ft[2]=(*(forcevalue1+2))/10+12.3;
//     ft[3]=(*(forcevalue1+3))/1000-0.033;ft[4]=(*(forcevalue1+4))/1000-0.239;ft[5]=(*(forcevalue1+5))/1000-0.037;
//     qDebug()<<ft[0]<<" "<<ft[1]<<" "<<ft[2]<<" "<<ft[3]<<" "<<ft[4]<<" "<<ft[5]<<" "<<" \n";
//     Mat sample(1,6,CV_32FC1);
//     for(int i=0;i<6;i++)
//     {
//        sample.at<float>(0,i)=q_actual[i];
//     }
//     Mat response;
//     ann.predict(sample,response);
//     response = response / 1.63;
//     qDebug()<<response.at<float>(0,0)*20<<" "<<response.at<float>(0,1)*20<<" "<<response.at<float>(0,2)*20<<" "<<response.at<float>(0,3)*1.3<<" "<<response.at<float>(0,4)*1.3<<" "<<response.at<float>(0,5)*1.3;
//     double bp[6];
//     bp[0]=20*response.at<float>(0,0);bp[1]=response.at<float>(0,1)*20;bp[2]=response.at<float>(0,2)*20;bp[3]=response.at<float>(0,3)*1.3;bp[4]=response.at<float>(0,4)*1.3;bp[5]=response.at<float>(0,5)*1.3;
//     Fx=ft[0]-bp[0];Fy=ft[1]-bp[1];Fz=ft[2]-bp[2];
//     Tx=ft[3]-bp[3];Ty=ft[4]-bp[4];Tz=ft[5]-bp[5];

//      Vd=compensation(Fx,Fy,Fz,Tx,Ty,Tz);
//      //dymMat.transpose();
//      v1(0)=Vd(0);v1(1)=Vd(1);v1(2)=Vd(2);
//     // v2(0)=Vd(3);v2(1)=Vd(4);v2(2)=Vd(5);
//      v1=dymMat.transpose()*v1;
//      v2=dymMat.transpose()*v2;
//      qDebug()<<v1(0)<<" "<<v1(1)<<" "<<v1(2)<<" "<<v2(0)<<" "<<v2(1)<<" "<<v2(2);
//      targetjoint[0]=q_actual[12]+v1(0);
//      targetjoint[1]=q_actual[13]+v1(1);
//      targetjoint[2]=q_actual[14]+v1(2);
//      targetjoint[3]=q_actual[15]+v2(0);
//      targetjoint[4]=q_actual[16]+v2(1);
//      targetjoint[5]=q_actual[17]+v2(2);
//     qDebug()<<targetjoint[0]<<" "<<targetjoint[1]<<" "<<targetjoint[2]<<" "<<targetjoint[3]<<" "<<targetjoint[4]<<" "<<targetjoint[5];
//     strInstruct=movelpos(targetjoint[0],targetjoint[1],targetjoint[2],targetjoint[3],targetjoint[4],targetjoint[5],1.5,0.8);
//      int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
//          qDebug()<<strInstruct<<";"<<write_byte;
   }

}
/*............ 打开力控制.............*/
void MainWindow::on_openforce_clicked()
{
    ifopen=open_sensor();
    if(ifopen)
    {
      ui->openforce->setEnabled(false);
     forcevalue1=force_measure();
     std::cout<<"Fx: "<<*forcevalue1<<" Fy: "<<*(forcevalue1+1)<<" Fz: "<<*(forcevalue1+2)<<" ";
     std::cout<<"Tx: "<<*(forcevalue1+3)<<" Ty: "<<*(forcevalue1+4)<<" Tz: "<<*(forcevalue1+5)<<std::endl;
    }
    //bp1=ANN_MLP::load<ANN_MLP>("/home/linzc/QT/2-4/untitled_test/bp_nn/bp_param.xml");
    PDinitial();
   robot_timer->stop();
    waitKey(500);
  if (!thread.isRunning())
     thread.ifopen=ifopen;
     thread.start();

}
/*............ 关闭力控制.............*/
void MainWindow::on_closeforce_clicked()
{
   ui->openforce->setEnabled(true);
  //forcecontrol_timer->stop();
    robot_timer->start(200);
   close_sensor();
   thread.terminate();

}
/*............ 人脸模型训练.............*/
void MainWindow::on_pushButton_clicked()
{
  facebegin();
}
/*............ 患者人脸识别.............*/
void MainWindow::on_pushButton_2_clicked()
{
     int identity;
    identity=recognized();

    QString id= QString::number(identity, 10);
     QString filename="/home/linzc/QT-workspace/untitled_test_1/facexml/";
     filename+=id+ "/" + id+".txt";
    // filename = QFileDialog :: getOpenFileName(this,NULL,NULL,filename1);
     QFile file(filename);
       if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
         {
             cout << "Open failed." << endl;
         }
         else
             {
               QTextStream in(&file);
                     //---QtextEdit按行显示文件内容
                     ui->patient_data->setPlainText(in.readAll());
             }
        filename="/home/linzc/QT-workspace/untitled_test_1/facexml/";
        filename+=id+ "/" + id+".jpg";
        QPixmap pixmap(filename);
      ui->patien_jpg->setPixmap(pixmap);
      ui->patien_jpg->show();
}
/*............  神经网络数据采集.............*/
void MainWindow::on_pushButton_3_clicked()
{
 /*............  神经网络数据采集.............*/
   float F[6];
    F[0]=ft[0]-1.7;F[1]=ft[1]+0.5;F[2]=ft[2]+11.2;
    F[3]=ft[3]-0.035;F[4]=ft[4]-0.21;F[5]=ft[5]-0.038;
   qDebug()<<F[0]-kin[0]<<" "<<F[1]-kin[1]<<" "<<F[2]-kin[2]<<" "<<F[3]-kin[3]<<" "<<F[4]-kin[4]<<" "<<F[5]-kin[5]<<" \n";

    if(fabs(F[0]-kin[0])<2.5&&fabs(F[1]-kin[1])<2.5&&fabs(F[2]-kin[2])<2.5&&fabs(F[3]-kin[3])<0.2&&fabs(F[4]-kin[4])<0.2&&fabs(F[5]-kin[5])<0.2)
    {

    QFile data("/home/linzc/QT/2-4/untitled_test/facexml/forcece.txt");
    if (data.open(QFile::WriteOnly | QIODevice::Append))
    {

        QTextStream out(&data);
        out <<F[0]<<","<<F[1]<<","<<F[2]<<","<<F[3]<<","<<F[4]<<","<<F[5]<<endl;
    }
    QFile data1("/home/linzc/QT/2-4/untitled_test/facexml/jointce.txt");
    if (data1.open(QFile::WriteOnly | QIODevice::Append))
    {

        QTextStream out1(&data1);
        out1 <<q_actual[0]<<","<<q_actual[1]<<","<<q_actual[2]<<","<<q_actual[3]<<","<<q_actual[4]<<","<<q_actual[5]<<","<<endl;
    }
    QFile data2("/home/linzc/QT/2-4/untitled_test/facexml/kin.txt");
    if (data2.open(QFile::WriteOnly | QIODevice::Append))
    {

        QTextStream out2(&data2);
        out2 <<kin[0]<<","<<kin[1]<<","<<kin[2]<<","<<kin[3]<<","<<kin[4]<<","<<kin[5]<<","<<endl;
    }
    j++;
    qDebug()<<j;
    }
/*............ 惯性矩阵数据采集.............*/
    
}
/*............  机械臂数据读取.............*/
//void MainWindow::robotstate1(double*q_actual)
//{
//    uint64_t unpack_to;
//    uint16_t offset = 0;
//    char  *buf;
//    int len;
//    uint64_t q;
//    int bytes_read;
//    QByteArray dataread;
//    offset=0;

//    dataread.resize(tcpClient->bytesAvailable());
//    bytes_read=tcpClient->read(dataread.data(),dataread.size());
//     qDebug()<< bytes_read;
//    //dataread=tcpClient->readAll();
//    if(bytes_read%1060==0&&bytes_read!=0)
//    {
//     buf=dataread.data()+bytes_read-1060;
//    memcpy(&len, &buf[0], sizeof(len));
//    offset += sizeof(len);
//    //qDebug()<< offset;
//    len=ntohl(len);//是将一个无符号长整形数从网络字节顺序转换为主机字节顺序， ntohl()返回一个以主机字节顺序表达的数。
//   //printf("%d",len);
//    memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
//    unpack_to = be64toh(unpack_to);//函数返回一个大字节排序整数转换为系统的本机字节顺序。返回值将与大端系统上的参数相同。所谓的大端模式（Big-endian），是指数据的高字节，保存在内存的低地址中，而数据的低字节，保存在内存的高地址中
//    double k;
//     memcpy(&k, &unpack_to, sizeof(k));
//    offset += sizeof(unpack_to)+30*sizeof(double);
//    for(int i=0;i<6;i++)
//    {
//    memcpy(&q, &buf[offset], sizeof(double));
//    q=be64toh(q);//字节序转换

//    memcpy(&q_actual[i], &q, sizeof(q));
//        //qDebug()<<q_actual[i];
//    q_actual[i]=q_actual[i]*180/pi;
//    offset += sizeof(double);
//    }

//    for(int i=0;i<6;i++)
//    {
//    memcpy(&q, &buf[offset], sizeof(double));
//    q=be64toh(q);//字节序转换

//    memcpy(&q_actual[i+6], &q, sizeof(q));
//        //qDebug()<<q_speed[i];
//    offset += sizeof(double);
//    }
//    offset+=sizeof(double)*12;
//    for(int i=0;i<6;i++)
//    {
//        memcpy(&q, &buf[offset], sizeof(double));
//        q=be64toh(q);//字节序转换
//        memcpy(&q_actual[i+12], &q, sizeof(q));
//        offset += sizeof(double);
//    }
//    for(int i=0;i<6;i++)
//    {
//        memcpy(&q, &buf[offset], sizeof(double));
//        q=be64toh(q);//字节序转换
//        memcpy(&q_actual[i+18], &q, sizeof(q));
//        offset += sizeof(double);
//    }
/*............  机械臂数据读取问题.............*/
//    //Matrix4d T;
//    double R[9];
//    AngleAxisd  V2;
//    VectorXd v(3);
//    double q[6] = { q_actual[0]*3.14 / 180, q_actual[1]*3.14 / 180, q_actual[2]*3.14 / 180, q_actual[3]*3.14 / 180, q_actual[4]*3.14 / 180, q_actual[5]*3.14 / 180 };
//    double T[16];
//    forward(q, T);
//    double T02 = -T[0];  double T00 = T[1];  double T01 = T[2];
//    double T12 = -T[4]; double T10 = T[5]; double T11 = T[6];
//    double T22 = T[8]; double T20 = -T[9]; double T21 = -T[10];
//    R[0]=T00;R[1]=T01;R[2]=T02;R[3]=T10;
//    R[4]=T11;R[5]=T12;R[6]=T20;R[7]=T21;
//    R[8]=T22;
//    Map<MatrixXd> dymMat(R,3,3);//以列存储
//    Matrix3d rt;
//    rt=dymMat.transpose();
//     V2.fromRotationMatrix(rt);
//     v=V2.axis()*V2.angle();
//     //q_actual[15]=v(0);q_actual[16]=v(1);q_actual[17]=v(2);
//}
//}


void MainWindow::on_level_clicked()
{
    QVector<double> temp(20);
       QVector<double> temp1(20);

       for(int i=0;i<20;i++)
       {
           temp[i] = i;
           temp1[i] =rand()%(45-55)+20;
       }
    ui->plot->addGraph();  //添加一条曲线
    ui->plot->graph(0)->setPen(QPen(Qt::red));//x是曲线序号，添加的第一条是0，设置曲线颜色
    ui->plot->graph(0)->setData(temp,temp1); //输出各点的图像，x和y都是QVector类
    //其原型如下：
    //void QCPGraph::setData(const QVector<double> &key, const QVector<double> &value)

    ui->plot->xAxis->setLabel("患者治疗前后30天内血糖含量");   //x轴的文字
    ui->plot->yAxis->setLabel("含量值");   //y轴的文字
    ui->plot->xAxis->setRange(0,20);  //x轴范围
    ui->plot->yAxis->setRange(0,50);  //y轴范围
    ui->plot->replot();   //重绘
}
