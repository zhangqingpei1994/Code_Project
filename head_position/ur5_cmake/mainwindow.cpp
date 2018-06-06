#include "mainwindow.h"
#include "ui_mainwindow.h"
#include"tcpip_robot.h"
#include<arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>
using namespace ARToolKitPlus;
double targetpos[6];//存储控制姿态
QString strInstruct;//存储控制的姿态字符串
double targetjoint[6];
double *sensor_measure;
//ARToolKitPlus::TrackerSingleMarker tracker(640,480,8, 6, 6, 6, 0);
ARToolKitPlus::TrackerSingleMarker tracker(1280,720,8, 6, 6, 6, 0);
int j=0,m=0,move_l=1,position=0;
double Fz_current;
bool first_last=false;
MatrixXd state_change(4,4);
MatrixXd endT(4,4);
MatrixXd robot_camera(4,4);
CvANN_MLP bp;
Vector3d last_time;
double old_position[6];
/*.............手眼矩阵操作*/
double camere_arm[16]={-0.8825,-0.47018,-0.011119,0.24744,-0.018058,0.010249,-0.99978,0.51586,-0.46997,0.88251,0.017536,1.2445,0,0,0,1};//以行存储
double ft[6];
float kin[6];
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //thread=new forceThread(this);

    connect_ur5=false;

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
    ui->line_Host->setText(QString("192.168.1.105"));
    ui->line_port->setText(QString("30003"));
    ui->Pos_control->setEnabled(false);
    ui->Joint_control->setEnabled(false);
   // ui->patien_jpg->setPixmap( QPixmap(/home/linzc/QT/2-4/untitled_test/facexml/12/12.jpg));
}

MainWindow::~MainWindow()
{
    capture.release();
    delete ui;
}
void MainWindow::on_Open_Video_clicked()
{
    if (capture.isOpened())
       capture.release();     //decide if capture is already opened; if so,close it
    capture.open(0);           //open the default camera
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);//set·œ·š²»œöÓÃÓÚÈ¡ÊÓÆµÖ¡µÄÎ»ÖÃ£¬»¹¿ÉÒÔÉèÖÃÊÓÆµµÄÖ¡ÂÊ¡¢ÁÁ¶È
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);//ÉèÖÃÍŒÏñµÄŽóÐ¡640X480
       //sleep(10);
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
    Map<MatrixXd>Tbase_arm(camere_arm,4,4);//实际矩阵的转置,按列顺序形成矩阵
    Matrix4d T;
    Matrix3d T1;
    VectorXd pos1(3);
    AngleAxisd  V2;
    VectorXd v(3);
    if (capture.isOpened())
    {
        capture >> frame;
          //  qDebug()<<frame.rows;
        if (!frame.empty())
        {
           // init_traker(&tracker,100);//二维码检测
           // get_visual_pose(frame,&frame,&tracker);
            image = Mat2QImage(frame);
             // qDebug()<<image.size();
            Map<MatrixXd> dymMat(tool_Position,4,4);//按列排布
            // Map<MatrixXd> Tbase_arm1(tbase,4,4);
            qDebug()<<dymMat(0,3)<<" "<<dymMat(1,3)<<" "<<dymMat(2,3)<<" ";
            AngleAxisd V3;
            for (int i = 0; i <  3; i++)
            {
                for(int j = 0; j <  3; j++)
                {
                    T1(i,j)=dymMat(i,j);
                }
            }
            V3.fromRotationMatrix(T1);
            Vector3d last;
            last=V3.axis()*V3.angle();
            qDebug()<<last(0)<<" "<<last(1)<<" "<<last(2)<<endl;

            T=(Tbase_arm.transpose()).inverse()*dymMat;//*(end_checkboard.transpose()).inverse();//*end_checkboard;

            for (int i = 0; i <  3; i++)
            {
                pos1(i)=T(i,3);
               // qDebug()<< pos1(i);
                for(int j = 0; j <  3; j++)
                {
                    T1(i,j)=T(i,j);
                  // qDebug()<< T1(i,j);
                }
            }
          //cout<<T1;
           // cout<<T1;
             //T=Tend_arm*(dymMat)*Tbase_arm;
          // qDebug()<<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2);
//           V2.fromRotationMatrix(T1);//旋转向量（轴角）到旋转矩阵

//              v=V2.axis()*V2.angle();
//              ui->poscontrol_x->setText((QString::number(pos1(0), 10, 4)));
//              ui->poscontrol_y->setText((QString::number(pos1(1), 10, 4)));
//              ui->poscontrol_z->setText((QString::number(pos1(2), 10, 4)));
//              ui->poscontrol_rx->setText((QString::number(v(0), 10, 4)));
//              ui->poscontrol_ry->setText((QString::number(v(1), 10, 4)));
//              ui->poscontrol_rz->setText((QString::number(v(2), 10, 4)));
            QPixmap autoimage=QPixmap::fromImage(image);
            autoimage.scaled(ui->vedio_frame->size(),Qt::KeepAspectRatio);
            ui->vedio_frame->setScaledContents(true);
            ui->vedio_frame->setPixmap(autoimage);
        }
    }
}
/*------------------------TCPIP连接------------------------*/
void MainWindow::on_robot_connect_clicked()
{
     ui->robot_connect->setEnabled(false);
     ui->connect_info->setText(tr("连接中。。"));
     ui->label_State_arm->setText(tr("连接中。。"));
     tcpClient->connectToHost(ui->line_Host->text(),ui->line_port->text().toInt());
}
void MainWindow::connectok()
{
     ui->connect_info->setText(tr("连接成功，准备发送数据"));
     robot_timer->start(100);
     //forcecontrol_timer->start(221);
     ui->Pos_control->setEnabled(true);
     ui->Joint_control->setEnabled(true);
     //ann.load("/home/linzc/QT-workspace/untitled_test_2/bp_nn/bp_param.xml");
     // PDinitial();

     connect_ur5=true;
     ui->label_State_arm->setText(tr("机械臂连接成功"));

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
    //qDebug()<<"Fz";
/*...........二维码跟踪控制......*/
    if(m==1)
    {
        double new_position[6];
        double error[6];
        double Fz;
        double translate[9];
        double q[6] = { q_actual[0]*3.14 / 180, q_actual[1]*3.14 / 180, q_actual[2]*3.14 / 180, q_actual[3]*3.14 / 180, q_actual[4]*3.14 / 180, q_actual[5]*3.14 / 180 };
        double T[16];
        forward(q, T);
        double T02 = -T[0];  double T00 = T[1];  double T01 = T[2];
        double T12 = -T[4]; double T10 = T[5]; double T11 = T[6];
        double T22 = T[8]; double T20 = -T[9]; double T21 = -T[10];
        translate[0]=T00;translate[1]=T01;translate[2]=T02;translate[3]=T10;
        translate[4]=T11;translate[5]=T12;translate[6]=T20;translate[7]=T21;
        translate[8]=T22;
        Map<MatrixXd> dymMat(translate,3,3);//以列存储
        automation_track(new_position);
/*..............采数据专用......................*/
        double real_state[16];
        real_state[0]=T00;real_state[1]=T01;real_state[2]=T02;real_state[3]=-T[3];
        real_state[4]=T10;real_state[5]=T11;real_state[6]=T12;real_state[7]=-T[7];
        real_state[8]=T20;real_state[9]=T21;real_state[10]=T22;real_state[11]=T[11];
        real_state[12]=0;real_state[13]=0;real_state[14]=0;real_state[15]=1;
        Map<MatrixXd>  robot__translate(real_state,4,4);
        MatrixXd robot__to_marker(4,4);
        Map<MatrixXd> camera_state(tool_Position,4,4);
        MatrixXd marker_in_robot(4,4);
        robot__to_marker=robot__translate.transpose()*endT*state_change;
        marker_in_robot=robot_camera.inverse()*camera_state;

        AngleAxisd V3;
        Matrix3d T2;
        Vector3d last;
        double robot_marker[6];
        double marker_in[6];
        for (int i = 0; i <  3; i++)
        {
             for(int j = 0; j <  3; j++)
               {
                  T2(i,j)=robot__to_marker(i,j);
               }
         }

       V3.fromRotationMatrix(T2);
       last=V3.axis()*V3.angle();
       robot_marker[0]=robot__to_marker(0,3);
       robot_marker[1]=robot__to_marker(1,3);
       robot_marker[2]=robot__to_marker(2,3);
       robot_marker[3]=last(0);
       robot_marker[4]=last(1);
       robot_marker[5]=last(2);
       for (int i = 0; i <  3; i++)
       {
            for(int j = 0; j <  3; j++)
              {
                 T2(i,j)=marker_in_robot(i,j);
              }
        }

      V3.fromRotationMatrix(T2);
      last=V3.axis()*V3.angle();
      marker_in[0]=marker_in_robot(0,3);
      marker_in[1]=marker_in_robot(1,3);
      marker_in[2]=marker_in_robot(2,3);
      marker_in[3]=last(0);
      marker_in[4]=last(1);
      marker_in[5]=last(2);
     // qDebug()<<robot_marker[0]<<" "<<robot_marker[1]<<" "<<robot_marker[2]<<" "<<robot_marker[3]<<" "<<robot_marker[4]<<" "<<robot_marker[5];
      //qDebug()<<marker_in[0]<<" "<<marker_in[1]<<" "<<marker_in[2]<<" "<<marker_in[3]<<" "<<marker_in[4]<<" "<<marker_in[5];
/*..............采数据专用......................*/
       if(position==0)
       {

            for(int i=0;i<6;i++)
            {
                old_position[i]=new_position[i];
               // qDebug()<<"old"<<old_position[i];
               // qDebug()<<"new"<<new_position[i];
            }

            position=1;
        }
        error[0]=fabs(new_position[0]-old_position[0]);
        error[1]=fabs(new_position[1]-old_position[1]);
        error[2]=fabs(new_position[2]-old_position[2]);
        error[3]=fabs(new_position[3]-old_position[3]);
        error[4]=fabs(new_position[4]-old_position[4]);
        error[5]=fabs(new_position[5]-old_position[5]);
        sensor_measure=force_measure();
        if(move_l==1)
        {
            Fz_current=(*(sensor_measure+2))/10;
        }
        //bp[0]=20*response.at<float>(0,0);bp[1]=response.at<float>(0,1)*20;bp[2]=response.at<float>(0,2)*20;bp[3]=response.at<float>(0,3)*1.2;bp[4]=response.at<float>(0,4)*1.2;bp[5]=response.at<float>(0,5)*1.2;
        Fz=(*(sensor_measure+2))/10;
        Fz=Fz-Fz_current+5;
/*......................二维码跟踪数据采集............................*/
//        QFile data("/home/zhang/brain_share/brain_robot/linzecai/ur5_cmake/facexml/robot.txt");
//        if (data.open(QFile::WriteOnly | QIODevice::Append))
//        {

//            QTextStream out(&data);
//            out <<robot_marker[0]<<","<<robot_marker[1]<<","<<robot_marker[2]<<","<<robot_marker[3]<<","<<robot_marker[4]<<","<<robot_marker[5]<<endl;
//        }
//        QFile data1("/home/zhang/brain_share/brain_robot/linzecai/ur5_cmake/facexml/marker.txt");
//        if (data1.open(QFile::WriteOnly | QIODevice::Append))
//        {

//            QTextStream out1(&data1);
//            out1 <<marker_in[0]<<","<<marker_in[1]<<","<<marker_in[2]<<","<<marker_in[3]<<","<<marker_in[4]<<","<<marker_in[5]<<","<<endl;
//        }
//        QFile data2("/home/zhang/brain_share/brain_robot/linzecai/ur5_cmake/facexml/force_z.txt");
//        if (data2.open(QFile::WriteOnly | QIODevice::Append))
//        {

//            QTextStream out2(&data2);
//            out2 <<Fz<<endl;
//        }
 /*......................二维码跟踪数据采集............................*/

        if(error[0]<0.015&&error[1]<0.015&&error[2]<0.015&&error[3]<0.1&&error[4]<0.1&&error[5]<0.1)
        {
            VectorXd Vd(6);//末端坐标系
            Vector3d v1;
            Vector3d v11;

            Vd=compensation(0,0,Fz,0,0,0);
            v1[0]=0;v1[1]=0;v1[2]=Vd[2];
            //qDebug()<<"Fz"<<Fz<<"Fz_curren"<<Fz_current<<"Z"<<Vd(2);
            v11=dymMat.transpose()*v1;
            qDebug()<<"Fz"<<Fz<<"Fz_curren"<<Fz_current<<"Z"<<Vd(2)<<"REAL z"<<v11[2];
            new_position[0]=q_actual[12]+v11[0];
            new_position[1]=q_actual[13]+v11[1];
            new_position[2]=q_actual[14]+v11[2];
            new_position[3]=q_actual[15];
            new_position[4]=q_actual[16];
            new_position[5]=q_actual[17];
            if(fabs(v11[0])<0.035&&fabs(v11[1])<0.035&&fabs(v11[2])<0.035)
            {

//                if((fabs(q_actual[18])<0.01)&&(fabs(q_actual[19])<0.01)&&(fabs(q_actual[20])<0.01)&&(fabs(q_actual[21])<0.01)&&(fabs(q_actual[22])<0.01)&&(fabs(q_actual[23])<0.01))
//                {
//                // qDebug()<<
//                    strInstruct=movelpos(new_position[0],new_position[1],new_position[2],new_position[3],new_position[4],new_position[5],0.1,0.05);
//                    int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
//                    qDebug()<<strInstruct<<";"<<write_byte;
//                    bool ifwritten=tcpClient->waitForBytesWritten();
//                    move_l=0;
//                    for(int i=0;i<6;i++)
//                        old_position[i]=new_position[i];
//                }
            }
         }
        else if(error[0]<0.15&&error[1]<0.15&&error[2]<0.15&&error[3]<1&&error[4]<1&&error[5]<1)
         {

            if((fabs(q_actual[18])<0.01)&&(fabs(q_actual[19])<0.01)&&(fabs(q_actual[20])<0.01)&&(fabs(q_actual[21])<0.01)&&(fabs(q_actual[22])<0.01)&&(fabs(q_actual[23])<0.01))
            {
               // qDebug()<<
                strInstruct=movelpos(new_position[0],new_position[1],new_position[2],new_position[3],new_position[4],new_position[5],0.1,0.08);
                int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
                qDebug()<<strInstruct<<";"<<write_byte;
                bool ifwritten=tcpClient->waitForBytesWritten();
                move_l=1;
                for(int i=0;i<6;i++)
                    old_position[i]=new_position[i];
           }
        }
        else
        {
            qDebug()<<"the move is so big,please come become";
        }
      }



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
        ft[0]=forcevalue1[0]/10;ft[1]=forcevalue1[1]/10;ft[2]=forcevalue1[2]/10;
        ft[3]=forcevalue1[3]/1000;ft[4]=forcevalue1[4]/1000;ft[5]=forcevalue1[5]/1000;
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
        qDebug()<<"Fx: "<<forcevalue1[0]<<" Fy: "<<forcevalue1[1]<<" Fz: "<<forcevalue1[2]<<" ";
        qDebug()<<"Tx: "<<*(forcevalue1+3)<<" Ty: "<<*(forcevalue1+4)<<" Tz: "<<*(forcevalue1+5);
    }
    //bp1.load("/home/linzc/QT-workspace/untitled_test_2/bp_nn/bp_param.xml");
    PDinitial();
//    robot_timer->stop();
//    waitKey(500);
//   if (!thread.isRunning())
//     thread.ifopen=ifopen;
//    thread.start();

}
/*............ 关闭力控制.............*/
void MainWindow::on_closeforce_clicked()
{
   /*Vector3d Md;
   //Md(0)=0;Md(1)=0;Md(2)=1;
    double CD;
    q_actual[15]=-2.33;q_actual[16]=2.098;q_actual[17]=0.0157;
    qDebug()<<-2.33<<" "<<2.098<<" "<<0.0157;
    CD=sqrt(q_actual[15]*q_actual[15]+q_actual[16]*q_actual[16]+q_actual[17]*q_actual[17]);
    Md(0)=q_actual[15]/CD;
    Md(1)=q_actual[16]/CD;
    Md(2)=q_actual[17]/CD;
    AngleAxisd zj(CD,Md);
    Md=zj.axis()*zj.angle();
    Vector3d eulerAngle=zj.matrix().eulerAngles(0,1,2);
    eulerAngle(0)=eulerAngle(0);//+v22(0);
    eulerAngle(1)=eulerAngle(1);//+v22(1);
    eulerAngle(2)=eulerAngle(2)+0.031;
    AngleAxisd angleAxis2 = AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX());
    AngleAxisd angleAxis1 = AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY());
    AngleAxisd angleAxis0 = AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ());
    Matrix3d test =angleAxis2.matrix()*angleAxis1.matrix()*angleAxis0.matrix();
    AngleAxisd V3(test);
    //V3.fromRotationMatrix(test);
   Vector3d last;
   last=V3.axis()*V3.angle();*/



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
    Kinect2_FaceRecog=new QTimer(this);
    connect(Kinect2_FaceRecog,SIGNAL(timeout()),this,SLOT(readKinect2_Frame()));
    Kinect2_FaceRecog->start(70);
    grab_kinect2.Grab_image_KinectV2(rgb_corrected,depth_corrected,rgb_ori,depth_ori);
    rgb_ori.copyTo(Kinect2_image);

    int identity;
    identity=recognized();

    QString id= QString::number(identity, 10);
    QString filename="/home/zhang/brain_share/brain_robot/linzecai/ur5_cmake/facexml/";
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
    filename="/home/zhang/brain_share/brain_robot/linzecai/ur5_cmake/facexml/";
    filename+=id+ "/" + id+".jpg";
    QPixmap pixmap(filename);
    ui->patien_jpg->setPixmap(pixmap);
    ui->patien_jpg->show();


}

void MainWindow::readKinect2_Frame()
{
    grab_kinect2.Grab_image_KinectV2(rgb_corrected,depth_corrected,rgb_ori,depth_ori);
    rgb_ori.copyTo(Kinect2_image);
}

/*............ 自动跟踪函数.............*/
void  MainWindow::automation_track(double *track_positon)
{

    Map<MatrixXd> camera_state(tool_Position,4,4);
//    MatrixXd camera_state;
//    camera_state=MatrixXd::Identity(4,4);
//    camera_state(0,3)=tool_Position[12];
//    camera_state(1,3)=tool_Position[13];
//    camera_state(2,3)=tool_Position[14];
      AngleAxisd V3;
      Matrix3d T2;
//      for (int i = 0; i <  3; i++)
//      {
//          for(int j = 0; j <  3; j++)
//          {
//              T2(i,j)=camera_state(i,j);
//          }
//      }
//      V3.fromRotationMatrix(T2);
      Vector3d last;
//      last=V3.axis()*V3.angle();
//      if(!first_last)
//      {
//         last_time=last;
//         first_last=true;
//      }
//     qDebug()<<"artoolrx";
//     qDebug()<<last(0)<<" "<<last(1)<<" "<<last(2)<<endl;
     MatrixXd robot_real(4,4);
     MatrixXd robot_matrix(4,4);
     MatrixXd robot_calcu(4,4);
     if(camera_state(0,3)==0&&camera_state(1,3)==0&&camera_state(2,3)==0)
     {
         *track_positon=q_actual[12];track_positon++;
         *track_positon=q_actual[13];track_positon++;
         *track_positon=q_actual[14];track_positon++;
         *track_positon=q_actual[15];track_positon++;
         *track_positon=q_actual[16];track_positon++;
         *track_positon=q_actual[17];
     }
//     else if(last_time(0)-last(0)>0.05||last_time(1)-last(1)>0.05||last_time(2)-last(2)>0.05)
//     {
//         *track_positon=q_actual[12];track_positon++;
//         *track_positon=q_actual[13];track_positon++;
//         *track_positon=q_actual[14];track_positon++;
//         *track_positon=q_actual[15];track_positon++;
//         *track_positon=q_actual[16];track_positon++;
//         *track_positon=q_actual[17];
//     }
     else
     {
     robot_calcu=robot_camera.inverse()*camera_state;
     robot_real=robot_calcu*state_change.inverse();
     robot_matrix=robot_real*endT.inverse();
      for (int i = 0; i <  3; i++)
      {
           for(int j = 0; j <  3; j++)
             {
                T2(i,j)=robot_matrix(i,j);
             }
       }

     V3.fromRotationMatrix(T2);
     last=V3.axis()*V3.angle();
     *track_positon=robot_matrix(0,3);track_positon++;
     *track_positon=robot_matrix(1,3);track_positon++;
     *track_positon=robot_matrix(2,3);track_positon++;
     *track_positon=last(0);track_positon++;
     *track_positon=last(1);track_positon++;
     *track_positon=last(2);
     }
     //last_time=last;

}
/*............ 数据采集专用及手眼标定设置.............*/
void MainWindow::on_pushButton_3_clicked()
{
 /*............  神经网络数据采集.............*/
 /*  float F[6];
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
    }*/
 /*............  神经网络数据采集.............*/
/*............ 机械臂相机标定数据采集（棋盘格）............*/
//    double a = -329.44,b =-96.24, c =112.73, d = -156.94, e = -4.55, f = -68.65;
//    double q[6] = { a*3.14 / 180, b*3.14 / 180, c*3.14 / 180, d*3.14 / 180, e*3.14 / 180, f*3.14 / 180 };
//    double* T = new double[16];
//    forward(q, T);
//    double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
//    double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
//    double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;
//    double translate[16];
//    translate[0]=T00;translate[1]=T01;translate[2]=T02;translate[3]=T03;
//    translate[4]=T10;translate[5]=T11;translate[6]=T12;translate[7]=T13;
//    translate[8]=T20;translate[9]=T21;translate[10]=T22;translate[11]=T23;
//    translate[12]=0;translate[13]=0;translate[14]=0;translate[15]=1;
//    QFile data("/home/linzc/QT-workspace/untitled_test_2/facexml/robot_state.txt");
//    if (data.open(QFile::WriteOnly | QIODevice::Append))
//    {

//        QTextStream out1(&data);
//        for (int i=0;i<4;i++)
//          {
//            out1 <<translate[4*i]<<","<<translate[4*i+1]<<","<<translate[4*i+2]<<","<<translate[4*i+3]<<";";
//            qDebug()<<translate[4*i]<<","<<translate[4*i+1]<<","<<translate[4*i+2]<<","<<translate[4*i+3]<<";";
//          }
//        out1 <<endl;
//    }
/*............ 机械臂相机标定数据采集(二维码）.............*/
    double a = q_actual[0], b = q_actual[1], c =q_actual[2], d = q_actual[3], e = q_actual[4], f = q_actual[5];
    double q[6] = { a*3.14 / 180, b*3.14 / 180, c*3.14 / 180, d*3.14 / 180, e*3.14 / 180, f*3.14 / 180 };
    double* T = new double[16];
    forward(q, T);
    double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
    double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
    double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;
    double translate[16];
    translate[0]=T00;translate[1]=T01;translate[2]=T02;translate[3]=T03;
    translate[4]=T10;translate[5]=T11;translate[6]=T12;translate[7]=T13;
    translate[8]=T20;translate[9]=T21;translate[10]=T22;translate[11]=T23;
    translate[12]=0;translate[13]=0;translate[14]=0;translate[15]=1;
    Map<MatrixXd> camera_state(tool_Position,4,4);
//    QFile data("/home/linzc/QT-workspace/untitled_test_2/facexml/robot_state.txt");
//    if (data.open(QFile::WriteOnly | QIODevice::Append))
//    {

//        QTextStream out1(&data);
//       // for (int i=0;i<4;i++)
//          //{
//            out1 <<q_actual[12]<<","<<q_actual[13]<<","<<q_actual[14]<<","<<q_actual[15]<<","<<q_actual[16]<<","<<q_actual[17];
//            qDebug()<<q_actual[12]<<","<<q_actual[13]<<","<<q_actual[14]<<","<<q_actual[15]<<","<<q_actual[16]<<","<<q_actual[17]<<",";
//         // }
//        out1 <<endl;
//    }
//    QFile data1("/home/linzc/QT-workspace/untitled_test_2/facexml/camera_state.txt");
//    if (data1.open(QFile::WriteOnly | QIODevice::Append))
//    {

//        QTextStream out2(&data1);
//        for (int i=0;i<4;i++)
//          {
//            out2 <<translate[4*i]<<","<<translate[4*i+1]<<","<<translate[4*i+2]<<","<<translate[4*i+3]<<";";
//            qDebug()<<translate[4*i]<<","<<translate[4*i+1]<<","<<translate[4*i+2]<<","<<translate[4*i+3]<<";";
//          }
//        out2 <<endl;
//    }

//    for(int i=0;i<4;i++)
//    {

//        qDebug()<<camera_state(i,0)<<","<<camera_state(i,1)<<","<<camera_state(i,2)<<","<<camera_state(i,3)<<",";

//    }

    MatrixXd robot_real(4,4);
    double last_position[6];
    endT=MatrixXd::Identity(4,4);
    endT(0,3)=0.15589;endT(1,3)=0.00201;endT(2,3)=0.08609;
    Map<MatrixXd> robot_return(translate,4,4);
    MatrixXd robot_calculate(4,4);
    robot_real=robot_return.transpose();
//    robot_camera=camera_state*endT.inverse()*(robot_state.transpose()).inverse();
//    for(int i=0;i<4;i++)
//     {
//        qDebug()<<endT.inverse()(i,0)<<","<<endT.inverse()(i,1)<<","<<endT.inverse()(i,2)<<","<<endT.inverse()(i,3)<<",";}
//     for(int i=0;i<4;i++)
//       { qDebug()<<robot_state(i,0)<<","<<robot_state(i,1)<<","<<robot_state(i,2)<<","<<robot_state(i,3)<<",";}
    for(int i=0;i<4;i++)
    {
        qDebug()<<robot_real(i,0)<<","<<robot_real(i,1)<<","<<robot_real(i,2)<<","<<robot_real(i,3)<<",";
    }
//     robot_camera(0,0)=0.984937;robot_camera(0,1)=0.160144;robot_camera(0,2)=0.0652091;robot_camera(0,3)=0.165777;
//     robot_camera(1,0)=0.0777747;robot_camera(1,1)=-0.0734865;robot_camera(1,2)=-0.994259;robot_camera(1,3)=0.489426;
//     robot_camera(2,0)=-0.154433;robot_camera(2,1)=0.984354;robot_camera(2,2)=-0.0848347;robot_camera(2,3)=1.24091;
/*.................1280x720.....................................*/
//    robot_camera(0,0)=0.98612;robot_camera(0,1)=0.1601;robot_camera(0,2)=0.043973;robot_camera(0,3)=0.19138;
//    robot_camera(1,0)=0.053547;robot_camera(1,1)=-0.055985;robot_camera(1,2)=-0.99699;robot_camera(1,3)=0.48755;
//    robot_camera(2,0)=-0.15716;robot_camera(2,1)= 0.98551;robot_camera(2,2)=-0.063781;robot_camera(2,3)=1.2158;
//    robot_camera(3,0)=0;robot_camera(3,1)=0;robot_camera(3,2)=0;robot_camera(3,3)=1;
/*.................1920x1080.....................................*/
    robot_camera(0,0)=0.9846;robot_camera(0,1)=0.16752;robot_camera(0,2)=0.049983;robot_camera(0,3)=0.20112;
//    robot_camera(1,0)=0.054682;robot_camera(1,1)=-0.02355;robot_camera(1,2)=-0.99823;robot_camera(1,3)=0.52848;
//    robot_camera(2,0)=-0.16604;robot_camera(2,1)= 0.98559;robot_camera(2,2)=-0.032347;robot_camera(2,3)=1.1996;
//    robot_camera(3,0)=0;robot_camera(3,1)=0;robot_camera(3,2)=0;robot_camera(3,3)=1;
    //robot_calculate=robot_camera.inverse()*camera_state*endT.inverse();
/*.................shi di.....................................*/
    robot_camera(0,0)=0.9846;robot_camera(0,1)=0.16752;robot_camera(0,2)=0.049983;robot_camera(0,3)=0.21312;
    robot_camera(1,0)=0.054682;robot_camera(1,1)=-0.02555;robot_camera(1,2)=-0.99823;robot_camera(1,3)=0.52848;
    robot_camera(2,0)=-0.16604;robot_camera(2,1)= 0.98559;robot_camera(2,2)=-0.034347;robot_camera(2,3)=1.2196;
    robot_camera(3,0)=0;robot_camera(3,1)=0;robot_camera(3,2)=0;robot_camera(3,3)=1;
        //robot_calculate=robot_camera.inverse()*camera_state*endT.inverse();
    robot_calculate=robot_camera.inverse()*camera_state;
    if(m==0)
    {
//        for(int i=0;i<3;i++)
//        {
//            for(int j=0;j<3;j++)
//                robot_matrix(i,j)=robot_real(i,j);
//            robot_space(i)=robot_real(i,3);
//        }
       // bp.load("/home/zhang/brain_face_xml/bp_param.xml");
       // PDinitial();//PD initial force composation
        state_change=(robot_real*endT).inverse()*robot_calculate;
        if(camera_state(0,3)!=0||camera_state(1,3)!=0||camera_state(2,3)!=0)
        {
            ifopen=open_sensor();//open sensor
            m=1;
        }
        for(int i=0;i<4;i++)
        {
            qDebug()<<state_change(i,0)<<","<<state_change(i,1)<<","<<state_change(i,2)<<","<<state_change(i,3)<<",";
        }
    }
    automation_track(last_position);
    qDebug()<<last_position[0]<<","<<last_position[1]<<","<<last_position[2]<<","<<last_position[3]<<","<<last_position[4]<<","<<last_position[5]<<",";
    ui->poscontrol_x->setText((QString::number(last_position[0], 10, 4)));
    ui->poscontrol_y->setText((QString::number(last_position[1], 10, 4)));
    ui->poscontrol_z->setText((QString::number(last_position[2], 10, 4)));
    ui->poscontrol_rx->setText((QString::number(last_position[3], 10, 4)));
    ui->poscontrol_ry->setText((QString::number(last_position[4], 10, 4)));
    ui->poscontrol_rz->setText((QString::number(last_position[5], 10, 4)));
}
void MainWindow::on_level_clicked()
{
    QVector<double> temp(40);
    QVector<double> temp1(40);
 /*.................第一条曲线.....................................*/
    for(int i=0;i<40;i++)
    {
        temp[i] = i;
        temp1[i] =rand()%(45-48)+20;
    }
    ui->plot->addGraph();  //添加一条曲线
    ui->plot->graph(0)->setPen(QPen(Qt::red));//x是曲线序号，添加的第一条是0，设置曲线颜色
    ui->plot->graph(0)->setData(temp,temp1); //输出各点的图像，x和y都是QVector类
    ui->plot->graph(0)->setName("体温");
    //其原型如下：
    //void QCPGraph::setData(const QVector<double> &key, const QVector<double> &value)
    ui->plot->xAxis->setLabel("患者治疗前后30天内各项指标情况");   //x轴的文字
    ui->plot->yAxis->setLabel("含量值");   //y轴的文字
    ui->plot->xAxis->setRange(0,40);  //x轴范围
    ui->plot->yAxis->setRange(0,50);  //y轴范围
    ui->plot->replot(); //重绘
  /*.................第二条曲线.....................................*/
    for(int i=0;i<40;i++)
    {
        temp[i] = i;
        temp1[i] =rand()%(45-48)+10;
    }
    ui->plot->addGraph();  //添加一条曲线
    ui->plot->graph(1)->setName("血糖含量");
    ui->plot->graph(1)->setPen(QPen(Qt::blue));//x是曲线序号，添加的第一条是0，设置曲线颜色
    ui->plot->graph(1)->setData(temp,temp1); //输出各点的图像，x和y都是QVector类
    ui->plot->replot(); //重绘
  /*.................第三条曲线.....................................*/
    for(int i=0;i<40;i++)
    {
        temp[i] = i;
        temp1[i] =rand()%(45-48);
    }
    ui->plot->addGraph();  //添加一条曲线
    ui->plot->graph(2)->setName("脑电波强度");
    ui->plot->graph(2)->setPen(QPen(Qt::black));//x是曲线序号，添加的第一条是0，设置曲线颜色
    ui->plot->graph(2)->setData(temp,temp1); //输出各点的图像，x和y都是QVector类
    ui->plot->replot(); //重绘
    /*.................第四条曲线.....................................*/
     for(int i=0;i<40;i++)
      {
          temp[i] = i;
          temp1[i] =rand()%(45-48)+30;
      }
      ui->plot->addGraph();  //添加一条曲线
      ui->plot->graph(3)->setName("脑电流量");
      ui->plot->graph(3)->setPen(QPen(Qt::green));//x是曲线序号，添加的第一条是0，设置曲线颜色
      ui->plot->graph(3)->setData(temp,temp1); //输出各点的图像，x和y都是QVector类
      ui->plot->replot(); //重绘

}




/*************************************************************************
 * @brief on_pushButton_startKinect2_clicked
 *
 * Introduction:  start the kinect2 and Time schedule
 *
 *                It's the start wrote by zhang
 *
 ************************************************************************/
void MainWindow::on_pushButton_startKinect2_clicked()
{
    connect_kinect2=grab_kinect2.Initial_KinectV2_driver();
    track_head=false;
    target_last[6]={0};

    if(connect_kinect2)
    {
        ui->label_State_Kinect2->setText(tr("Kinect2 连接成功"));
        ui->pushButton_startKinect2->setEnabled(false);
        ui->pushButton_stopKinect2->setEnabled(true);

        grab_kinect2.Grab_image_KinectV2(rgb_corrected,depth_corrected,rgb_ori,depth_ori);

        Kinect2_Timer=new QTimer(this);
        connect(Kinect2_Timer,SIGNAL(timeout()),this,SLOT(Kinect2_cycle()));
        Kinect2_Timer->start(300);

        ui->lineEdit_handeyedata_path->setText(QString("/home/zhang/brain_face_xml/hand_eye_data/111.txt"));


        trackhead.init_track_head(rgb_corrected.cols,rgb_corrected.rows,grab_kinect2.cameraMatrixColor,grab_kinect2.cameraMatrixDepth);
    }
    else
    {
        ui->label_State_Kinect2->setText(tr("Kinect2 连接失败"));
        ui->pushButton_startKinect2->setEnabled(true);
        ui->pushButton_stopKinect2->setEnabled(false);
    }
}

void MainWindow::on_pushButton_stopKinect2_clicked()
{
   grab_kinect2.Close_KinectV2();
   connect_kinect2=false;
   ui->pushButton_startKinect2->setEnabled(true);
   ui->pushButton_stopKinect2->setEnabled(false);
   Kinect2_Timer->stop();
}


void MainWindow::Kinect2_cycle(void)
{
    grab_kinect2.Grab_image_KinectV2(rgb_corrected,depth_corrected,rgb_ori,depth_ori);

    bool detect_face;

    detect_face=trackhead.detect_face(rgb_corrected,depth_corrected);

    if(detect_face)
        ui->label_State_track->setText(tr("头部跟踪状态：正常"));
    else
        ui->label_State_track->setText(tr("头部跟踪失败,请病人停止运动!!!"));

    if(track_head && detect_face )
    {  
        double speed_now=sqrt(q_actual[6]*q_actual[6]+q_actual[7]*q_actual[7]+q_actual[8]*q_actual[8]);

        double target[6];
        trackhead.track_head(target,2);
/*.........................   添加力控用的内容(begin)....................................*/
        double q[6] = {q_actual[0]*3.14 / 180, q_actual[1]*3.14 / 180, q_actual[2]*3.14 / 180, q_actual[3]*3.14 / 180, q_actual[4]*3.14 / 180, q_actual[5]*3.14 / 180 };
        double T[16];
        double translate[9];
        double Fz;
        forward(q, T);
        double T02 = -T[0];  double T00 = T[1];  double T01 = T[2];
        double T12 = -T[4]; double T10 = T[5]; double T11 = T[6];
        double T22 = T[8]; double T20 = -T[9]; double T21 = -T[10];
        translate[0]=T00;translate[1]=T01;translate[2]=T02;translate[3]=T10;
        translate[4]=T11;translate[5]=T12;translate[6]=T20;translate[7]=T21;
        translate[8]=T22;
        Map<MatrixXd> dymMat(translate,3,3);//以列存储
        sensor_measure=force_measure();

        if(speed_now<0.01)
    {
            if(move_l==1)
            {
                Fz_current=(*(sensor_measure+2))/10;
            }
            //qDebug()<<"Fz_current:"<<Fz_current;
            //bp[0]=20*response.at<float>(0,0);bp[1]=response.at<float>(0,1)*20;bp[2]=response.at<float>(0,2)*20;bp[3]=response.at<float>(0,3)*1.2;bp[4]=response.at<float>(0,4)*1.2;bp[5]=response.at<float>(0,5)*1.2;
            Fz=(*(sensor_measure+2))/10;
            Fz=Fz-Fz_current+5;
            qDebug()<<"Fz_current:"<<Fz_current<<"Fz:"<<Fz;
            VectorXd Vd(6);//末端坐标系
            Vector3d v1;
            Vector3d v11;
            Vd=compensation(0,0,Fz,0,0,0);
            v1[0]=0;v1[1]=0;v1[2]=Vd[2];
            //qDebug()<<"Fz"<<Fz<<"Fz_curren"<<Fz_current<<"Z"<<Vd(2);
            v11=dymMat.transpose()*v1;
            qDebug()<<v11[0]<<" "<<v11[1]<<" "<<v11[2];

/*.........................   添加力控用的内容(end)....................................*/


            double error[3];

            qDebug()<<"target:"<<target[0]<<" "<<target[1]<<" "<<target[2];
            qDebug()<<"target_last:"<<target_last[0]<<" "<<target_last[1]<<" "<<target_last[2];
            error[0]=target[0]-target_last[0];
            error[1]=target[1]-target_last[1];
            error[2]=target[2]-target_last[2];

/*......................... 添加力控用的内容(begin)....................................*/
            if(fabs(error[0])<0.02&&fabs(error[1])<0.02&&fabs(error[2])<0.03)
            {
                if(fabs(v11[0])<0.035&&fabs(v11[1])<0.035&&fabs(v11[2])<0.035)
                {
                        target[0]=q_actual[12]+v11[0];
                        target[1]=q_actual[13]+v11[1];
                        target[2]=q_actual[14]+v11[2];
                        strInstruct=movelpos(target[0],target[1],target[2],q_actual[15],q_actual[16],q_actual[17],0.1,0.1);
                        int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
                        qDebug()<<strInstruct<<";"<<write_byte;
                        bool ifwritten=tcpClient->waitForBytesWritten();
                        move_l=0;
                        for(int i=0;i<6;i++)
                           target_last[i]=target[i];

                }
                qDebug()<<error[0]<<" "<<error[1]<<" "<<error[2];
            }
/*.........................添加力控用的内容(end)....................................*/
            else if(fabs(error[0])<0.2&&fabs(error[1])<0.2&&fabs(error[2])<0.2)
            {
                strInstruct=movelpos(target[0],target[1],target[2],target[3],target[4],target[5],0.1,0.1);
                int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());
                qDebug()<<strInstruct<<";"<<write_byte;
                //move_l=1;
                for(int i=0;i<6;i++)
                    target_last[i]=target[i];
            }
            else
            {
                qDebug()<<" the change is so big";
            }
        }

    }

    ui->label_Head_track->setPixmap(QPixmap::fromImage(Mat2QImage(rgb_corrected)));

    ui->label_Head_track->show();

    if(connect_ur5)
    {
        ui->lineEdit_PX->setText(QString::number(q_actual[12], 5, 3));
        ui->lineEdit_PY->setText(QString::number(q_actual[13], 5, 3));
        ui->lineEdit_PZ->setText(QString::number(q_actual[14], 5, 3));
        ui->lineEdit_Rx->setText(QString::number(q_actual[15], 4, 3));
        ui->lineEdit_Ry->setText(QString::number(q_actual[16], 4, 3));
        ui->lineEdit_Rz->setText(QString::number(q_actual[17], 4, 3));
    }
}


void MainWindow::on_pushButton_Stop_Trackhead_clicked()
{
    track_head=false;
}

void MainWindow::on_pushButton_Start_Trackhead_clicked()
{
    track_head=true;

}

void MainWindow::on_pushButton_get_weizi_clicked()
{
     fstream hand_eye_position;
     hand_eye_position.open(ui->lineEdit_handeyedata_path->text().toStdString());
     ifopen=open_sensor();//open sensor
     if(ifopen)
         qDebug()<<"force sensor open true";
     else
         qDebug()<<"force sensor open false";
     for(int i=0;i<4;i++)
     {
        for(int j=0;j<4;j++)
        {
          hand_eye_position>>trackhead.cam_to_armbase(i,j);
        }
     }
     hand_eye_position.close();

     trackhead.armbase_to_cam= trackhead.cam_to_armbase.inverse();

     Kinect2_Timer->stop();

     trackhead.get_end_to_base( q_actual[12],q_actual[13],q_actual[14],q_actual[15],q_actual[16],q_actual[17]);

     target_last[0]=q_actual[12];
     target_last[1]=q_actual[13];
     target_last[2]=q_actual[14];

     grab_kinect2.Grab_image_KinectV2(rgb_corrected,depth_corrected,rgb_ori,depth_ori);

     bool detected_face=trackhead.detect_face(rgb_corrected,depth_corrected);

     if(detected_face)
     {  
         std::cout<<"1111111"<<std::endl;
         double differ1[3],differ2[3];
         trackhead.get_illpoint_to_head(differ1,differ2);
         ui->lineEdit_target_X->setText(QString::number(differ1[0], 5, 2));
         ui->lineEdit_target_Y->setText(QString::number(differ1[1], 5, 2));
         ui->lineEdit_target_Z->setText(QString::number(differ1[2], 5, 2));
         ui->lineEdit_target_Rx->setText(QString::number(differ2[0], 5, 2));
         ui->lineEdit_target_Ry->setText(QString::number(differ2[1], 5, 2));
         ui->lineEdit_target_Rz->setText(QString::number(differ2[2], 5, 2));
     }

     Kinect2_Timer->start();

}

void MainWindow::on_pushButton_start_recons_clicked()
{
    kinfu_app.construction_flag=true;
    kinfu_app.execute();
}
void MainWindow::on_pushButton_6_clicked()
{
   kinfu_app.take_cloud();
}


void MainWindow::on_pushButton_end_recons_clicked()
{
   kinfu_app.construction_flag=false;
}





void MainWindow::on_close_system_clicked()
{
    double initial_pose[6];
    Kinect2_Timer->stop();
    robot_timer->stop();
    initial_pose[0]=-0.063;initial_pose[1]=-0.6099;initial_pose[2]=0.47698;initial_pose[3]=2.3474;initial_pose[4]=-2.0927;initial_pose[5]=0.0831;
    strInstruct=movelpos(initial_pose[0],initial_pose[1],initial_pose[2],initial_pose[3],initial_pose[4],initial_pose[5],0.1,0.1);
    int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());//toLatin1(); 将QString转为char
    qDebug()<<strInstruct<<";"<<write_byte;

}



