
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <unistd.h>
#include "CameraCalibrator.h"
#include <eigen3/Eigen/Geometry>
#include<arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

#define picture 1
#define video 2
#define pi  3.141592653





MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),boardSize(11,8),filename_yaw("/home/zhang/TransformPairsInput.yml"),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    timer = new QTimer(this);                           //开启定时器更新相机视频信息


    Camera_connected=false;                                     //默认相机是不打开的
    capture_count=0;
    ui->label_PictureShow->setScaledContents(true);       //照片展示label
    ui->label_video->setScaledContents(true);             //video展示label
    pic_savePath="picture_data/image";                  //照片保存路径
    ui->pushButton_take_picture->setEnabled(false);       //没连相机前take_picture按钮不能用
    ui->pushButton_cali->setEnabled(false);               //标定按钮失能



    tcpClient = new QTcpSocket(this); 
    ur5_data_to_write=cv::Mat_<double>::zeros(4,4);      //ur5机械臂TCP末端位姿状态
    ui->lineEdit_Host->setText(QString("192.168.1.100"));
    ui->lineEdit_Port->setText(QString("30003"));
    ui->pushButton_sendPosition->setEnabled(false);       //发送TCP位姿按钮
    ui->pushButton_stop->setEnabled(false);               //停止机械臂按钮


    connect(tcpClient,SIGNAL(connected()),this,SLOT(connectok()));
    connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout())); //定时器到时响应

    timer->start (30);         //30ms刷新一次


}

MainWindow::~MainWindow()
{
    delete ui;
}




/******************************************************************************/
/*****              机械臂模块相关程序                                   *********/
/*****                                                                *********/
/*****                                                                *********/
/******************************************************************************/



void MainWindow::on_pushButton_Connect_clicked()                 //连接机械臂按钮
{
    ui->pushButton_Connect->setEnabled(false);
    ui->label_State->setText(tr("连接中....."));

    tcpClient->connectToHost(ui->lineEdit_Host->text(),ui->lineEdit_Port->text().toInt());
}

void MainWindow::connectok()    //当连接服务器成功后，发出connecct()信号，开始传送文件
{
    if_ur5_connect=true;
    ui->label_State->setText(tr("连接成功，准备发送数据"));
    ui->pushButton_sendPosition->setEnabled(true);               //发送位姿按钮使能
    ui->pushButton_stop->setEnabled(true);                      //停机按钮


}

void MainWindow::on_pushButton_sendPosition_clicked()
{
    targetposition1[0]=ui->lineEdit_PX->text().toDouble();
    targetposition1[1]=ui->lineEdit_PY->text().toDouble();
    targetposition1[2]=ui->lineEdit_PZ->text().toDouble();
    targetposition1[3]=ui->lineEdit_Rx->text().toDouble();
    targetposition1[4]=ui->lineEdit_Ry->text().toDouble();
    targetposition1[5]=ui->lineEdit_Rz->text().toDouble();
    speed= ui->lineEdit_Speed->text().toDouble();
    acc  = ui->lineEdit_Acc->text().toDouble();

    movelpos(targetposition1[0],targetposition1[1],targetposition1[2],targetposition1[3],targetposition1[4],targetposition1[5],0.1,0.5);
    QString strInstruct=commondqueue;
    int write_byte=tcpClient->write(strInstruct.toLatin1(),strInstruct.length());     //toLatin1(); 将QString转为char
      qDebug()<<strInstruct<<";"<<write_byte;                                       //打印信息
}

void MainWindow::movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v)
{
    commondqueue="def f():\n\t";     //\t tab键

    QString strInstruct=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t").arg(x).arg(y).arg(z).arg(Rx).arg(Ry).arg(Rz);  //用字符串变量参数依次替代字符串中最小数值
    strInstruct+=QString("movel(wp1,a=%1,v=%2)\n\t").arg(a).arg(v);
    if(strInstruct.right(1).toLatin1() != "\n")   //right：最右边  toLatin1是一种编码格式
           strInstruct.append('\n');
    commondqueue+=strInstruct;
    commondqueue+="end\n";

}



void MainWindow::robotstate(double*q_actual)
{
    uint64_t unpack_to;
    uint16_t offset = 0;
    char  *buf;
    int len;
    uint64_t q;
    int bytes_read;
    QByteArray dataread;
    offset=0;
    dataread.resize(tcpClient->bytesAvailable());
    bytes_read=tcpClient->read(dataread.data(),dataread.size());
    //dataread=tcpClient->readAll();
    if(bytes_read%1060==0&&bytes_read!=0)
    {
         buf=dataread.data()+bytes_read-1060;
         memcpy(&len, &buf[0], sizeof(len));
         offset += sizeof(len);

         len=ntohl(len);                                  //是将一个无符号长整形数从网络字节顺序转换为主机字节顺序， ntohl()返回一个以主机字节顺序表达的数。

         memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
         unpack_to = be64toh(unpack_to);//函数返回一个大字节排序整数转换为系统的本机字节顺序。返回值将与大端系统上的参数相同。所谓的大端模式（Big-endian），是指数据的高字节，保存在内存的低地址中，而数据的低字节，保存在内存的高地址中
         double k;
         memcpy(&k, &unpack_to, sizeof(k));
         offset += sizeof(unpack_to)+30*sizeof(double);
         for(int i=0;i<6;i++)
         {
            memcpy(&q, &buf[offset], sizeof(double));
            q=be64toh(q);//字节序转换

            memcpy(&q_actual[i], &q, sizeof(q));
            //qDebug()<<q_actual[i];
            q_actual[i]=q_actual[i]*180/pi;
            offset += sizeof(double);
          }

        for(int i=0;i<6;i++)
        {
           memcpy(&q, &buf[offset], sizeof(double));
           q=be64toh(q);//字节序转换

           memcpy(&q_actual[i+6], &q, sizeof(q));
           //qDebug()<<q_speed[i];
           offset += sizeof(double);
         }
        offset+=sizeof(double)*11;

        for(int i=0;i<6;i++)
        {
          memcpy(&q, &buf[offset], sizeof(double));
          q=be64toh(q);//字节序转换
          memcpy(&q_actual[i+11], &q, sizeof(q));
          offset += sizeof(double);
        }
       for(int i=0;i<6;i++)
       {
          memcpy(&q, &buf[offset], sizeof(double));
          q=be64toh(q);//字节序转换
          memcpy(&q_actual[i+17], &q, sizeof(q));
          offset += sizeof(double);
       }
}
}

void MainWindow::save_TCP_Position(void)
{

    Mat R_vector = (Mat_<double>(3,1) << data[15], data[16],data[17]);
    Mat t_vector=  (Mat_<double>(3,1) << data[12], data[13],data[14]);

    TCP_Rotation_vector.push_back(R_vector);
    TCP_t_vector.push_back(t_vector);

}

void MainWindow::on_pushButton_stop_clicked()              //停机按钮
{
     tcpClient->disconnectFromHost();
     ui->label_State->setText(tr("已断开连接"));
     timer->stop();
     ui->pushButton_Connect->setEnabled(true);

}


/*******************************************************************/
/*******************************************************************/



void MainWindow::handleTimeout(void)    //40ms 中断周期
{

    if(Camera_connected)
    {

        bool has_frame = capture.grab(depth, image);      //grab获得深度图像和彩色图像 --
        if (!has_frame)
        {
          std::cout << "Can't grab" << std::endl;
          return;
        }

       display_picture(image,video);
    }


    /***************************************************************/
    /***********     实时更新机械臂位姿，并记录下来       ***************/
    /***************************************************************/
   if(if_ur5_connect)
   {

        robotstate(data);
        double temp1,temp2,temp3,temp4,temp5,temp6;

        temp1=data[12];
        temp2=data[13];
        temp3=data[14];
        temp4=data[15];
        temp5=data[16];
        temp6=data[17];

        ui->lineEdit_Rx_read->setText(QString::number(temp1, 10,4));
        ui->lineEdit_Ry_read->setText(QString::number(temp2, 10,4));
        ui->lineEdit_Rz_read->setText(QString::number(temp3, 10,4));
        ui->lineEdit_tx_read->setText(QString::number(temp4, 10,4));
        ui->lineEdit_ty_read->setText(QString::number(temp5, 10,4));
        ui->lineEdit_tz_read->setText(QString::number(temp6, 10,4));
   }


}




/******************************************************************************/
/*****              相机臂模块相关程序                                   *********/
/*****                                                                *********/
/*****                                                                *********/
/******************************************************************************/
void MainWindow::on_pushButton_START_Image_clicked()    //开启相机模块
{

    capture.open (0);  //相机初始化采集照片
    Camera_connected=true;
    ui->pushButton_take_picture->setEnabled(true);
    //if(if_ur5_connect)
    ui->pushButton_cali->setEnabled(true);


}


void MainWindow::on_pushButton_take_picture_clicked()         //采集标定图片,  并获得当前TCP在基坐标系下的位姿
{


      Mat image_temp,image_gray;
      std::vector<cv::Point2f> imageCorners;
      image.copyTo(image_temp);

      cvtColor( image_temp, image_gray, CV_BGR2GRAY );
      //下面这三句话是为了画出来找到的角点
      bool found=findChessboardCorners(image_gray, boardSize, imageCorners);
      cv::cornerSubPix(image_gray, imageCorners, cv::Size(5,5),  cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30, 0.1));
      ui->lineEdit_Cor_num->setText(QString::number(imageCorners.size(),10));      //显示采集的角点的数量

      if(imageCorners.size() == boardSize.area())                                  //检测到一定数量的角点后才会画出来
      {

         capture_count++;


        cv::drawChessboardCorners(image_temp, boardSize, imageCorners, found);

        std::string num_string=to_string(capture_count);

        if(capture_count<10)
          num_string="0"+num_string+".jpg";
        else
           num_string=num_string+".jpg";

        imwrite(pic_savePath+num_string,image);                                //把image_temp对应的图片存起来

        cout<<"aaa"<<endl;

        filename.push_back(pic_savePath+num_string);

       }

      display_picture(image_temp,picture);

      ui->lineEdit_Pic_num->setText(QString::number(capture_count,10));  //显示采集的数量


    if(if_ur5_connect)
     {
        save_TCP_Position();
     }
}


void MainWindow::display_picture(Mat  image_dis,int type)   //显示图片，type指明是显示图片还是视频序列
{

         Mat rgb,image_temp;
         QImage img;

         image_dis.copyTo(image_temp);


         //把Mat转换成AImage格式
        if(image_temp.channels() == 3)
        {
            cvtColor(image_temp,rgb,CV_BGR2RGB);
            img = QImage((const uchar*)(rgb.data),
                         rgb.cols,rgb.rows,
                         rgb.cols*rgb.channels(),
                         QImage::Format_RGB888);
        }
        else                     // gray image
        {
            img = QImage((const uchar*)(image_temp.data),
                         image_temp.cols,image_temp.rows,
                         image_temp.cols*image_temp.channels(),
                         QImage::Format_Indexed8);
        }

        //显示图片或者视频显示
        switch(type)
       {
        case picture:          //显示图片
           ui->label_PictureShow->setPixmap(QPixmap::fromImage(img));
           //ui->label_PictureShow->resize(ui->label_PictureShow->pixmap()->size());
           ui->label_PictureShow->show();
           break;
        case video:           //显示视频
            ui->label_video->setPixmap(QPixmap::fromImage(img));
            //ui->label_PictureShow->resize(ui->label_PictureShow->pixmap()->size());
            ui->label_video->show();
            break;
        }
}



void MainWindow::on_pushButton_cali_clicked()               //标定相机得到每一张图片的外参矩阵
{
  int picture_num=cameraCali.addChessboardPoints(filename,boardSize);

  cout<<picture_num<<endl;

  Size imageSize=image.size();
  cameraCali.calibrate(imageSize);

  cameraCali.get_Transform_matrix(image_Rotation_vector,image_t_vector);

  Write_data_to_yaw();

  //system("gnome-terminal -x bash -c 'roslaunch handeye_calib_camodocal handeye_example.launch'&");
  //system("gnome-terminal -x  '^c'");

}


void MainWindow::Write_data_to_yaw(void)
{

    Mat_<double> data_to_write_camera  = cv::Mat_<double>::zeros(4,4);
    Mat_<double> data_to_write_ur5  = cv::Mat_<double>::zeros(4,4);

    Mat Matrix_temp_camera;
    Mat Matrix_temp_ur5;


    cv::FileStorage fs(filename_yaw, cv::FileStorage::WRITE);

    int frameCount=image_Rotation_vector.size();

    fs << "frameCount" << frameCount;

    for(int i=0;i<image_Rotation_vector.size();i++)
    {
        std::stringstream ss1;
        std::stringstream ss2;

        Rodrigues(image_Rotation_vector[i],Matrix_temp_camera);
        Rodrigues(TCP_Rotation_vector[i],  Matrix_temp_ur5);

        cout<<i<<" frame"<<endl;
        cout<<"image_Rotation_vector.t:"<<image_Rotation_vector[i].t()<<endl;
        cout<<"image_t_vector.t:"<<image_t_vector[i].t()<<endl;
        cout<<"Matrix_temp_camera:"<<Matrix_temp_camera<<endl;
        cout<<"TCP_Rotation_vector.t:"<<TCP_Rotation_vector[i].t()<<endl;
        cout<<"TCP_t_vector.t:"<<TCP_t_vector[i].t()<<endl;
        cout<<"Matrix_temp_ur5:"<<Matrix_temp_ur5<<endl;




        ss1 << "T1_" << i;
        ss2 << "T2_" << i;

        for(int j=0;j<3;j++)
          for(int jj=0;jj<3;jj++)
            {
              data_to_write_camera.at<double>(j,jj)=Matrix_temp_camera.at<double>(j,jj);
              data_to_write_ur5.at<double>(j,jj)=Matrix_temp_ur5.at<double>(j,jj);

            }

        data_to_write_camera.at<double>(0,3)=image_t_vector[i].at<double>(0,0);
        data_to_write_camera.at<double>(1,3)=image_t_vector[i].at<double>(1,0);
        data_to_write_camera.at<double>(2,3)=image_t_vector[i].at<double>(2,0);
        data_to_write_camera.at<double>(3,3)=1.0;

        data_to_write_ur5.at<double>(0,3)=TCP_t_vector[i].at<double>(0,0);
        data_to_write_ur5.at<double>(1,3)=TCP_t_vector[i].at<double>(1,0);
        data_to_write_ur5.at<double>(2,3)=TCP_t_vector[i].at<double>(2,0);
        data_to_write_ur5.at<double>(3,3)=1.0;



        //data_to_write_camera=data_to_write_camera.inv();

        fs << ss1.str() <<data_to_write_ur5;
        fs << ss2.str() << data_to_write_camera;


     }
}






















void MainWindow::on_pushButton_clicked()
{

;
}



