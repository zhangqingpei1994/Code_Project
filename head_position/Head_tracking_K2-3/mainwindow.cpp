
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <unistd.h>


#include <arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>
#include <fstream>
#include <iterator>


using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),  boardSize(11,8),filename_txt("handeye_data/TCP_position.txt"),pic_savePath("handeye_data/image"),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //机械臂模块
    if_ur5_connect=false;
    ui->lineEdit_Host->setText(QString("192.168.1.101"));
    ui->lineEdit_Port->setText(QString("30003"));
    ui->pushButton_sendPosition->setEnabled(false);       //发送TCP位姿按钮
    ui->pushButton_stop->setEnabled(false);               //停止机械臂按钮
    ui->pushButton_stop_2->setEnabled(false);
    connect(robot_ur5.tcpClient,SIGNAL(connected()),this,SLOT(connectok()));

    //tab1  变量初始化
    capture_count=0;                                      //记录采集的照片数量
    ui->label_PictureShow->setScaledContents(true);       //照片展示label
    ui->label_video->setScaledContents(true);             //video展示label
    ui->pushButton_take_picture->setEnabled(false);       //没连相机前take_picture按钮不能用
    ui->pushButton_close_Image->setEnabled(false);
    ui->lineEdit_datasave_path->setText(QString("/home/zhang/calibrate_data/"));

    //tab2  变量初始化
    Kinect2_tracking_show=false;
    track_head=false;
    ui->lineEdit_handeyedata_path->setText(QString("/home/zhang/calibrate_data/111.txt"));
    cyc_count=0;

    //两页共有的
    initial_kinect2=false;

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleTimeout())); //定时器到时响应
    timer->start(30);
}

MainWindow::~MainWindow()
{
    delete ui;
}


/******************************************************************************/
/*****              机械臂模块相关程序                                   *********/
/*****                                                                *********/
/******************************************************************************/
void MainWindow::on_pushButton_Connect_clicked()                 //连接机械臂按钮
{ 
    ui->label_State->setText(tr("连接中....."));
    ui->label_State_2->setText(tr("机械臂状态:连接中....."));
    robot_ur5.tcpClient->connectToHost(ui->lineEdit_Host->text(),ui->lineEdit_Port->text().toInt());
}

void MainWindow::on_pushButton_Connect_2_clicked()
{  
    on_pushButton_Connect_clicked() ;
}
void MainWindow::connectok()          //当连接服务器成功后，发出connecct()信号，开始传送文件
{
    if_ur5_connect=true;
    ui->label_State->setText(tr("连接成功，准备发送数据"));
    ui->label_State_2->setText(tr("机械臂连接成功"));
    ui->pushButton_sendPosition->setEnabled(true);                 //发送位姿按钮使能
    ui->pushButton_stop->setEnabled(true);                         //停机按钮
    ui->pushButton_stop_2->setEnabled(true);
    ui->pushButton_Connect->setEnabled(false);
    ui->pushButton_Connect_2->setEnabled(false);
    timer->start(30);                                              //一共出现了四次
}

void MainWindow::on_pushButton_stop_clicked()
{
     robot_ur5.tcpClient->disconnectFromHost();
     ui->label_State->setText(tr("已断开连接"));
     ui->label_State_2->setText(tr("机械臂状态:已断开连接"));
     ui->pushButton_Connect->setEnabled(true);                    //连接按钮恢复可用
     ui->pushButton_Connect_2->setEnabled(true);
     ui->pushButton_sendPosition->setEnabled(false);              //发送TCP位姿按钮
     ui->pushButton_stop->setEnabled(false);                      //停止机械臂按钮
     ui->pushButton_stop_2->setEnabled(false);                    //停止机械臂按钮
     if_ur5_connect=false;
}
void MainWindow::on_pushButton_stop_2_clicked()
{
    on_pushButton_stop_clicked();
}


void MainWindow::on_pushButton_sendPosition_clicked()
{
    double targetposition[6];
    double speed,acc;
    targetposition[0]=ui->lineEdit_PX->text().toDouble();
    targetposition[1]=ui->lineEdit_PY->text().toDouble();
    targetposition[2]=ui->lineEdit_PZ->text().toDouble();
    targetposition[3]=ui->lineEdit_Rx->text().toDouble();
    targetposition[4]=ui->lineEdit_Ry->text().toDouble();
    targetposition[5]=ui->lineEdit_Rz->text().toDouble();
    speed= ui->lineEdit_Speed->text().toDouble();
    acc  = ui->lineEdit_Acc->text().toDouble();
    robot_ur5.movelpos(targetposition[0],targetposition[1],targetposition[2],targetposition[3],targetposition[4],targetposition[5],acc,speed);
}


/*******************************************************************/
/*******************************************************************/
void MainWindow::handleTimeout(void)
{
   cyc_count++;
   if(initial_kinect2)
   {
       grab_kinect2.Grab_image_KinectV2(rgb_undis,depth_undis,rgb_ori,depth_ori);
       display_picture(rgb_undis,2,1);
       if(Kinect2_tracking_show)
       {
           points_3d_show.cloudViewer(rgb_undis,depth_undis);
           display_picture(rgb_undis,2,2);
           if(track_head && if_ur5_connect && cyc_count>2)
           {
              points_3d_show.get_Nose_Position(nose1,nose2);
              double speed_now=sqrt(data[6]*data[6]+data[7]*data[7]+data[8]*data[8]);
              robot_ur5.track_head(nose1,nose2,speed_now);
              cyc_count=0;
           }
       }
   }
   if(if_ur5_connect)         //更新机械臂数据
   {
        robot_ur5.robotstate(data);
        ui->lineEdit_Rx_read->setText(QString::number(data[12], 10,4));    //tab1中的机械臂数据更新
        ui->lineEdit_Ry_read->setText(QString::number(data[13], 10,4));
        ui->lineEdit_Rz_read->setText(QString::number(data[14], 10,4));
        ui->lineEdit_tx_read->setText(QString::number(data[15], 10,4));
        ui->lineEdit_ty_read->setText(QString::number(data[16], 10,4));
        ui->lineEdit_tz_read->setText(QString::number(data[17], 10,4));
        ui->lineEdit_Rx_2->setText(QString::number(data[12], 10,4));        //tab2中的机械臂数据更新
        ui->lineEdit_Ry_2->setText(QString::number(data[13], 10,4));
        ui->lineEdit_Rz_2->setText(QString::number(data[14], 10,4));
        ui->lineEdit_PX_2->setText(QString::number(data[15], 10,4));
        ui->lineEdit_PY_2->setText(QString::number(data[16], 10,4));
        ui->lineEdit_PZ_2->setText(QString::number(data[17], 10,4));
   }

   if(if_ur5_connect== false  && initial_kinect2==false)
   {
        timer->stop();
   }


}

/******************************************************************************/
/*****              相机 模块相关程序                                   *********/
/*****                                                                *********/
/*****                                                                *********/
/******************************************************************************/
void MainWindow::on_pushButton_START_Image_clicked()    //开启相机模块
{
    initial_kinect2 = grab_kinect2.Initial_KinectV2_driver();      //grab获得深度图像和彩色图像
    if(initial_kinect2)
      {
        ui->label_State_Kinect->setText(tr("初始化成功,可以采集图像"));
        ui->pushButton_take_picture->setEnabled(true);
        ui->pushButton_START_Image->setEnabled(false);
        ui->pushButton_close_Image->setEnabled(true);

        if(ui->lineEdit_datasave_path->text().isEmpty() == false) //设置保存数据的路径,默认为当前可执行程序同级的handeye_data文件夹
        {
          filename_txt=ui->lineEdit_datasave_path->text().toStdString()+"TCP_position.txt";
          pic_savePath=ui->lineEdit_datasave_path->text().toStdString()+"image";
        }

        outFile=ofstream (filename_txt.c_str(), ios_base::out);     //按新建或覆盖方式写入
        if (!outFile.is_open())
        {
            cout << "打开文件失败" << endl;
        }

        timer->start(30);                //一共出现了四次

      }
    else
       ui->label_State_Kinect->setText(tr("初始化失败,请检查硬件设备"));

}
void MainWindow::on_pushButton_close_Image_clicked()
{
      grab_kinect2.Close_KinectV2();
      ui->label_State_Kinect->setText(tr("已断开连接"));
      ui->pushButton_START_Image->setEnabled(true);
      ui->pushButton_close_Image->setEnabled(false);
      ui->pushButton_take_picture->setEnabled(false);       //没连相机前take_picture按钮不能用

      outFile.close();

      initial_kinect2=false;
}

void MainWindow::on_pushButton_take_picture_clicked()
{
      cv::Mat image_temp,image_gray;
      std::vector<cv::Point2f> imageCorners;
      rgb_undis.copyTo(image_temp);
      cv::cvtColor( image_temp, image_gray, CV_BGR2GRAY );
      bool found=findChessboardCorners(image_gray, boardSize, imageCorners);
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
        imwrite(pic_savePath+num_string,rgb_undis);
        if(if_ur5_connect)
         {
            outFile << data[15] << "\t"<< data[16] << "\t"<< data[17]<<std::endl ;                 //每列数据用 tab 隔开
            outFile << data[12] << "\t"<< data[13] << "\t"<< data[14]<<std::endl<<std::endl ;
         }
       }
      display_picture(image_temp,1,1);
      ui->lineEdit_Pic_num->setText(QString::number(capture_count,10));  //显示采集的数量
}




void MainWindow::on_pushButton_startKinect2_2_clicked()
{
    //一开始开着 先关掉Kinect2
    if(initial_kinect2)
         on_pushButton_stopKinect2_2_clicked();
    initial_kinect2=grab_kinect2.Initial_KinectV2_driver();
    if(initial_kinect2)   //Kinect2已经打开
    {
        Kinect2_tracking_show=true;  
        ui->label_State_Kinect_2->setText(tr("Kinect2 连接成功"));
        ui->label_State_Kinect->setText(tr("Kinect2 连接成功"));
        points_3d_show.init_Point_3d_show(rgb_undis.cols,rgb_undis.rows,grab_kinect2.cameraMatrixColor,grab_kinect2.cameraMatrixDepth);
        ui->pushButton_startKinect2_2->setEnabled(false);
        ui->pushButton_stopKinect2_2->setEnabled(true);
        timer->start(30);
    }
    else
     ui->label_State_Kinect_2->setText(tr("连接失败,请检查硬件设备"));
}
void MainWindow::on_pushButton_stopKinect2_2_clicked()
{
    grab_kinect2.Close_KinectV2();
    initial_kinect2=false;
    Kinect2_tracking_show=false;
    ui->label_State_Kinect->setText(tr("Kinect2 已断开连接"));
    ui->label_State_Kinect_2->setText(tr("Kinect2 已断开连接"));
    ui->pushButton_startKinect2_2->setEnabled(true);
    ui->pushButton_stopKinect2_2->setEnabled(false);

    ui->pushButton_START_Image->setEnabled(true);
    ui->pushButton_close_Image->setEnabled(false);
    ui->pushButton_take_picture->setEnabled(false);
}

void MainWindow::on_pushButton_get_weizi_clicked()
{
    //把机械臂和相机的相对位姿从txt中读取出来
    fstream hand_eye_position;
    hand_eye_position.open(ui->lineEdit_handeyedata_path->text().toStdString());
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
          hand_eye_position>>robot_ur5.handeye_data_Affin3d(i,j);  
        }
    }

     points_3d_show.cloudViewer(rgb_undis,depth_undis);
     points_3d_show.get_Nose_Position(nose1,nose2);

     std::cout<<"nose1: "<<nose1(0,0)<<" "<<nose1(1,0)<<" "<<nose1(2,0)<<std::endl;
     std::cout<<"nose2: "<<nose2(0,0)<<" "<<nose2(1,0)<<" "<<nose2(2,0)<<std::endl;

     Eigen::Vector4d nose1_in_ur5, nose2_in_ur5;
     nose1_in_ur5=robot_ur5.handeye_data_Affin3d*nose1;
     nose2_in_ur5=robot_ur5.handeye_data_Affin3d*nose2;

     std::cout<<"nose1_ur5: "<<nose1_in_ur5(0,0)<<" "<<nose1_in_ur5(1,0)<<" "<<nose1_in_ur5(2,0)<<std::endl;
     std::cout<<"nose2_ur5: "<<nose2_in_ur5(0,0)<<" "<<nose2_in_ur5(1,0)<<" "<<nose2_in_ur5(2,0)<<std::endl;

     std::cout<<data[12]<<" "<<data[13]<<" "<<data[14]<<std::endl;

     robot_ur5.differ1(0,0)=data[12]-nose1_in_ur5(0,0);
     robot_ur5.differ1(1,0)=data[13]-nose1_in_ur5(1,0);
     robot_ur5.differ1(2,0)=data[14]-nose1_in_ur5(2,0);
     robot_ur5.differ1(3,0)=0;
     robot_ur5.differ2(0,0)=data[12]-nose2_in_ur5(0,0);
     robot_ur5.differ2(1,0)=data[13]-nose2_in_ur5(1,0);
     robot_ur5.differ2(2,0)=data[14]-nose2_in_ur5(2,0);
     robot_ur5.differ2(3,0)=0;

     std::cout<<"differ1: "<<robot_ur5.differ1(0,0)<<" "<<robot_ur5.differ1(1,0)<<" "<<robot_ur5.differ1(2,0)<<std::endl;
     std::cout<<"differ2: "<<robot_ur5.differ2(0,0)<<" "<<robot_ur5.differ2(1,0)<<" "<<robot_ur5.differ2(2,0)<<std::endl;


     robot_ur5.Rx=data[15];
     robot_ur5.Ry=data[16];
     robot_ur5.Rz=data[17];

     hand_eye_position.close();
}

void MainWindow::on_pushButton_trackHead_clicked()
{
    track_head=true;
}

void MainWindow::on_pushButton_stoptrack_clicked()
{
    track_head=false;
}

/*****************************************************
 * @brief MainWindow::display_picture
 * 这个函数不会对传进去的参数有任何操作,也不会对函数之外的
 * 任何变量进行操作
 * @param image_dis:要在界面上显示的图片
 * @param type:  picture  or  video
 ****************************************************/
void MainWindow::display_picture(cv::Mat image_dis,int type, int page)   //显示图片，type指明是显示图片还是视频序列
{

         cv::Mat rgb,image_temp;
         QImage img;
         image_dis.copyTo(image_temp);

         //把Mat转换成QImage格式
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
        case 1:          //显示图片
           ui->label_PictureShow->setPixmap(QPixmap::fromImage(img));
           //ui->label_PictureShow->resize(ui->label_PictureShow->pixmap()->size());
           ui->label_PictureShow->show();
           break;
        case 2:           //显示视频
            if(page==1)
           {
             ui->label_video->setPixmap(QPixmap::fromImage(img));
             ui->label_video->show();
           }
            else
           {
             ui->label_headtracking->setPixmap(QPixmap::fromImage(img));
             ui->label_headtracking->show();
           }
            break;
        }
}







