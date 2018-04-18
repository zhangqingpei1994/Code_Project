#include"cameraposition.h"
#include<QDebug>
double tool_Position[16]={0};
void init_traker(TrackerSingleMarker *tracker,int maker_width)
{
    const bool useBCH=false;
    //设置图像格式为灰度图PIXEL_FORMAT_ABGR
    tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    // 载入自己相机的标定内参
    if (!tracker->init("/home/linzc/QT/2-4/untitled_test/Mydatas/lojiInAir.cal", 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");

    }
    // 定义二维码大小，可定义成实际的二维码边框边长（以mm为单位）
    tracker->setPatternWidth(maker_width);
    // the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(useBCH ? 0.125 : 0.25);
    //如果采用自动阈值
    tracker->activateAutoThreshold(true);
    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    //这里图像像素为640 × 480,考虑畸变
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_STD);

}
void get_visual_pose(Mat frame_src,Mat* frame_dst,TrackerSingleMarker *tracker)
{


//    Mat frame_in;
//    Mat frame_out;
    double traslation_Matrix[16]={0};
    double eular[3]={0};
//    Mat frame_in;
//    Mat frame_out;
    Positing position;

//    frame_src.copyTo(frame_in);
//    frame_in.copyTo(frame_out);

   //position.newCalcuate(frame_src,*frame_dst,*tracker,traslation_Matrix);//main
    position.oldCalcuate(frame_src,*frame_dst,tracker,traslation_Matrix);//main
//    for (int i = 0; i < 16; i++)
//      std::cout << traslation_Matrix[i] << " ";
//    std::cout << std::endl << std::endl;
    rotToEular_rad(traslation_Matrix,eular);

}
int Positing::oldCalcuate(Mat src,Mat &dst,TrackerSingleMarker * tracker,double *outputMatrix)
{

    ARToolKitPlus::ARMarkerInfo* markInfo;

    cv::Mat image_gray(src.rows,src.cols,CV_8U);
    int rows, cols,cols_bits;
     rows = src.rows;
     cols = src.cols;
     cols_bits = cols*src.channels();
    for (int i = 1; i < rows-1; i++)
        {
            uchar *data = src.ptr<uchar>(i);
            uchar *data1 = image_gray.ptr<uchar>(i);
            for (int j = 1; j < cols-1; j++)
            {
                data1[j] = data[3 * j - 2];// -100;
                //data1[j] = data1[j]*1.7 ;
            }
        }
    //src.copyTo(image_gray);
//    if(image_gray.channels()==3)
//    {
//        cv::cvtColor(src,image_gray,cv::COLOR_BGR2GRAY);
//    }
    Positing::tryAutoThreshold(image_gray,tracker,markInfo,outputMatrix,100);

    Positing::drawMarkerInfo(dst,markInfo);

    return 0;
}
//设置自动阈值模式，默认尝试100次, 这里的 markInfo为指针类型的引用传递
int Positing::tryAutoThreshold(cv::Mat src, TrackerSingleMarker* tracker,ARMarkerInfo* & markInfo,double*outputMatrix,int tryNum)
{
    cv::Mat dst;
    src.copyTo(dst);
    if(dst.data == NULL)
    {
        printf("no image ");
        return 1;
    }
    if(dst.channels()==3)
    {
        printf("read color image,convert gray image ");
        cv::cvtColor(dst,dst,cv::COLOR_BGR2GRAY);
    }
    int markNum=0;
    std::vector<int> markerId; //store the transformation matrix information

    //tracker->activateAutoThreshold(true);
    tracker->setNumAutoThresholdRetries(tryNum);

    markerId= tracker->calc(dst.data,&markInfo,&markNum);

    tracker->selectBestMarkerByCf();

    for(int i=0 ; i<16 ;i++)
    {
        outputMatrix[i]=tracker->getModelViewMatrix()[i];//按列取出
       // qDebug()<<outputMatrix[i];
        //printf("%.2f  %s", tracker->getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    }
    printf("\n\n");

    return 0;

}

std::string  int2str(int num)
{

    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}

void Positing::drawMarkerInfo(cv::Mat &image, ARToolKitPlus::ARMarkerInfo*  markInfo)
{

    cv::Point center,corner0,corner1,corner2,corner3 ;
    center=cv::Point(markInfo->pos[0],markInfo->pos[1]);
    //qDebug()<<"X="<<markInfo->pos[0];
    //qDebug()<<"Y="<<markInfo->pos[1];
    corner0=cv::Point(markInfo->vertex[(4-markInfo->dir+0)%4][0],markInfo->vertex[(4-markInfo->dir+0)%4][1]);
    corner1=cv::Point(markInfo->vertex[(4-markInfo->dir+1)%4][0],markInfo->vertex[(4-markInfo->dir+1)%4][1]);
    corner2=cv::Point(markInfo->vertex[(4-markInfo->dir+2)%4][0],markInfo->vertex[(4-markInfo->dir+2)%4][1]);
    corner3=cv::Point(markInfo->vertex[(4-markInfo->dir+3)%4][0],markInfo->vertex[(4-markInfo->dir+3)%4][1]);

    cv::line(image,corner0,corner1,CV_RGB(255,0,0),1,8);
    cv::line(image,corner1,corner2,CV_RGB(255,0,0),1,8);
    cv::line(image,corner2,corner3,CV_RGB(255,0,0),1,8);
    cv::line(image,corner3,corner0,CV_RGB(255,0,0),1,8);
    cv::rectangle(image,cv::Point(center.x-1, center.y-1),cv::Point(center.x+1, center.y+1),CV_RGB(0,255,0),1,8); //圈取图像中心点

    //string dir_str = int2str(one_mark.dir);
    std::string tx0 = "0";
    std::string tx1 = "1";
    std::string tx2 = "2";
    std::string tx3 = "3";

    cv::putText(image,tx0,corner0,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx1,corner1,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx2,corner2,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::putText(image,tx3,corner3,CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));


    std::string text ="Id:"+ int2str(markInfo->id);
    cv::putText(image,text,cv::Point(center.x+80,center.y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

}
void rotToEular_rad(double *rot, double *eular)
{
    double R11=rot[0];
    double R21=rot[4];
    double R31=rot[8];
    double R32=rot[9];
    double R33=rot[10];
    double pitch=asin(R31);
    double yaw = atan2(R32,R33);
    double roll =atan2(R21,R11);
    rot[12]=rot[12]/1000;
    rot[13]=rot[13]/1000;
    rot[14]=rot[14]/1000;
    eular[0]=pitch;
    eular[1]=yaw;
    eular[2]=roll;
    for(int i=0 ; i<16 ;i++)
    {
        tool_Position[i]=rot[i];//按列取出
        //qDebug()<<tool_Position[i];
        //printf("%.2f  %s", tracker->getModelViewMatrix()[i], (i % 4 == 3) ? "\n  " : "");
    }
}
void rotToEular_angle(double *rot, double *eular)
{
    double R11=rot[0];
//    double R21=rot[4];
//    double R31=rot[8];
//    double R32=rot[9];
//    double R33=rot[10];
//    double pitch=asin(R31);
//    double yaw = atan2(R32,R33);
//    double roll =atan2(R21,R11);
    //pitch=pitch/pi*180;
    //yaw=yaw/pi*180;
   // roll=roll/pi*180;
   // eular[0]=pitch;
   // eular[1]=yaw;
   // eular[2]=roll;
}
QImage  Mat2QImage(cv::Mat cvImg)//图片格式转换，opencv转为QT
{
    QImage qImg;
    if(cvImg.channels()==3)                             //3 channels color image
    {

        cv::cvtColor(cvImg,cvImg,CV_BGR2RGB);
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols, cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    else if(cvImg.channels()==1)                    //grayscale image
    {
        qImg =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_Indexed8);
    }
    else
    {
            qImg =QImage((const unsigned char*)(cvImg.data),
                        cvImg.cols,cvImg.rows,
                        cvImg.cols*cvImg.channels(),
                        QImage::Format_RGB888);
        }

        return qImg;

    }
