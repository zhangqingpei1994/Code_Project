#include"mythread.h"
#include"UR5_kin/ur5_kin.h"
#define pi 3.1415926;
forceThread::forceThread(QObject *parent) :
    QThread(parent)
{
    moveToThread(this);
}
forceThread::~forceThread()
{
    delete tcpClient2;
    tcpClient2=NULL;
}

void forceThread::run()
{
    tcpClient2= new QTcpSocket(this);
    double *forcevalue;
    double ft[6];
    double newtarget[6];
    double oldtarget[6];
    double translate[9];
    bool firstmove=true;
    Vector3d rc;
    rc(0)=0.154;rc(1)=0;rc(2)=0.045;
    CvANN_MLP bp;
    bp.load("../textxml/bp_param.xml");
    PDinitial();
    Mat sample(1,6,CV_32FC1);
    Mat response;

    tcpClient2->connectToHost("192.168.1.103",30003,QTcpSocket::ReadWrite);
    qDebug()<<"tcp2连接。";
    if(tcpClient2->waitForConnected(500))
    {
        qDebug()<<"tcp2连接成功。";
        ifok=true;
    }
     else
        qDebug()<<"连接失败";

//    /*............神经网络训练专用.............*/
//    //bp_train()e ft[6];

    while(1)
    {
       if(ifok)
       {
           robotstate(q_actual);
           for(int i=0;i<6;i++)
           {
                sample.at<float>(0,i)=q_actual[i];
                // qDebug()<<q_actual[i];
            }
            double Fx=0,Fy=0,Fz=0;
            double Tx=0,Ty=0,Tz=0;
            Vector3d v1;
            Vector3d v2;
            Vector3d v11;
            Vector3d v22;
            VectorXd Vd(6);//末端坐标系
            AngleAxisd  V2;
            VectorXd v(3);
            Matrix3d rt;
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
            rt=dymMat.transpose();
            V2.fromRotationMatrix(rt);
            v=V2.axis()*V2.angle();
            //q_actual[15]=v(0);q_actual[16]=v(1);q_actual[17]=v(2);
            //qDebug()<<q_actual[15]<<" "<<q_actual[16]<<" "<<q_actual[17];
            bool ifstop=false;
            if((fabs(q_actual[18])<0.001)&&(fabs(q_actual[19])<0.001)&&(fabs(q_actual[20])<0.001)&&(fabs(q_actual[21])<0.001)&&(fabs(q_actual[22])<0.001)&&(fabs(q_actual[23])<0.001))
                ifstop=true;
            if(ifopen&&ifstop)
            {
                forcevalue=force_measure();
                ft[0]=(*forcevalue)/10-1.7;ft[1]=(*(forcevalue+1))/10+0.5;ft[2]=(*(forcevalue+2))/10+12.3;
                ft[3]=(*(forcevalue+3))/1000-0.033;ft[4]=(*(forcevalue+4))/1000-0.239;ft[5]=(*(forcevalue+5))/1000-0.037;
                qDebug()<<ft[0]<<" "<<ft[1]<<" "<<ft[2]<<" "<<ft[3]<<" "<<ft[4]<<" "<<ft[5]<<" "<<" \n";
                bp.predict(sample,response);
                response = response / 1.63;
                qDebug()<<response.at<float>(0,0)*20<<" "<<response.at<float>(0,1)*20<<" "<<response.at<float>(0,2)*20<<" "<<response.at<float>(0,3)*1.2<<" "<<response.at<float>(0,4)*1.2<<" "<<response.at<float>(0,5)*1.2;
                double bp[6];
                bp[0]=20*response.at<float>(0,0);bp[1]=response.at<float>(0,1)*20;bp[2]=response.at<float>(0,2)*20;bp[3]=response.at<float>(0,3)*1.2;bp[4]=response.at<float>(0,4)*1.2;bp[5]=response.at<float>(0,5)*1.2;
                Fx=ft[0]-bp[0];Fy=ft[1]-bp[1];Fz=ft[2]-bp[2]+8;

                Vector3d ef;
                Vector3d et;
                ef(0)=Fx;ef(1)=Fy;ef(2)=Fz-8;
                et=rc.cross(ef);
                //Tx=ft[3]-bp[3]-et(0);Ty=ft[4]-bp[4]-et(1);Tz=ft[5]-bp[5]-et(2);//}
                Tx=ft[3]-bp[3]+Fy*0.045;Ty=ft[4]-bp[4]+(Fz-5)*0.154-Fx*0.045;Tz=ft[5]-bp[5]-Fy*0.154;
                // else
                //Tx=ft[3]-bp[3];Ty=ft[4]-bp[4];Tz=ft[5]-bp[5];
                //qDebug()<<Tz;
                Vd=compensation(Fx,Fy,Fz,Tx,Ty,Tz);
                v1(0)=Vd(0);
                v1(1)=Vd(1);
                v1(2)=Vd(2);
                v2(0)=-Vd(3);//v2(0)=0;//v2(0)=Vd(3);
                v2(1)=Vd(4);//v2(1)=0;//v2(1)=Vd(4);
                v2(2)=-Vd(5);//v2(2)=0;//v2(2)=Vd(5)
                //qDebug()<<v2(2);
                v11=dymMat.transpose()*v1;
                v22=dymMat.transpose()*v2;
                Vector3d Md;
                double CD;
                qDebug()<<q_actual[15]<<" "<<q_actual[16]<<" "<<q_actual[17];
                CD=sqrt(q_actual[15]*q_actual[15]+q_actual[16]*q_actual[16]+q_actual[17]*q_actual[17]);
                Md(0)=q_actual[15]/CD;
                Md(1)=q_actual[16]/CD;
                Md(2)=q_actual[17]/CD;
                AngleAxisd zj(CD,Md);
                Vector3d eulerAngle=zj.matrix().eulerAngles(0,1,2);
                eulerAngle(0)=eulerAngle(0)+v22(0);
                eulerAngle(1)=eulerAngle(1)+v22(1);
                eulerAngle(2)=eulerAngle(2)+v22(2);
                AngleAxisd angleAxis2 = AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX());
                AngleAxisd angleAxis1 = AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY());
                AngleAxisd angleAxis0 = AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ());
                Matrix3d test = angleAxis2.matrix() * angleAxis1.matrix() * angleAxis0.matrix();
                AngleAxisd V3;
                V3.fromRotationMatrix(test);
                Vector3d last;
                last=V3.angle()*V3.axis();
                qDebug()<<v11(0)<<" "<<v11(1)<<" "<<v11(2)<<" "<<v22(0)<<" "<<v22(1)<<" "<<v22(2);
         /*..........考虑欧拉角................*/
             //  double w=cos(v2(0)/2)*cos(v2(1)/2)*cos(v2(2)/2)+sin(v2(0)/2)*sin(v2(1)/2)*sin(v2(2)/2);
        //       double x=sin(v2(0)/2)*cos(v2(1)/2)*cos(v2(2)/2)-cos(v2(0)/2)*sin(v2(1)/2)*sin(v2(2)/2);
        //       double y=cos(v2(0)/2)*sin(v2(1)/2)*cos(v2(2)/2)+sin(v2(0)/2)*cos(v2(1)/2)*sin(v2(2)/2);
        //       double z=cos(v2(0)/2)*cos(v2(1)/2)*sin(v2(2)/2)-sin(v2(0)/2)*sin(v2(1)/2)*cos(v2(2)/2);
        //       double x=sin(v2(2)/2)*cos(v2(1)/2)*cos(v2(0)/2)-cos(v2(2)/2)*sin(v2(1)/2)*sin(v2(0)/2);
        //       double y=cos(v2(2)/2)*sin(v2(1)/2)*cos(v2(0)/2)+sin(v2(2)/2)*cos(v2(1)/2)*sin(v2(0)/2);
        //       double z=cos(v2(2)/2)*cos(v2(1)/2)*sin(v2(0)/2)-sin(v2(2)/2)*sin(v2(1)/2)*cos(v2(0)/2);
        //       Quaterniond Q1;
        //       Q1.w()=w;Q1.x()=x;Q1.y()=y;Q1.z()=z;
        //       AngleAxisd V6(Q1);
        //       v2=V6.axis()*V6.angle();
               //qDebug()<<v2(0)<<" "<<v2(1)<<" "<<v2(2);
               qDebug()<<q_actual[12]<<" "<<q_actual[13]<<" "<<q_actual[14]<<" "<<q_actual[15]<<" "<<q_actual[16]<<" "<<q_actual[17];
               newtarget[0]=q_actual[12]+v11(0);
               newtarget[1]=q_actual[13]+v11(1);
               newtarget[2]=q_actual[14]+v11(2);
               newtarget[3]=last(0);
               newtarget[4]=last(1);
               newtarget[5]=last(2);
       /*..........数据采集专用................*/
               QFile data("../untitled_test_2/textxml/f_t.txt");
               if (data.open(QFile::WriteOnly | QIODevice::Append))
               {
                   QTextStream out(&data);
                   out <<Fx<<","<<Fy<<","<<Fz<<","<<Tx<<","<<Ty<<","<<Tz<<endl;
               }
               QFile data1("../untitled_test_2/textxml/pos.txt");
               if (data1.open(QFile::WriteOnly | QIODevice::Append))
               {
                   QTextStream out1(&data1);
                   out1 <<v1(0)<<","<<v1(1)<<","<<v1(2)<<","<<v2(0)<<","<<v2(1)<<","<<v2(2)<<endl;
               }
       /*..........数据采集专用................*/
               qDebug()<<q_actual[12]<<" "<<q_actual[13]<<" "<<q_actual[14]<<" "<<q_actual[15]<<" "<<q_actual[16]<<" "<<q_actual[17];
               qDebug()<<newtarget[0]<<" "<<newtarget[1]<<" "<<newtarget[2]<<" "<<newtarget[3]<<" "<<newtarget[4]<<" "<<newtarget[5];
               if(fabs(v1(0))<1e-3&&fabs(v1(1))<1e-3&&fabs(v1(2))<1e-3&&fabs(v2(0))<1e-3&&fabs(v2(1))<1e-3&&fabs(v2(2))<1e-3)
               {
                   msleep(8);
               }
               else
               {
                    if(firstmove)
                    {
                        for(int i=0;i<6;i++)
                        oldtarget[i]=q_actual[12+i];
                        firstmove=false;
                     }
                    double erro[6];
                    for(int i=0;i<6;i++)
                    {
                        erro[i]=fabs(newtarget[i]-oldtarget[i]);
                     }
                    // qDebug()<<erro[0]<<" "<<erro[1]<<" "<<erro[2]<<" "<<erro[3]<<" "<<erro[4]<<" "<<erro[5];
                    // qDebug()<<newtarget[0]<<" "<<newtarget[1]<<" "<<newtarget[2]<<" "<<newtarget[3]<<" "<<newtarget[4]<<" "<<newtarget[5];
                    // qDebug()<<oldtarget[0]<<" "<<oldtarget[1]<<" "<<oldtarget[2]<<" "<<oldtarget[3]<<" "<<oldtarget[4]<<" "<<oldtarget[5];
                    // if(erro[0]>0.031||erro[1]>0.031||erro[2]>0.036||erro[3]>0.15||erro[4]>0.2||erro[5]>0.2){msleep(8);}
                    if(erro[0]>0.035||erro[1]>0.035||erro[2]>0.046)
                    {
                        msleep(8);
                    }
                    else
                    {

                        strInstruct1=movelpos(newtarget[0],newtarget[1],newtarget[2],newtarget[3],newtarget[4],newtarget[5],0.5,0.5);
                        int write_byte=tcpClient2->write(strInstruct1.toLatin1(),strInstruct1.length());//toLatin1(); 将QString转为char
                        qDebug()<<strInstruct1<<";"<<write_byte;
                        bool ifwritten=tcpClient2->waitForBytesWritten();
                        for(int i=0;i<6;i++)
                        {
                            oldtarget[i]=newtarget[i];
                        }
                        msleep(8);
                    }
            }
        }
    else

      msleep(8);
       }
}
}
void forceThread::robotstate(double*q_actual)
{
    uint64_t unpack_to;
    uint16_t offset = 0;
    char  *buf;
    int len;
    uint64_t q;
    qint64 bytes_read;
    offset=0;
 if(tcpClient2->waitForReadyRead())
 {
     dataread.resize(tcpClient2->bytesAvailable());
     //qDebug()<<"A";
     bytes_read=tcpClient2->read(dataread.data(),dataread.size());
     qDebug()<<bytes_read;
     //dataread=tcpClient->readAll();
     if(bytes_read%1060==0&&bytes_read!=0)
     {
        buf=dataread.data()+bytes_read-1060;
        memcpy(&len, &buf[0], sizeof(len));
        offset += sizeof(len);
        //qDebug()<< offset;
        len=ntohl(len);//是将一个无符号长整形数从网络字节顺序转换为主机字节顺序， ntohl()返回一个以主机字节顺序表达的数。
         //printf("%d",len);
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
        offset+=sizeof(double)*12;
        for(int i=0;i<6;i++)
        {
            memcpy(&q, &buf[offset], sizeof(double));
            q=be64toh(q);//字节序转换
            memcpy(&q_actual[i+12], &q, sizeof(q));
            offset += sizeof(double);
        }
        for(int i=0;i<6;i++)
        {
            memcpy(&q, &buf[offset], sizeof(double));
            q=be64toh(q);//字节序转换
            memcpy(&q_actual[i+18], &q, sizeof(q));
            offset += sizeof(double);
        }
    }
}
}
