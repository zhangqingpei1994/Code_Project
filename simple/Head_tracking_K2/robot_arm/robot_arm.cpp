
#include"robot_arm/robot_arm.h"

using namespace std;

/******************************************************
 * @brief robotstate(double*q_actual)
 * @param q_actual:  传进来的数组,用来存取机械臂的位置量
 *     说明:  只对 q_actual进行操作,过程中使用了 tcpClient
 *****************************************************/
void Robot_arm::robotstate(double*q_actual)
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

    if(bytes_read%1060==0&&bytes_read!=0)
    {
         buf=dataread.data()+bytes_read-1060;
         memcpy(&len, &buf[0], sizeof(len));
         offset += sizeof(len);
         len=ntohl(len);      //是将一个无符号长整形数从网络字节顺序转换为主机字节顺序， ntohl()返回一个以主机字节顺序表达的数。
         memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
         //函数返回一个大字节排序整数转换为系统的本机字节顺序。返回值将与大端系统上的参数相同。所谓的大端模式（Big-endian），是指数据的高字节，保存在内存的低地址中，而数据的低字节，保存在内存的高地址中
         unpack_to = be64toh(unpack_to);
         double k;
         memcpy(&k, &unpack_to, sizeof(k));
         offset += sizeof(unpack_to)+30*sizeof(double);
         for(int i=0;i<6;i++)
         {
            memcpy(&q, &buf[offset], sizeof(double));
            q=be64toh(q);//字节序转换
            memcpy(&q_actual[i], &q, sizeof(q));
            q_actual[i]=q_actual[i]*180/pi;
            offset += sizeof(double);
          }

        for(int i=0;i<6;i++)
        {
           memcpy(&q, &buf[offset], sizeof(double));
           q=be64toh(q);//字节序转换
           memcpy(&q_actual[i+6], &q, sizeof(q));

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

/********************************************************************
 * @brief Robot_arm::movelpos
 *          说明:  根据传入的参数 只对私有变量commondqueue进行了一些操作
 **********************************************************************/
void Robot_arm::movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v)
{
    commondqueue="def f():\n\t";     //\t tab键
    //用字符串变量参数依次替代字符串中最小数值
    QString strInstruct=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t").arg(x).arg(y).arg(z).arg(Rx).arg(Ry).arg(Rz);

    strInstruct+=QString("movel(wp1,a=%1,v=%2)\n\t").arg(a).arg(v);

    if(strInstruct.right(1).toLatin1() != "\n")   //right：最右边  toLatin1是一种编码格式
           strInstruct.append('\n');

    commondqueue+=strInstruct;

    commondqueue+="end\n";
    int write_byte=tcpClient->write(commondqueue.toLatin1(),commondqueue.length());     //toLatin1(); 将QString转为char
    qDebug()<<strInstruct<<";"<<write_byte;                                       //打印信息
}


void Robot_arm::track_head(Eigen::Vector4d & nose1,Eigen::Vector4d &nose2)
{
   target_nose1=handeye_data_Affin3d*nose1;
   target_nose2=handeye_data_Affin3d*nose2;

   movelpos(target_nose1(0,0), target_nose1(1,0), target_nose1(2,0), Rx, Ry, Rz, 0.1, 0.1);

}







