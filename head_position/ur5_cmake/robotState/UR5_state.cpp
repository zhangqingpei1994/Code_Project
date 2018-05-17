#include"UR5_state.h"
#define pi 3.1415926;
void UR5_state(double*q_actual,QTcpSocket*tcpClient)
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
 //qDebug()<< bytes_read;
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
/*............  机械臂数据读取问题.............*/
//Matrix4d T;
//double R[9];
//AngleAxisd  V2;
//VectorXd v(3);
//double q[6] = { q_actual[0]*3.14 / 180, q_actual[1]*3.14 / 180, q_actual[2]*3.14 / 180, q_actual[3]*3.14 / 180, q_actual[4]*3.14 / 180, q_actual[5]*3.14 / 180 };
//double T[16];
//forward(q, T);
//double T02 = -T[0];  double T00 = T[1];  double T01 = T[2];
//double T12 = -T[4]; double T10 = T[5]; double T11 = T[6];
//double T22 = T[8]; double T20 = -T[9]; double T21 = -T[10];
//R[0]=T00;R[1]=T01;R[2]=T02;R[3]=T10;
//R[4]=T11;R[5]=T12;R[6]=T20;R[7]=T21;
//R[8]=T22;
//Map<MatrixXd> dymMat(R,3,3);//以列存储
//Matrix3d rt;
//rt=dymMat.transpose();
// V2.fromRotationMatrix(rt);
// v=V2.axis()*V2.angle();
 //q_actual[15]=v(0);q_actual[16]=v(1);q_actual[17]=v(2);
}
}
