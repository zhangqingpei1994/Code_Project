
#include<uart.h>
using namespace std;

//#define rec_buf_wait_2s 2

#define buffLen 1024
#define rcvTimeOut 2

unsigned char buffRcvData[buffLen] = { 0 };
unsigned char buffRcvData1[10]={ 0 };
int fd;
float PIzj=3.1415926;
float width_robot = 0.19;
float steelD=0.063;
float right_vel = 0.0;
float left_vel = 0.0;
    int  flag=1;
   int reciveleftspeed,reciverightspeed;
void Delay(int cnt)
{
    int i,j;
    i=0xff00;
    while(i--)
    {
      j=0x100;
      while(j--);
    }
}
/*************Linux and Serial Port *********************/
int openPort(int comport)
{

    if (comport == 1)
    {
        fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);//O_RDWR读写模式,O_NOCTTY如果路径名指向终端设备，不要把这个设备用作控制终端,
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS0 .....\n");
        }
    }
    else if (comport == 2)
    {
        fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS1 .....\n");
        }
    }
    else if (comport == 3)
    {
        fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS2 .....\n");
        }
    }
    /*************************************************/
    else if (comport == 4)
    {
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyUSB0 .....\n");
        }
    }
    if (fcntl(fd, F_SETFL, 0)<0)//功能描述：根据文件描述词来操作文件的特性，返回-1代表出错
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    }
    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("is a tty success!\n");
    }
    printf("fd-open=%d\n", fd);
    return fd;
}
int setOpt(int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) //tcgetattr可以初始化一个终端对应的termios结构,把当前的参数写入oldtio结构
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;//CLOCAL忽略所有调制解调器的状态行，保证程序不会占用串口，CREAD代表启用字符接收器，能够从串口中读取输入的数据。
    newtio.c_cflag &= ~CSIZE;//CS5/6/7/8表示发送或接收字符时使用5/6/7/8比特。
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch (nEvent)
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if (nStop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (nStop == 2)
    {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);//tcflush用于清空中端为完成的输入/输出请求及数据，
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
int readDataTty(unsigned char *rcv_buf, int TimeOut, int Len)
{
    int retval;
    fd_set rfds;
    struct timeval tv;
    int ret, pos;
    tv.tv_sec = TimeOut / 1000;  //set the rcv wait time
    tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s

    pos = 0;
    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (retval == -1)
        {
            perror("select()");
            break;
        }
        else if (retval)
        {
            ret = read(fd, rcv_buf + pos, 1);
            if (-1 == ret)
            {
                break;
            }

            pos++;
            if (Len <= pos)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return pos;
}

void sendDataTty(float l)
{
   ssize_t ret;
   char x=0x0d;
   char y=0x0a;
//   float vel_x=0.2;
//   float vel_th=0;
//   unsigned int vll,vrr;
//   if(vel_x == 0&&vel_th !=0) // turning
//        {
//            //vel_th> turing left   vel_th <0 turing right
//            right_vel = 20*5*vel_th * width_robot / (PIzj*steelD*2.0);//ratio 20  5 is my ratio(by cg)
//            //left_vel = (-1) * right_vel;
//            left_vel =  (-1)*right_vel;
//        }
//  else if(vel_th != 0&&vel_x ==0)
//        { // fordward / backward
//            left_vel = 100*vel_x/(PIzj*steelD);
//            //left_vel = vel_x;
//            right_vel = 100*vel_x/(PIzj*steelD);
//        }
//  else
//        { // moving doing arcs
//            left_vel = 100*(vel_x - vel_th * width_robot / 2.0)/(PIzj*steelD);
//            right_vel =100*( vel_x + vel_th * width_robot / 2.0)/(PIzj*steelD);
//            //ROS_INFO("moving while turing");
//        }

//  if(left_vel>0)//deal with linzece
//        {
//            vll=(unsigned int)left_vel;
//            //by wds
//            buffRcvData[2]=(unsigned char)vll;
//            buffRcvData[1]=0x0F;
//           //buf[2] = (unsigned char)((vll>>12)|0X0F);//high 8bit
//           // buf[3] = (unsigned char)(vll&0XFF);//low 8 bit
//        }
//        else
//        {
//            vll=(unsigned int)abs(left_vel);
//           buffRcvData[2]=(unsigned char)(vll);
//            buffRcvData[1]=0xF0;
//        }

//        //------------
//        if(right_vel>0)
//        {
//            vrr=(unsigned int)right_vel;
//            buffRcvData[4]=(unsigned char)vrr;
//            buffRcvData[3]=0x0F;
//        }
//        else
//        {
//            vrr=(unsigned int)abs(right_vel);
//            buffRcvData[4]=(unsigned char)vrr;
//            buffRcvData[3]=0xF0;
//        }
    buffRcvData[0] = l;
    buffRcvData[1] = x;
    buffRcvData[2] = y;
    ret = write(fd, &buffRcvData, 3);
    if (ret == -1)
    {
        printf("write device error\n");

    }

//for(i=0;i<6;i++)
//{readDataNum = readDataTty(buffRcvData1, rcvTimeOut,1);
//if(buffRcvData1[0]==0xaa)
//{
//  i=6;
//  j=1;
//}}
//if(j==1)
//{
  flag=0;
}
int uart()
{
    int iSetOpt = 0;//SetOpt 的增量i
    //int fdSerial = 0;
    if ((fd = openPort(4))<0)
    {
        perror("open_port error");
        return -1;
    }
    //setOpt(fdSerial, 9600, 8, 'N', 1)
    if ((iSetOpt = setOpt(9600, 8, 'N', 1))<0)
    {
        perror("set_opt error");
        return -1;
    }
    printf("Serial fdSerial=%d\n", fd);

    tcflush(fd, TCIOFLUSH);//清掉串口缓存
    fcntl(fd, F_SETFL, 0);

}
//int readData()
//{  int readDataNum ;
//    readDataNum = readDataTty(buffRcvData1, rcvTimeOut,1);

//    return buffRcvData1[0];
//}
