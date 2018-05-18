#ifndef TCPIP_ROBOT
#define TCPIP_ROBOT
#include<QString>

//#include <QTcpSocket>

QString movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v);

QString moveljiont(double q1, double q2, double q3, double q4, double q5, double q6, double a, double v);

QString forcecontrol(double Fx, double Fy, double Fz, double Tx, double Ty, double Tz);

QString closeforce();

//QTcpSocket *tcpClient;

#endif // TCPIP_ROBOT

