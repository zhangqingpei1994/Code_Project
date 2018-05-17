#ifndef UR5_STATE_H
#define UR5_STATE_H
#include<arpa/inet.h>
#include <stddef.h>
#include <stdlib.h>
#include <QTcpSocket>
void UR5_state(double*q_actual,QTcpSocket*tcpClient);
#endif

