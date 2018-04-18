#ifndef UART_H
#define UART_H
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iostream>
#include <cmath>

void sendDataTty(float l);
int readData();
int uart();
#endif // UART_H

