#ifndef FTUSB_H
#define FTUSB_H
#include<iostream>
#include<stdio.h>
#include <unistd.h>
#include <opencv2/objdetect/objdetect.hpp>
#include"omd/opto.h"
bool open_sensor();
double *force_measure();
void close_sensor();
#endif // FTUSB

