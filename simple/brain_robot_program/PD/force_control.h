#ifndef FORCE_CONTROL
#define FORCE_CONTROL
#include<Eigen>
#include <iostream>
using namespace Eigen;
struct PDparam
{
    double kp;
    double kd;
    double errP;
    double errD;
    double errOld;
    double ctrOut;
};
double PDcontrol(PDparam &Kp,double &errNow);
void PDinitial();
VectorXd compensation(double& Fx,double& Fy,double&Fz,double& Tx,double& Ty,double& Tz);
#endif // FORCE_CONTROL

