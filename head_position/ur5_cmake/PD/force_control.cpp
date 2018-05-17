#include "force_control.h"
PDparam pdFx;PDparam pdFy;PDparam pdFz;
PDparam pdTx;PDparam pdTy;PDparam pdTz;
void PDinitial()
{
pdFx.kp=2.33;pdFx.kd=0.3;pdFx.errOld=0;
pdFy.kp=2.33;pdFy.kd=0.3;pdFy.errOld=0;
pdFz.kp=1.32;pdFz.kd=0.25;pdFz.errOld=0;
pdTx.kp=2.68;pdTx.kd=0.45;pdTx.errOld=0;
pdTy.kp=2.38;pdTy.kd=0.45;pdTy.errOld=0;
pdTz.kp=2.68;pdTz.kd=0.45;pdTz.errOld=0;
}
double PDcontrolF(PDparam &Kp,double &errNow)
{
double rlkp=Kp.kp+0.8*log(fabs(errNow)+1)/log(4);
double rlkd=Kp.kd+0.2*log(fabs(errNow)+1)/log(4);
Kp.errP=errNow;
Kp.errD=Kp.errP-Kp.errOld;
Kp.ctrOut=rlkp*Kp.errP+rlkd*Kp.errD;
Kp.errOld=Kp.errP;
//Kp.ctrOut=Kp.kp*Kp.errP;
return Kp.ctrOut;
}
double PDcontrolT(PDparam &Kp,double &errNow)
{
double rlkp=Kp.kp+0.6*log(fabs(errNow)+1)/log(1.4);
double rlkd=Kp.kd+0.2*log(fabs(errNow)+1)/log(1.4);
Kp.errP=errNow;
Kp.errD=Kp.errP-Kp.errOld;
Kp.errOld=Kp.errP;
Kp.ctrOut=rlkp*Kp.errP+rlkd*Kp.errD;
//Kp.ctrOut=Kp.kp*Kp.errP;
return Kp.ctrOut;
}
VectorXd compensation(double Fx,double Fy,double Fz,double Tx,double Ty,double Tz)
{
    VectorXd pos(6);
    double dx=0,dy=0,dz=0;
    double rx=0,ry=0,rz=0;
    if(fabs(Fx)>1.5)
    {if(Fx<0)Fx=Fx+1.5;
        else
            Fx=Fx-1.5;
       dx=PDcontrolF(pdFx,Fx);
       dx=dx/1000;
       if(dx>0.03)
           dx=0.03;
       if(dx<-0.03)
           dx=-0.03;
    }
    if(fabs(Fy)>1.5)
    {if(Fy<0)Fy=Fy+1.5;
        else
            Fy=Fy-1.5;
       dy=PDcontrolF(pdFy,Fy);
       dy=dy/1000;
       if(dy>0.03)
           dy=0.03;
       if(dy<-0.03)
           dy=-0.03;
    }
    if(fabs(Fz)>3)
    {if(Fz<0)Fz=Fz+3;
        else
        Fz=Fz-3;
       dz=PDcontrolF(pdFz,Fz);
       dz=dz/1000;
       if(dz>0.035)
           dz=0.035;
       if(dz<-0.035)
           dz=-0.035;
    }
    if(fabs(Tx)>0.15)
    {if(Tx<0)Tx=Tx+0.15;
        else
            Tx=Tx-0.15;
       rx=PDcontrolT(pdTx,Tx);
       rx=rx*3.14/180;
       if(rx>0.0872)
           rx=0.0872;
       if(rx<-0.0872)
           rx=-0.0872;

    }
    if(fabs(Ty)>0.15)
    {if(Ty<0)Ty=Ty+0.15;
        else
            Ty=Ty-0.15;
       ry=PDcontrolT(pdTy,Ty);
       ry=ry*3.14/180;
       if(ry>0.0872)
           ry=0.0872;
       if(ry<-0.0872)
           ry=-0.0872;
    }
    if(fabs(Tz)>0.15)
    {if(Tz<0)Tz=Tz+0.15;
        else
            Tz=Tz-0.15;
       rz=PDcontrolT(pdTz,Tz);
       rz=rz*3.14/180;
       if(rz>0.0872)
           rz=0.0872;
       if(rz<-0.0872)
           rz=-0.0872;
    }
    pos(0)=dx;pos(1)=dy;pos(2)=dz;pos(3)=rx;pos(4)=ry;pos(5)=rz;
    return pos;
}
