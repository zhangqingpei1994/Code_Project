#include"tcpip_robot.h"
QString movelpos(double x, double y, double z, double Rx, double Ry, double Rz, double a, double v)
{
    QString  commondqueue="def f():\n\t";

    QString strInstruct=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t").arg(x).arg(y).arg(z).arg(Rx).arg(Ry).arg(Rz);

    strInstruct+=QString("movel(wp1,a=%1,v=%2)\n\t").arg(a).arg(v);

    if(strInstruct.right(1).toLatin1() != "\n")
         strInstruct.append('\n');

    commondqueue+=strInstruct;

    commondqueue+="end\n";

    return commondqueue;

}
QString moveljiont(double q1, double q2, double q3, double q4, double q5, double q6, double a, double v)
{
    QString  commondjoint="def f():\n\t";
    QString strInstruct1=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t").arg(q1).arg(q2).arg(q3).arg(q4).arg(q5).arg(q6);
       strInstruct1+=QString("movej(wp1,a=%1,v=%2)\n\t").arg(a).arg(v);
       if(strInstruct1.right(1).toLatin1() != "\n")
               strInstruct1.append('\n');
           commondjoint+=strInstruct1;
           commondjoint+="end\n";
           return commondjoint;
}
QString forcecontrol(double Fx, double Fy, double Fz, double Tx, double Ty, double Tz)
{

     //commondjoint="def f():\n\t";// \n huiche huanhang \t xia yi zifu biao
    QString commondjoint="thread Force_properties_calculation_thread_1():\n\t";
             commondjoint+=" while (True):\n\t\t";
    QString strInstruct1=QString("wp1=p[%1,%2,%3,%4,%5,%6]\n\t\t").arg(Fx).arg(Fy).arg(Fz).arg(Tx).arg(Ty).arg(Tz);
       strInstruct1+=QString("force_mode(tool_pose(), [1, 1, 1, 1, 1, 1],[%1,%2,%3,%4,%5,%6], 2, [0.1, 0.1, 0.1, 0.7, 0.7, 0.7])\n\t\t").arg(Fx).arg(Fy).arg(Fz).arg(Tx).arg(Ty).arg(Tz);
        strInstruct1+=QString("sync()\n\t");
      //strInstruct1+=QString("sleep(0.110)\n");
        //strInstruct1+=QString("end\n");
        strInstruct1+=QString("end\n");
        //strInstruct1+=QString("thread_handler_1 = run Force_properties_calculation_thread_1()\n");
//       if(strInstruct1.right(1).toLatin1() != "\n")
//               strInstruct1.append('\n');
          commondjoint+=strInstruct1;
           commondjoint+="end\n";
          commondjoint+=QString("thread_handler_1 = run Force_properties_calculation_thread_1()\n");
          //commondjoint+="def f():\n\t";
         // commondjoint+= QString("thread_handler_1 = run Force_properties_calculation_thread_1()\n");

           return commondjoint;
}
QString closeforce()
{

QString  killforce=QString("kill thread_handler_1\n");
         killforce+=QString("end_force_mode()\n");

return killforce;
}
