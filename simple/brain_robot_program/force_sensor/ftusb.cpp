#include"ftusb.h"
OptoDAQ daq;
OptoPorts ports;
OPort *portlist;
OptoPackage pack3D;
OptoPackage6D pack6D;
void MySleep(unsigned long p_uMillisecs)
{
    usleep(p_uMillisecs * 1000);
}
bool open_sensor()
{
   bool ifopen;
    MySleep(400);
   portlist=ports.listPorts(true);
   if (ports.getLastSize()>0)
   {
      daq.open(portlist[0]);
      ifopen=true;
   }
   else
   {
      std::cout<<"No sensor available"<<std::endl;
      ifopen=false;
   }
   return ifopen;
}

double * force_measure()
{
    double forcevalue[6];

//     if (OpenPort(daq, ports, 1) == false) {
//             std::cout<<"Could not open port"<<std::endl;
//            // return 0;
//         }
    //MySleep(500);
//   if (daq.getVersion()!=_95 && daq.getVersion() != _64) // It is a 3D sensor
//   {

//       int size=daq.read(pack3D,false);	// Reading Sensor #0 (up to 16 Sensors)
//       std::cout<<"x: "<<pack3D.x<<" y: "<<pack3D.y<<" z: "<<pack3D.z<<std::endl;

//   }
//   else					  // It is a 6D sensor
//   {

  int size=daq.read6D(pack6D,false);
  forcevalue[0]=pack6D.Fx;
  forcevalue[1]=pack6D.Fy;
  forcevalue[2]=pack6D.Fz;
  forcevalue[3]=pack6D.Tx;
  forcevalue[4]=pack6D.Ty;
  forcevalue[5]=pack6D.Tz;
  return forcevalue;



}
void close_sensor()
{
    daq.close();
}
