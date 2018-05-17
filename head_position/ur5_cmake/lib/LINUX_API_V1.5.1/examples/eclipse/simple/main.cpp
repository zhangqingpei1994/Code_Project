#include <iostream>
#include "omd/opto.h"
#include <unistd.h>

void MySleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}


int main()
{
	OptoDAQ daq;
	OptoPorts ports;

	MySleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList

    OPort* portlist=ports.listPorts(true);

    if (ports.getLastSize()>0)
    {
    	daq.open(portlist[0]);

        if (daq.getVersion()!=_95 && daq.getVersion() != _64) // It is a 3D sensor
        {
        	OptoPackage pack3D;
            int size=daq.read(pack3D,false);	// Reading Sensor #0 (up to 16 Sensors)
            std::cout<<"x: "<<pack3D.x<<" y: "<<pack3D.y<<" z: "<<pack3D.z<<std::endl;

        }
        else					  // It is a 6D sensor
        {
        	OptoPackage6D pack6D;
            int size=daq.read6D(pack6D,false);
            std::cout<<"Fx: "<<pack6D.Fx<<" Fy: "<<pack6D.Fy<<" Fz: "<<pack6D.Fz<<" ";
            std::cout<<"Tx: "<<pack6D.Tx<<" Ty: "<<pack6D.Ty<<" Tz: "<<pack6D.Tz<<std::endl;
        }
        daq.close();
    }
    else
    {
    	std::cout<<"No sensor available"<<std::endl;
    }
    return 0;
}
