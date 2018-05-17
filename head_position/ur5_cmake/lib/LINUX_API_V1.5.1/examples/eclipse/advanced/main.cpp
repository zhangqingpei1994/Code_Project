#include <iostream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "omd/opto.h"
#include "omd/sensorconfig.h"
#include "omd/optopackage.h"


typedef unsigned long long mytime_t;

mytime_t Now()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t millisecs = t.tv_sec * 1000;
    millisecs += t.tv_nsec / (1000 * 1000);
    return millisecs;
}


mytime_t NowMicro()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t microsecs = t.tv_sec * 1000 * 1000;
    microsecs += t.tv_nsec / (1000);
    return microsecs;
}


mytime_t ElapsedTime(mytime_t p_Time)
{
	return Now() - p_Time;
}


mytime_t ElapsedTimeMicro(mytime_t p_Time)
{
	return NowMicro() - p_Time;
}


void MySleep(unsigned long p_uMillisecs)
{
	usleep(p_uMillisecs * 1000);
}

/*
 * Set the config to the DAQ
 * it is a blocking function; returns true, if the sending of
 * configuration is succeeded otherwise it returns false
 */
bool SetConfig(OptoDAQ & p_optoDAQ, int p_iSpeed, int p_iFilter)
{
	SensorConfig sensorConfig;
	sensorConfig.setSpeed(p_iSpeed);
	sensorConfig.setFilter(p_iFilter);
	mytime_t tNow = Now();

	bool bSuccess = false;
	do {
		bSuccess = p_optoDAQ.sendConfig(sensorConfig);
		if (bSuccess) {
			return true;
		}
		if (ElapsedTime(tNow) > 1000) {
			// 1 sec timeout
			return false;
		}
		MySleep(1);
	} while (bSuccess == false);
	return false;
}


void ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port)
{
	std::string deviceName = std::string(p_Port.deviceName);
	std::string name = std::string(p_Port.name);
	std::string serialNumber = std::string (p_Port.serialNumber);
	int version = p_optoDAQ.getVersion();
	std::cout<<"Device Name: "<<deviceName<<std::endl;
	std::cout<<"Name: "<<name<<std::endl;
	std::cout<<"Serial Number: "<<serialNumber<<std::endl;
	std::cout<<"Version: "<<version<<std::endl;
}



/*
 * Opens the desired port
 * it returns true if port could be opened otherwise returns false
 */
bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex)
{
	MySleep(2500); // We wait some ms to be sure about OptoPorts enumerated PortList
	OPort * portList = p_Ports.listPorts(true);
	int iLastSize = p_Ports.getLastSize();
	if (p_iIndex >= iLastSize) {
		// index overflow
		return false;
	}
	bool bSuccess = p_optoDAQ.open(portList[p_iIndex]);
	if (bSuccess) {
		ShowInformation(p_optoDAQ, portList[p_iIndex]);
	}
	return bSuccess;
}


/*
 * Blocking call to read one 3D package (with one second timeout)
 * it return a non-negative number if it succeeded (p_Package will be the read package)
 * otherwise returns -1
 */
int ReadPackage3D(OptoDAQ & p_optoDAQ, OptoPackage & p_Package)
{
	int iSize = -1;
	mytime_t tNow = Now();
	for (;;) {
		iSize = p_optoDAQ.read(p_Package, 0, false);
		if (iSize < 0 || iSize > 0) {
			break;
		}
		// No packages in the queue so we check the timeout
		if (ElapsedTime(tNow) >= 1000) {
			break;
		}
		MySleep(1);
	}
	return iSize;
}


/*
 * Blocking call to read one 6D package (with one second timeout)
 * it return a non-negative number if it succeeded (p_Package will be the read package)
 * otherwise returns -1
 */
int ReadPackage6D(OptoDAQ & p_optoDAQ, OptoPackage6D & p_Package)
{
	int iSize = -1;
	mytime_t tNow = Now();
	for (;;) {
		iSize = p_optoDAQ.read6D(p_Package, false);
		if (iSize < 0 || iSize > 0) {
			break;
		}
		// No packages in the queue so we check the timeout
		if (ElapsedTime(tNow) >= 1000) {
			break;
		}
		MySleep(1);
	}
	return iSize;
}


/*
 * The function determines if the sensor is a 3D sensor
 */
bool Is3DSensor(OptoDAQ & p_optoDAQ)
{
	opto_version optoVersion = p_optoDAQ.getVersion();
	if (optoVersion != _95 && optoVersion != _64) {
		return true;
	}
	return false;
}



void Run3DSensorExample(OptoDAQ & p_optoDAQ, mytime_t p_tTimeout)
{
	mytime_t tNow = Now();
	unsigned int uTotalReadPackages = 0;
	do {
		mytime_t tLoopNow = NowMicro();
		OptoPackage optoPackage;
		int iReadSize = ReadPackage3D(p_optoDAQ, optoPackage);
		if (iReadSize < 0) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}
		uTotalReadPackages += (unsigned int)iReadSize;

		// Formatting output in C style
		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
		mytime_t TotalElapsedTime = ElapsedTime(tNow);
		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
		double dFrequency = 0.0;
		if (dTotalTime > 0.0) {
			dFrequency = (uTotalReadPackages / dTotalTime);
		}
		fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
		fflush(stdout);
	} while (ElapsedTime(tNow) < p_tTimeout);
}


void Run6DSensorExample(OptoDAQ & p_optoDAQ, mytime_t p_tTimeout)
{
	mytime_t tNow = Now();
	unsigned int uTotalReadPackages = 0;
	do {
		mytime_t tLoopNow = NowMicro();
		OptoPackage6D optoPackage;
		int iReadSize = ReadPackage6D(p_optoDAQ, optoPackage);
		if (iReadSize < 0) {
			std::cout<<"Something went wrong, DAQ closed!"<<std::endl;
			return;
		}
		uTotalReadPackages += (unsigned int)iReadSize;

		// Formatting output in C style
		double dLoopTime = ElapsedTimeMicro(tLoopNow) / 1000.0;
		mytime_t TotalElapsedTime = ElapsedTime(tNow);
		double dTotalTime = (double)TotalElapsedTime / 1000.0; // Elapsed time in sec
		double dFrequency = 0.0;
		if (dTotalTime > 0.0) {
			dFrequency = (uTotalReadPackages / dTotalTime);
		}
		fprintf(stdout, "Elapsed: %.1f s Loop time: %.2f ms Samples: %u Sample rate: %.2f Hz\r\n", dTotalTime, dLoopTime, uTotalReadPackages, dFrequency);
		fflush(stdout);
	} while (ElapsedTime(tNow) < p_tTimeout);
}


int main()
{
	OptoDAQ optoDaq;
	OptoPorts optoPorts;
	// Changeable values, feel free to play with them
	int iPortIndex = 0;  // The index of the port which will be opened
	mytime_t tRunningTime = 60 * 1000;  // Total running time in milliseconds
	int iSpeed = 1000; // Speed in Hz
	int iFilter = 15;  // Filter in Hz
	///////////////////
	if (OpenPort(optoDaq, optoPorts, iPortIndex) == false) {
		std::cout<<"Could not open port"<<std::endl;
		return 0;
	}
	bool bConfig = SetConfig(optoDaq, iSpeed, iFilter);
	if (bConfig == false) {
		std::cout<<"Could not set config"<<std::endl;
		optoDaq.close();
		return 0;
	}
	if (Is3DSensor(optoDaq)) {
		Run3DSensorExample(optoDaq, tRunningTime);
	}
	else {
		Run6DSensorExample(optoDaq, tRunningTime);
	}
	optoDaq.close();
    return 0;
}

