#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include <fstream>      //file management
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()
#include "Tiffscope.h"
using namespace Const;

/*Define the full path of the bitfile. The bitfile is the FPGA code*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

//Low-level FPGA functions
void printHex(int input);
double ConvertSpatialCoord2Time(double x);
double getDiscreteTime(int pix);
double calculateDwellTime(int pix);
double calculatePracticalDwellTime(int pix);
U32 packU32(U16 t, U16 x);
U16 convertUs2tick(double x);
I16 convertVolt2I16(double x);
U32 generateSingleAnalogOut(double t, double V);
U32 generateSingleDigitalOut(double t, bool DO);
U32 generateSinglePixelClock(double t, bool DO);
U32Q concatenateQueues(U32Q& headQ, U32Q& tailQ);
int sendQueueToFPGA(NiFpga_Status* status, NiFpga_Session session, U32QV& VectorOfQueues);
U32Q generateLinearRamp(double dt, double T, double Vi, double Vf);
int readPhotonCount(NiFpga_Status* status, NiFpga_Session session);
void readFIFO(NiFpga_Status* status, NiFpga_Session session, int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray);
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);

int writeFrameToTxt(unsigned char *imageArray, std::string fileName);

//FPGA initialization and trigger
int initializeFPGA(NiFpga_Status* status, NiFpga_Session session);
int triggerFIFOIN(NiFpga_Status* status, NiFpga_Session session);
int triggerLineGate(NiFpga_Status* status, NiFpga_Session session);
int triggerFIFOflush(NiFpga_Status* status, NiFpga_Session session);
int configureFIFO(NiFpga_Status* status, NiFpga_Session session, U32 depth);

//Vibratome functions
int vibratome_SendCommand(NiFpga_Status* status, NiFpga_Session session, double dt, VibratomeChannel channel);
int vibratome_StartStop(NiFpga_Status* status, NiFpga_Session session);

//Resonant scanner
NiFpga_Status resonantScanner_StartStop(NiFpga_Status* status, NiFpga_Session session, bool state);
int resonantScanner_SetOutputVoltager(NiFpga_Status* status, NiFpga_Session session, double Vout);
double resonantScanner_Amp2Volt(double Amplitude);

//Shutters
int shutter1_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);
int shutter2_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);