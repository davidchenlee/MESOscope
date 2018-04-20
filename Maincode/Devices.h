#pragma once
#include "FPGAapi.h"
//#include "PIstages.h"
#include "UARTscope.h"
#include "Tiffscope.h"

//Photon counter
int readPhotonCount(NiFpga_Status* status, NiFpga_Session session);
void readFIFO(NiFpga_Status* status, NiFpga_Session session, int &NelementsReadFIFOa, int &NelementsReadFIFOb, U32 *dataFIFOa, U32 **bufArrayb, int *NelementsBufArrayb, int &bufArrayIndexb, int NmaxbufArray);
unsigned char *unpackFIFObuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
int correctInterleavedImage(unsigned char *interleavedImage);
int writeFrameToTxt(unsigned char *imageArray, std::string fileName);

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