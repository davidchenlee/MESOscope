#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include <fstream>      //file management
#include "windows.h"	//the stages use this lib. also Sleep
#include <ctime>		//Clock()
#include "Tiffscope.h"
using namespace Const;

/*Define the full path of the bitfile*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

//Low-level FPGA functions
void printHex(U32 input);
U32 u32pack(U16 t, U16 x);
U16 us2tick(double x);
I16 volt2I16(double x);
U32 AnalogOut(double t, double V);
U32 DigitalOut(double t, bool DO);
U32 PixelClock(double t, bool DO);
U32Q PushQ(U32Q& headQ, U32Q& tailQ);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
U32Q linearRamp(double dt, double T, double Vi, double Vf);
void CountPhotons(NiFpga_Status* status, NiFpga_Session session);
unsigned char *UnpackFIFOBuffer(int bufArrayIndexb, int *NelementsBufArrayb, U32 **bufArrayb);
void CorrectInterleavedImage(unsigned char *interleavedImage);

void WriteFrameToTxt(unsigned char *imageArray, std::string fileName);

//FPGA initialization and trigger
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void TriggerFIFOIN(NiFpga_Status* status, NiFpga_Session session);
void TriggerLineGate(NiFpga_Status* status, NiFpga_Session session);
void ConfigureFIFO(NiFpga_Status* status, NiFpga_Session session, U32 depth);

//Vibratome functions
int Vibratome_SendCommand(NiFpga_Status* status, NiFpga_Session session, double dt, VibratomeChannel channel);
int Vibratome_StartStop(NiFpga_Status* status, NiFpga_Session session);

//Resonant scanner
NiFpga_Status ResonantScanner_StartStop(NiFpga_Status* status, NiFpga_Session session, bool state);
int ResonantScanner_SetOutputVoltager(NiFpga_Status* status, NiFpga_Session session, double Vout);
double ResonantScanner_Amp2Volt(double Amplitude);

//Shutters
int Shutter1_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);
int Shutter2_OpenClose(NiFpga_Status* status, NiFpga_Session session, bool state);