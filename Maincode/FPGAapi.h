#pragma once
#include <iostream>
#include "NiFpga_FPGAvi.h"
#include "Const.h"
#include <string>	//printing the error codes
using namespace Const;


/*Define the full path of the bitfile. The bitfile is the FPGA code*/
static const char* Bitfile = "D:\\OwnCloud\\Codes\\MESOscope\\LabView\\FPGA Bitfiles\\" NiFpga_FPGAvi_Bitfile;

void checkFPGAstatus(char functionName[], NiFpga_Status status);

namespace GenericFPGAfunctions {
	void printHex(int input);
	U32 packU32(U16 t, U16 x);
	U16 convertUs2tick(double x);
	I16 convertVolt2I16(double x);
	U32 singleAnalogOut(double t, double V);
	U32 singleDigitalOut(double t, bool DO);
	U32 singlePixelClock(double t, bool DO);
	QU32 generateLinearRamp(double TimeStep, double RampLength, double Vinitial, double Vfinal);
}



class FPGAapi {	
	NiFpga_Session mSession;
public:
	FPGAapi();
	~FPGAapi();
	void initialize() const;
	void writeFIFO(VQU32 &vectorqueues) const;
	void runRTsequence() const;
	void flushFIFO() const;
	void close(const bool reset) const;
	NiFpga_Session getSession() const;
};

class FPGAexception : public std::runtime_error {
public:
	//FPGAexception(const char *message) : std::runtime_error(message) {}
	FPGAexception(const std::string& message) : std::runtime_error(message.c_str()) {}
};