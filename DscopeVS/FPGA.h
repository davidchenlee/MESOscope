#pragma once
#include "NiFpga_FPGA.h"
#include <queue>
#include <vector>
#include <iostream>
#include <exception>
//#include <limits.h>		//for _I16_MAX??
//using namespace std;

#define _tick 1						//a tick of the FPGA's clock
#define tickPerUs 40				//Number of ticks in 1 us. Depends on the FPGA hardware clock
#define AOtickPerUs 40				//Number of ticks in 1 us. Depends on the FPGA hardware clock
#define _us 1
#define _ms 1000*_us				//millisecond
#define _s 1000000*_us				//second
#define _V 1						//volt
#define AO_dt 4*_us					//Analog output time increament in us. Currently, the AO works well with 4us or more.
#define Nchannels 3					//Number of channels available. WARNING: This number has to match the implementation on the FPGA!


typedef   signed    char	I8;
typedef unsigned    char	U8;
typedef            short	I16;
typedef unsigned   short	U16;
typedef              int	I32;
typedef unsigned     int	U32;
typedef          __int64	I64;
typedef unsigned __int64	U63;		
typedef std::queue<uint32_t> U32Q;	//Queue of unsigned integers
typedef std::vector<U32Q> U32QV;	//Vector of queues of unsigned integers


/*DELAY. Sync AO and DO by delaying DO*/
static const U16 DODelayTick = 80 * _tick; //relative delay between AO and DO. This is because the AO takes longer than DO to write the output

/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;

//prototypes
U32Q linearRamp(double dt, double T, double Vi, double Vf);
U32 u32pack(U16 t, U16 x);
U16 us2tick(double x);
I16 AOUT(double x);
U32 AnalogOut(double t, double V);
U32 DigitalOut(double t, bool DO);
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void PulseTrigger(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
U32Q NewConcatenatedQ(U32Q& headQ, U32Q& tailQ);