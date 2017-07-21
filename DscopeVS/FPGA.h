#pragma once
#include "NiFpga_FPGA.h"
#include <queue>

#define tickPerUs 40				//Master clock. Number of ticks in 1 us
#define _us 1
#define _ms 1000*_us
#define _s 1000000*_us
#define _tick 1
#define _V 1
#define LSBmask 0x0000FFFF			//Select the 16 least significant bits
#define Abits 16					//Number of bits of the analog output
#define AO_dt 4*_us					//Analog output time increament in us. Currently, the AO works well with 4us or more.
#define Nchannels 3					//Number of channels available. WARNING: This number has to match the implementation on the FPGA!

typedef uint32_t tt_t;				//Time type

/*DELAY. Sync AO and DO by delaying DO*/
static const uint16_t DODelayTick = 80 * _tick; //relative delay between AO and DO. This is because the AO takes longer than DO to write the output

/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;

//prototypes
void RunFPGA();
tt_t us2tick(double x);
int16_t AOUT(double x);
uint32_t u32pack(tt_t t, uint16_t x);
void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void PulseTrigger(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, std::queue<uint32_t>* Qarray);

