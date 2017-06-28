#pragma once
#include "NiFpga_FPGA.h"


#define tickPerUs 40				// Number of ticks in 1 us
#define us 1
//#define ms 1000
#define tick 1
#define LSBmask 0x0000FFFF			// Select the 16 least significant bits
#define Abits 16					// Number of bits of the analog output
#define AO_dt 2*us					// Analog output time increament in us. AO takes 43 ticks (40ticks-1us) to read and write the data

typedef uint16_t tt_t;				// Time type

/*DELAY. Sync AO and DO by delaying DO*/
static const uint16_t DODelayTick = 80 * tick; //relative delay between AO and DO. This is because the AO takes longer than DO to write the output

/*Define the full path of the bitfile*/
static const char* const Bitfile = "D:\\OwnCloud\\Codes\\Dscope\\DscopeVS\\LabView\\FPGA Bitfiles\\" NiFpga_FPGA_Bitfile;

//prototypes
void RunFPGA();
tt_t us2tick(double x);
int16_t AOUT(double x);
uint32_t u32pack(tt_t t, uint16_t x);
void PulseTrigger(NiFpga_Status* status, NiFpga_Session session);
void PulseStart(NiFpga_Status* status, NiFpga_Session session);
