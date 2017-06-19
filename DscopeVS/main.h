#pragma once


#define tickPerUs 40				// Number of ticks in 1 us
#define us 1
#define ms 1000
#define tick 1
#define LSBmask 0x0000FFFF			// Select the 16 least significant bits
#define Abits 16					// Number of bits of the analog output
#define AO_dt 50					// Analog output time increament
typedef uint16_t tt_t;				// Time type

#define VOUT(x) (int16_t)(1.0*x / 10 *_I16_MAX)		//converts voltage +-10 to hex
#define TIME(x) (int16_t)(1.0*x / 10 *_I16_MAX)		//convert time to hex

/*FIFO variables*/
static const uint32_t timeout = 1; // in ms. 


/*DELAY. Sync AO and DO by delaying DO*/
static const uint16_t DOfifoDelayTick = (76+80) * tick; //relative delay between AO and DO
static const uint16_t calibrateAOtiming = 40 * tick; //fine-tune the AO timing
static const uint16_t calibrateDOtiming = 5 * tick; //fine-tune the DO timing
static const uint16_t initialWait = 500 * tick; //Initial wait-time before the entire sequence starts



