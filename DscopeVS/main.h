#pragma once


#define tickPerUs 40
#define us 1
#define tick 1

/*FIFO variables*/
static const uint32_t timeout = 1; // in ms


/*DELAY. Sync AO and DO by delaying DO*/
static const uint16_t DOfifoDelayTick = 80 * tick; //relative delay between AO and DO
static const uint16_t calibrateAOtiming = 40 * tick; //fine-tune the AO timing
static const uint16_t calibrateDOtiming = 5 * tick; //fine-tune the DO timing
static const uint16_t initialWait = 100 * tick; //Initial wait-time before the entire sequence starts
