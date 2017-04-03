#pragma once


#define tickPerUs 40
#define us 1
#define tick 1

/*DELAY. Sync AO and DO by delaying DO*/
static const uint8_t DOfifoDelayTick = 80 * tick; //relative delay between AO and DO
static const uint16_t calibrateAOtiming = 40 * tick; //fine-tune the AO timing
static const uint16_t calibrateDOtiming = 5 * tick; //fine-tune the DO timing