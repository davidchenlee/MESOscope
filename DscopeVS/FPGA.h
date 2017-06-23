#pragma once

#define tickPerUs 40				// Number of ticks in 1 us
#define us 1
#define ms 1000
#define tick 1
#define LSBmask 0x0000FFFF			// Select the 16 least significant bits
#define Abits 16					// Number of bits of the analog output
#define AO_dt 2*us					// Analog output time increament in us. AO takes 43 ticks (1us=40ticks) to read and write the data

#define v2hex(x) (int16_t)(1.0 * x / 10 *_I16_MAX)		//converts voltage from +-10 to hex
#define us2tick(x) (tt_t) (x * tickPerUs)				//converts us to ticks