#include "FPGA.h"
#include "NiFpga_FPGA.h"
#include <limits.h>		//for _I16_MAX??




/*FIFO variables*/
extern const uint32_t timeout = 1; // in ms. 


								   /*DELAY. Sync AO and DO by delaying DO*/
extern const uint16_t DOfifoDelayTick = 76 * tick; //relative delay between AO and DO. This is because the AO takes longer than DO to write the output


