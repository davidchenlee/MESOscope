/*
 * Generated with the FPGA Interface C API Generator 16.0.0
 * for NI-RIO 16.0.0 or later.
 */

#ifndef __NiFpga_FPGA_h__
#define __NiFpga_FPGA_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1600
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGA_Bitfile;
 */
#define NiFpga_FPGA_Bitfile "NiFpga_FPGA.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGA_Signature = "760E13E0BBEC66E148FD7511AF94E2FE";

typedef enum
{
   NiFpga_FPGA_IndicatorU8_Countreading = 0x12,
} NiFpga_FPGA_IndicatorU8;

typedef enum
{
   NiFpga_FPGA_IndicatorU32_Detector = 0x14,
   NiFpga_FPGA_IndicatorU32_NAO0 = 0x20,
   NiFpga_FPGA_IndicatorU32_NAO1 = 0x1C,
   NiFpga_FPGA_IndicatorU32_NDO0 = 0x18,
} NiFpga_FPGA_IndicatorU32;

typedef enum
{
   NiFpga_FPGA_ControlBool_Trigger = 0x36,
   NiFpga_FPGA_ControlBool_restartgenerator = 0x2A,
} NiFpga_FPGA_ControlBool;

typedef enum
{
   NiFpga_FPGA_ControlU16_DOdelaytick = 0x3A,
} NiFpga_FPGA_ControlU16;

typedef enum
{
   NiFpga_FPGA_ControlI32_FIFOtimeout = 0x30,
   NiFpga_FPGA_ControlI32_Nchannels = 0x24,
   NiFpga_FPGA_ControlI32_Setcount = 0x2C,
} NiFpga_FPGA_ControlI32;

typedef enum
{
   NiFpga_FPGA_HostToTargetFifoU32_FIFO = 0,
} NiFpga_FPGA_HostToTargetFifoU32;

#endif
