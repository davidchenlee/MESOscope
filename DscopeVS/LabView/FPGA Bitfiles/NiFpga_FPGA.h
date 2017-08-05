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
static const char* const NiFpga_FPGA_Signature = "409B86FDAFE7636D0A5A8EEA6D12919B";

typedef enum
{
   NiFpga_FPGA_IndicatorU8_Counter = 0x22,
} NiFpga_FPGA_IndicatorU8;

typedef enum
{
   NiFpga_FPGA_ControlBool_Enable = 0x16,
   NiFpga_FPGA_ControlBool_Trigger = 0x2E,
} NiFpga_FPGA_ControlBool;

typedef enum
{
   NiFpga_FPGA_ControlU16_DOdelaytick = 0x32,
   NiFpga_FPGA_ControlU16_Nmax = 0x1A,
} NiFpga_FPGA_ControlU16;

typedef enum
{
   NiFpga_FPGA_ControlI32_FIFOtimeout = 0x28,
   NiFpga_FPGA_ControlI32_Nchannels = 0x24,
} NiFpga_FPGA_ControlI32;

typedef enum
{
   NiFpga_FPGA_IndicatorArrayU8_outputarray2 = 0x10,
} NiFpga_FPGA_IndicatorArrayU8;

typedef enum
{
   NiFpga_FPGA_IndicatorArrayU8Size_outputarray2 = 10,
} NiFpga_FPGA_IndicatorArrayU8Size;

typedef enum
{
   NiFpga_FPGA_ControlArrayBool_Array = 0x1E,
} NiFpga_FPGA_ControlArrayBool;

typedef enum
{
   NiFpga_FPGA_ControlArrayBoolSize_Array = 5,
} NiFpga_FPGA_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_HostToTargetFifoU32_FIFO = 0,
} NiFpga_FPGA_HostToTargetFifoU32;

#endif
