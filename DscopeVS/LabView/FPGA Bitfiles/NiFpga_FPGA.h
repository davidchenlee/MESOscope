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
static const char* const NiFpga_FPGA_Signature = "7FBB069DCD5968D9C4E0D6990BCEAEB6";

typedef enum
{
   NiFpga_FPGA_ControlBool_start = 0x810E,
} NiFpga_FPGA_ControlBool;

typedef enum
{
   NiFpga_FPGA_ControlU8_DOFIFODelayTicks = 0x811A,
} NiFpga_FPGA_ControlU8;

typedef enum
{
   NiFpga_FPGA_ControlU16_AOCalibratetick = 0x8116,
   NiFpga_FPGA_ControlU16_DOCalibratetick = 0x8112,
} NiFpga_FPGA_ControlU16;

typedef enum
{
   NiFpga_FPGA_HostToTargetFifoU32_A0FIFO = 1,
   NiFpga_FPGA_HostToTargetFifoU32_DOFIFO = 0,
} NiFpga_FPGA_HostToTargetFifoU32;

#endif
