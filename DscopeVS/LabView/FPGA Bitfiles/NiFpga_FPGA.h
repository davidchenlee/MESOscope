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
static const char* const NiFpga_FPGA_Signature = "DC3CCB7EEC834DAD040764FC92D86C90";

typedef enum
{
   NiFpga_FPGA_ControlBool_ONOFF = 0x1E,
   NiFpga_FPGA_ControlBool_Start_acquisition = 0x3A,
   NiFpga_FPGA_ControlBool_Trigger = 0x4A,
   NiFpga_FPGA_ControlBool_VT_NC = 0x2E,
   NiFpga_FPGA_ControlBool_VT_back = 0x26,
   NiFpga_FPGA_ControlBool_VT_forward = 0x2A,
   NiFpga_FPGA_ControlBool_VT_start = 0x22,
} NiFpga_FPGA_ControlBool;

typedef enum
{
   NiFpga_FPGA_ControlI16_PC1_voltage = 0x16,
   NiFpga_FPGA_ControlI16_RS_voltage = 0x1A,
} NiFpga_FPGA_ControlI16;

typedef enum
{
   NiFpga_FPGA_ControlU16_FIFO_timeout = 0x46,
   NiFpga_FPGA_ControlU16_Nchannels = 0x42,
   NiFpga_FPGA_ControlU16_Ncounters = 0x36,
   NiFpga_FPGA_ControlU16_Nmax_lines = 0x12,
   NiFpga_FPGA_ControlU16_Sync_AODO_to_LineGate = 0x32,
   NiFpga_FPGA_ControlU16_Sync_DO_to_AO = 0x4E,
} NiFpga_FPGA_ControlU16;

typedef enum
{
   NiFpga_FPGA_ControlArrayBool_Pulsesequence = 0x3C,
} NiFpga_FPGA_ControlArrayBool;

typedef enum
{
   NiFpga_FPGA_ControlArrayBoolSize_Pulsesequence = 20,
} NiFpga_FPGA_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGA_TargetToHostFifoU8_FIFOOUT = 0,
} NiFpga_FPGA_TargetToHostFifoU8;

typedef enum
{
   NiFpga_FPGA_HostToTargetFifoU32_FIFOIN = 1,
} NiFpga_FPGA_HostToTargetFifoU32;

#endif
