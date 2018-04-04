/*
 * Generated with the FPGA Interface C API Generator 16.0.0
 * for NI-RIO 16.0.0 or later.
 */

#ifndef __NiFpga_FPGAvi_h__
#define __NiFpga_FPGAvi_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 1600
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_FPGAvi_Bitfile;
 */
#define NiFpga_FPGAvi_Bitfile "NiFpga_FPGAvi.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_FPGAvi_Signature = "CB9C2912FEC97BEE640256EA49532129";

typedef enum
{
   NiFpga_FPGAvi_ControlBool_FIFOOUTdebug = 0x16,
   NiFpga_FPGAvi_ControlBool_ONOFF = 0x22,
   NiFpga_FPGAvi_ControlBool_Start_acquisition = 0x3A,
   NiFpga_FPGAvi_ControlBool_Trigger = 0x4A,
   NiFpga_FPGAvi_ControlBool_VT_NC = 0x32,
   NiFpga_FPGAvi_ControlBool_VT_back = 0x2A,
   NiFpga_FPGAvi_ControlBool_VT_forward = 0x2E,
   NiFpga_FPGAvi_ControlBool_VT_start = 0x26,
} NiFpga_FPGAvi_ControlBool;

typedef enum
{
   NiFpga_FPGAvi_ControlI16_PC1_voltage = 0x1A,
   NiFpga_FPGAvi_ControlI16_RS_voltage = 0x1E,
} NiFpga_FPGAvi_ControlI16;

typedef enum
{
   NiFpga_FPGAvi_ControlU16_FIFO_timeout = 0x46,
   NiFpga_FPGAvi_ControlU16_Height_pix = 0x12,
   NiFpga_FPGAvi_ControlU16_Nchannels = 0x42,
   NiFpga_FPGAvi_ControlU16_Sync_AODO_to_LineGate = 0x36,
   NiFpga_FPGAvi_ControlU16_Sync_DO_to_AO = 0x4E,
} NiFpga_FPGAvi_ControlU16;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBool_Pulsesequence = 0x3C,
} NiFpga_FPGAvi_ControlArrayBool;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBoolSize_Pulsesequence = 20,
} NiFpga_FPGAvi_ControlArrayBoolSize;

typedef enum
{
   NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa = 1,
   NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTb = 0,
} NiFpga_FPGAvi_TargetToHostFifoU32;

typedef enum
{
   NiFpga_FPGAvi_HostToTargetFifoU32_FIFOIN = 2,
} NiFpga_FPGAvi_HostToTargetFifoU32;

#endif
