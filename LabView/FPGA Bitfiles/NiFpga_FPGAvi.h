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
static const char* const NiFpga_FPGAvi_Signature = "39FBAEA875C24EF9E55DAB73A3C64B54";

typedef enum
{
   NiFpga_FPGAvi_ControlBool_EnableFIFO = 0x3E,
   NiFpga_FPGAvi_ControlBool_FIFOINtrigger = 0x6A,
   NiFpga_FPGAvi_ControlBool_FlushTrigger = 0x1E,
   NiFpga_FPGAvi_ControlBool_LinegateTrigger = 0x32,
   NiFpga_FPGAvi_ControlBool_Pockels1_EnableManualControl = 0x1A,
   NiFpga_FPGAvi_ControlBool_RS_ON_OFF = 0x46,
   NiFpga_FPGAvi_ControlBool_Shutter1 = 0x3A,
   NiFpga_FPGAvi_ControlBool_Shutter2 = 0x36,
   NiFpga_FPGAvi_ControlBool_VT_NC = 0x56,
   NiFpga_FPGAvi_ControlBool_VT_back = 0x4E,
   NiFpga_FPGAvi_ControlBool_VT_forward = 0x52,
   NiFpga_FPGAvi_ControlBool_VT_start = 0x4A,
} NiFpga_FPGAvi_ControlBool;

typedef enum
{
   NiFpga_FPGAvi_ControlU8_LineclockInputSelector = 0x26,
   NiFpga_FPGAvi_ControlU8_PhotoncounterInputSelector = 0x2A,
} NiFpga_FPGAvi_ControlU8;

typedef enum
{
   NiFpga_FPGAvi_ControlI16_RS_voltage = 0x42,
} NiFpga_FPGAvi_ControlI16;

typedef enum
{
   NiFpga_FPGAvi_ControlU16_FIFOtimeout_tick = 0x66,
   NiFpga_FPGAvi_ControlU16_Nchannels = 0x62,
   NiFpga_FPGAvi_ControlU16_NlinesAll = 0x12,
   NiFpga_FPGAvi_ControlU16_NlinesPerFrame = 0x2E,
   NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips = 0x22,
   NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate_tick = 0x5A,
   NiFpga_FPGAvi_ControlU16_SyncDOtoAOtick = 0x6E,
} NiFpga_FPGAvi_ControlU16;

typedef enum
{
   NiFpga_FPGAvi_ControlU32_StageTriggerDuration_tick = 0x14,
} NiFpga_FPGAvi_ControlU32;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBool_Pulsesequence = 0x5C,
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
