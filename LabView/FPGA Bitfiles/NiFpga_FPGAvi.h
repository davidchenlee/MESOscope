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
static const char* const NiFpga_FPGAvi_Signature = "F68463DE219BF5B72FC109181B991001";

typedef enum
{
   NiFpga_FPGAvi_IndicatorBool_RSisRunning = 0x32,
} NiFpga_FPGAvi_IndicatorBool;

typedef enum
{
   NiFpga_FPGAvi_IndicatorI16_RSvoltageMon_I16 = 0x52,
} NiFpga_FPGAvi_IndicatorI16;

typedef enum
{
   NiFpga_FPGAvi_IndicatorU16_RescanGalvoMon = 0x2E,
   NiFpga_FPGAvi_IndicatorU16_ScanGalvoMon = 0x12,
} NiFpga_FPGAvi_IndicatorU16;

typedef enum
{
   NiFpga_FPGAvi_ControlBool_FIFOINtrigger = 0x96,
   NiFpga_FPGAvi_ControlBool_FIFOOUTgateEnable = 0x26,
   NiFpga_FPGAvi_ControlBool_FlushTrigger = 0x62,
   NiFpga_FPGAvi_ControlBool_LineclockInputSelector = 0x3A,
   NiFpga_FPGAvi_ControlBool_PcTrigger = 0x6A,
   NiFpga_FPGAvi_ControlBool_PockelsAutoOffEnable = 0x5A,
   NiFpga_FPGAvi_ControlBool_RSrun = 0x7A,
   NiFpga_FPGAvi_ControlBool_ShutterFidelity = 0x6E,
   NiFpga_FPGAvi_ControlBool_ShutterVision = 0x72,
   NiFpga_FPGAvi_ControlBool_TriggerAODOexternal = 0x3E,
   NiFpga_FPGAvi_ControlBool_VTback = 0x82,
   NiFpga_FPGAvi_ControlBool_VTforward = 0x86,
   NiFpga_FPGAvi_ControlBool_VTstart = 0x7E,
   NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable = 0x36,
} NiFpga_FPGAvi_ControlBool;

typedef enum
{
   NiFpga_FPGAvi_ControlU8_PhotocounterInputSelector = 0x2A,
} NiFpga_FPGAvi_ControlU8;

typedef enum
{
   NiFpga_FPGAvi_ControlI16_Nframes = 0x5E,
   NiFpga_FPGAvi_ControlI16_NlinesAll = 0x8A,
   NiFpga_FPGAvi_ControlI16_NlinesPerFrame = 0x66,
   NiFpga_FPGAvi_ControlI16_Npreframes = 0x1E,
   NiFpga_FPGAvi_ControlI16_RSvoltage_I16 = 0x76,
} NiFpga_FPGAvi_ControlI16;

typedef enum
{
   NiFpga_FPGAvi_ControlU16_FIFOINtimeout_tick = 0x4E,
   NiFpga_FPGAvi_ControlU16_Nchannels = 0x92,
   NiFpga_FPGAvi_ControlU16_RescanGalvoDelay_tick = 0x22,
   NiFpga_FPGAvi_ControlU16_ScanGalvoDelay_tick = 0x1A,
   NiFpga_FPGAvi_ControlU16_SyncDOtoAO_tick = 0x4A,
} NiFpga_FPGAvi_ControlU16;

typedef enum
{
   NiFpga_FPGAvi_ControlU32_LinegateTimeout_tick = 0x54,
   NiFpga_FPGAvi_ControlU32_PockelsMainDelay_tick = 0x44,
   NiFpga_FPGAvi_ControlU32_PockelsSecondaryDelay_tick = 0x14,
   NiFpga_FPGAvi_ControlU32_StagePulseStretcher_tick = 0x40,
} NiFpga_FPGAvi_ControlU32;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBool_PulseSequence = 0x8C,
} NiFpga_FPGAvi_ControlArrayBool;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBoolSize_PulseSequence = 20,
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
