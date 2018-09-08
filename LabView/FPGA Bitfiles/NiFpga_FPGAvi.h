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
static const char* const NiFpga_FPGAvi_Signature = "4DCE472DB0BDEB6AE396B94942397556";

typedef enum
{
   NiFpga_FPGAvi_IndicatorBool_RSenableMon = 0x1A,
} NiFpga_FPGAvi_IndicatorBool;

typedef enum
{
   NiFpga_FPGAvi_IndicatorI16_RScontrolMon_I16 = 0x3A,
} NiFpga_FPGAvi_IndicatorI16;

typedef enum
{
   NiFpga_FPGAvi_IndicatorU16_Galvo1Mon = 0x12,
   NiFpga_FPGAvi_IndicatorU16_Galvo2Mon = 0x16,
} NiFpga_FPGAvi_IndicatorU16;

typedef enum
{
   NiFpga_FPGAvi_ControlBool_FIFOINtrigger = 0x8E,
   NiFpga_FPGAvi_ControlBool_FIFOOUTfpgaEnable = 0x6A,
   NiFpga_FPGAvi_ControlBool_FlushTrigger = 0x4E,
   NiFpga_FPGAvi_ControlBool_LineclockInputSelector = 0x22,
   NiFpga_FPGAvi_ControlBool_MasterTrigger = 0x5E,
   NiFpga_FPGAvi_ControlBool_Pockels1AutoOffEnable = 0x46,
   NiFpga_FPGAvi_ControlBool_RSenable = 0x72,
   NiFpga_FPGAvi_ControlBool_ScanDirection = 0x42,
   NiFpga_FPGAvi_ControlBool_Shutter1 = 0x66,
   NiFpga_FPGAvi_ControlBool_Shutter2 = 0x62,
   NiFpga_FPGAvi_ControlBool_StageTrigAcqEnable = 0x1E,
   NiFpga_FPGAvi_ControlBool_TriggerAODOexternal = 0x26,
   NiFpga_FPGAvi_ControlBool_VTback = 0x7A,
   NiFpga_FPGAvi_ControlBool_VTforward = 0x7E,
   NiFpga_FPGAvi_ControlBool_VTstart = 0x76,
} NiFpga_FPGAvi_ControlBool;

typedef enum
{
   NiFpga_FPGAvi_ControlU8_Nframes = 0x4A,
   NiFpga_FPGAvi_ControlU8_PhotoncounterInputSelector = 0x56,
} NiFpga_FPGAvi_ControlU8;

typedef enum
{
   NiFpga_FPGAvi_ControlI16_RScontrol_I16 = 0x6E,
} NiFpga_FPGAvi_ControlI16;

typedef enum
{
   NiFpga_FPGAvi_ControlU16_FIFOINtimeout_tick = 0x36,
   NiFpga_FPGAvi_ControlU16_LinegateTimeout_tick = 0x3E,
   NiFpga_FPGAvi_ControlU16_Nchannels = 0x8A,
   NiFpga_FPGAvi_ControlU16_NlinesAll = 0x82,
   NiFpga_FPGAvi_ControlU16_NlinesPerFrame = 0x5A,
   NiFpga_FPGAvi_ControlU16_NlinesPerFramePlusSkips = 0x52,
   NiFpga_FPGAvi_ControlU16_SyncAODOtoLinegate_tick = 0x2E,
   NiFpga_FPGAvi_ControlU16_SyncDOtoAO_tick = 0x32,
} NiFpga_FPGAvi_ControlU16;

typedef enum
{
   NiFpga_FPGAvi_ControlU32_StagePulseStretcher_tick = 0x28,
} NiFpga_FPGAvi_ControlU32;

typedef enum
{
   NiFpga_FPGAvi_ControlArrayBool_PulseSequence = 0x84,
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
