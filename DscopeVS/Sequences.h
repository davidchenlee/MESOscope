#pragma once
#include "FPGA.h"
//using namespace Const;

U32QV Seq1();
U32QV Seq2();
U32Q PixelClockSeq();
U32Q GalvoSeq();
U32QV GalvoTest();
U32QV AnalogLatencyCalib();
U32QV DigitalLatencyCalib();