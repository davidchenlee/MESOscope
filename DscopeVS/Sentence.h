#pragma once
#include "Word.h"
#include "windows.h"	//for  Sleep
//using namespace Const;

U32QV Seq1();
U32QV Seq2();
U32Q PixelClockSeq();
U32Q GalvoSeq();
U32QV GalvoTest();
U32QV DigitalOutSeq(bool DO);
U32QV DigitalTimingCheck();
U32QV AnalogLatencyCalib();
U32QV DigitalLatencyCalib();

void VTpulse(NiFpga_Status* status, NiFpga_Session session, double tstep, int channel);

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);