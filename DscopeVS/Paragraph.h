#pragma once
#include "Sentence.h"
#include "windows.h"	//for  Sleep

void NonDetDigitalPulse(NiFpga_Status* status, NiFpga_Session session, double dt);
void MainSequence(NiFpga_Status* status, NiFpga_Session session);

void InitializeFPGA(NiFpga_Status* status, NiFpga_Session session);
void SendOutQueue(NiFpga_Status* status, NiFpga_Session session, U32QV& Qarray);
void TriggerAODO(NiFpga_Status* status, NiFpga_Session session);
void TriggerAcquisition(NiFpga_Status* status, NiFpga_Session session);