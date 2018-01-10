#pragma once
#include <windows.h>
#include <conio.h>
#include <time.h>
#include "PI_GCS2_DLL.h"
#include <iostream>
#include <fstream>


int runPIstageSequence();
bool ReferenceIfNeeded(int PIdeviceId, char* axis);
void CloseConnectionWithComment(int PIdeviceId, const char* comment);
void ReportError(int PIdeviceId);
BOOL WaitForMovementStop(int PIdeviceId, char* zAxis);
BOOL GetControllerID(int stageID);
BOOL GetStageBondaries(int stageID);
BOOL MoveStage(int stageID, double SetPosition);
BOOL GetStagePosition(int stageID, double SetPosition);
