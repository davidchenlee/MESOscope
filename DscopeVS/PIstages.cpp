#include "PIstages.h"

//To set up the include files, follow the instructions in 'C:\Users\Public\PI\C-891\Samples\VC++\Read.me'
//Add the respective include directory (right-click on the project, select Properties -> C/C++ -> General -> Additional include directories)
//Add the libraries 'PI_GCS2_DLL.lib' and 'PI_GCS2_DLL_x64.lib' (right-click on the project, select Properties -> Linker -> Input -> Additional dependencies)


// Additional Sample Functions
int XstageID, YstageID, ZstageID;
char NumberOfAxesPerController[2] = "1"; //There is only 1 stage per controller
double SetPosition[3] = { 0, 0, 23 };

int runPIstageSequence()
{

	//Start USB connection. Make sure that the stages and servo are enabled on supplied software PIMikroMove
	XstageID = PI_ConnectUSB("116049107"); //	X-stage (V-551.4B)
	YstageID = PI_ConnectUSB("116049105"); //	Y-stage (V-551.2B)
	ZstageID = PI_ConnectUSB("0165500631");	//  Z-stage (ES-100)

	if (XstageID < 0)
	{
		std::cout << "Could not connect to the controller X.\n";
		_getch();
		return FALSE;
	}
	else if (YstageID < 0)
	{
		std::cout << "Could not connect to the controller Y.\n";
		_getch();
		return FALSE;
	}
	else if (ZstageID < 0) {
		std::cout << "Could not connect to the controller Z.\n";
		_getch();
		return FALSE;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// Startup stage
	//////////////////////////////////////////////////////////////////////////////////////////////////

	// Switch on servo
	const BOOL ServoOn = true;
	if (!PI_SVO(ZstageID, NumberOfAxesPerController, &ServoOn))
	{
		CloseConnectionWithComment(ZstageID, "SVO failed. Exiting.\n");
		return FALSE;
	}

	// Reference stage
	if (!ReferenceIfNeeded(ZstageID, NumberOfAxesPerController))
	{
		CloseConnectionWithComment(ZstageID, "Not referenced, Referencing failed.\n");
		return FALSE;
	}

	// Check if referencing was successful
	BOOL referenceCompleted;
	referenceCompleted = FALSE;

	if (!PI_qFRF(ZstageID, NumberOfAxesPerController, &referenceCompleted))
	{
		CloseConnectionWithComment(ZstageID, "Failed to query reference status.\n");
		return FALSE;
	}

	// Abort execution if stage could not be referenced
	if (FALSE == referenceCompleted)
	{
		CloseConnectionWithComment(ZstageID, "Referencing failed.\n");
		return FALSE;
	}

	std::cout << "Stage is referenced.\n";
	Sleep(1000);




	// Determine boundaries for stage movement
	if (!GetStageBondaries(XstageID) | !GetStageBondaries(YstageID) | !GetStageBondaries(ZstageID))
		return FALSE;

	// Move stage to set position within boundaries.
	if (!MoveStage(XstageID, SetPosition[0]) | !MoveStage(YstageID, SetPosition[1]) | !MoveStage(ZstageID, SetPosition[2]))
		return FALSE;

	//Wait For Movement to Stop
	if (!WaitForMovementStop(XstageID, NumberOfAxesPerController) | !WaitForMovementStop(YstageID, NumberOfAxesPerController) | !WaitForMovementStop(ZstageID, NumberOfAxesPerController))
		return FALSE;

	// Query stage position
	if (!GetStagePosition(XstageID, SetPosition[0]) | !GetStagePosition(YstageID, SetPosition[1]) | !GetStagePosition(ZstageID, SetPosition[2]))
		return FALSE;

	// Close Connection		
	PI_CloseConnection(XstageID);
	PI_CloseConnection(YstageID);
	PI_CloseConnection(ZstageID);
	std::cout << "Connection closed.\n";
	_getch();
	return 0;
}



//////////////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions																		
//////////////////////////////////////////////////////////////////////////////////////////////////

bool ReferenceIfNeeded(int PIdeviceId, char* axis)
{
	BOOL Referenced;
	BOOL Flag;
	if (!PI_qFRF(PIdeviceId, axis, &Referenced))
		return false;

	// If stage is equipped with absolute sensors, Referenced will always be set to true.
	if (!Referenced)
	{
		std::cout << "Referencing axis " << axis << "\n";
		if (!PI_FRF(PIdeviceId, axis))
		{
			return false;
		}

		// Wait until the reference move is done.
		Flag = false;
		while (Flag != TRUE)
		{
			if (!PI_IsControllerReady(PIdeviceId, &Flag))
				return false;
		}
	}
	return true;
}

void CloseConnectionWithComment(int PIdeviceId, const char* comment)
{
	std::cout << comment << "\n";
	ReportError(PIdeviceId);
	PI_CloseConnection(PIdeviceId);
	_getch();
}

void ReportError(int PIdeviceId)
{
	int err = PI_GetError(PIdeviceId);
	char zErrMsg[300];
	if (PI_TranslateError(err, zErrMsg, 299))
	{
		std::cout << "Error " << err << " occurred: " << zErrMsg << "\n";
	}
}

BOOL WaitForMovementStop(int PIdeviceId, char* zAxis)
{
	BOOL isMoving = true;

	while (isMoving)
	{
		if (!PI_IsMoving(PIdeviceId, zAxis, &isMoving))
		{
			CloseConnectionWithComment(PIdeviceId, "Unable to query movement status.\n");
			return FALSE;
		}
		Sleep(100);
	}
	return TRUE;
}

// Query controller identification
BOOL GetControllerID(int stageID)
{
	char IDN[200];
	if (PI_qIDN(stageID, IDN, 199) == FALSE)
	{
		CloseConnectionWithComment(stageID, "qIDN failed. Exiting.\n");
		return FALSE;
	}
	std::cout << "qIDN returned: " << IDN;
}

// Determine boundaries for stage movement
BOOL GetStageBondaries(int stageID)
{
	double MinPositionValue, MaxPositionValue;
	if (!PI_qTMN(stageID, NumberOfAxesPerController, &MinPositionValue))
	{
		CloseConnectionWithComment(stageID, "TMN? unable to query min. position of axis.\n");
		return FALSE;
	}

	if (!PI_qTMX(stageID, NumberOfAxesPerController, &MaxPositionValue))
	{
		CloseConnectionWithComment(stageID, "TMX?, Unable to query max. position of axis.\n");
		return FALSE;
	}

	std::cout << "Allowed range of movement: min: " << MinPositionValue << "\t max: " << MaxPositionValue << "\n";
}

// Move stage to set position within boundaries.
BOOL MoveStage(int stageID, double SetPosition)
{
	if (!PI_MOV(stageID, NumberOfAxesPerController, &SetPosition))
	{
		CloseConnectionWithComment(stageID, "MOV, unable to move stage to target position.\n");
		return FALSE;
	}
}

// Query stage position
BOOL GetStagePosition(int stageID, double SetPosition)
{
	double Position;
	if (!PI_qPOS(stageID, NumberOfAxesPerController, &Position))
	{
		CloseConnectionWithComment(stageID, "Unable to query stage position\n");
		return FALSE;
	}
	std::cout << "Stage successfully moved to " << Position << " target was " << SetPosition << "\n";
}