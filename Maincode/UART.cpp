//example from https://msdn.microsoft.com/en-us/library/windows/desktop/aa363201(v=vs.85).aspx
#include "UART.h"

void PrintCommState(DCB dcb)
{
	//  Print some of the DCB structure values
	_tprintf(TEXT("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n"),
		dcb.BaudRate,
		dcb.ByteSize,
		dcb.Parity,
		dcb.StopBits);
}

int runUARTsequence()
{

	DCB dcb = { 0 };
	HANDLE hCom;
	TCHAR *pcCommPort = TEXT("COM4");

	//Open a handle to the specified com port
	hCom = CreateFile(pcCommPort,
		GENERIC_READ | GENERIC_WRITE, //access (read and write)
		0, //share (0: cannot share the COM port)
		0, // security (0: None)
		OPEN_EXISTING, //creation (open existing)
		FILE_ATTRIBUTE_NORMAL, //we don't want overlapped operation
		0); //no template file for COM port

	if (hCom == INVALID_HANDLE_VALUE)
	{
		//Handle the error
		printf("CreateFile failed with error %d.\n", GetLastError());
		return (1);
	}


	//Initialize the DCB structure
	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(dcb);

	//Build on the current configuration by first retrieving all current settings
	if ((GetCommState(hCom, &dcb) == 0))
	{
		//  Handle the error.
		printf("GetCommState failed with error %d.\n", GetLastError());
		return (2);
	}

	//  Fill in some DCB values and set the com state
	dcb.BaudRate = CBR_19200;
	dcb.ByteSize = DATABITS_8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;

	if (!SetCommState(hCom, &dcb))
	{
		//  Handle the error.
		printf("SetCommState failed with error %d.\n", GetLastError());
		return (3);
	}

	// Set timeouts
	COMMTIMEOUTS timeout = { 0 };
	timeout.ReadIntervalTimeout = 50;
	timeout.ReadTotalTimeoutConstant = 50;
	timeout.ReadTotalTimeoutMultiplier = 50;
	timeout.WriteTotalTimeoutConstant = 50;
	timeout.WriteTotalTimeoutMultiplier = 10;
	SetCommTimeouts(hCom, &timeout);

	//  Get the comm config again.
	if (!GetCommState(hCom, &dcb))
	{
		//  Handle the error.
		printf("GetCommState failed with error %d.\n", GetLastError());
		return (2);
	}

	PrintCommState(dcb);       //  Output to console

	_tprintf(TEXT("Serial port %s successfully reconfigured.\n"), pcCommPort);



	//WRITE DATA
	string TxCommand = "?VW";
	DWORD dwBytesWritten;
	int totalBytesWritten = 0;
	for (int ii = 0; ii < TxCommand.length(); ii++)
	{
		WriteFile(hCom, //COM port handle
			&TxCommand[ii], //data to write
			(DWORD) sizeof(TxCommand[ii]), // # of bytes to write
			&dwBytesWritten, // # of bytes written
			NULL);

		totalBytesWritten += dwBytesWritten;
	}
	cout << "Command sent: " << TxCommand << "\n";
	cout << "Total bytes written: " << totalBytesWritten << "\n";

	//send CR
	char CRchar = 0x0D;
	WriteFile(hCom, &CRchar, (DWORD) sizeof(CRchar), &dwBytesWritten, NULL);

	//send LF
	char LFchar = 0x0A;
	WriteFile(hCom, &LFchar, (DWORD) sizeof(LFchar), &dwBytesWritten, NULL);



	//READ DATA
	char TempChar; //Temporary character used for reading
	char RxBuffer[256];//Buffer for storing Rxed Data
	DWORD numberBytesToRead; //Bytes successfully read by the ReadFile()
	int numberBytesActuallyRead = 0;


	do
	{
		ReadFile(hCom,       //Handle of the Serial port
			&TempChar,       //Temporary variable used to store the byte read from serial port buffer.
			sizeof(TempChar),//used to calculate the number of bytes to read
			&numberBytesToRead,    //Bytes successfully read by the ReadFile()
			NULL);
		if (numberBytesToRead > 0)
		{
			RxBuffer[numberBytesActuallyRead] = TempChar;// Store Tempchar into buffer
			numberBytesActuallyRead++;
		}
	} while (numberBytesToRead > 0);


	//PRINT OUT THE RECEIVED MESSAGE
	if (numberBytesActuallyRead > 0)
	{
		for (int ii = 0; ii < numberBytesActuallyRead; ii++)
		{
			cout << RxBuffer[ii];
		}
		//cout << "\n";
	}



	getchar();


	CloseHandle(hCom);
	return 0;
}