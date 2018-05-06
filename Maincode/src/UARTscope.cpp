//example from https://msdn.microsoft.com/en-us/library/windows/desktop/aa363201(v=vs.85).aspx
//May do: use this cross-platform serial class http://wjwwood.io/serial/doc/1.1.0/classserial_1_1_serial.html

#include "UARTscope.h"

void PrintCommState(DCB dcb)
{
	//  Print some of the DCB structure values
	_tprintf(TEXT("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n"),
		dcb.BaudRate,
		dcb.ByteSize,
		dcb.Parity,
		dcb.StopBits);
}

int FilterWheel()
{

	DCB dcb = { 0 };
	HANDLE hCom;
	TCHAR *pcCommPort = TEXT("COM6");	//filterwheel 1

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
	dcb.BaudRate = CBR_115200;
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
	string TxCommand = "pos=2";
	DWORD dwBytesWritten;
	int totalBytesWritten = 0;
	for (int ii = 0; ii < TxCommand.length(); ii++)
	{
		//(COM port handle, data to write, # of bytes to write, # of bytes written)
		WriteFile(hCom, &TxCommand[ii], (DWORD) sizeof(TxCommand[ii]), &dwBytesWritten, NULL);
		totalBytesWritten += dwBytesWritten;
	}
	cout << "Command sent: " << TxCommand << "\n";
	//cout << "Total bytes written: " << totalBytesWritten << "\n";

	//send CR
	char CRchar = 0x0D;
	WriteFile(hCom, &CRchar, (DWORD) sizeof(CRchar), &dwBytesWritten, NULL);

	//send LF
	//char LFchar = 0x0A;
	//WriteFile(hCom, &LFchar, (DWORD) sizeof(LFchar), &dwBytesWritten, NULL);



	//READ DATA
	char TempChar; //Temporary character used for reading
	char RxBuffer[256];//Buffer for storing Rxed Data
	DWORD numberBytesToRead; //Bytes successfully read by the ReadFile()
	int numberBytesActuallyRead = 0;

	do
	{
		//(Handle of the Serial port, Temporary variable used to store the byte read from serial port buffer, used to calculate the number of bytes to read, Bytes successfully read by the ReadFile() )
		ReadFile(hCom, &TempChar, sizeof(TempChar), &numberBytesToRead, NULL);

		if (numberBytesToRead > 0 && TempChar != 0x0D && TempChar != 0x3E)	//Ignore 0x0D (ASCII CR) and 0x3E (ASCII >)
		{
			RxBuffer[numberBytesActuallyRead] = TempChar;// Store Tempchar into buffer
			numberBytesActuallyRead++;
		}
	} while (numberBytesToRead > 0);

	//PRINT OUT THE RECEIVED MESSAGE
	cout << "Command outcome: ";
	if (numberBytesActuallyRead > 0)
	{
		for (int ii = 0; ii < numberBytesActuallyRead; ii++)
		{
			std::cout << RxBuffer[ii] ;
		}
		cout << "\n";
	}

	CloseHandle(hCom);
	return 0;
}