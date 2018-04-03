#pragma once
#include <windows.h> //contains the Serial library
#include <iostream>
#include <tchar.h> //unicode
#include <string>
using namespace std;

void PrintCommState(DCB dcb);
int runUARTsequence();
int FilterWheel();