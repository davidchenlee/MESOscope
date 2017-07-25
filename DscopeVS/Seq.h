#pragma once
#include <iostream>
#include "FPGA.h"
#include "Const.h"

class Seq
{
	int s = 0;
	int a = 0;

public:
	Seq();
	~Seq();

	void print();
	int status();
};



