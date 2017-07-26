#pragma once
#include <iostream>
#include "FPGA.h"
#include "Const.h"

class Seq
{
	int Status = 0;
	U32QV QV;

public:
	Seq::Seq();
	~Seq();

	void Seq::print();
	int Seq::status();
	void Seq::push(U8 chan, U32 x);
	int Seq::size();
	U32QV Seq::vector();
	void Seq::galvo(double t, double val);
	void Seq::shutter(double t, bool val);
};



