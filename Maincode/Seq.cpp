/*
Sequence class
WORK IN PROGRESS

*/


#include "Seq.h"

//constructor
Seq::Seq():QV(Nchan)
{
}

//destructor
Seq::~Seq()
{
}

void Seq::print()
{
	std::cout << "this is a test: " << Status << "\n";
}

int Seq::status()
{
	return Status;
}

void Seq::push(U8 chan, U32 x)
{
	QV[chan].push(x);
}

int Seq::size()
{
	return QV.size();
}

U32QV Seq::vector()
{
	return QV;
}

void Seq::galvo(double t, double val)
{
	QV[ABUF0].push(AnalogOut(t, val));
}

void Seq::shutter(double t, bool val)
{
	QV[DBUF0].push(DigitalOut(t, val));
}
