//#include <math.h>
//#include <time.h>
//#include <concrt.h> 	//Concurrency::wait(2000);
#include <iostream>
#include "FPGA.h"
#include <queue>


int main()
{
	RunFPGA();
	/*
	std::queue<uint32_t> myqueue;
	myqueue.push(12);
	myqueue.push(144);
	std::cout << "the current size is: " << myqueue.size() << "\n";
	while (!myqueue.empty())
	{
		std::cout << myqueue.front() << "\n";
		myqueue.pop();
	}
	std::cout << "the current size is: " << myqueue.size() << "\n";
	getchar();
	*/

	return 0;
}
