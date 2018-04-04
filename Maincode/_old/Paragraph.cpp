#include "Paragraph.h"

void MainSequence(NiFpga_Status* status, NiFpga_Session session)
{
	//control sequences
	//SendOutQueue(status, session, GalvoTest());
	SendOutQueue(status, session, Acquire2D());
	TriggerAODO(status, session);			//trigger the analog and digital outputs

	//TriggerAcquisition(status, session); // trigger the data acquisition
	CountPhotons(status, session);


	//SECOND ROUND
	if (0)
	{
		SendOutQueue(status, session, TestAODOSeq());
		TriggerAODO(status, session);
		TriggerAcquisition(status, session);
	}
}

/* Test the Seq class
void SeqClassTest()
{
		Seq ss;
		ss.shutter(1 * us, 1);
		std::cout << "size of the vector" << ss.size() << std::endl;
		std::cout << "" << (ss.vector())[0].size() << std::endl;
		Sleep(1000);
}
*/

