#include "Paragraph.h"

void MainSequence(NiFpga_Status* status, NiFpga_Session session)
{
	SendOutQueue(status, session, Seq1());
	TriggerAODO(status, session);
	Sleep(1000);

	// start acquiring data
	TriggerAcquisition(status, session);
	CountPhotons(status, session);

	//SECOND ROUND
	if (0)
	{
		SendOutQueue(status, session, Seq1());
		TriggerAODO(status, session);
		TriggerAcquisition(status, session);
	}
}

/* Testing the Seq class
void SeqClassTest()
{
		Seq ss;
		ss.shutter(1 * us, 1);
		std::cout << "size of the vector" << ss.size() << "\n";
		std::cout << "" << (ss.vector())[0].size() << "\n";
		Sleep(1000);
}
*/

