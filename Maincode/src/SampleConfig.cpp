#include "SampleConfig.h"

const extern std::vector<LIMIT2> PetridishPosLimit{ { 27. * mm, 57. * mm}, { 0. * mm, 30. * mm}, { 15. * mm, 24. * mm} };		//Soft limit of the stage for the petridish
const extern std::vector<LIMIT2> ContainerPosLimit{ { -65. * mm, 65. * mm}, { 1.99 * mm, 30. * mm}, { 10. * mm, 24. * mm} };	//Soft limit of the stage for the oil container

//SAMPLE PARAMETERS
//const extern POSITION3 g_stackCenterXYZ{ (44.300 + 1.456) * mm, (24.003 + 9.904/2 - 0.285)* mm, (17.840 + 0.000) * mm };
//const extern POSITION3 g_stackCenterXYZ{ (44.300) * mm, (24.003)* mm, (18.051 + 0.000) * mm };//For contScanX
const extern POSITION3 g_stackCenterXYZ{ (53.360 - 0.035) * mm, (25.000 - 0.017)* mm, (17.923) * mm };

#if multibeam
const extern Sample g_currentSample{ "Beads4um16X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, multiply16X(50. * mW), multiply16X(1000. * um) },
																				   { "GFP", 920, multiply16X(55. * mW), multiply16X(1000. * um) },
																				   { "TDT", 1040, multiply16X(15. * mW), multiply16X(1000. * um) } }} };
/*const extern Sample liver{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{ {"TDT", 1040, multiply16X(50. * mW), multiply16X(0.0) },
																							{ "GFP", 920, multiply16X(40. * mW), multiply16X(0.0) },
																							{ "DAPI", 750, multiply16X(50. * mW), multiply16X(0.) } }} };*/
																							/*Sample g_currentSample{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit, {{ {"TDT", 1040, multiply16X(50. * mW), 150., 4 },
																																																  { "DAPI", 750, multiply16X(20. * mW), 120., 2 } }} };*/
#else
/*const extern Sample g_currentSample{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit,  {{{"TDT", 1040, 30. * mW, 150. * um, 4 },
																									  { "DAPI", 750, 12. * mW, 120. * um, 2 }}} };*/

const extern Sample g_currentSample{ "Beads4um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 40. * mW, 1000. * um},
																				  { "GFP", 920, 40. * mW, 1000. * um},
																				  { "TDT", 1040, 10. * mW, 1000. * um}}} };
/*const extern Sample g_currentSample{ "Beads1um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 40. * mW, 0. },
																					{ "GFP", 920, 40. * mW, 0. },
																					{ "TDT", 1040, 15. * mW, 0. }}} };*/
																					/*Sample g_currentSample{ "fluorBlue1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{ "DAPI", 750, 10. * mW, 0. }}} };*/
#endif