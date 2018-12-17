#include "Routines.h"

void seq_main(const FPGAns::FPGA &fpga)
{
	//const RunMode acqMode = SINGLEMODE;			//Single shot
	//const RunMode acqMode = CONTMODE;				//Image the same z plane many times
	//const RunMode acqMode = AVGMODE;				//Image the same z plane many times for averaging
	//const RunMode acqMode = STACKMODE;			//Stack volume from the initial z position
	const RunMode acqMode = STACKCENTEREDMODE;	//Stack volume centered at the initial z position

	//ACQUISITION SETTINGS
	const int widthPerFrame_pix = 300;
	const int heightPerFrame_pix = 400;
	const int nFramesCont = 1;										//Number of frames for continuous acquisition
	const double3 stagePosition0_mm = { 36.050, 14.150, 18.697 };	//Stage initial position. For 5% overlap: x=+-0.190, y=+-0.142

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);
	RScanner.isRunning();					//Make sure that the RS is running

	//STACK
	const double stepSize_um = 0.5 * um;
	double stackRangeZ_um = 10 * um;		//Acquire a stack covering this interval

	//LASER
	const int wavelength_nm = 1040;
	const std::vector<double> Pif_mW = { 15, 15 };
	double P_mW = Pif_mW.front();
	Laser vision(VISION);
	vision.setWavelength(wavelength_nm);

	//SAMPLE
	const std::string sampleName("Bead4um");
	const std::string immersionMedium("Glycerol");
	const double collar = 1.47;

	//FILTERWHEEL
	Filterwheel FWdetection(FWDET);
	FWdetection.setColor(wavelength_nm);

	//STAGES
	Stage stage;
	std::vector<double3> stagePosition_mm;

	int nDiffZ;				//Number of frames at different Z for discontinuous acquisition
	int nSameZ;				//Number of frames at same Z for discontinuous acquisition
	OverrideFileSelector overrideFlag;
	switch (acqMode)
	{
	case SINGLEMODE:
		nSameZ = 1;
		nDiffZ = 1; //Do not change this
		overrideFlag = NOOVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case CONTMODE:
		nSameZ = 500;
		nDiffZ = 1; //Do not change this
		overrideFlag = OVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case AVGMODE:
		nSameZ = 10;
		nDiffZ = 1; //Do not change this
		overrideFlag = NOOVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case STACKMODE:
		nSameZ = 1;
		nDiffZ = (int)(stackRangeZ_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(XX), stagePosition0_mm.at(YY), stagePosition0_mm.at(ZZ) + iterDiffZ * stepSize_um / 1000 });
		break;
	case STACKCENTEREDMODE:
		nSameZ = 1;
		nDiffZ = (int)(stackRangeZ_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(XX), stagePosition0_mm.at(YY), stagePosition0_mm.at(ZZ) - 0.5 * stackRangeZ_um / 1000 + iterDiffZ * stepSize_um / 1000 });
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
	}

	FPGAns::RTsequence RTsequence(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

	//GALVO FOR RT
	const double FFOVgalvo_um = 200 * um;	//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;
	Galvo galvo(RTsequence, GALVO1);
	const double duration = 62.5 * us * RTsequence.mHeightPerFrame_pix;				//= halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo

	//POCKELS CELL FOR RT
	PockelsCell pockels(RTsequence, VISION, wavelength_nm);
	//pockelsVision.voltageLinearRamp(4*us, 40*us, 0, 1*V);
	//pockelsVision.voltageLinearRamp(galvoTimeStep, duration, 0.5*V, 1*V);	//Ramp up the laser intensity in a frame and repeat for each frame
	//pockelsVision.scalingLinearRamp(1.0, 2.0);								//Linearly scale the laser intensity across all the frames

	//DATALOG
	{
		Logger datalog("datalog_" + sampleName);
		datalog.record("SAMPLE-------------------------------------------------------");
		datalog.record("Sample = ", sampleName);
		datalog.record("Immersion medium = ", immersionMedium);
		datalog.record("Correction collar = ", collar);
		datalog.record("FPGA---------------------------------------------------------");
		datalog.record("FPGA clock (MHz) = ", tickPerUs);
		datalog.record("LASER--------------------------------------------------------");
		datalog.record("Laser wavelength (nm) = ", wavelength_nm);
		datalog.record("Laser power first frame (mW) = ", Pif_mW.front());
		datalog.record("Laser power last frame (mW) = ", Pif_mW.back());
		datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod);
		datalog.record("SCAN---------------------------------------------------------");
		datalog.record("RS FFOV (um) = ", RScanner.mFFOV_um);
		datalog.record("RS period (us) = ", 2 * halfPeriodLineclock_us);
		datalog.record("Pixel dwell time (us) = ", RTsequence.mDwell_us);
		datalog.record("RS fill factor = ", RScanner.mFillFactor);
		datalog.record("Galvo FFOV (um) = ", FFOVgalvo_um);
		datalog.record("Galvo time step (us) = ", galvoTimeStep);
		datalog.record("IMAGE--------------------------------------------------------");
		datalog.record("Max count per pixel = ", RTsequence.mPulsesPerPixel);
		datalog.record("8-bit upscaling factor = ", RTsequence.mUpscaleU8);
		datalog.record("Width X (RS) (pix) = ", RTsequence.mWidthPerFrame_pix);
		datalog.record("Height Y (galvo) (pix) = ", RTsequence.mHeightPerFrame_pix);
		datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes_umPerPix);
		datalog.record("Resolution Y (galvo) (um/pix) = ", FFOVgalvo_um / RTsequence.mHeightPerFrame_pix);
		datalog.record("STAGE--------------------------------------------------------");
	}

	//CREATE CONTAINERS FOR STORING THE STACK OF TIFFs
	TiffU8 stackDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ);
	TiffU8 stackSameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ);

	//OPEN THE UNIBLITZ SHUTTERS
	pockels.setShutter(true);
	Sleep(50);

	//ACQUIRE FRAMES AT DIFFERENT Zs
	for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
	{
		stage.moveStage3(stagePosition_mm.at(iterDiffZ));
		stage.waitForMotionToStop3();
		stage.printPosition3();		//Print the stage position
		//P_mW += 0.5;				//Increase the laser power by this amount

		//Acquire many frames at the same Z via discontinuous acquisition
		for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
		{
			std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
				"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << "\n";

			pockels.pushPowerSinglet(8 * us, P_mW, OVERRIDE);	//Override the previous laser power

			//EXECUTE THE RT SEQUENCE
			Image image(RTsequence);
			image.acquire();			//Execute the RT sequence and acquire the image
			image.mirrorOddFrames();
			image.average();			//Average the frames acquired via continuous acquisition
			stackSameZ.pushImage(iterSameZ, image.accessTiff());

			if (acqMode == SINGLEMODE || acqMode == CONTMODE)
			{
				//Save individual files
				std::string singleFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_" + toString(P_mW, 1) + "mW" +
					"_x=" + toString(stagePosition_mm.at(iterDiffZ).at(XX), 3) + "_y=" + toString(stagePosition_mm.at(iterDiffZ).at(YY), 3) + "_z=" + toString(stagePosition_mm.at(iterDiffZ).at(ZZ), 4));
				image.saveTiffSinglePage(singleFilename, overrideFlag);
				Sleep(500);
			}
		}
		stackSameZ.average();	//Average the frames acquired via discontinuous acquisition
		stackDiffZ.pushImage(iterDiffZ, stackSameZ.accessTiff());

		std::cout << "\n";
		P_mW += (Pif_mW.back() - Pif_mW.front()) / nDiffZ;
	}
	pockels.setShutter(false);

	if (acqMode == AVGMODE || acqMode == STACKMODE || acqMode == STACKCENTEREDMODE)
	{
		//Save the stackDiffZ to a file
		std::string stackFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(Pif_mW.front(), 1) + "mW_Pf=" + toString(Pif_mW.back(), 1) + "mW" +
			"_x=" + toString(stagePosition_mm.front().at(XX), 3) + "_y=" + toString(stagePosition_mm.front().at(YY), 3) +
			"_zi=" + toString(stagePosition_mm.front().at(ZZ), 4) + "_zf=" + toString(stagePosition_mm.back().at(ZZ), 4) + "_Step=" + toString(stepSize_um / 1000, 4));

		stackDiffZ.saveToFile(stackFilename, MULTIPAGE, overrideFlag);
	}

}

void seq_mainFidelity(const FPGAns::FPGA &fpga)
{
	//const RunMode acqMode = SINGLEMODE;			//Single shot
	//const RunMode acqMode = CONTMODE;				//Image the same z plane many times
	//const RunMode acqMode = AVGMODE;				//Image the same z plane many times for averaging
	//const RunMode acqMode = STACKMODE;			//Stack volume from the initial z position
	const RunMode acqMode = STACKCENTEREDMODE;	//Stack volume centered at the initial z position

	//ACQUISITION SETTINGS
	const int widthPerFrame_pix = 300;
	const int heightPerFrame_pix = 400;
	const int nFramesCont = 1;									//Number of frames for continuous acquisition
	const double3 stagePosition0_mm = { 36.050, 14.150, 18.686 };	//Stage initial position. For 5% overlap: x=+-0.190, y=+-0.142

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);
	RScanner.isRunning();					//Make sure that the RS is running

	//STACK
	const double stepSize_um = 0.5 * um;
	double stackRangeZ_um = 10 * um;				//Acquire a stack covering this interval

	//LASER
	const int wavelength_nm = 1040;						//Needed for the filterwheels
	const std::vector<double> Pif_mW = { 20, 20 };		//For 750 nm over 200 um
	double P_mW = Pif_mW.front();
	//Laser fidelity(FIDELITY);							//This does not do anything

	//SAMPLE
	const std::string sampleName("Bead4um");
	const std::string immersionMedium("Glycerol");
	const double collar = 1.47;

	//FILTERWHEEL
	Filterwheel FWexcitation(FWEXC);
	Filterwheel FWdetection(FWDET);
	FWexcitation.setColor(wavelength_nm);
	FWdetection.setColor(wavelength_nm);

	//STAGES
	Stage stage;
	std::vector<double3> stagePosition_mm;

	int nDiffZ;				//Number of frames at different Z for discontinuous acquisition
	int nSameZ;				//Number of frames at same Z for discontinuous acquisition
	OverrideFileSelector overrideFlag;
	switch (acqMode)
	{
	case SINGLEMODE:
		nSameZ = 1;
		nDiffZ = 1; //Do not change this
		overrideFlag = NOOVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case CONTMODE:
		nSameZ = 500;
		nDiffZ = 1; //Do not change this
		overrideFlag = OVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case AVGMODE:
		nSameZ = 10;
		nDiffZ = 1; //Do not change this
		overrideFlag = NOOVERRIDE;
		stagePosition_mm.push_back(stagePosition0_mm);
		break;
	case STACKMODE:
		nSameZ = 1;
		nDiffZ = (int)(stackRangeZ_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(XX), stagePosition0_mm.at(YY), stagePosition0_mm.at(ZZ) + iterDiffZ * stepSize_um / 1000 });
		break;
	case STACKCENTEREDMODE:
		nSameZ = 1;
		nDiffZ = (int)(stackRangeZ_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(XX), stagePosition0_mm.at(YY), stagePosition0_mm.at(ZZ) - 0.5 * stackRangeZ_um / 1000 + iterDiffZ * stepSize_um / 1000 });
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
	}

	FPGAns::RTsequence RTsequence(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

	//GALVO (RT sequence)
	const double FFOVgalvo_um = 200 * um;	//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;
	Galvo galvo(RTsequence, GALVO1);
	const double duration = 62.5 * us * RTsequence.mHeightPerFrame_pix;				//= halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo

	//POCKELS CELLS (RT sequence)
	PockelsCell pockelsVision(RTsequence, VISION, wavelength_nm);
	PockelsCell pockelsFidelity(RTsequence, FIDELITY, 1040);
	//pockelsVision.voltageLinearRamp(galvoTimeStep, duration, 0.5*V, 1*V);			//Ramp up the laser intensity in a frame and repeat for each frame
	//pockelsVision.scalingLinearRamp(1.0, 2.0);									//Linearly scale the laser intensity across all the frames

	//DATALOG
	{
		Logger datalog("datalog_" + sampleName);
		datalog.record("\nSAMPLE-------------------------------------------------------");
		datalog.record("Sample = ", sampleName);
		datalog.record("Immersion medium = ", immersionMedium);
		datalog.record("Correction collar = ", collar);
		datalog.record("\nFPGA---------------------------------------------------------");
		datalog.record("FPGA clock (MHz) = ", tickPerUs);
		datalog.record("\nLASER--------------------------------------------------------");
		datalog.record("Laser wavelength (nm) = ", wavelength_nm);
		datalog.record("Laser power first frame (mW) = ", Pif_mW.front());
		datalog.record("Laser power last frame (mW) = ", Pif_mW.back());
		datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod);
		datalog.record("\nSCAN---------------------------------------------------------");
		datalog.record("RS FFOV (um) = ", RScanner.mFFOV_um);
		datalog.record("RS period (us) = ", 2 * halfPeriodLineclock_us);
		datalog.record("Pixel dwell time (us) = ", RTsequence.mDwell_us);
		datalog.record("RS fill factor = ", RScanner.mFillFactor);
		datalog.record("Galvo FFOV (um) = ", FFOVgalvo_um);
		datalog.record("Galvo time step (us) = ", galvoTimeStep);
		datalog.record("\nIMAGE--------------------------------------------------------");
		datalog.record("Max count per pixel = ", RTsequence.mPulsesPerPixel);
		datalog.record("8-bit upscaling factor = ", RTsequence.mUpscaleU8);
		datalog.record("Width X (RS) (pix) = ", RTsequence.mWidthPerFrame_pix);
		datalog.record("Height Y (galvo) (pix) = ", RTsequence.mHeightPerFrame_pix);
		datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes_umPerPix);
		datalog.record("Resolution Y (galvo) (um/pix) = ", FFOVgalvo_um / RTsequence.mHeightPerFrame_pix);
		datalog.record("\nSTAGE--------------------------------------------------------");
	}

	//CREATE CONTAINERS FOR STORING THE STACK OF TIFFs
	TiffU8 stackDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ);
	TiffU8 stackSameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ);

	//SELECT A POCKELS AND OPEN THE UNIBLITZ SHUTTERS
	PockelsCell pockels(pockelsFidelity);
	pockels.setShutter(true);

	//ACQUIRE FRAMES AT DIFFERENT Zs
	for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
	{
		stage.moveStage3(stagePosition_mm.at(iterDiffZ));
		stage.waitForMotionToStop3();
		stage.printPosition3();		//Print the stage position
		//P_mW += 0.5;				//Increase the laser power by this amount

		//Acquire many frames at the same Z via discontinuous acquisition
		for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
		{
			std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
				"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << "\n";

			pockels.pushPowerSinglet(8 * us, P_mW, OVERRIDE);	//Override the previous laser power

			//EXECUTE THE RT SEQUENCE
			Image image(RTsequence);
			image.acquire();			//Execute the RT sequence and acquire the image
			image.mirrorOddFrames();
			image.average();			//Average the frames acquired via continuous acquisition
			stackSameZ.pushImage(iterSameZ, image.accessTiff());

			if (acqMode == SINGLEMODE || acqMode == CONTMODE)
			{
				//Save individual files
				std::string singleFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_" + toString(P_mW, 1) + "mW" +
					"_x=" + toString(stagePosition_mm.at(iterDiffZ).at(XX), 3) + "_y=" + toString(stagePosition_mm.at(iterDiffZ).at(YY), 3) + "_z=" + toString(stagePosition_mm.at(iterDiffZ).at(ZZ), 4));
				image.saveTiffSinglePage(singleFilename, overrideFlag);
				Sleep(500);
			}
		}
		stackSameZ.average();	//Average the frames acquired via discontinuous acquisition
		stackDiffZ.pushImage(iterDiffZ, stackSameZ.accessTiff());

		std::cout << "\n";
		P_mW += (Pif_mW.back() - Pif_mW.front()) / nDiffZ;
	}
	pockels.setShutter(false);

	if (acqMode == AVGMODE || acqMode == STACKMODE || acqMode == STACKCENTEREDMODE)
	{
		//Save the stackDiffZ to a file
		std::string stackFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(Pif_mW.front(), 1) + "mW_Pf=" + toString(Pif_mW.back(), 1) + "mW" +
			"_x=" + toString(stagePosition_mm.front().at(XX), 3) + "_y=" + toString(stagePosition_mm.front().at(YY), 3) +
			"_zi=" + toString(stagePosition_mm.front().at(ZZ), 4) + "_zf=" + toString(stagePosition_mm.back().at(ZZ), 4) + "_Step=" + toString(stepSize_um / 1000, 4));

		stackDiffZ.saveToFile(stackFilename, MULTIPAGE, overrideFlag);
	}

}

void seq_testGalvo(const FPGAns::FPGA &fpga)
{
	const int width_pix = 300;
	const int height_pix = 400;
	const int nFramesDiscont = 1;
	const int nFramesCont = 2;

	//GALVO
	const double FFOVgalvo_um = 200 * um;				//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	//CREATE A REAL-TIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga, FG, nFramesCont, width_pix, height_pix);

	//GALVO FOR RT
	Galvo galvo(RTsequence, GALVO1);
	const double duration = halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);			//Linear ramp for the galvo

	for (int iter = 0; iter < nFramesDiscont; iter++)
	{
		std::cout << "Iteration: " << iter + 1 << "\n";

		//Execute the realtime sequence and acquire the image
		Image image(RTsequence);
		image.acquire(); //Execute the RT sequence and acquire the image
		image.saveTiffSinglePage("Untitled", OVERRIDE);
	}
}

void seq_testPixelclock(const FPGAns::FPGA &fpga)
{
	std::vector<unsigned char> stackOfAverages;

	FPGAns::RTsequence RTsequence(fpga); 		//Create a realtime sequence
	Image image(RTsequence);
	image.acquire();						//Execute the realtime sequence and acquire the image
	//image.pushToVector(stackOfAverages);
	//std::cout << "size: " << stackOfAverages.size() << "\n";
	//TiffU8 acqParam(stackOfAverages, 300, 400);
	//acqParam.saveTiff("Untitled");
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAns::FPGA &fpga)
{
	FPGAns::RTsequence RTsequence(fpga);

	//DO
	RTsequence.pushDigitalSinglet(DODEBUG, 4 * us, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, 4 * us, 0);

	//AO
	RTsequence.pushAnalogSinglet(GALVO1, 8 * us, 4);
	RTsequence.pushAnalogSinglet(GALVO1, 4 * us, 2);
	RTsequence.pushAnalogSinglet(GALVO1, 4 * us, 1);

	RTsequence.triggerRT();	//Execute the realtime sequence
}

void seq_testAOramp(const FPGAns::FPGA &fpga)
{
	const double Vmax = 5;
	const double step = 4 * us;

	FPGAns::RTsequence RTsequence(fpga);
	RTsequence.pushLinearRamp(GALVO1, step, 2 * ms, 0, -Vmax);
	RTsequence.pushLinearRamp(GALVO1, step, 20 * ms, -Vmax, Vmax);
	RTsequence.pushLinearRamp(GALVO1, step, 2 * ms, Vmax, 0);

	const double pulsewidth = 300 * us;
	RTsequence.pushDigitalSinglet(DODEBUG, pulsewidth, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, 4 * us, 0);
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAns::FPGA &fpga)
{
	const double step = 400 * us;

	FPGAns::RTsequence RTsequence(fpga);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 0);
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAns::FPGA &fpga)
{
	const double step = 4 * us;

	FPGAns::RTsequence RTsequence(fpga);

	RTsequence.pushDigitalSinglet(DODEBUG, step, 1);

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		RTsequence.pushDigitalSinglet(DODEBUG, step, 0);

	RTsequence.pushDigitalSinglet(DODEBUG, step, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 0);
}

//First calibrate the digital channels, then use it as a time reference
void seq_calibAnalogLatency(const FPGAns::FPGA &fpga)
{
	const double delay = 400 * us;
	const double step = 4 * us;

	FPGAns::RTsequence RTsequence(fpga);
	RTsequence.pushAnalogSinglet(GALVO1, step, 10);						//Initial pulse
	RTsequence.pushAnalogSinglet(GALVO1, step, 0);
	RTsequence.pushLinearRamp(GALVO1, 4 * us, delay, 0, 5 * V);			//Linear ramp to accumulate the error
	RTsequence.pushAnalogSinglet(GALVO1, step, 10);						//Initial pulse
	RTsequence.pushAnalogSinglet(GALVO1, step, 0);						//Final pulse

	//DO0
	RTsequence.pushDigitalSinglet(DODEBUG, step, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 0);
	RTsequence.pushDigitalSinglet(DODEBUG, delay, 0);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 1);
	RTsequence.pushDigitalSinglet(DODEBUG, step, 0);
}

void seq_testFilterwheel()
{
	Filterwheel FWexcitation(FWEXC);
	Filterwheel FWdetection(FWDET);
		
	if (0)
	{
		FWdetection.setColor(1040);
		FWexcitation.setColor(1040);
	}
	else
	{
		FWdetection.setColor(750);
		FWexcitation.setColor(750);
	}
}

void seq_testShutter(const FPGAns::FPGA &fpga)
{
	//CREATE A REALTIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga);

	PockelsCell fidelity(RTsequence, FIDELITY, 1040);
	fidelity.setShutter(true);
	Sleep(5000);
	fidelity.setShutter(false);

	//Shutter shutterFidelity(fpga, FIDELITY);
	//shutterFidelity.open();
	//Sleep(5000);
	//shutterFidelity.close();
}

void seq_testStagePosition()
{
	double duration;
	const double3 stagePosition0_mm = { 35.020, 19.808, 18.542 };	//Stage initial position
	Stage stage;

	std::cout << "Stages initial position:" << "\n";
	stage.printPosition3();

	auto t_start = std::chrono::high_resolution_clock::now();

	stage.moveStage3(stagePosition0_mm);

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << "\n";

	stage.waitForMotionToStop(ZZ);

	std::cout << "Stages final position:" << "\n";
	stage.printPosition3();
	
	/*
	int input = 1;
	while (input)
	{
		std::cout << "Stage X position = " << stage.downloadPosition_mm(XX) << "\n";
		std::cout << "Stage Y position = " << stage.downloadPosition_mm(YY) << "\n";
		std::cout << "Stage X position = " << stage.downloadPosition_mm(ZZ) << "\n";

		std::cout << "Enter command: ";
		std::cin >> input;
		//input = 0;
	}
	*/
}

//Test configuring setDOenable and CTO for the stages
void seq_testStageConfig()
{
	Stage stage;

	//std::cout << "Stages initial position:" << "\n";
	//stage.printPosition3();
	//stage.isDOenable(XX, 1);
	//stage.isDOenable(ZZ, 1);
	//stage.setDOenable(ZZ, 1, 1);
	//stage.downloadDOconfig(ZZ, 1, 1);
	//std::cout << "x stage vel: " << stage.qVEL(XX) << " mm/s" << "\n";
	//std::cout << "y stage vel: " << stage.qVEL(YY) << " mm/s" << "\n";
	//std::cout << "z stage vel: " << stage.qVEL(ZZ) << " mm/s" << "\n";
	stage.printStageConfig(ZZ, 1);
	//stage.downloadConfiguration(ZZ, 2);

	std::cout << "Press any key to continue...\n";
	getchar();
}

void seq_testPMT16X()
{
	PMT16X pmt;
	pmt.readAllGain();
	//pmt.setSingleGain(2, 300);
	//pmt.setAllGain(255);
	//pmt.readTemp();
	//pmt.setAllGain({ 100,255,255,255,255,255,255,255,255,255,255,255,255,255,100,255});

	std::cout << "Press any key to continue...\n";
	getchar();
}

void seq_testLaser(const FPGAns::FPGA &fpga)
{
	Laser laser(VISION);
	//Laser laser(FIDELITY);
	laser.setShutter(false);
	//laser.setWavelength(940);
	//laser.printWavelength_nm();

	std::cout << "Press any key to continue...\n";
	getchar();
}

void seq_testVirtualLaser(const FPGAns::FPGA &fpga)
{
	//CREATE A REALTIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga);

	VirtualLaser laser(RTsequence, 1030, 10 * mW);
	laser.setShutter(true);
	Sleep(3000);
	laser.setShutter(false);

	std::cout << "Press any key to continue...\n";
	getchar();
}

//For keeping the pockels on to check the the laser power
//0. Make sure that the function generator feeds the lineclock
//1. Manually open the Vision shutter and Uniblitz shutter. The latter because the class destructor closes the shutter automatically
//2. Set pockelsAutoOff = DISABLE for holding the last value
//3. Tune Vision's wavelength manually
void seq_testPockels(const FPGAns::FPGA &fpga)
{
	//CREATE A REALTIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga);

	//DEFINE THE POCKELS CELLS
	PockelsCell pockelsVision(RTsequence, VISION, 1040);			//Vision
	PockelsCell pockelsFidelity(RTsequence, FIDELITY, 1040);		//Fidelity

	//DEFINE A POCKELS CELL DYNAMICALLY THROUGH THE COPY CONTRUCTOR
	//PockelsCell pockels(pockelsVision);
	PockelsCell pockels(pockelsFidelity);
	pockels.pushPowerSinglet(400 * us, 30 * mW);
	pockels.pushPowerSinglet(8 * us, 0 * mW);
	//pockels.pushVoltageSinglet(8 * us, 0.0 * V);

	//LOAD AND EXECUTE THE SEQUENCE ON THE FPGA
	pockels.setShutter(true);
	Image image(RTsequence);
	image.acquire();
	pockels.setShutter(false);

	std::cout << "Press any key to continue...\n";
	getchar();
}

void seq_testRS(const FPGAns::FPGA &fpga)
{
	ResonantScanner RScanner(fpga);
	std::cout << "aaa = " << RScanner.downloadControl_V() << "\n";
	//RScanner.turnOn_um(150);
	//RScanner.turnOff();
}

void seq_testConvertI16toVolt()
{
	std::cout << "volt to I16: " << FPGAns::convertVoltToI16(1) << "\n";
	std::cout << "I16 to colt: " << FPGAns::convertI16toVolt(32767) << "\n";
	std::cout << "volt to I16 to volt: " << FPGAns::convertI16toVolt(FPGAns::convertVoltToI16(0)) << "\n";
}

void seq_testTiffU8()
{
	std::string inputFilename("Beads_4um_750nm_50mW_x=35.120_y=19.808_z=18.4610");
	std::string outputFilename("test");

	const int nFramesCont = 10;
	TiffU8 image(inputFilename, nFramesCont);
	
	image.mirrorOddFrames();
	//image.average();
	image.averageEvenOdd();
	image.saveToFile(outputFilename, MULTIPAGE, OVERRIDE);


	//image.saveToFile(outputFilename, 2);

	//image.mirrorOddFrames(nFramesCont);
	//image.averageEvenOdd(nFramesCont);
	
}

//To measure the saving speed of a Tiff file, either locally or remotely
//Select a local or remote folder accordingly
void seq_testEthernetSpeed()
{
	std::string filename = "testEthernetSpeed";

	//The goal is to stream a stackDiffZ composed of 200 z-planes (100 um in 0.5 um-steps), where each frame has 400x400 pixels. Therefore, the stackDiffZ has 400x400x200 = 32 Mega pixels
	//The stackDiffZ size is 8 bits x 32M = 32 MB
	const int width = 400;
	const int height = 400;
	const int nFramesCont = 200;

	TiffU8 image(width, height, nFramesCont);

	//Declare and start a stopwatch
	double duration;
	auto t_start = std::chrono::high_resolution_clock::now();

	//overriding the file saving has some overhead
	//Splitting the stackDiffZ into a page structure (by assigning nFramesCont = 200 in saveToFile) gives a large overhead
	image.saveToFile(filename, MULTIPAGE, OVERRIDE);
	   	 
	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << "\n";

}

void seq_testVibratome(const FPGAns::FPGA &fpga)
{
	Vibratome vibratome(fpga);
	//vibratome.reset(20);
	vibratome.cutAndRetract(20);

	std::cout << "Press any key to continue...\n";
	getchar();
}

//The pc triggers the z-stage motion, then the position of the stage triggers the control sequence and data acquisition
//Remember that I am not using MACROS on the stages anymore
//With the PI monitor for the step reponse I see that the motion start and end of the stage is somewhat nonlinear (+/- 1 um off target).
//1. My current sol is to first trigger the stage motion, then after a certain fixed distance use the stage DO to trigger the data acquisition. The problem right now is that I can not find a way to reset the stage DO
void seq_testStageTrigAcq(const FPGAns::FPGA &fpga)
{
	//ACQUISITION SETTINGS
	const int width = 300;
	const int height = 400;
	const int nFramesCont = 100;		//Number of frames for continuous acquisition

	//STAGES
	const double3 stagePosition0_mm = { 36.0, 14.2, 18.415 - 0.020 };	//Stage initial position
	Stage stage;
	stage.moveStage3(stagePosition0_mm);
	stage.waitForMotionToStop3();

	//LASER
	const int wavelength_nm = 750;
	double laserPower_mW = 40 * mW;
	Laser vision(VISION);
	vision.setWavelength(wavelength_nm);

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);

	//FILTERWHEEL
	Filterwheel fw(FWDET);
	fw.setColor(wavelength_nm);

	//CREATE THE REAL-TIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga, RS, nFramesCont, width, height, STAGETRIG);	//Notice the STAGETRIG flag

	//GALVO FOR RT
	const double FFOVgalvo_um = 200 * um;	//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;
	Galvo galvo(RTsequence, GALVO1);
	const double duration = 62.5 * us * RTsequence.mHeightPerFrame_pix;				//= halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo

	//POCKELS CELL FOR RT
	PockelsCell pockelsVision(RTsequence, VISION, wavelength_nm);
	pockelsVision.pushPowerSinglet(8 * us, laserPower_mW);

	//OPEN THE SHUTTER
	pockelsVision.setShutter(true);
	Sleep(50);

	//EXECUTE THE RT SEQUENCE
	Image image(RTsequence);
	image.initialize();

	//Move the stage, which will trigger the control sequence and data acquisition
	stage.moveStage_(ZZ, stagePosition0_mm.at(ZZ) + 0.060); //Why did I used 60 um and not 100 um?
	
	image.startFIFOOUTpc();
	image.download();
	image.mirrorOddFrames();
	image.saveTiffMultiPage("Untitled");

	stage.waitForMotionToStop3();
	pockelsVision.setShutter(false);

	//Disable ZstageAsTrigger to be able to move the stage without triggering the acquisition sequence
	NiFpga_WriteBool(fpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable, false);
}

void seq_testSequencer()
{
	//Generate the command list and keep it in memory.
	//I prefer generating such list before execution because then I can inspect all the parameters offline

	//Configure the sample
	const ROI roi_mm = { 0, 10, 10, 0 };
	const double sampleLengthZ_mm = 10;
	SampleConfig sampleConfig("Beads_4um", "grycerol_1.47", roi_mm, sampleLengthZ_mm);

	//Configure the lasers {wavelength_nm, laser power mW, laser power incremental mW}
	using SingleLaserConfig = LaserListConfig::SingleLaserConfig;
	const SingleLaserConfig blueLaser{ 750, 10, 5 };
	const SingleLaserConfig greenLaser{ 940, 11, 6 };
	const SingleLaserConfig redLaser{ 1040, 12, 7 };
	const std::vector<SingleLaserConfig> laserList{ blueLaser, greenLaser, redLaser };
	
	//Configure the stacks
	const double2 FOV_um = { 150,200 };
	const double stepSizeZ_um = 0.5;					//Image resolution in z
	const double stackDepth_um = 100;					//Stack depth or thickness
	const double3 stackOverlap_frac = { 0.1,0.1,0.1 };	//Percentage of stack overlap in x, y, and z
	StackConfig stackConfig(FOV_um, stepSizeZ_um, stackDepth_um, stackOverlap_frac);

	//Configure the vibratome
	const double cutAboveBottomOfStack_um = 9;			//Cut this much above the bottom of the stack
	const VibratomeConfig vibratomeConfig(cutAboveBottomOfStack_um);

	//Configure the stages
	const double stageInitialZ_mm = 10;					//Initial height of the stage
	StageConfig stageConfig(stageInitialZ_mm);

	//Create a sequence
	Sequencer sequence(sampleConfig, laserList, stackConfig, vibratomeConfig, stageConfig);
	sequence.generateCommandList();
	sequence.printToFile("Commandlist");

	if (0)
	{
		//Read the commands line by line
		for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != 10; iterCommandline++)
		{
			Commandline commandline = sequence.getCommandline(iterCommandline);
			commandline.printParameters();

			//Initialize to unreal values for safety
			double scanZi_mm = -1, stackDepth_mm = -1, scanPi_mW = -1, stackPinc_mW = -1;
			double2 stackCenter_mm{1000,1000};
			int wavelength_nm = -1;

			switch (commandline.mAction)
			{
			case MOV:
				//Move the x and y stages to stackCenter_mm
				stackCenter_mm = commandline.mCommand.moveStage.stackCenter_mm;
				break;
			case ACQ:
				//Acquire a stack using the parameters:
				wavelength_nm = commandline.mCommand.acqStack.wavelength_nm;
				scanZi_mm = commandline.mCommand.acqStack.scanZi_mm;
				stackDepth_mm = commandline.mCommand.acqStack.stackDepth_um;
				scanPi_mW = commandline.mCommand.acqStack.scanPi_mW;
				stackPinc_mW = commandline.mCommand.acqStack.stackPinc_mW;
			case SAV:
				//Save the stack to file and label it with the scan parameters:
				wavelength_nm, scanZi_mm, stackDepth_mm, scanPi_mW, stackPinc_mW;
				stackCenter_mm;
				break;
			case CUT:
				//Move the stage to
				double3 stagePosition_mm = commandline.mCommand.cutSlice.samplePosition_mm;
				//and then cut a slice off
				break;
			default:
				throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
			}
		}
	}


	//std::cout << "Press any key to continue...\n";
	//getchar();
}