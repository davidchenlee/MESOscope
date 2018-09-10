#include "Sequences.h"

void seq_main(const FPGAns::FPGA &fpga)
{
	const RunMode acqMode = SINGLEMODE;			//Single shot
	//const RunMode acqMode = CONTMODE;				//Image the same z plane many times
	//const RunMode acqMode = AVGMODE;				//Image the same z plane many times for averaging
	//const RunMode acqMode = STACKMODE;				//Stack volume from the initial z position
	//const RunMode acqMode = STACKCENTEREDMODE;	//Stack volume centered at the initial z position

	//ACQUISITION SETTINGS
	const int widthPerFrame_pix = 300;
	const int heightPerFrame_pix = 400;
	const int nFramesCont = 1;									//Number of frames for continuous acquisition
	const double3 stagePosition0_mm = { 36.0, 14.2, 18.415 };	//Stage initial position. For 5% overlap: x=+-0.190, y=+-0.142

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);
	RScanner.isRunning();

	//STACK
	const double stepSize_um = 0.5 * um;
	double zDelta_um = 10 * um;				//Acquire a stack covering this interval

	//LASER
	const int wavelength_nm = 750;
	const std::vector<double> Pif_mW = { 30, 30};		//For 750 nm over 200 um
	//const std::vector<double> Pif_mW = { 70 , 100 };	//For 1040 nm over 200 um
	double P_mW = Pif_mW.front();
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//SAMPLE
	const std::string sampleName("Bead4um");
	const std::string immersionMedium("Glycerol");
	const double collar = 1.47;

	//FILTERWHEEL
	Filterwheel fw(FW1);
	fw.setColor(wavelength_nm);

	//Stages
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
		nDiffZ = (int)(zDelta_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(xx), stagePosition0_mm.at(yy), stagePosition0_mm.at(zz) + iterDiffZ * stepSize_um / 1000 });

		break;
	case STACKCENTEREDMODE:
		nSameZ = 1;
		nDiffZ = (int)(zDelta_um / stepSize_um);
		overrideFlag = NOOVERRIDE;
		//Push the stage sequence
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			stagePosition_mm.push_back({ stagePosition0_mm.at(xx), stagePosition0_mm.at(yy), stagePosition0_mm.at(zz) - 0.5 * zDelta_um / 1000 + iterDiffZ * stepSize_um / 1000 });

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
	PockelsCell pockels(RTsequence, POCKELS1, wavelength_nm);
	//pockels.voltageLinearRamp(4*us, 40*us, 0, 1*V);
	//pockels.voltageLinearRamp(galvoTimeStep, duration, 0.5*V, 1*V);	//Ramp up the laser intensity in a frame and repeat for each frame
	//pockels.scalingLinearRamp(1.0, 2.0);								//Linearly scale the laser intensity across all the frames

	//Datalog
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
	}

	//Create container-stacks for storing the Tiff images
	TiffU8 stackDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ);
	TiffU8 stackSameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ);

	//OPEN THE SHUTTER
	Shutter shutterVision(fpga, SHUTTER1);
	shutterVision.open();
	Sleep(50);

	//Acquire frames at different Z
	for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
	{
		stage.moveStage3(stagePosition_mm.at(iterDiffZ));
		stage.waitForMotionToStop3();
		stage.printPosition3();		//Print the stage position
		//P_mW += 0.5;				//Increase the laser power by this much

		//Acquire many frames at the same Z via discontinuous acquisition
		for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
		{
			std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
				"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << std::endl;

			pockels.pushPowerSinglet(8 * us, P_mW, OVERRIDE);	//OverrideFileSelector the previous laser power

			//EXECUTE THE RT SEQUENCE
			Image image(RTsequence);
			image.acquire();			//Execute the RT sequence and acquire the image
			image.mirrorOddFrames();
			image.average();			//Average the frames acquired via continuous acquisition
			stackSameZ.pushImage(iterSameZ, image.accessTiff());

			if (acqMode == SINGLEMODE || acqMode == CONTMODE)
			{
				//Save individual files
				std::string singleFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_" + toString(P_mW, 0) + "mW" +
					"_x=" + toString(stagePosition_mm.at(iterDiffZ).at(xx), 3) + "_y=" + toString(stagePosition_mm.at(iterDiffZ).at(yy), 3) + "_z=" + toString(stagePosition_mm.at(iterDiffZ).at(zz), 4));
				image.saveTiffSinglePage(singleFilename, overrideFlag);
				Sleep(500);
			}
		}
		stackSameZ.average();	//Average the frames acquired via discontinuous acquisition
		stackDiffZ.pushImage(iterDiffZ, stackSameZ.accessTiff());

		std::cout << std::endl;
		P_mW += (Pif_mW.back() - Pif_mW.front()) / nDiffZ;
	}
	shutterVision.close();

	if (acqMode == AVGMODE || acqMode == STACKMODE || acqMode == STACKCENTEREDMODE)
	{
		//Save the stackDiffZ to a file
		std::string stackFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(Pif_mW.front(), 1) + "mW_Pf=" + toString(Pif_mW.back(), 1) + "mW" +
			"_x=" + toString(stagePosition_mm.front().at(xx), 3) + "_y=" + toString(stagePosition_mm.front().at(yy), 3) +
			"_zi=" + toString(stagePosition_mm.front().at(zz), 4) + "_zf=" + toString(stagePosition_mm.back().at(zz), 4) + "_Step=" + toString(stepSize_um / 1000, 4));

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
		std::cout << "Iteration: " << iter + 1 << std::endl;

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
	//std::cout << "size: " << stackOfAverages.size() << std::endl;
	//TiffU8 aa(stackOfAverages, 300, 400);
	//aa.saveTiff("Untitled");
}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAns::FPGA &fpga)
{
	FPGAns::RTsequence RTsequence(fpga);

	//DO
	RTsequence.pushDigitalSinglet(DOdebug, 4 * us, 1);
	RTsequence.pushDigitalSinglet(DOdebug, 4 * us, 0);

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
	RTsequence.pushDigitalSinglet(DOdebug, pulsewidth, 1);
	RTsequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAns::FPGA &fpga)
{
	const double step = 400 * us;

	FPGAns::RTsequence RTsequence(fpga);
	RTsequence.pushDigitalSinglet(DOdebug, step, 1);
	RTsequence.pushDigitalSinglet(DOdebug, step, 0);
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAns::FPGA &fpga)
{
	const double step = 4 * us;

	FPGAns::RTsequence RTsequence(fpga);

	RTsequence.pushDigitalSinglet(DOdebug, step, 1);

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		RTsequence.pushDigitalSinglet(DOdebug, step, 0);

	RTsequence.pushDigitalSinglet(DOdebug, step, 1);
	RTsequence.pushDigitalSinglet(DOdebug, step, 0);
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
	RTsequence.pushDigitalSinglet(DOdebug, step, 1);
	RTsequence.pushDigitalSinglet(DOdebug, step, 0);
	RTsequence.pushDigitalSinglet(DOdebug, delay, 0);
	RTsequence.pushDigitalSinglet(DOdebug, step, 1);
	RTsequence.pushDigitalSinglet(DOdebug, step, 0);
}

void seq_testFilterwheel()
{
	Filterwheel FW(FW1);
	//FW.setColor(RED);
	
	if(1)
		FW.setColor(1040);
	else
		FW.setColor(940);
}

void seq_testStagePosition()
{
	double duration;
	const double3 stagePosition0_mm = { 35.020, 19.808, 18.542 };	//Stage initial position
	Stage stage;

	std::cout << "Stages initial position:" << std::endl;
	stage.printPosition3();

	auto t_start = std::chrono::high_resolution_clock::now();

	stage.moveStage3(stagePosition0_mm);

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

	stage.waitForMotionToStop(zz);

	std::cout << "Stages final position:" << std::endl;
	stage.printPosition3();
	
	/*
	int input = 1;
	while (input)
	{
		std::cout << "Stage X position = " << stage.downloadPosition_mm(xx) << std::endl;
		std::cout << "Stage Y position = " << stage.downloadPosition_mm(yy) << std::endl;
		std::cout << "Stage X position = " << stage.downloadPosition_mm(zz) << std::endl;

		std::cout << "Enter command: ";
		std::cin >> input;
		//input = 0;
	}
	*/
}

void seq_testmPMT()
{
	mPMT pmt;
	pmt.readAllGain();
	//pmt.setSingleGain(2, 300);
	//pmt.setAllGain(255);
	//pmt.readTemp();
	//pmt.setAllGain({ 100,255,255,255,255,255,255,255,255,255,255,255,255,255,100,255});
}

//Keep the pockels on to check the setpoint of the laser power
//1. Manually open the laser and Uniblitz shutters
//2. Set pockels1AutoOff = DISABLE
//3. Tune the laser wavelength manually
void seq_testPockels(const FPGAns::FPGA &fpga)
{
	//Create a realtime sequence
	FPGAns::RTsequence RTsequence(fpga);

	//Turn on the pockels cell
	PockelsCell pockels(RTsequence, POCKELS1, 750);
	pockels.pushPowerSinglet(8 * us, 30 * mW);
	//pockels.pushVoltageSinglet_(8 * us, 2.508 * V);

	{
		//Execute the sequence
		Image image(RTsequence);
		image.acquire();
	}
}

void seq_testLaserComm(const FPGAns::FPGA &fpga)
{
	Laser vision;
	//vision.setShutter(0);
	vision.setWavelength(940);
}

void seq_testRS(const FPGAns::FPGA &fpga)
{
	ResonantScanner RScanner(fpga);
	std::cout << "aaa = " << RScanner.downloadControl_V() << std::endl;
	//RScanner.turnOn_um(150);
	//RScanner.turnOff();
}

void seq_testConvertI16toVolt()
{
	std::cout << "volt to I16: " << FPGAns::convertVoltToI16(1) << std::endl;
	std::cout << "I16 to colt: " << FPGAns::convertI16toVolt(32767) << std::endl;
	std::cout << "volt to I16 to volt: " << FPGAns::convertI16toVolt(FPGAns::convertVoltToI16(0)) << std::endl;
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

//Test configuring TRO_ and CTO for the stages
void seq_testStageConfig()
{
	Stage stage;

	//std::cout << "Stages initial position:" << std::endl;
	//stage.printPosition3();
	//stage.qTRO_(xx, 1);
	//stage.qTRO_(zz, 1);
	//stage.TRO_(zz, 1, 1);
	//stage.qCTO_(zz, 1, 1);
	//stage.qVEL(xx);
	//stage.qVEL(yy);
	//stage.qVEL(zz);
	stage.downloadConfiguration(zz, 1);
	stage.downloadConfiguration(zz, 2);

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
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

}

//The pc triggers the z-stage motion. Its position triggers the control sequence and data acquisition
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
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);

	//FILTERWHEEL
	Filterwheel fw(FW1);
	fw.setColor(wavelength_nm);

	//CREATE THE REAL-TIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga, RS, nFramesCont, width, height, STAGETRIG);

	//GALVO FOR RT
	const double FFOVgalvo_um = 200 * um;	//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;
	Galvo galvo(RTsequence, GALVO1);
	const double duration = 62.5 * us * RTsequence.mHeightPerFrame_pix;				//= halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo

	//POCKELS CELL FOR RT
	PockelsCell pockels(RTsequence, POCKELS1, wavelength_nm);
	pockels.pushPowerSinglet(8 * us, laserPower_mW);

	//OPEN THE SHUTTER
	Shutter shutterVision(fpga, SHUTTER1);
	shutterVision.open();
	Sleep(50);

	//EXECUTE THE RT SEQUENCE
	Image image(RTsequence);
	image.initialize();

	stage.moveStage(zz, stagePosition0_mm.at(zz) + 0.060); //Move the stage, which will trigger the control sequenceand data acquisition
	
	image.startFIFOOUTpc();
	image.download();
	image.mirrorOddFrames();
	image.saveTiffMultiPage("Untitled");

	stage.waitForMotionToStop3();
	shutterVision.close();

	//Disable ZstageAsTrigger to position the stage in the next run without triggering the sequence
	NiFpga_WriteBool(fpga.getFpgaHandle(), NiFpga_FPGAvi_ControlBool_ZstageAsTriggerEnable, false);

}