#include "Sequences.h"

/*
There are basically 2 imaging modes :
1. Frame by frame: For each frame, a RT sequence is created, loaded onto the fpga, and a corresponding image is acquired. The z stage is moved after each image is acquired.
2. Continuous: A single long RT sequence contains all the frames. Such sequence is loaded onto the fpga and run once. A stream of images is acquired. The z stage moves continuously
*/

void seq_main(const FPGAns::FPGA &fpga)
{
	const int runmode = 4;
	/*
	0 - Single shot
	1 - Continuous: image the same plane many times
	2 - Average: Image many times the same plane for averaging
	3 - Stack volume from the initial z position
	4 - Stack volume around the initial z position
	*/
	const RunMode runMode = static_cast<RunMode>(runmode);

	//STAGE
	//double3 position_mm = { 37.950, 29.150, 16.950 };	//Initial position
	double3 position_mm = { 35.120, 19.808, 18.507 };	//Initial position

	//STACK
	const double stepSize_um = 0.5 * um;
	double zDelta_um = 5 * um;				//Acquire a stack within this interval

	//LASER
	const int wavelength_nm = 750;
	double laserPower_mW = 40 * mW;
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//GALVO
	const double FFOVgalvo_um = 200 * um;	//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RScanner(fpga);
	RScanner.setFFOV(FFOVrs_um);

	//SAMPLE
	const std::string sampleName = "Beads_4um";
	const double collar = 1.47;

	//FILTERWHEEL
	Filterwheel fw(FW1);
	fw.setColor(wavelength_nm);

	//ACQUISITION SETTINGS
	const int nFrames = 10;
	const int widthSingleFrame_pix = 300;
	const int heightSingleFrame_pix = 400;
	int nDiffZ;
	int nSameZ;
	bool overrideFlag;
	switch (runMode)
	{
	case single:
		nSameZ = 1;
		nDiffZ = 1; //Do not change this
		overrideFlag = FALSE;
		break;
	case continuous:
		nSameZ = 500;
		nDiffZ = 1; //Do not change this
		overrideFlag = TRUE;
		break;
	case average:
		nSameZ = 10;
		nDiffZ = 1; //Do not change this
		overrideFlag = FALSE;
		break;
	case stack:
		nSameZ = 1;
		nDiffZ = (int)(zDelta_um / stepSize_um);
		overrideFlag = FALSE;
		break;
	case stack_centered:
		nSameZ = 1;
		nDiffZ = (int)(zDelta_um / stepSize_um);
		position_mm.at(zz) -= 0.5 * zDelta_um / 1000; //Shift the stage to the middle of the interval
		overrideFlag = FALSE;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
	}

	//PRESET THE STAGES
	Stage stage;
	stage.moveStage3(position_mm);
	stage.waitForMovementToStop3();

	//OPEN THE SHUTTER
	Shutter shutter1(fpga, Shutter1);
	shutter1.open();
	Sleep(50);

	//Store the average associated with each z plane
	TiffU8 stackOfAverages(widthSingleFrame_pix, heightSingleFrame_pix * nDiffZ);

	//Frames at different Z
	for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
	{
		stage.printPosition3();		//Print the stage position

		//Frames at the same Z
		for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
		{
			std::cout << "Frame # (diff-Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same-Z): " << (iterSameZ + 1) << "/" << nSameZ <<
				"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << std::endl;

			//CREATE THE REAL-TIME SEQUENCE
			FPGAns::RTsequence RTsequence(fpga, RS, nFrames, widthSingleFrame_pix, heightSingleFrame_pix);

			//GALVO FOR RT
			Galvo galvo(RTsequence, GALVO1);
			const double duration = 62.5 * us * RTsequence.mHeightPerFrame_pix;				//= halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix
			galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo
			if ( RTsequence.mNframes == 1 )
				galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);		//set the output back to the initial value

			//POCKELS CELL FOR RT
			PockelsCell pockels(RTsequence, POCKELS1, wavelength_nm);
			pockels.pushPowerSinglet(8 * us, laserPower_mW);
			//pockels.voltageLinearRamp(4*us, 40*us, 0, 1*V);
			//pockels.voltageLinearRamp(galvoTimeStep, duration, 0.5*V, 1*V);	//Ramp up the laser intensity in a frame and repeat for each frame
			//pockels.scalingLinearRamp(1.0, 2.0);								//Linearly scale the laser intensity across all the frames

			//EXECUTE THE RT SEQUENCE
			std::string filename = sampleName + "_" + toString(wavelength_nm, 0) + "nm_" + toString(laserPower_mW, 0) + "mW" +
				"_x=" + toString(position_mm.at(xx), 3) + "_y=" + toString(position_mm.at(yy), 3) + "_z=" + toString(position_mm.at(zz), 4);

			Image image(RTsequence);
			image.acquire(); //Execute the RT sequence and acquire the image
			image.mirrorVertical();
			image.average();
			//image.saveTiff(filename, overrideFlag);
			//image.saveTxt(filename);
			stackOfAverages.pushImage(iterDiffZ, nDiffZ, image.accessTiff());
			
			if (iterDiffZ == 0 && iterSameZ == 0)
			{
				//DATALOG
				Logger datalog("datalog_" + sampleName);
				datalog.record("SAMPLE-------------------------------------------------------");
				datalog.record("Sample = ", sampleName);
				datalog.record("Correction collar = ", collar);
				datalog.record("FPGA---------------------------------------------------------");
				datalog.record("FPGA clock (MHz) = ", tickPerUs);
				datalog.record("LASER--------------------------------------------------------");
				datalog.record("Laser wavelength (nm) = ", wavelength_nm);
				datalog.record("Laser power (mW) = ", laserPower_mW);
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
		}
		std::cout << std::endl;

		//Move the z stage to the new position
		if (runMode == stack || runMode == stack_centered)
		{
			position_mm.at(zz) += stepSize_um / 1000;
			stage.moveStage(zz, position_mm.at(zz));
			stage.waitForMovementToStop3();
			//Sleep(300);
			//laserPower_mW += 0.5; //Increase the laser power by this much
		}
	}
	shutter1.close();

	//Save the stack to file
	stackOfAverages.saveTiff("StackOfAverages", nDiffZ);
}


//For live optimization of the objective's correction collar
void seq_contAcquisition(const FPGAns::FPGA &fpga)
{
	int nFramesSameZ = 1000;

	//LASER
	const int wavelength_nm = 750;
	double laserPower_mW = 30 * mW;
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//GALVO
	const double FFOVgalvo_um = 300 * um;				//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	//FILTERWHEEL
	Filterwheel fw(FW1);
	fw.setColor(wavelength_nm);

	//SHUTTER
	Shutter shutter1(fpga, Shutter1);

	//SEQUENCE
	shutter1.open();
	Sleep(50);

	for (int jj = 0; jj < nFramesSameZ; jj++)
	{
		std::cout << "Iteration: " << jj+1 << std::endl;

		//CREATE A REAL-TIME SEQUENCE
		FPGAns::RTsequence RTsequence(fpga, RS);

		//GALVO FOR RT
		Galvo galvo(RTsequence, GALVO1);
		const double duration = halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
		galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);			//Linear ramp for the galvo
		galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);				//set the output back to the initial value

		//POCKELS CELL FOR RT
		PockelsCell pockels(RTsequence, POCKELS1, wavelength_nm);
		pockels.pushPowerSinglet(8 * us, laserPower_mW);

		//Execute the realtime sequence and acquire the image
		Image image(RTsequence);
		image.acquire(); //Execute the RT sequence and acquire the image
		image.saveTiff("Untitled", TRUE);
	}
	shutter1.close();
}


void seq_testInterframeTiming(const FPGAns::FPGA &fpga)
{
	const int width_pix = 300;
	const int height_pix = 400;
	const int nFrames = 2;

	//GALVO
	const double FFOVgalvo_um = 300 * um;				//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	for (int jj = 0; jj < nFrames; jj++)
	{
		std::cout << "Iteration: " << jj + 1 << std::endl;

		//CREATE A REAL-TIME SEQUENCE
		FPGAns::RTsequence RTsequence(fpga, FG, nFrames, width_pix, height_pix);

		//GALVO FOR RT
		Galvo galvo(RTsequence, GALVO1);
		const double duration = halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
		galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);			//Linear ramp for the galvo
		if (RTsequence.mNframes == 1)
			galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);				//set the output back to the initial value

		//Execute the realtime sequence and acquire the image
		Image image(RTsequence);
		image.acquire(); //Execute the RT sequence and acquire the image
		image.saveTiff("Untitled", TRUE);
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

void seq_testStageSetPosition()
{
	double duration;
	const double newPosition_mm = 5;
	Stage stage;

	std::cout << "Stages initial position:" << std::endl;
	stage.printPosition3();

	auto t_start = std::chrono::high_resolution_clock::now();

	stage.moveStage(zz, newPosition_mm);
	//stage.waitForMovementToStop(zz);

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

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

void seq_testStageTriggerConfig()
{
	Stage stages;
	stages.printPosition3();
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

//Keep the pockels cell on.
//1. Manually open the laser and Uniblitz shutters
//2. Set pockels1_enableAutoOff = 0
//3. Set lineclockInput = FG
void seq_testPockels(const FPGAns::FPGA &fpga)
{
	//Create a realtime sequence
	FPGAns::RTsequence RTsequence(fpga);

	//Open the Uniblitz shutter
	//Shutter shutter1(fpga, Shutter1);
	//shutter1.open();

	//Turn on the pockels cell
	PockelsCell pockels(RTsequence, POCKELS1, 750);
	pockels.pushPowerSinglet(8 * us, 40 * mW);
	//pockels.pushVoltageSinglet_(8 * us, 2.508 * V);

	//Execute the sequence
	Image image(RTsequence);
	image.acquire();
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

void seq_testGalvoSync(const FPGAns::FPGA &fpga)
{
	//GALVO
	const double FFOVgalvo_um = 200 * um;		//Full FOV in the slow axis
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	//CREATE A REAL-TIME SEQUENCE
	FPGAns::RTsequence RTsequence(fpga, FG, 3);

	//GALVO FOR RT
	Galvo galvo(RTsequence, GALVO1);
	const double duration = halfPeriodLineclock_us * RTsequence.mHeightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
	galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);			//Linear ramp for the galvo
	if ( RTsequence.mNframes == 1 )
		galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);			//Linear ramp for the galvo
	//galvo.pushVoltageSinglet_(2 * us, 1);												//Mark the end of the galvo ramp
																						
	//Execute the realtime sequence and acquire the image
	Image image(RTsequence);
	image.acquire(); //Execute the RT sequence and acquire the image
}

void seq_testTiff()
{
	std::string inputFilename = "Beads_4um_750nm_50mW_x=35.120_y=19.808_z=18.4610";
	std::string outputFilename = "test";

	const int nFrames = 10;
	//TiffU8 image(inputFilename);
	
	//image.mirrorVertical(nFrames);
	//image.averageSeparately(nFrames);
	//image.saveTiff(outputFilename, 2);

	//image.mirrorVertical(nFrames);
	//image.averageSeparately(nFrames);
	//image.saveTiff(outputFilename, 2);//The second argument specifies the number of Frames

	TiffU8 aa(300, 4*300);
	aa.saveTiff("test", 4);


}
