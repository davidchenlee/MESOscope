#include "Sequences.h"

/*
There are basically 2 imaging modes :
1. Frame by frame: For each frame, a RT sequence is created, loaded onto the fpga, and a corresponding image is acquired. The z stage is moved after each image is acquired.
2. Continuous: A single long RT sequence contains all the frames. Such sequence is loaded onto the fpga and run once. A stream of images is acquired. The z stage moves continuously
*/

void seq_main(const FPGAapi::Session &fpga)
{	
	const int runmode = 4;
	/*
	0 - Single image
	1 - Image continuously the same plane
	2 - Image many times the same plane for subsequent averaging
	3 - Stack volume from the initial z position
	4 - Stack volume around the initial z position
	*/
		
	const RunMode runMode = static_cast<RunMode>(runmode);
	
	//STAGE
	//double3 position_mm = { 37.950, 29.150, 16.950 };	//Initial position
	double3 position_mm = { 35.120, 19.808, 18.4160 };	//Initial position

	//STACK
	const double stepSize_um = 0.5 * um;
	double zDelta_um = 5 * um;				//Acquire a stack within this interval

	//LASER
	const int wavelength_nm = 750;
	double laserPower_mW = 50 * mW;
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//GALVO
	const double FFOVgalvo_um = 200 * um;									//Full FOV in the slow axis
	const double duration = halfPeriodLineclock_us * heightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
	const double galvoTimeStep = 8 * us;
	const double posMax_um = FFOVgalvo_um / 2;

	//RS
	const double FFOVrs_um = 150 * um;
	ResonantScanner RS(fpga);
	RS.setFFOV(FFOVrs_um);

	//SAMPLE
	const std::string filename = "Beads_4um";
	const double collar = 1.47;

	//FILTERWHEEL
	Filterwheel fw(FW1);
	fw.setColor(wavelength_nm);

	//ACQUISITION MODE SETTING
	int nFramesStack;
	int nFramesAvg;
	bool overrideFlag;
	switch (runMode)
	{
	case single:
		nFramesAvg = 1;
		nFramesStack = 1; //Do not change this
		overrideFlag = FALSE;
		break;
	case continuous:
		nFramesAvg = 500;
		nFramesStack = 1; //Do not change this
		overrideFlag = TRUE;
		break;
	case average:
		nFramesAvg = 10;
		nFramesStack = 1; //Do not change this
		overrideFlag = FALSE;
		break;
	case stack:
		nFramesAvg = 1;
		nFramesStack = (int)(zDelta_um / stepSize_um);
		overrideFlag = FALSE;
		break;
	case stack_centered:
		nFramesAvg = 1;
		nFramesStack = (int)(zDelta_um / stepSize_um);
		position_mm.at(zz) -= 0.5 * zDelta_um / 1000; //Shift the stage to the middle of the interval
		overrideFlag = FALSE;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
	}

	//DATALOG
	Logger datalog("datalog_" + filename);
	datalog.record("SAMPLE-------------------------------------------------------");
	datalog.record("Sample = ", filename);
	datalog.record("Correction collar = ", collar);

	datalog.record("FPGA---------------------------------------------------------");
	datalog.record("FPGA clock (MHz) = ", tickPerUs);

	datalog.record("LASER--------------------------------------------------------");
	datalog.record("Laser wavelength (nm) = ", wavelength_nm);
	datalog.record("Laser power (mW) = ", laserPower_mW);
	datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod);

	datalog.record("SCAN---------------------------------------------------------");
	datalog.record("RS FFOV (um) = ", RS.mFFOV_um);
	datalog.record("RS period (us) = ", 2 * halfPeriodLineclock_us);
	datalog.record("Pixel dwell time (us) = ", dwell_us);
	datalog.record("RS fill factor = ", RS.mFillFactor);
	datalog.record("Galvo FFOV (um) = ", FFOVgalvo_um);
	datalog.record("Galvo time step (us) = ", galvoTimeStep);


	datalog.record("IMAGE--------------------------------------------------------");
	datalog.record("Max count per pixel = ", pulsesPerPixel);
	datalog.record("8-bit upscaling factor = ", upscaleU8);
	datalog.record("Width X (RS) (pix) = ", widthPerFrame_pix);
	datalog.record("Height Y (galvo) (pix) = ", heightPerFrame_pix);
	datalog.record("Resolution X (RS) (um/pix) = ", RS.mSampRes_umPerPix);
	datalog.record("Resolution Y (galvo) (um/pix) = ", FFOVgalvo_um/heightPerFrame_pix);

	//PRESET THE STAGES
	Stage stage;
	stage.moveStage3(position_mm);
	stage.waitForMovementToStop3();

	//OPEN THE SHUTTER
	Shutter shutter1(fpga, Shutter1);
	shutter1.open();
	Sleep(50);

	for (int ii = 0; ii < nFramesStack; ii++)
	{
		stage.printPosition3();		//Print the stage position

		for (int jj = 0; jj < nFramesAvg; jj++)
		{
			std::cout << "Z-plane " << (ii + 1) << "/" << nFramesStack <<
				"\tFrame " << (jj + 1) << "/" << nFramesAvg <<
				"\tTotal frame " << ii * nFramesAvg + (jj + 1) << "/" << nFramesStack * nFramesAvg << std::endl;

			//CREATE A REAL-TIME SEQUENCE
			FPGAapi::RTsequence sequence(fpga);

			//GALVO FOR RT
			Galvo galvo(sequence, GALVO1);
			galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo
			galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);			//set the output back to the initial value

			//POCKELS CELL FOR RT
			PockelsCell pockels(sequence, POCKELS1, wavelength_nm);
			pockels.pushPowerSinglet(8 * us, laserPower_mW);
			//pockels.voltageLinearRamp(4*us, 40*us, 0, 1*V);
			//pockels.voltageLinearRamp(galvoTimeStep, duration, 0.5*V, 1*V);	//Ramp up the laser intensity in a frame and repeat for each frame
			//pockels.scalingLinearRamp(1.0, 2.0);								//Linearly scale the laser intensity across all the frames

			//Upload the realtime sequence to the FPGA but don't execute it yet
			sequence.uploadRT();

			//Execute the realtime sequence and acquire the image
			Image image(fpga);
			image.acquire(TRUE, filename + "_" + toString(wavelength_nm, 0) + "nm_" + toString(laserPower_mW, 0) + "mW" +
				"_x=" + toString(position_mm.at(xx), 3) + "_y=" + toString(position_mm.at(yy), 3) + "_z=" + toString(position_mm.at(zz), 4), overrideFlag); //Execute the RT sequence and acquire the image
		}
		std::cout << std::endl;

		//Move to the new position z
		if (runMode == stack || runMode == stack_centered)
		{
			position_mm.at(zz) += stepSize_um / 1000;
			stage.moveStage(zz, position_mm.at(zz));
			//stage.waitForMovementToStop3();
			Sleep(500);
			//laserPower_mW += 0.5; //Increase the laser power by this much
		}
	}
	shutter1.close();
}

void seq_contAcquisition(const FPGAapi::Session &fpga)
{
	int nFrames = 1000;

	//LASER
	const int wavelength_nm = 750;
	double laserPower_mW = 30 * mW;
	Laser vision;
	vision.setWavelength(wavelength_nm);

	//GALVO
	const double FFOVgalvo_um = 300 * um;									//Full FOV in the slow axis
	const double duration = halfPeriodLineclock_us * heightPerFrame_pix;	//= 62.5us * 400 pixels = 25 ms
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

	for (int jj = 0; jj < nFrames; jj++)
	{
		std::cout << "Iteration: " << jj+1 << std::endl;

		//CREATE A REAL-TIME SEQUENCE
		FPGAapi::RTsequence sequence(fpga);

		//GALVO FOR RT
		Galvo galvo(sequence, GALVO1);
		galvo.positionLinearRamp(galvoTimeStep, duration, posMax_um, -posMax_um);		//Linear ramp for the galvo
		galvo.positionLinearRamp(galvoTimeStep, 1 * ms, -posMax_um, posMax_um);			//set the output back to the initial value

		//POCKELS CELL FOR RT
		PockelsCell pockels(sequence, POCKELS1, wavelength_nm);
		pockels.pushPowerSinglet(8 * us, laserPower_mW);

		//Upload the realtime sequence to the FPGA but don't execute it yet
		sequence.uploadRT();

		//Execute the realtime sequence and acquire the image
		Image image(fpga);
		image.acquire(TRUE,"Untitled",TRUE); //Execute the RT sequence and acquire the image
	}
	shutter1.close();
}

void seq_contAcquisitionTest(const FPGAapi::Session &fpga)
{
	int nFrames = 100;
	for (int jj = 0; jj < nFrames; jj++)
	{
		std::cout << "Iteration: " << jj + 1 << std::endl;

		//CREATE A REAL-TIME SEQUENCE
		FPGAapi::RTsequence sequence(fpga);

		//Upload the realtime sequence to the FPGA but don't execute it yet
		sequence.uploadRT();

		//Execute the realtime sequence and acquire the image

		FPGAapi::checkStatus(__FUNCTION__, NiFpga_StartFifo(fpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));		//Establish the connection between FIFOOUTfpga and FIFOOUTpc

		FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(fpga.getSession(), NiFpga_FPGAvi_ControlBool_LinegateTrigger, 1));
		FPGAapi::checkStatus(__FUNCTION__, NiFpga_WriteBool(fpga.getSession(), NiFpga_FPGAvi_ControlBool_LinegateTrigger, 0));


		U32 nRemainFIFOOUT = 0;						//Elements remaining in FIFOOUTpc
		U32 *bufArray = new U32[nPixAllFrames];		//Array to read FIFOOUTpc
		int nElemReadFIFOOUT = 0;
		int timeoutCounter_iter = 10;
		const U32 timeout_ms = 100;
		const int readFifoWaitingTime_ms = 10;

		U32 *dummy = new U32[0];

		while (nElemReadFIFOOUT < nPixAllFrames)
		{
			Sleep(readFifoWaitingTime_ms); //Wait till collecting big chuncks of data. Adjust the waiting time to max transfer bandwidth

			FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(fpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, dummy, 0, timeout_ms, &nRemainFIFOOUT));
			std::cout << "Number of elements remaining in FIFOOUT: " << nRemainFIFOOUT << std::endl;

			if (nRemainFIFOOUT > 0)
			{
				nElemReadFIFOOUT += nRemainFIFOOUT;
				FPGAapi::checkStatus(__FUNCTION__, NiFpga_ReadFifoU32(fpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa, bufArray, nRemainFIFOOUT, timeout_ms, &nRemainFIFOOUT));
				//std::cout << "Number of elements remaining in FIFOOUT: " << nRemainFIFOOUT << std::endl;
			}

			timeoutCounter_iter--;
			//Transfer timeout
			if (timeoutCounter_iter == 0)
				break;
		}
		std::cout << "Total of elements read: " << nElemReadFIFOOUT << std::endl; //Print out the total number of elements read
		std::cout << std::endl;

		//If all the expected data is NOT read successfully
		if (nElemReadFIFOOUT != nPixAllFrames)
			getchar();

		delete[] dummy;
		delete[] bufArray;

		//Sleep(100);//Simulate the shifting time of the stages

		FPGAapi::checkStatus(__FUNCTION__, NiFpga_StopFifo(fpga.getSession(), NiFpga_FPGAvi_TargetToHostFifoU32_FIFOOUTa));


	}//For loop
}


void seq_testPixelclock(const FPGAapi::Session &fpga)
{
	FPGAapi::RTsequence sequence(fpga); 	//Create a realtime sequence
	sequence.uploadRT();					//Upload the realtime sequence to the FPGA but don't execute it yet
	Image image(fpga);
	image.acquire(TRUE);					//Execute the realtime sequence and acquire the image

}

//Test the analog and digital output and the relative timing wrt the pixel clock
void seq_testAODO(const FPGAapi::Session &fpga)
{
	FPGAapi::RTsequence sequence(fpga);

	//DO
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);

	//AO
	sequence.pushAnalogSinglet(GALVO1, 8 * us, 4);
	sequence.pushAnalogSinglet(GALVO1, 4 * us, 2);
	sequence.pushAnalogSinglet(GALVO1, 4 * us, 1);

	sequence.uploadRT();	//Upload the realtime sequence to the FPGA but don't execute it yet
	sequence.triggerRT();	//Execute the realtime sequence
}

void seq_testAOramp(const FPGAapi::Session &fpga)
{
	const double Vmax = 5;
	const double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, 0, -Vmax);
	sequence.pushLinearRamp(GALVO1, step, 20 * ms, -Vmax, Vmax);
	sequence.pushLinearRamp(GALVO1, step, 2 * ms, Vmax, 0);

	const double pulsewidth = 300 * us;
	sequence.pushDigitalSinglet(DOdebug, pulsewidth, 1);
	sequence.pushDigitalSinglet(DOdebug, 4 * us, 0);
}

//Generate a long digital pulse and check the duration with the oscilloscope
void seq_checkDigitalTiming(const FPGAapi::Session &fpga)
{
	const double step = 400 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

//Generate many short digital pulses and check the overall duration with the oscilloscope
void seq_calibDigitalLatency(const FPGAapi::Session &fpga)
{
	const double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);

	sequence.pushDigitalSinglet(DOdebug, step, 1);

	//Many short digital pulses to accumulate the error
	for (U32 ii = 0; ii < 99; ii++)
		sequence.pushDigitalSinglet(DOdebug, step, 0);

	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
}

//First calibrate the digital channels, then use it as a time reference
void seq_calibAnalogLatency(const FPGAapi::Session &fpga)
{
	const double delay = 400 * us;
	const double step = 4 * us;

	FPGAapi::RTsequence sequence(fpga);
	sequence.pushAnalogSinglet(GALVO1, step, 10);	//Initial pulse
	sequence.pushAnalogSinglet(GALVO1, step, 0);
	sequence.pushLinearRamp(GALVO1, 4 * us, delay, 0, 5 * V);			//Linear ramp to accumulate the error
	sequence.pushAnalogSinglet(GALVO1, step, 10);	//Initial pulse
	sequence.pushAnalogSinglet(GALVO1, step, 0);	//Final pulse

	//DO0
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
	sequence.pushDigitalSinglet(DOdebug, delay, 0);
	sequence.pushDigitalSinglet(DOdebug, step, 1);
	sequence.pushDigitalSinglet(DOdebug, step, 0);
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
//1. Manually open the shutter
//2. Set pockels1_enableAutoOff = 0
//3. Set lineclockInput = FG
void seq_testPockels(const FPGAapi::Session &fpga)
{
	//Create a realtime sequence
	FPGAapi::RTsequence sequence(fpga);

	//Open the Uniblitz shutter
	//Shutter shutter1(fpga, Shutter1);
	//shutter1.open();

	//Turn on the pockels cell
	PockelsCell pockels(sequence, POCKELS1, 750);
	pockels.pushPowerSinglet(8 * us, 50 * mW);
	//pockels.pushVoltageSinglet_(8 * us, 2.508 * V);

	//Upload the pockels sequence to the FPGA but don't execute it yet
	sequence.uploadRT();

	//Execute the sequence
	Image image(fpga);
	image.acquire();
}

void seq_testLaserComm(const FPGAapi::Session &fpga)
{
	Laser vision;
	//vision.setShutter(0);
	vision.setWavelength(940);
}

void seq_testRS(const FPGAapi::Session &fpga)
{
	ResonantScanner RS(fpga);
	std::cout << "aaa = " << RS.downloadControl_V() << std::endl;
	//RS.turnOn_um(150);
	//RS.turnOff();
}

void seq_testConvertI16toVolt()
{
	std::cout << "volt to I16: " << FPGAapi::convertVoltToI16(1) << std::endl;

	std::cout << "I16 to colt: " << FPGAapi::convertI16toVolt(32767) << std::endl;

	std::cout << "volt to I16 to volt: " << FPGAapi::convertI16toVolt(FPGAapi::convertVoltToI16(0)) << std::endl;
}


//Test the speed for saving a Tiff file
void seq_saveAsTiffTest()
{

	//const std::string foldername = "D:\\OwnCloud\\Data\\_output_D\\";
	const std::string foldername = "Z:\\_output_Z\\";
	std::string filename = "Untitled";


	const int width_pix = 400;
	const int height_pix = 400 * 200; // 100 um in 0.5 um-steps = 200 planes

	U8 *image = new U8[width_pix * height_pix]();

	//Declare and start a stopwatch
	double duration;
	auto t_start = std::chrono::high_resolution_clock::now();

	//This gives some overhead
	//filename = file_exists(filename);

	TIFF *tiffHandle = TIFFOpen((foldername + filename + ".tif").c_str(), "w");

	if (tiffHandle == nullptr)
		throw ImageException((std::string)__FUNCTION__ + ": Saving Tiff failed");

	//TAGS
	TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, width_pix);							//Set the width of the image
	TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, height_pix);							//Set the height of the image
	TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);								//Set number of channels per pixel
	TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);									//Set the size of the channels
	TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);					//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
																						//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);				//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
	TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);				//Single channel with min as black

	tsize_t bytesPerLine = width_pix;			//Length in memory of one row of pixel in the image.
	unsigned char *buffer = nullptr;			//Buffer used to store the row of pixel information for writing to file

	//Allocating memory to store pixels of current row
	if (TIFFScanlineSize(tiffHandle))
		buffer = (unsigned char *)_TIFFmalloc(bytesPerLine);
	else
		buffer = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(tiffHandle));

	//Set the strip size of the file to be size of one row of pixels
	TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, width_pix));

	//Now writing image to the file one strip at a time. CURRENTLY ONLY ONE FRAME IS SAVED!!!
	for (int row = 0; row < height_pix; row++)
	{
		memcpy(buffer, &image[(height_pix - row - 1)*bytesPerLine], bytesPerLine);    // check the index here, and figure tiffHandle why not using h*bytesPerLine
		if (TIFFWriteScanline(tiffHandle, buffer, row, 0) < 0)
			break;
	}

	delete[] image;

	//Close the output file
	(void)TIFFClose(tiffHandle);

	//Destroy the buffer
	if (buffer)
		_TIFFfree(buffer);

	//Stop the stopwatch
	duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
	std::cout << "Elapsed time: " << duration << " ms" << std::endl;

}