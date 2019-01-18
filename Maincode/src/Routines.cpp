#include "Routines.h"

namespace MainRoutines
{
	void discreteScanZ(const FPGAns::FPGA &fpga)
	{
		//Each of the following modes can be used under 'continuous XY acquisition' by setting nFramesCont > 1, meaning that the galvo is scanned back and
		//forth on the same z plane. The images the can be averaged
		//const RunMode acqMode = SINGLEMODE;			//Single shot
		//const RunMode acqMode = LIVEMODE;				//Image the same z plane many times as single shots. Used it for adjusting the microscope live
		//const RunMode acqMode = AVGMODE;				//Image the same z plane many times and average the images
		const RunMode acqMode = STACKMODE;			//Stack volume from the initial z position
		//const RunMode acqMode = STACKCENTEREDMODE;		//Stack volume centered at the initial z position

		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const int nFramesCont(1);												//Number of frames for continuous XY acquisition
		//const double3 stagePosition0{ 34.750 * mm, 10.110 * mm, 18.481 * mm };	//Stage initial position
		const double3 stagePosition0{ 34.925 * mm, 10.130 * mm, 18.451 * mm };	//Stage initial position

		//RS
		const ResonantScanner RScanner(fpga);
		RScanner.isRunning();					//Make sure that the RS is running

		//STACK
		const double stepSizeZ(1 * um);
		double stackDepthZ(80 * um);			//Acquire a stack of this depth or thickness in Z

		//SAMPLE
		const std::string sampleName("Beads4um");
		const std::string immersionMedium("SiliconMineralOil5050");
		const std::string collar("1.49");

		//STAGES
		Stage stage;
		std::vector<double3> stagePosition;

		int nDiffZ;		//Number of frames at different Zs
		int nSameZ;		//Number of frames at the same Z
		OverrideFileSelector overrideFlag;
		switch (acqMode)
		{
		case SINGLEMODE:
			nSameZ = 1;
			nDiffZ = 1; //Do not change this
			overrideFlag = NOOVERRIDE;
			stagePosition.push_back(stagePosition0);
			break;
		case LIVEMODE:
			nSameZ = 500;
			nDiffZ = 1; //Do not change this
			overrideFlag = OVERRIDE;
			stagePosition.push_back(stagePosition0);
			break;
		case AVGMODE:
			nSameZ = 10;
			nDiffZ = 1; //Do not change this
			overrideFlag = NOOVERRIDE;
			stagePosition.push_back(stagePosition0);
			break;
		case STACKMODE:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			overrideFlag = NOOVERRIDE;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePosition.push_back({ stagePosition0.at(XX), stagePosition0.at(YY), stagePosition0.at(ZZ) + iterDiffZ * stepSizeZ });
			break;
		case STACKCENTEREDMODE:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			overrideFlag = NOOVERRIDE;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePosition.push_back({ stagePosition0.at(XX), stagePosition0.at(YY), stagePosition0.at(ZZ) - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

		//GALVO RT linear scan
		const double FFOVgalvo(200 * um);			//Full FOV in the slow axis
		Galvo galvo(RTcontrol, RTGALVO1, FFOVgalvo / 2);

		//LASER
		LaserSelector whichLaser = VISION;
		int wavelength_nm;
		double laserPowerMin, laserPowerMax;
		switch (whichLaser)
		{
		case VISION:
			wavelength_nm = 750;
			laserPowerMin = 55. * mW;
			laserPowerMax = 55. * mW;
			break;
		case FIDELITY:
			wavelength_nm = 1040;
			laserPowerMin = 25. * mW;
			laserPowerMax = 25. * mW;
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + "Select VISION OR FIDELITY");
		}
		double laserPower = laserPowerMin;
		VirtualLaser laser(RTcontrol, wavelength_nm, whichLaser);

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
			datalog.record("Laser power first frame (mW) = ", laserPowerMin / mW);
			datalog.record("Laser power last frame (mW) = ", laserPowerMax / mW);
			datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Galvo FFOV (um) = ", FFOVgalvo / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", RTcontrol.mHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (FFOVgalvo / um) / RTcontrol.mHeightPerFrame_pix);
			datalog.record("\nSTAGE--------------------------------------------------------");
		}

		//CREATE A STACK FOR STORING THE TIFFS
		TiffStack tiffStack(widthPerFrame_pix, heightPerFrame_pix, nDiffZ, nSameZ);

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();

		//ACQUIRE FRAMES AT DIFFERENT Zs
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
		{
			stage.moveXYZstages(stagePosition.at(iterDiffZ));
			stage.waitForMotionToStopAllStages();
			stage.printPositionXYZ();		//Print the stage position		

			//Acquire many frames at the same Z via discontinuous acquisition
			for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
			{
				std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
					"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << "\n";

				laser.setPower(laserPower);	//Update the laser power

				//EXECUTE THE RT CONTROL SEQUENCE
				Image image(RTcontrol);
				image.acquire();			//Execute the RT control sequence and acquire the image
				image.mirrorOddFrames();
				image.averageFrames();		//Average the frames acquired via continuous XY acquisition
				tiffStack.pushSameZ(iterSameZ, image.pointerToTiff());

				if (acqMode == SINGLEMODE || acqMode == LIVEMODE)
				{
					//Save individual files
					std::string singleFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_" + toString(laserPower / mW, 1) + "mW" +
						"_x=" + toString(stagePosition.at(iterDiffZ).at(XX) / mm, 3) + "_y=" + toString(stagePosition.at(iterDiffZ).at(YY) / mm, 3) + "_z=" + toString(stagePosition.at(iterDiffZ).at(ZZ) / mm, 4));
					image.saveTiffSinglePage(singleFilename, overrideFlag);
					Sleep(500);
				}
			}
			tiffStack.pushDiffZ(iterDiffZ);

			std::cout << "\n";
			laserPower += (laserPowerMin - laserPowerMax) / nDiffZ;		//calculate the new laser power
		}
		laser.closeShutter();

		if (acqMode == AVGMODE || acqMode == STACKMODE || acqMode == STACKCENTEREDMODE)
		{
			//Save the stackDiffZ to file
			std::string stackFilename(sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(laserPowerMin / mW, 1) + "mW_Pf=" + toString(laserPowerMax / mW, 1) + "mW" +
				"_x=" + toString(stagePosition.front().at(XX) / mm, 3) + "_y=" + toString(stagePosition.front().at(YY) / mm, 3) +
				"_zi=" + toString(stagePosition.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePosition.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4));
			tiffStack.saveToFile(stackFilename, overrideFlag);
		}

		pressAnyKeyToCont();
	}

	/*
	The galvo scans the FOV back and forth continuously while the z-stage travels nonstop
	The pc triggers the z-stage motion, then the position of the stage triggers the scanning routine
	Using the PI step-response monitor, I observe that the motion of the stage is somewhat nonlinear (+/- 1 um off target) at the start or end.
	I then thought that, due to such asymmetry, the forth and back z-scanning would generate slightly different registrations
	I therefore set DO1 to output a 50us-pulse every time the stage travels 0.3 um and triggered the stack acquisition with such signal
	(see the implementation on LV and also the DO config of the stage driver). Because the position of the stage is supposed to be absolute and precise,
	I thought that the registration would perfect. However, the beads differed in 3 or more planes.
	I triggered the stack acquisition using DO2 for both scanning directions: top-down and bottom-up. Under DO2 triggering, the beads' z-position in both cases looks
	almost identical, with a difference of maybe 1 plane only (0.5 um)
	Remember that I do not use MACROS on the stages anymore
	*/
	void continuousScanZ(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const int nFramesCont(160);				//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ(0.5 * um);

		//STAGES
		const int stackScanDirZ = 1;	//Scan direction in z: 1 for top-down, -1 for bottom-up
		const double3 stackCenterXYZ{ 34.750 * mm, 10.110 * mm, 18.451 * mm };												//Center of x, y, z stack
		const double stackDepth(static_cast<int>(stackScanDirZ) * nFramesCont * stepSizeZ);
		const double3 stageXYZi{ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) - stackDepth / 2 };	//Initial position of the stages
		const double frameDurationTmp = halfPeriodLineclock * heightPerFrame_pix;											//TODO: combine this with the galvo's one
		Stage stage(5 * mmps, 5 * mmps, stepSizeZ / frameDurationTmp);
		stage.moveXYZstages(stageXYZi);
		stage.waitForMotionToStopAllStages();

		//RS
		ResonantScanner RScanner(fpga);
		RScanner.isRunning();		//Make sure that the RS is running

		//CREATE THE REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, STAGETRIG);	//Notice the STAGETRIG flag

		//LASER: wavelength_nm, laserPower, whichLaser
		VirtualLaser laser(RTcontrol, 750, 60. * mW, AUTO);
		//VirtualLaser laser(RTcontrol, 1040, 25. * mW, AUTO);

		//GALVO RT linear scan
		const double FFOVgalvo(200 * um);	//Full FOV in the slow axis
		Galvo galvo(RTcontrol, RTGALVO1, FFOVgalvo / 2);

		//OPEN THE SHUTTER
		laser.openShutter();

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image(RTcontrol);
		image.initialize();

		image.startFIFOOUTpc();
		std::cout << "Scanning the stack...\n";
		stage.moveSingleStage(ZZ, stageXYZi.at(ZZ) + stackDepth);	//Move the stage to trigger the control sequence and data acquisition
		image.download();
		image.mirrorOddFrames();

		laser.closeShutter();
		image.saveTiffMultiPage("Untitled", NOOVERRIDE, stackScanDirZ);

		//Disable ZstageAsTrigger to be able to move the z-stage without triggering the acquisition sequence
		//RTcontrol.setZstageTriggerEnabled(false);

		//pressAnyKeyToCont();
	}
}//namespace


namespace CalibrationRoutines
{
	void fineTuneGalvoScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const int nFramesCont(30);												//Number of frames for continuous XY acquisition
		const double3 stagePosition0{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//RS
		const ResonantScanner RScanner(fpga);
		RScanner.isRunning();		//Make sure that the RS is running

		//STAGES
		Stage stage;

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

		//GALVO
		const double FFOVgalvo(200 * um);			//Full FOV in the slow axis
		const double posMax(FFOVgalvo / 2);
		Galvo galvo(RTcontrol, RTGALVO1);
		galvo.generateFrameScan(posMax, -posMax);

		//LASER
		const int wavelength_nm = 1040;
		const double P(25. * mW);		//Laser power
		VirtualLaser laser(RTcontrol, wavelength_nm, P, FIDELITY);

		//ACQUIRE FRAMES AT DIFFERENT Zs
		stage.moveXYZstages(stagePosition0);
		stage.waitForMotionToStopAllStages();

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image(RTcontrol);
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition
		image.mirrorOddFrames();
		image.averageEvenOddFrames();
		image.saveTiffMultiPage("Untitled", NOOVERRIDE);

		laser.closeShutter();

		pressAnyKeyToCont();
	}

	//Generate many short digital pulses and check the overall frameDuration with the oscilloscope
	void digitalLatency(const FPGAns::FPGA &fpga)
	{
		const double step(4 * us);

		FPGAns::RTcontrol RTcontrol(fpga);

		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);

		//Many short digital pulses to accumulate the error
		for (U32 ii = 0; ii < 99; ii++)
			RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);

		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
	}

	//First calibrate the digital channels, then use it as a time reference
	void analogLatency(const FPGAns::FPGA &fpga)
	{
		const double delay(400 * us);
		const double step(4 * us);

		FPGAns::RTcontrol RTcontrol(fpga);
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 10 * V);					//Initial pulse
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 0);
		RTcontrol.pushLinearRamp(RTGALVO1, 4 * us, delay, 0, 5 * V);			//Linear ramp to accumulate the error
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 10 * V);					//Initial pulse
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 0);							//Final pulse

		//DO0
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, delay, 0);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
	}
}//namespace


namespace TestRoutines
{
	void galvo(const FPGAns::FPGA &fpga)
	{
		const int width_pix(300);
		const int height_pix(400);
		const int nFramesDiscont(1);
		const int nFramesCont(2);

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, FG, nFramesCont, width_pix, height_pix);

		//GALVO
		const double FFOVgalvo(200 * um);				//Full FOV in the slow axis
		const double galvoTimeStep(8 * us);
		const double posMax(FFOVgalvo / 2);
		Galvo galvo(RTcontrol, RTGALVO1);
		const double frameDuration(halfPeriodLineclock * RTcontrol.mHeightPerFrame_pix);		//= 62.5us * 400 pixels = 25 ms
		galvo.positionLinearRamp(galvoTimeStep, frameDuration, posMax, -posMax);				//Linear ramp for the galvo

		for (int iter = 0; iter < nFramesDiscont; iter++)
		{
			std::cout << "Iteration: " << iter + 1 << "\n";

			//Execute the realtime control sequence and acquire the image
			Image image(RTcontrol);
			image.acquire(); //Execute the RT control sequence and acquire the image
			//image.saveTiffSinglePage("Untitled", OVERRIDE);
		}
	}

	void pixelclock(const FPGAns::FPGA &fpga)
	{
		std::vector<unsigned char> stackOfAverages;

		FPGAns::RTcontrol RTcontrol(fpga); 			//Create a realtime control sequence
		Image image(RTcontrol);
		image.acquire();							//Execute the realtime control sequence and acquire the image
		//image.pushToVector(stackOfAverages);
		//std::cout << "size: " << stackOfAverages.size() << "\n";
		//TiffU8 acqParam(stackOfAverages, 300, 400);
		//acqParam.saveTiff("Untitled");
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGAns::FPGA &fpga)
	{
		FPGAns::RTcontrol RTcontrol(fpga);

		//DO
		RTcontrol.pushDigitalSinglet(RTDODEBUG, 4 * us, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, 4 * us, 0);

		//AO
		RTcontrol.pushAnalogSinglet(RTGALVO1, 8 * us, 4 * V);
		RTcontrol.pushAnalogSinglet(RTGALVO1, 4 * us, 2 * V);
		RTcontrol.pushAnalogSinglet(RTGALVO1, 4 * us, 1 * V);

		RTcontrol.triggerRT();	//Execute the realtime control sequence
	}

	void analogRamp(const FPGAns::FPGA &fpga)
	{
		const double Vmax(5 * V);
		const double step(4 * us);

		FPGAns::RTcontrol RTcontrol(fpga);
		RTcontrol.pushLinearRamp(RTGALVO1, step, 2 * ms, 0, -Vmax);
		RTcontrol.pushLinearRamp(RTGALVO1, step, 20 * ms, -Vmax, Vmax);
		RTcontrol.pushLinearRamp(RTGALVO1, step, 2 * ms, Vmax, 0);

		const double pulsewidth(300 * us);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, pulsewidth, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, 4 * us, 0);
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGAns::FPGA &fpga)
	{
		const double step(400 * us);

		FPGAns::RTcontrol RTcontrol(fpga);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
	}

	void filterwheel()
	{
		Filterwheel FWexcitation(FWEXC);
		Filterwheel FWdetection(FWDET);

		int wavelength_nm;
		if (0)
			wavelength_nm = 1040;
		else
			wavelength_nm = 750;

		//FWexcitation.setWavelength(wavelength);
		//FWdetection.setWavelength(wavelength);
		std::thread th1(&Filterwheel::setWavelength, &FWexcitation, wavelength_nm);
		std::thread th2(&Filterwheel::setWavelength, &FWdetection, wavelength_nm);
		th1.join();
		th2.join();

		pressAnyKeyToCont();
	}

	void shutter(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga);

		PockelsCell fidelity(RTcontrol, 1040, FIDELITY);
		fidelity.setShutter(true);
		Sleep(5000);
		fidelity.setShutter(false);

		//Shutter shutterFidelity(fpga, FIDELITY);
		//shutterFidelity.open();
		//Sleep(5000);
		//shutterFidelity.close();

		pressAnyKeyToCont();
	}

	void stagePosition()
	{
		double duration;
		const double3 stagePosition0{ 35.020 * mm, 19.808 * mm, 18.542 * mm };	//Stage initial position
		Stage stage;

		std::cout << "Stages initial position:" << "\n";
		stage.printPositionXYZ();

		auto t_start = std::chrono::high_resolution_clock::now();

		stage.moveXYZstages(stagePosition0);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		stage.waitForMotionToStopSingleStage(ZZ);

		std::cout << "Stages final position:" << "\n";
		stage.printPositionXYZ();

		/*
		int input = 1;
		while (input)
		{
			std::cout << "Stage X position = " << stage.downloadPosition(XX) << "\n";
			std::cout << "Stage Y position = " << stage.downloadPosition(YY) << "\n";
			std::cout << "Stage X position = " << stage.downloadPosition(ZZ) << "\n";

			std::cout << "Enter command: ";
			std::cin >> input;
			//input = 0;
		}
		*/
	}

	//Test configuring setDOtriggerEnabled and CTO for the stages
	void stageConfig()
	{
		Stage stage;
		const int DOchan = 1;
		//std::cout << "Stages initial position:" << "\n";
		//stage.printPositionXYZ();
		//stage.isDOtriggerEnabled(ZZ, DOchannel);
		//stage.setDOtriggerEnabled(ZZ, DOchannel , true);
		//const int triggerParam = 1;
		//stage.downloadDOtriggerSingleParam(ZZ, DOchannel , triggerParam);
		std::cout << "x stage vel: " << stage.downloadSingleVelocity(XX) / mmps << " mm/s" << "\n";
		std::cout << "y stage vel: " << stage.downloadSingleVelocity(YY) / mmps << " mm/s" << "\n";
		std::cout << "z stage vel: " << stage.downloadSingleVelocity(ZZ) / mmps << " mm/s" << "\n";
		//stage.printStageConfig(ZZ, DOchan);

		pressAnyKeyToCont();
	}

	void PMT16Xconfig()
	{
		PMT16X pmt;
		pmt.readAllGain();
		//pmt.setSingleGain(2, 300);
		//pmt.setAllGain(255);
		//pmt.readTemp();
		//pmt.setAllGain({ 100,255,255,255,255,255,255,255,255,255,255,255,255,255,100,255});

		pressAnyKeyToCont();
	}

	void lasers(const FPGAns::FPGA &fpga)
	{
		Laser laser(VISION);
		//Laser laser(FIDELITY);
		//std::cout << laser.isShutterOpen() << std::endl;
		//laser.setShutter(false);
		laser.setWavelength(1040);
		//laser.printWavelength_nm();

		pressAnyKeyToCont();
	}

	void virtualLasers(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const int nFramesCont(10);			//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, FG, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

		const int wavelength_nm(750);
		const double P(50. * mW);		//Laser power
		VirtualLaser laser(RTcontrol, wavelength_nm, P);

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image(RTcontrol);
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition

		//laser.openShutter();
		//Sleep(3000);
		//laser.closeShutter();

		pressAnyKeyToCont();
	}

	//For keeping the pockels on to check the the laser power
	//0. Make sure that the function generator feeds the lineclock
	//1. Manually open the Vision shutter and Uniblitz shutter. The latter because the class destructor closes the shutter automatically
	//2. Set pockelsAutoOff = DISABLE for holding the last value
	//3. Tune Vision's wavelength manually
	void pockels(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga);

		//DEFINE THE POCKELS CELLS
		PockelsCell pockelsVision(RTcontrol, 1040, VISION);			//Vision
		PockelsCell pockelsFidelity(RTcontrol, 1040, FIDELITY);		//Fidelity

		PockelsCell pockels(pockelsVision);
		//PockelsCell pockels(pockelsFidelity);
		pockels.pushPowerSinglet(400 * us, 30 * mW);
		//pockels.pushPowerSinglet(8 * us, 0 * mW);
		//pockels.pushVoltageSinglet(8 * us, 0.0 * V);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		Image image(RTcontrol);
		image.acquire();

		pressAnyKeyToCont();
	}

	void pockelsRamp(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const int nFramesCont(10);			//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol(fpga, FG, nFramesCont, widthPerFrame_pix, heightPerFrame_pix);

		//POCKELS CELL
		const int wavelength_nm(750);
		const double P(25 * mW);		//Laser power
		PockelsCell pockels(RTcontrol, wavelength_nm, VISION);

		//Test the voltage setpoint
		//pockels.pushVoltageSinglet(8 * us, 0.375 * V);
		//pockels.voltageLinearRamp(0.25 * V, 0.5 * V);		//Linearly scale the pockels voltage from the first to the last frame

		//Test the laser power setpoint
		//pockels.pushPowerSinglet(8 * us, P);
		//pockels.powerLinearRamp(P, 2 * P);		//Linearly scale the laser power from the first to the last frame

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image(RTcontrol);
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition

		pressAnyKeyToCont();
	}

	void resonantScanner(const FPGAns::FPGA &fpga)
	{
		ResonantScanner RScanner(fpga);
		std::cout << "aaa = " << RScanner.downloadControlVoltage() << "\n";
		//RScanner.turnOn(150 * um);
		//RScanner.turnOff();
	}

	void convertI16toVolt()
	{
		std::cout << "volt to I16: " << FPGAns::voltageToI16(1) << "\n";
		std::cout << "I16 to colt: " << FPGAns::I16toVoltage(32767) << "\n";
		std::cout << "volt to I16 to volt: " << FPGAns::I16toVoltage(FPGAns::voltageToI16(0)) << "\n";
	}

	void tiffU8()
	{
		std::string inputFilename("Beads_4um_750nm_50mW_x=35.120_y=19.808_z=18.4610");
		std::string outputFilename("test");

		const int nFramesCont(10);
		TiffU8 image(inputFilename, nFramesCont);

		image.mirrorOddFrames();
		//image.averageFrames();
		image.averageEvenOddFrames();
		image.saveToFile(outputFilename, MULTIPAGE, OVERRIDE);

		//image.saveToFile(outputFilename, 2);

		//image.mirrorOddFrames(nFramesCont);
		//image.averageEvenOddFrames(nFramesCont);
	}

	//To measure the saving speed of a Tiff file, either locally or remotely
	//Select a local or remote folder accordingly
	void ethernetSpeed()
	{
		std::string filename = "testEthernetSpeed";

		//The goal is to stream a stackDiffZ composed of 200 z-planes (100 um in 0.5 um-steps), where each frame has 300x560 pixels. Therefore, the stackDiffZ has 300x560x200 = 33.6 Mega pixels
		//The stackDiffZ size is 8 bits x 33.6M = 33.6 MB
		const int width(300);
		const int height(560);
		const int nFramesCont(200);

		TiffU8 image(width, height, nFramesCont);

		//Declare and start a stopwatch
		double duration;
		auto t_start = std::chrono::high_resolution_clock::now();

		//overriding the file saving has some overhead
		//Splitting the stackDiffZ into a page structure (by assigning nFramesCont = 200 in saveToFile) gives a large overhead
		image.saveToFile(filename, SINGLEPAGE, OVERRIDE);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		pressAnyKeyToCont();
	}

	void vibratome(const FPGAns::FPGA &fpga)
	{
		Vibratome vibratome(fpga);
		//vibratome.retractDistance(20 * mm);
		vibratome.cutAndRetractDistance(20 * mm);

		pressAnyKeyToCont();
	}

	void multithread()
	{
		class FUNC
		{
			int dummy;
		public:
			FUNC(const int i) : dummy(i) {}
			void func1(const int x)
			{
				Sleep(1000);
				std::cout << "func1 " << x << "\n";
				dummy = x;
			}
			void func2(const int x)
			{
				std::cout << "func2 " << x << "\n";
				dummy = x;
			}
		};

		unsigned int n = std::thread::hardware_concurrency();
		std::cout << n << " concurrent threads are supported.\n";

		std::cout << "func1 and func2 will execute concurrently\n";

		FUNC x(1);

		std::thread first(&FUNC::func1, &x, 123);
		std::thread second(&FUNC::func2, &x, 314);

		first.join();//pauses until first finishes
		second.join();//pauses until second finishes

		pressAnyKeyToCont();
	}

	void sequencer(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix(300);
		const int heightPerFrame_pix(400);
		const double2 FFOV{ 200. * um, 150. * um };
		const int nFramesCont(80);											//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ(0.5 * um);									//Step size in z
		const ROI roi{ 34.850 * mm, 10.150 * mm, 35.050 * mm, 9.950 * mm }; //Region of interest
		const double3 stackOverlap_frac{ 0.05, 0.05, 0.05 };					//Stack overlap
		const double cutAboveBottomOfStack(15 * um);						//height to cut above the bottom of the stack
		const double sampleLengthZ(0.01 * mm);								//Sample thickness
		const double initialZ(18.471 * mm);

		const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW }, { 1040, 30. * mW, 0. * mW } };
		//const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW } };
		//const std::vector<LaserList::SingleLaser> laserList{{ 1040, 25. * mW, 0. * mW } };
		Sample sample("Beads4um", "Grycerol", "1.47", roi, sampleLengthZ, initialZ, cutAboveBottomOfStack);
		Stack stack(FFOV, stepSizeZ, stepSizeZ * nFramesCont, stackOverlap_frac);

		//Create a sequence
		Sequencer sequence(laserList, sample, stack);
		sequence.generateCommandList();
		sequence.printToFile("CommandlistLight");

		if (1)
		{
			//STAGES. Specify the velocity
			Stage stage(5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix));
			stage.moveSingleStage(ZZ, sample.mInitialZ);	//Move to the initial position

			//RS
			ResonantScanner RScanner(fpga);
			RScanner.isRunning();		//Make sure that the RS is running

			//CREATE THE REALTIME CONTROL SEQUENCE
			FPGAns::RTcontrol RTcontrol(fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, STAGETRIG);	//Notice the STAGETRIG flag

			//LASER: wavelength_nm, laserPower, whichLaser
			VirtualLaser laser(RTcontrol, laserList.front().mWavelength_nm);

			//GALVO RT linear ramp	
			Galvo galvo(RTcontrol, RTGALVO1, FFOV.at(XX) / 2);

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image(RTcontrol);

			//Read the commands line by line
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.mCommandCounter; iterCommandline++)
			//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Commandline commandline = sequence.getCommandline(iterCommandline); //Implement read-from-file?
				commandline.printParameters();

				double scanZi, scanZf, stackDepth, scanPi, scanPf, stackPinc;
				double2 stackCenter;
				int wavelength_nm, scanDirZ;
				std::string filename;
				switch (commandline.mAction)
				{
				case MOV:
					//Move the x and y stages to mStackCenter
					stackCenter = commandline.mCommand.moveStage.mStackCenter;
					stage.moveXYstages(stackCenter);
					stage.waitForMotionToStopAllStages();

					break;
				case ACQ:
					//Acquire a stack using the parameters:
					wavelength_nm = commandline.mCommand.acqStack.mWavelength_nm;
					scanDirZ = commandline.mCommand.acqStack.mScanDirZ;
					scanZi = commandline.mCommand.acqStack.mScanZi;
					stackDepth = commandline.mCommand.acqStack.mStackDepth;
					scanZf = scanZi + scanDirZ * stackDepth;
					scanPi = commandline.mCommand.acqStack.mScanPi;
					stackPinc = commandline.mCommand.acqStack.mStackPinc;
					scanPf = scanPi + scanDirZ * stackPinc;

					//Update the laser parameters
					laser.setWavelength(wavelength_nm);
					laser.setPower(scanPi, stackPinc);

					//OPEN THE SHUTTER
					laser.openShutter();
					
					image.initialize();
					image.startFIFOOUTpc();
					std::cout << "Scanning the stack...\n";
					stage.moveSingleStage(ZZ, scanZf);		//Move the stage to trigger the control sequence and data acquisition
					image.download();

					filename = toString(wavelength_nm, 0) + "nm_Pi=" + toString(scanPi / mW, 1) + "mW_Pf=" + toString(scanPf / mW, 1) + "mW" +
						"_x=" + toString(stackCenter.at(XX) / mm, 3) + "_y=" + toString(stackCenter.at(YY) / mm, 3) +
						"_zi=" + toString(scanZi / mm, 4) + "_zf=" + toString(scanZf / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4);
					image.mirrorOddFrames();
					image.saveTiffMultiPage(filename, NOOVERRIDE, scanDirZ);

					break;
				case SAV:
					//Save the stack to file and label it with the scan parameters:
					wavelength_nm, scanZi, stackDepth, scanPi, stackPinc;
					stackCenter;


					break;
				case CUT:
					//Move the stage to
					double3 stagePosition = commandline.mCommand.cutSlice.mBladePosition;
					//and then cut a slice off

					break;
				default:
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
				}//switch

			}//for
			laser.closeShutter();
		}//if

		pressAnyKeyToCont();
	}
}//namespace