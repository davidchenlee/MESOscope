#include "Routines.h"

namespace Routines
{
	//The "Swiss knife" of my routines
	void stepwiseScan(const FPGA &fpga)
	{
		//const RUNMODE acqMode{ RUNMODE::SINGLE };			//Single frame. The same location is imaged continuously if nFramesCont>1 (the galvo is scanned back and forth at the same location) and the average is returned
		const RUNMODE acqMode{ RUNMODE::AVG };			//Single frame. The same location is imaged stepwise and the average is returned
		//const RUNMODE acqMode{ RUNMODE::SCANZ };			//Scan in the Z-stage axis stepwise with stackCenterXYZ.at(STAGEZ) as the starting position
		//const RUNMODE acqMode{ RUNMODE::SCANZCENTERED };	//Scan in the Z-stage axis stepwise with stackCenterXYZ.at(STAGEZ) as the center of the stack
		//const RUNMODE acqMode{ RUNMODE::SCANX };			//Scan in the X-stage axis stepwise
		//const RUNMODE acqMode{ RUNMODE::COLLECTLENS };	//For optimizing the collector lens
		//const RUNMODE acqMode{ RUNMODE::FIELD_ILLUM };	//Field illumination measurement for 16X using beads
		
		//ACQUISITION SETTINGS
		const FluorMarkerList::FluorMarker fluorMarker{ g_currentSample.findFluorMarker("TDT") };	//Select a particular fluorescence channel
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const POSITION3 stackCenterXYZ{ g_stackCenterXYZ };;
		const int nFramesCont{ 1 };	
		const double stackDepthZ{ 100. * um };								//Stack deepth in the Z-stage axis
		const double pixelSizeZ{ 1.0 * um };
	
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (g_multibeam)
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<double>(FFOVslow / g_nChanPMT);
		}
		else//Singlebeam. When using a fluorescent slide, set selectScanFFOV = 0 and PMT16Xchan_int = PMT16XCHAN::CENTERED to let the laser scan over the PMT16X channels
		{
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOVslow;
		}

		std::vector<POSITION3> stagePosXYZ;								//Vector with the sample locations to image
		int nSameLocation{ 1 };											//Number of frames at the same location
		double stepSizeX{ 0 };											//Step for lateral scanning
		bool saveAllPMT{ false };										//Save all PMT16X channels in separate pages in a Tiff
		double cLensPosIni{ 0 }, cLensPosFinal{ 0 }, cLensStep{ 0 };	//For debugging the collector lens
		int sleepTime_ms{ 100 };										//Wait after every single shot to avoid overheating the sample
		switch (acqMode)
		{
		case RUNMODE::SINGLE:
			stagePosXYZ.push_back(stackCenterXYZ);
			if (!g_multibeam) //No need for saving all the PMT channels for multibeam,
			{
				//saveAllPMT = true;
			}
			break;
		case RUNMODE::AVG:
			//saveAllPMT = true;

			nSameLocation = 10;
			for (int iterSameZ = 0; iterSameZ < nSameLocation; iterSameZ++)
				stagePosXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::SCANZ:
			for (int iterDiffZ = 0; iterDiffZ < static_cast<int>(stackDepthZ / pixelSizeZ); iterDiffZ++)
				stagePosXYZ.push_back({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ + iterDiffZ * pixelSizeZ });
			break;
		case RUNMODE::SCANZCENTERED:
			//Generate the discrete scan sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < static_cast<int>(stackDepthZ / pixelSizeZ); iterDiffZ++)
				stagePosXYZ.push_back({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ - 0.5 * stackDepthZ + iterDiffZ * pixelSizeZ });
			break;
		case RUNMODE::SCANX:
		{
			//saveAllPMT = true;
			//stagePosXYZ.push_back({ stackCenterXYZ.at(Stage::X), stackCenterXYZ.at(Stage::Y), stackCenterXYZ.at(Stage::Z) });
			//stagePosXYZ.push_back({ stackCenterXYZ.at(Stage::X) + 0.250 * mm, stackCenterXYZ.at(Stage::Y), stackCenterXYZ.at(Stage::Z) - 0.003 * mm});

			stepSizeX = 17.5 * um;
			const int nLocationsX{ 60 };
			for (int iterDiffX = 0; iterDiffX < nLocationsX; iterDiffX++)
				stagePosXYZ.push_back({ stackCenterXYZ.XX + iterDiffX * stepSizeX, stackCenterXYZ.YY, stackCenterXYZ.ZZ });
			break;
		}
		case RUNMODE::COLLECTLENS:
			if(g_multibeam)
				throw std::invalid_argument((std::string)__FUNCTION__ + ": Collector-lens scanning available only for single beam");
			saveAllPMT = true;
			cLensPosIni = 0.0 * mm;
			cLensPosFinal = 5.0 * mm;
			cLensStep = 0.5 * mm;;
			nSameLocation = static_cast<int>( std::floor((cLensPosFinal - cLensPosIni)/ cLensStep) ) + 1;
			for (int iterSameZ = 0; iterSameZ < nSameLocation; iterSameZ++)
				stagePosXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::FIELD_ILLUM:
		{
			//Measure how the fluorescence intensity differs across the FOV in 16X
			//Move the beads across the 16 PMT channels and average several images at each location
			sleepTime_ms = 1000; //Avoid overheating the sample

			stepSizeX = 17.5 * um;
			const int nLocationsX{ 16+7 };
			for (int iterDiffX = 0; iterDiffX < nLocationsX; iterDiffX++)
			{
				nSameLocation = 10;
				for (int iterSameZ = 0; iterSameZ < nSameLocation; iterSameZ++)
					stagePosXYZ.push_back({ stackCenterXYZ.XX + iterDiffX * stepSizeX, stackCenterXYZ.YY, stackCenterXYZ.ZZ });
			}
			break;
		}
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };
		Mesoscope mesoscope{ realtimeSeq, whichLaser };
		mesoscope.configure(fluorMarker.mWavelength_nm);

		//RS
		mesoscope.ResonantScanner::isRunning();		//To make sure that the RS is running

		//STAGES
		mesoscope.moveXYZ(stagePosXYZ.front());		//Move the stage to the initial position
		Sleep(500);									//Give the stages enough time to settle at the initial position
		mesoscope.waitForMotionToStopAll();

		//CREATE A STACK FOR STORING THE TIFFS
		const int nLocations{ static_cast<int>(stagePosXYZ.size()) };
		TiffU8 output{ heightPerFrame_pix, widthPerFrame_pix, nLocations };
		std::string filename{ g_currentSample.readName() + "_" + mesoscope.readCurrentLaser_s(true) + Util::toString(fluorMarker.mWavelength_nm, 0) + "nm" };
		
		//OPEN THE UNIBLITZ SHUTTERS
		mesoscope.openShutter();				//The destructor will close the shutter automatically

		//ACQUIRE FRAMES
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			//Reload the imaging parameters because they are deleted after each individual sequence
			realtimeSeq.reconfigure(heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam);

			//SCANNERS
			const Galvo scanner{ realtimeSeq, FFOVslowPerBeamlet / 2. };
			const Galvo rescanner{ realtimeSeq, FFOVslowPerBeamlet / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };
			//const Galvo rescanner{ realtimeSeq, 0, fluorMarker.mWavelength_nm, mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations  << "\n";
			mesoscope.moveXYZ(stagePosXYZ.at(iterLocation));
			mesoscope.waitForMotionToStopAll();
			//stage.printPosXYZ();				//Print the stage position	
			
			mesoscope.setPower(Util::exponentialFunction(fluorMarker.mScanPmin, iterLocation * pixelSizeZ, fluorMarker.mScanPLexp));
	
			//Used to optimize the collector lens position
			if (acqMode == RUNMODE::COLLECTLENS)
			{
				mesoscope.closeShutter();	//To avoid photobleaching when the galvo is on standby
				mesoscope.CollectorLens::move(cLensPosIni + iterLocation * cLensStep);
				mesoscope.openShutter();
				Sleep(100);
			}

			//EXECUTE THE CONTROL SEQUENCE
			realtimeSeq.run();					//Execute the control sequence and acquire the image
			Image image{ realtimeSeq };
			image.acquire(saveAllPMT);
			image.averageFrames();				//Average the frames imaged via continuous acquisition
			//image.averageEvenOddFrames();		//For debugging
			//image.correct(mesoscope.readFFOV());

			if (acqMode == RUNMODE::SINGLE && !saveAllPMT)
			{
				//Save individual files
				filename.append("_P=" + Util::toString(fluorMarker.mScanPmin / mW, 1) + "mW" +
					"_x=" + Util::toString(stagePosXYZ.at(iterLocation).XX / mm, 3) + "_y=" + Util::toString(stagePosXYZ.at(iterLocation).YY / mm, 3) + "_z=" + Util::toString(stagePosXYZ.at(iterLocation).ZZ / mm, 4) +
					"_avg=" + Util::toString(nFramesCont, 0));
				std::cout << "Saving the stack...\n";
				image.save(g_imagingFolderPath, filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
			}

			output.pushImage(image.data(), iterLocation);
			Sleep(sleepTime_ms);

			Util::pressESCforEarlyTermination();
		}//for-loop

		if (acqMode == RUNMODE::AVG)
		{
			filename.append("_P=" + Util::toString(fluorMarker.mScanPmin / mW, 1) +
				"_x=" + Util::toString(stagePosXYZ.front().XX / mm, 3) + "_y=" + Util::toString(stagePosXYZ.front().YY / mm, 3) +
				"_z=" + Util::toString(stagePosXYZ.front().ZZ / mm, 4) + "_avg=" + Util::toString(nFramesCont * nSameLocation, 0));

			output.binFrames(nSameLocation);		//Divide the images in bins and return the binned image
			std::cout << "Saving the stack...\n";
			output.saveToFile(g_imagingFolderPath, filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
		}

		if (acqMode == RUNMODE::SCANZ || acqMode == RUNMODE::SCANZCENTERED)
		{	
			filename.append( "_Pmin=" + Util::toString(fluorMarker.mScanPmin / mW, 1) + "mW_PLexp=" + Util::toString(fluorMarker.mScanPLexp / um, 0) + "um" +
				"_x=" + Util::toString(stagePosXYZ.front().XX / mm, 3) + "_y=" + Util::toString(stagePosXYZ.front().YY / mm, 3) +
				"_zi=" + Util::toString(stagePosXYZ.front().ZZ / mm, 4) + "_zf=" + Util::toString(stagePosXYZ.back().ZZ / mm, 4) + "_StepZ=" + Util::toString(pixelSizeZ / mm, 4) +
				"_avg=" + Util::toString(nFramesCont * nSameLocation, 0) );

			//output.binFrames(nSameLocation);		//Divide the images in bins and return the binned image
			std::cout << "Saving the stack...\n";
			output.saveToFile(g_imagingFolderPath, filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
		}

		if (acqMode == RUNMODE::SCANX || acqMode == RUNMODE::FIELD_ILLUM)
		{
			filename.append( "_P=" + Util::toString(fluorMarker.mScanPmin / mW, 1) + "mW" +
				"_xi=" + Util::toString(stagePosXYZ.front().XX / mm, 4) + "_xf=" + Util::toString(stagePosXYZ.back().XX / mm, 4) +
				"_y=" + Util::toString(stagePosXYZ.front().YY / mm, 4) +
				"_z=" + Util::toString(stagePosXYZ.front().ZZ / mm, 4) + "_StepX=" + Util::toString(stepSizeX / mm, 4) +
				"_avg=" + Util::toString(nFramesCont, 0) );

			output.binFrames(nSameLocation);		//Divide the images in bins and return the binned image
			std::cout << "Saving the stack...\n";
			output.saveToFile(g_imagingFolderPath, filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
		}

		//DATALOG
		{
			Logger datalog(g_imagingFolderPath, filename, OVERRIDE::DIS);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", g_currentSample.readName());
			datalog.record("Immersion medium = ", g_currentSample.readImmersionMedium());
			datalog.record("Correction collar = ", g_currentSample.readObjectiveCollar());
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", g_tickPerUs);
			datalog.record("\nLASER--------------------------------------------------------");
			datalog.record("Laser used = ", mesoscope.readCurrentLaser_s(false));
			datalog.record("Laser wavelength (nm) = ", mesoscope.readCurrentWavelength_nm());
			datalog.record("Min laser power (mW) = ", fluorMarker.mScanPmin / mW);
			datalog.record("Power exponential length (um) = ", fluorMarker.mScanPLexp / um);
			datalog.record("Laser repetition period (us) = ", g_laserPulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", mesoscope.ResonantScanner::readFFOV() / um);
			datalog.record("RS period (us) = ", 2 * g_lineclockHalfPeriod / us);
			datalog.record("Pixel dwell time (us) = ", g_pixelDwellTime / us);
			datalog.record("RS fill factor = ", mesoscope.ResonantScanner::readFillFactor());
			datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
			datalog.record("nFramesCont = ", nFramesCont);
			datalog.record("nSameLocation= ", nSameLocation);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", g_pulsesPerPix);
			datalog.record("Upscaling factor = ", g_upscalingFactor);
			datalog.record("Height Y (slow) (pix) = ", heightPerFrame_pix);
			datalog.record("Width X (fast) (pix) = ", widthPerFrame_pix);
			datalog.record("Resolution X (fast) (um/pix) = ", mesoscope.ResonantScanner::readSampleRes() / um);
			datalog.record("Resolution Y (slow) (um/pix) = ", pixelSizeXY / um);
			datalog.record("\nSTAGE--------------------------------------------------------");
			datalog.record("Stack center X (mm) = ", stackCenterXYZ.XX / mm);
			datalog.record("Stack center Y (mm) = ", stackCenterXYZ.YY / mm);
		}
		//Util::pressAnyKeyToCont();
	}

	//I triggered the stack acquisition using DO2 (stage motion) for both scan directions: top-down and bottom-up. In both cases the bead z-position looks almost identical with a difference of maybe only 1 plane (0.5 um)
	//Remember that I do not use MACROS on the stages anymore*/
	void contScanZ(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorMarkerList::FluorMarker fluorMarker{ g_currentSample.findFluorMarker("TDT") };		//Select a particular laser
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const SCANDIR scanDirZ{ SCANDIR::UPWARD };														//Scan direction for imaging in Z
		const int nFramesBinning{ fluorMarker.nFramesBinning };											//For binning
		const double stackDepth{ 100. * um };
		const double pixelSizeZafterBinning{ 1.0 * um  };

		const int nFrames{ static_cast<int>(nFramesBinning * stackDepth / pixelSizeZafterBinning) };	//Number of frames BEFORE binning for continuous acquisition
		const double pixelSizeZbeforeBinning{ pixelSizeZafterBinning / nFramesBinning };				//Pixel size per z frame
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };										//Full FOV in the slow axis

		POSITION3 stackCenterXYZ{ g_stackCenterXYZ };
		//stackCenterXYZ.ZZ -= nFrames * pixelSizeZbeforeBinning /2;//Center the stack<--------------------For beads!

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (g_multibeam)
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<double>(FFOVslow / g_nChanPMT);
		}
		else//Singlebeam
		{
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOVslow;
		}

		//CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFrames, g_multibeam };	//Note the STAGEZ flag
		Mesoscope mesoscope{ realtimeSeq, whichLaser };
		mesoscope.configure(fluorMarker.mWavelength_nm);
		mesoscope.setPowerExponentialScaling(fluorMarker.mScanPmin, pixelSizeZbeforeBinning, Util::convertScandirToInt(scanDirZ) * fluorMarker.mScanPLexp);

		//RS
		mesoscope.ResonantScanner::isRunning();		//To make sure that the RS is running

		//SCANNERS
		const Galvo scanner{ realtimeSeq, FFOVslowPerBeamlet / 2. };
		const Galvo rescanner{ realtimeSeq, FFOVslowPerBeamlet / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

		//STAGES
		const double stageZi = determineInitialScanPos(stackCenterXYZ.ZZ, stackDepth, 0. * mm, scanDirZ);
		const double stageZf = determineFinalScanPos(stackCenterXYZ.ZZ, stackDepth, 0.010 * mm, scanDirZ);						
		mesoscope.moveXYZ({ stackCenterXYZ.XX, stackCenterXYZ.YY, stageZi });		//Move the stage to the initial position
		mesoscope.waitForMotionToStopAll();
		Sleep(500);																	//Give the stages enough time to settle at the initial position
		//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix
		mesoscope.setVelSingle(AXIS::ZZ, pixelSizeZbeforeBinning / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));		

		//EXECUTE THE CONTROL SEQUENCE
		realtimeSeq.initialize(MAINTRIG::STAGEZ, fluorMarker.mWavelength_nm, scanDirZ);
		mesoscope.openShutter();				//Open the shutter. The destructor will close the shutter automatically
		std::cout << "Scanning the stack...\n";
		mesoscope.moveSingle(AXIS::ZZ, stageZf);//Move the stage to trigger the ctl&acq sequence
		realtimeSeq.downloadData();

		mesoscope.closeShutter();				//Close the shutter manually even though the destructor does it because the post-processing could take a long time
		Image image{ realtimeSeq };
		image.acquire();
		image.binFrames(nFramesBinning);
		image.correct(mesoscope.readFFOV());

		const std::string filename{ g_currentSample.readName() + "_" + mesoscope.readCurrentLaser_s(true) + Util::toString(fluorMarker.mWavelength_nm, 0) +
			"nm_Pmin=" + Util::toString(fluorMarker.mScanPmin / mW, 1) + "mW_PLexp=" + Util::toString(fluorMarker.mScanPLexp / um, 0) +
			"um_x=" + Util::toString(stackCenterXYZ.XX / mm, 3) + "_y=" + Util::toString(stackCenterXYZ.YY / mm, 3) +
			"_zi=" + Util::toString(stageZi / mm, 4) + "_zf=" + Util::toString(stageZf / mm, 4) + "_Step=" + Util::toString(pixelSizeZafterBinning / mm, 4) +
			"_bin=" + Util::toString(nFramesBinning, 0) };
		
		std::cout << "Saving the stack...\n";
		image.save(g_imagingFolderPath, filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);

		//Util::pressAnyKeyToCont();
	}

	void panoramicScan(const FPGA &fpga)
	{
		FluorMarkerList fluorMarkerList{ {{"TDT", 1040, 30. * mW }, { "DAPI", 750, 6.5 * mW }} };

		//ACQUISITION SETTINGS
		const FluorMarkerList::FluorMarker fluorMarker{ fluorMarkerList.findFluorMarker("TDT") };	//Select a particular fluorescence channel
		SCANDIR iterScanDirX{ SCANDIR::RIGHTWARD };			//Initial scan direction of stage 
		const double fullWidth{ 8.000 * mm };				//Total width of the tile array

		const double tileHeight{ 280. * um };
		const double tileWidth{ 150. * um };				//Width of a strip
		const double fullHeight{ 53 * tileHeight };			//Total height of the tile array = height of the strip (long vertical tile). If changed, the X-stage timing must be recalibrated
		const double pixelSizeX{ 1.0 * um };				//WARNING: the image becomes distorted at the edges of the strip when PANpixelSizeX > 1 um (check this again)
		const double pixelSizeY{ 0.5 * um };

		POSITION3 stackCenterXYZ{ g_stackCenterXYZ };
		PanoramicScan panoramicScan{ { stackCenterXYZ.XX, stackCenterXYZ.YY }, { tileHeight, tileWidth }, { pixelSizeX, pixelSizeY }, { fullHeight, fullWidth } };

		//CONTROL SEQUENCE. The Image height is 2 (two galvo swings) and nFrames is stitchedHeight_pix/2. The total height of the final image is therefore stitchedHeight_pix. Note the STAGEX flag
		RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, 2, panoramicScan.readTileWidth_pix(), panoramicScan.readTileHeight_pix() / 2, 0 };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::AUTO };
		mesoscope.configure(fluorMarker.mWavelength_nm);
		mesoscope.setPower(fluorMarker.mScanPmin);

		//RS
		mesoscope.ResonantScanner::isRunning();				//To make sure that the RS is running

		//SCANNERS. Keep them fixed at amplitude 0
		const double galvoScanAmplitude{ 0 };
		const Galvo scanner{ realtimeSeq, galvoScanAmplitude };
		const Galvo rescanner{ realtimeSeq, galvoScanAmplitude, mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

		//STAGES
		mesoscope.moveXYZ({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ });			//Move the stage to the initial position
		mesoscope.waitForMotionToStopAll();
		Sleep(500);																				//Give the stages enough time to settle at the initial position
		mesoscope.setVelSingle(AXIS::XX, pixelSizeX / g_lineclockHalfPeriod);					//Set the vel for imaging

		mesoscope.openShutter();	//Open the shutter. The destructor will close the shutter automatically

		//LOCATIONS on the sample to image
		const int nLocations{ panoramicScan.readNumberStageYpos() };
		double stageXi, stageXf;		//Stage final position
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			const double travelOverhead{ 1.0 * mm};
			stageXi = panoramicScan.determineInitialScanPosX(travelOverhead, iterScanDirX);
			stageXf = panoramicScan.determineFinalScanPosX(travelOverhead, iterScanDirX);

			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations << "\n";
			mesoscope.moveXY({ stageXi, panoramicScan.readStageYposAt(iterLocation) });
			mesoscope.waitForMotionToStopAll();

			Sleep(300);													//Avoid iterations too close to each other, otherwise the X-stage will fail to trigger the ctl&acq sequence.
																		//This might be because of g_postSequenceTimer

			realtimeSeq.initialize(MAINTRIG::STAGEX);
			std::cout << "Scanning the stack...\n";
			mesoscope.moveSingle(AXIS::XX, stageXf);					//Move the stage to trigger the ctl&acq sequence
			realtimeSeq.downloadData();

			Image image{ realtimeSeq };
			image.acquireVerticalStrip(iterScanDirX);
			image.correctRSdistortion(tileWidth);						//Correct the image distortion induced by the nonlinear scanning of the RS
			panoramicScan.push(image.data(), { 0, iterLocation });		//for now, only allow to stack up strips to the right

			reverseSCANDIR(iterScanDirX);
			Util::pressESCforEarlyTermination();
		}
			mesoscope.closeShutter();

			const std::string filename{ g_currentSample.readName() + "_" + mesoscope.readCurrentLaser_s(true) + Util::toString(fluorMarker.mWavelength_nm, 0) +
				"nm_P=" + Util::toString(fluorMarker.mScanPmin / mW, 1) +
				"mW_xi=" + Util::toString(stageXi / mm, 3) + "_xf=" + Util::toString(stageXf / mm, 3) +
				"_yi=" + Util::toString(panoramicScan.readStageYposFront() / mm, 3) + "_yf=" + Util::toString(panoramicScan.readStageYposBack() / mm, 3) +
				"_z=" + Util::toString(stackCenterXYZ.ZZ / mm, 4) };
			std::cout << "Saving the stack...\n";
			panoramicScan.saveToFile(g_imagingFolderPath, filename, OVERRIDE::DIS);

			//Tile size for the slow scan. Do not call the tile size from panoramicScan because the tiles are long strips. 
			const PIXDIM2 overlayTileSize_pix{ Util::intceil(tileHeight / pixelSizeX), Util::intceil(tileWidth / pixelSizeY) };
			const TILEOVERLAP3 overlayTileOverlapXYZ_frac{ 0.0, 0.0, 0.0 };
			const double threshold{ 0.02 };

			//Boolmap boolmap{ panoramicScan, overlayTileSize_pix, overlayTileOverlapXYZ_frac, threshold };
			//boolmap.saveBoolmapToText("Boolmap_" + filename);
			//boolmap.saveTiffWithBoolmapTileOverlay("TileArrayMap_" + filename);
	}

	//Full sequence to image and cut an entire sample automatically. Note that the stack starts at stackCenterXYZ.at(Z) (i.e., the stack is not centered at stackCenterXYZ.at(Z))
	//When forceScanAllStacks = true, the full boolmap is set to 1 (i.e., the boolmap does not have any effect on the scanning)
	void sequencer(const FPGA &fpga, const int firstCommandIndex, const bool forceScanAllStacks, const RUN runSeq)
	{
		//for beads, center the stack around g_stackCenterXYZ.at(Z) -----> //const double sampleSurfaceZ{ g_stackCenterXYZ.ZZ - nFramesCont * pixelSizeZ / 2 };

		//ACQUISITION SETTINGS
		const double stackDepth{ 100. * um };
		const double pixelSizeZafterBinning{ 1.0 * um };												//Step size in the Z-stage axis

		//TILE SCAN
		const int nFramesAfterBinning{ static_cast<int>(stackDepth / pixelSizeZafterBinning) };
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };			//Full FOV in the (slow axis, fast axis)
		const LENGTH3 LOIxyz{ 10.000 * mm, 8.000 * mm, 4.500 * mm };
		//const LENGTH3 LOIxyz{ 1.000 * mm, 1.000 * mm, 0.150 * mm };
		const double cutAboveBottomOfStack{ 50. * um };													//Distance to cut above the bottom of the stack
		const double sampleSurfaceZ{ g_stackCenterXYZ.ZZ };
		const SCANDIR ScanDirZini{ SCANDIR::UPWARD };
		const TILEOVERLAP3 stackOverlap_frac{ 0.15, 0.10, 0.50 };										//Stack overlap

		//PANORAMIC SCAN
		const double PANwidth{ 10.000 * mm };															//Total width of the panoramic scan
		const double PANtileHeight{ heightPerFrame_pix * pixelSizeXY };
		const double PANtileWidth{ widthPerFrame_pix * pixelSizeXY };									//Width of a strip of the panoramic scan
		const double PANheight{ 53 * PANtileHeight };													//Total height of the panoramic scan = height of the strip (long vertical tile). If changed, the X-stage timing must be recalibrated
		const double PANpixelSizeX{ 1.0 * um };															//WARNING: the image becomes distorted at the edges of the strip when PANpixelSizeX > 1 um (check this again)
		const double PANpixelSizeY{ pixelSizeXY };
		const double PANlaserPower{ 30. * mW };
		const int PANwavelength_nm{ 1040 };
		const double threshold{ 0.02 };

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (g_multibeam)
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<double>(FFOV.XX / g_nChanPMT);
		}
		else //Singlebeam
		{
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOV.XX;
		}

		//Create a sequence
		const Sample sample{ g_currentSample, {g_stackCenterXYZ.XX, g_stackCenterXYZ.YY}, LOIxyz, sampleSurfaceZ, cutAboveBottomOfStack };
		const Stack stack{ FFOV, heightPerFrame_pix, widthPerFrame_pix, pixelSizeZafterBinning, nFramesAfterBinning, stackOverlap_frac };
		Sequencer sequence{ sample, stack };
		const TILEDIM2 tileArraySizeIJ{ sequence.readTileArraySizeIJ() };	//Dimension of the tile array

		sequence.generateCommandList();
		sequence.printToFile(g_imagingFolderPath, "_Commandlist", OVERRIDE::EN);

		if (runSeq == RUN::EN)
		{
			//CONTROL SEQUENCE
			RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, 100, 1 };
			Mesoscope mesoscope{ realtimeSeq, Laser::ID::AUTO };
			mesoscope.configure(1040);								//Initialize the laser wavelength for determining the initial chromatic shift correction of the stages

			//RS
			mesoscope.ResonantScanner::isRunning();					//To make sure that the RS is running

			//Read the commands line by line
			POSITION2 tileCenterXY;
			std::string shortName, longName;
			SCANDIR iterScanDirZ{ ScanDirZini };
			Logger datalogPanoramic(g_imagingFolderPath, "_Panoramic", OVERRIDE::DIS);
			Logger datalogStacks(g_imagingFolderPath, "_TileConfiguration", OVERRIDE::DIS);
			datalogStacks.record("dim=3"); //Needed for GridStitcher on Fiji

			//BOOLMAP. Declare the boolmap here to pass it between different actions
			std::vector<bool> vec_boolmap(tileArraySizeIJ.II * tileArraySizeIJ.JJ, forceScanAllStacks);
			int brightStackIndex{ 0 };
			for (int iterCommandline = firstCommandIndex; iterCommandline < sequence.readNtotalCommands(); iterCommandline++)
			{
				Sequencer::Commandline commandline{ sequence.readCommandline(iterCommandline) };

				//These parameters must be accessible to all the switch-cases
				int wavelength_nm, nFramesBinning, cutNumber, tileIndexII, tileIndexJJ;
				double scanZi, scanZf, scanPmin, scanPLexp;
				switch (commandline.mActionID)
				{
				case Action::ID::MOV://Move the X and Y-stages to mStackCenterXY
					cutNumber = commandline.mAction.moveStage.readCutNumber();
					tileIndexII = commandline.mAction.moveStage.readTileIndex(TileArray::Axis::II);
					tileIndexJJ = commandline.mAction.moveStage.readTileIndex(TileArray::Axis::JJ);
					tileCenterXY = commandline.mAction.moveStage.readTileCenterXY();

					if (Util::isBright(vec_boolmap, tileArraySizeIJ, { tileIndexII, tileIndexJJ }))
					{
						mesoscope.moveXY(tileCenterXY);
						mesoscope.waitForMotionToStopAll();
					}
					break;
				case Action::ID::ACQ://Acquire a stack
					//std::cout << "is (" << tileIndexII_s << "," << tileIndexJJ_s << ") bright? = " << Util::isBright(vec_boolmap, tileArraySizeIJ, { tileIndexII_s, tileIndexJJ_s });
					//Util::pressAnyKeyToCont();

					if (Util::isBright(vec_boolmap, tileArraySizeIJ, { tileIndexII, tileIndexJJ }))
					{
						Action::AcqStack acqStack{ commandline.mAction.acqStack };
						{
							//Set the number of frames considering that binning will be performed
							nFramesBinning = acqStack.readNframeBinning();
							const int nFramesBeforeBinning{ nFramesAfterBinning * nFramesBinning };
							realtimeSeq.reconfigure(heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesBeforeBinning, g_multibeam);

							//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix	
							const double pixelSizeZbeforeBinning{ stackDepth / nFramesBeforeBinning };
							mesoscope.setVelSingle(AXIS::ZZ, pixelSizeZbeforeBinning / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));

							//These parameters must be accessible for saving the tiff
							//stackIndex = acqStack.readStackIndex();
							wavelength_nm = acqStack.readWavelength_nm();
							scanZi = determineInitialScanPos(acqStack.readScanZmin(), stackDepth, 0. * mm, iterScanDirZ);
							scanZf = determineFinalScanPos(acqStack.readScanZmin(), stackDepth, 0. * mm, iterScanDirZ);

							//Update the laser parameters
							mesoscope.configure(wavelength_nm);									//The uniblitz shutter is closed by the pockels destructor when switching wavelengths
							scanPmin = acqStack.readScanPmin();
							scanPLexp = acqStack.readScanPLexp();
							mesoscope.setPowerExponentialScaling(scanPmin, pixelSizeZbeforeBinning, Util::convertScandirToInt(iterScanDirZ) * scanPLexp);

							//SCANNERS
							const Galvo scanner{ realtimeSeq, FFOVslowPerBeamlet / 2. };
							Galvo rescanner{ realtimeSeq, FFOVslowPerBeamlet / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };
						}

						//Move the stage to the initial Z position
						mesoscope.moveSingle(AXIS::ZZ, scanZi);
						mesoscope.waitForMotionToStopAll();

						realtimeSeq.initialize(MAINTRIG::STAGEZ, wavelength_nm, iterScanDirZ);	//Use the scan direction determined dynamically
						mesoscope.openShutter();												//Re-open the Uniblitz shutter if closed by the pockels destructor

						//Print out the stackIndex starting from 1 (stackIndex indexes from 0, so add a 1) and the cutNumber_s starting from 1 (cutNumber_s indexes from 0, so add a 1)
						std::cout << "Scanning cut = " << std::to_string(cutNumber + 1) << "/" << sequence.readTotalNumberOfCuts() <<
							"\tstack = " << std::to_string(brightStackIndex + 1) << "/" << Util::determineNumberOf1s(vec_boolmap) <<
							"\tStack index = (" << tileIndexII << "," << tileIndexJJ << ")\n";

						mesoscope.moveSingle(AXIS::ZZ, scanZf);	//Move the stage to trigger the ctl&acq sequence
						realtimeSeq.downloadData();
						reverseSCANDIR(iterScanDirZ);
						brightStackIndex++;
					}//if
					break;
				case Action::ID::SAV:
					if (Util::isBright(vec_boolmap, tileArraySizeIJ, { tileIndexII, tileIndexJJ }))
					{
						const std::string tileIndexIIpadded{ Util::zeroPadding(tileIndexII, 2) };
						const std::string tileIndexJJpadded{ Util::zeroPadding(tileIndexJJ, 2) };

						shortName = Util::zeroPadding(cutNumber, 3) + "_" + Util::convertWavelengthToFluorMarker_s(wavelength_nm) + "_" + tileIndexIIpadded + "_" + tileIndexJJpadded;		//cutNumber_stackIndex_wavelengthIndex
						longName = mesoscope.readCurrentLaser_s(true) + Util::toString(wavelength_nm, 0) + "nm_Pmin=" + Util::toString(scanPmin / mW, 1) + "mW_PLexp=" + Util::toString(scanPLexp / um, 0) + "um" +
							"_x=" + Util::toString(tileCenterXY.XX / mm, 3) +
							"_y=" + Util::toString(tileCenterXY.YY / mm, 3) +
							"_zi=" + Util::toString(scanZi / mm, 4) + "_zf=" + Util::toString(scanZf / mm, 4) +
							"_Step=" + Util::toString(pixelSizeZafterBinning / mm, 4) + "_bin=" + Util::toString(nFramesBinning, 0);

						Image image{ realtimeSeq };
						image.acquire();
						image.binFrames(nFramesBinning);
						image.save(g_imagingFolderPath, shortName, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);

						//Convert the absolute tile position to pixels to be called by Grid/collection stitcher in Fiji.
						datalogStacks.record(shortName + ".tif;;\t(" + Util::toString(-tileCenterXY.YY/pixelSizeXY, 0) + "," +	//Note that in Fiji II and JJ are interchanged. Also note the minus sign to change the sign convention
																	   Util::toString(-tileCenterXY.XX/pixelSizeXY, 0) + "," +
																	   Util::toString((std::min)(scanZi, scanZf)/pixelSizeZafterBinning, 0) + ")");
						datalogStacks.record("#(" + tileIndexIIpadded + "," + tileIndexJJpadded + ")\t" + longName);
						std::cout << "\n";
					}
					break;
				case Action::ID::CUT://Move the stage to the vibratome and then cut off the surface
				{
					mesoscope.closeShutter();

					std::cout << "Cut number = " << std::to_string(cutNumber + 1) << "/" << sequence.readTotalNumberOfCuts() << "\n";
					const double planeZtoCut{ commandline.mAction.cutTissue.readStageZheightForFacingTheBlade() };
					mesoscope.cutTissue(planeZtoCut);

					//Reset the scan direction
					iterScanDirZ = ScanDirZini;

					brightStackIndex = 0;

					//Reset the boolmap
					vec_boolmap.assign(tileArraySizeIJ.II * tileArraySizeIJ.JJ, forceScanAllStacks);
				}
				break;
				case Action::ID::PAN:
				{
					const double PANplaneZ{ commandline.mAction.panoramicScan.readPlaneZ() };
					SCANDIR iterScanDirX{ SCANDIR::RIGHTWARD };												//Initial scan direction of stage 

					//CONTROL SEQUENCE. The Image height is 2 (two galvo swings) and nFrames is stitchedHeight_pix/2. The total height of the final image is therefore stitchedHeight_pix
					PanoramicScan panoramicScan{ { g_stackCenterXYZ.XX, g_stackCenterXYZ.YY }, { PANtileHeight, PANtileWidth }, { PANpixelSizeX, PANpixelSizeY }, { PANheight, PANwidth } };
					realtimeSeq.reconfigure(2, panoramicScan.readTileWidth_pix(), panoramicScan.readTileHeight_pix()/2, 0);
					mesoscope.configure(PANwavelength_nm);
					mesoscope.setPower(PANlaserPower);

					//SCANNERS. Keep them fixed at amplitude 0
					const Galvo scanner{ realtimeSeq, 0 };
					const Galvo rescanner{ realtimeSeq, 0, mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

					//STAGES
					mesoscope.moveXYZ({ g_stackCenterXYZ.XX, g_stackCenterXYZ.YY, PANplaneZ });				//Move the stage to the center of the panoramic view
					mesoscope.waitForMotionToStopAll();
					Sleep(500);																				//Give the stages enough time to settle at the initial position
					mesoscope.setVelSingle(AXIS::XX, PANpixelSizeX / g_lineclockHalfPeriod);				//Set the vel for imaging
					mesoscope.openShutter();																//Open the shutter

					//LOCATIONS of the sample to image
					const int nLocations{ panoramicScan.readNumberStageYpos() };
					double stageXi, stageXf;		//Stage final position
					for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
					{
						const double travelOverhead{ 1.0 * mm };
						stageXi = panoramicScan.determineInitialScanPosX(travelOverhead, iterScanDirX);
						stageXf = panoramicScan.determineFinalScanPosX(travelOverhead, iterScanDirX);

						std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations << "\n";
						mesoscope.moveXY({ stageXi, panoramicScan.readStageYposAt(iterLocation) });			//Move the stage to the start of the "ribbon" scan
						mesoscope.waitForMotionToStopAll();
						Sleep(300);																			//Avoid iterations too close to each other, otherwise the X-stage will fail to trigger the ctl&acq sequence.
																											//This might be because of g_postSequenceTimer
						realtimeSeq.initialize(MAINTRIG::STAGEX);
						std::cout << "Scanning the stack...\n";
						mesoscope.moveSingle(AXIS::XX, stageXf);											//Move the stage to trigger the ctl&acq sequence
						realtimeSeq.downloadData();

						Image image{ realtimeSeq };
						image.acquireVerticalStrip(iterScanDirX);
						image.correctRSdistortion(PANtileWidth);											//Correct the image distortion induced by the nonlinear scanning of the RS
						panoramicScan.push(image.data(), { 0, iterLocation });								//for now, only allow to stack up strips to the right

						reverseSCANDIR(iterScanDirX);
						Util::pressESCforEarlyTermination();
					}
					mesoscope.closeShutter();

					//SAVE THE FILES
					const std::string PANlongName{ mesoscope.readCurrentLaser_s(true) + Util::toString(PANwavelength_nm, 0) +
						"nm_P=" + Util::toString(PANlaserPower / mW, 1) +
						"mW_xi=" + Util::toString(stageXi / mm, 3) + "_xf=" + Util::toString(stageXf / mm, 3) +
						"_yi=" + Util::toString(panoramicScan.readStageYposFront() / mm, 3) + "_yf=" + Util::toString(panoramicScan.readStageYposBack() / mm, 3) +
						"_z=" + Util::toString(PANplaneZ / mm, 4) };
					
					const std::string PANcutNumberPadded{ Util::zeroPadding(commandline.mAction.panoramicScan.readCutNumber(), 3) };
					panoramicScan.saveToFile(g_imagingFolderPath, "Panoramic_" + PANcutNumberPadded, OVERRIDE::DIS);//For large tiffs, tifflib sometimes returns "no space for output buffer" error
					datalogPanoramic.record(PANcutNumberPadded + "\t" + PANlongName);

					//DETERMINE THE BOOLMAP
					const LENGTH2 LOIxy_pix{ LOIxyz.XX / (2 * pixelSizeXY), LOIxyz.YY / pixelSizeXY };
					const PIXDIM2 overlayTileSize_pix{ heightPerFrame_pix / 2, widthPerFrame_pix };								//Tile size for the slow scan. Do not call the tile size from panoramicScan because the tiles are long strips
																																//Note the factor of 2 because PANpixelSizeX=1.0*um whereas pixelSizeXY=0.5*um
					Boolmap boolmap{ panoramicScan, tileArraySizeIJ, overlayTileSize_pix, stackOverlap_frac, threshold };		//NOTE THE FACTOR OF 2 IN X
					boolmap.fillBoolmapHoles();
					boolmap.saveBoolmapToText(g_imagingFolderPath, "Boolmap_" + PANcutNumberPadded, OVERRIDE::DIS);
					boolmap.replaceInputBoolmapByUnion(vec_boolmap);															//Save the boolmap for the next iterations
					
					//boolmap.saveTiffWithBoolmapGridOverlay("GridOverlay", OVERRIDE::EN);//For debugging
				}
				break;
				default:
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
				}//switch(mAction)
				Util::pressESCforEarlyTermination();
			}//for(iterCommandline)
			mesoscope.closeShutter();
			//Util::saveBoolmapToText("Union", vec_boolmap, tileArraySizeIJ, OVERRIDE::EN);//For debugging
		}//if (run)
		Util::pressAnyKeyToCont();
	}

	//Image the sample non-stop. Use PIMikroMove to move the stages manually
	//20190916: an stage object class member was included in Mesoscope. It will block the connection with PIMikroMove
	void liveScan(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorMarkerList::FluorMarker fluorMarker{ g_currentSample.findFluorMarker("TDT") };	//Select a particular fluorescence channel
		const int nFramesCont{ 1 };																	//Number of frames for continuous acquisition
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };									//Full FOV in the slow axis

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::VISION };
		mesoscope.configure(fluorMarker.mWavelength_nm);

		//RS
		mesoscope.ResonantScanner::isRunning();														//To make sure that the RS is running

		//SCANNERS
		const Galvo scanner{ realtimeSeq, FFOVslow / 2. };
		const Galvo rescanner{ realtimeSeq, FFOVslow / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

		//OPEN THE UNIBLITZ SHUTTERS
		mesoscope.openShutter();											//The destructor will close the shutter automatically

		while (true)
		{
			mesoscope.setPower(fluorMarker.mScanPmin);						//Set the laser power

			realtimeSeq.run();
			Image image{ realtimeSeq };
			image.acquire();												//Execute the control sequence
			image.averageFrames();											//Average the frames acquired via continuous acquisition
			//image.correct(RScanner.mFFOV);
			image.save(g_imagingFolderPath, "Untitled", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);	//Save individual files
			Sleep(700);

			Util::pressESCforEarlyTermination();							//Early termination if ESC is pressed
		}
	}

	//Post-process the tiff stacks: RS, flatfield, and crosstalk correction
	//Read the file names from "_TileConfiguration.txt"
	//Create a configuration text file for each vibratome cut and laser wavelength for Fiji's GridStitcher
	void correctTiffReadFromTileConfiguration(const int firstCutNumber, const int lastCutNumber, const std::vector<int> vec_wavelengthIndex)
	{
		const std::string inputPath{ "D:\\20191129_Liver20190812_03_lobe_sorted\\" };
		const std::string outputPath{ "D:\\20191129_Liver20190812_03_lobe_corrected\\" };

		if (firstCutNumber > lastCutNumber)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The first cut number must be <= last cut number");
		if (vec_wavelengthIndex.size() == 0)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": At least one wavelength must be input as argument");

		//For Fiji's GridStitcher, generate a configuration text file for each vibratome cut and each laser wavelength
		//The file format is "_TileConfigurationCorrected_cutNumber_wavelengthIndex"
		std::vector<std::ofstream> vec_outputConfigTxt( (lastCutNumber - firstCutNumber + 1) * vec_wavelengthIndex.size());
		for (int iterCutNumber = firstCutNumber; iterCutNumber <= lastCutNumber; iterCutNumber++)
			for (std::vector<int>::size_type iterVec = 0; iterVec != vec_wavelengthIndex.size(); iterVec++)
			{		
				const unsigned int index{ (iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec };
				const std::string subFolderName{ Util::zeroPadding(iterCutNumber, 3) + "\\" };
				vec_outputConfigTxt.at(index).open(outputPath + subFolderName + "_TileConfigurationCorrected_" + Util::zeroPadding(iterCutNumber, 3) + "_" + Util::toString(vec_wavelengthIndex.at(iterVec), 0) + ".txt");
				vec_outputConfigTxt.at(index) << "dim=3\n";	//Needed at the start of the txt for GridStitcher
			}

		//Open the text file with the Tiff parameters
		std::ifstream inputConfigTxt{ inputPath + "_TileConfiguration.txt" };

		if (!inputConfigTxt)
			throw std::runtime_error((std::string)__FUNCTION__ + ": The file _TileConfiguration.txt failed to open");
		
		//Read the text file with the stack parameters line by line
		TiffU8 image{ 560, 300, 100 };
		std::string line;
		getline(inputConfigTxt, line);			//Skip the first line that contains "dim=3"
		while (getline(inputConfigTxt, line))
			if (line.front() != '#')	//Skip all the lines that are commented out with #
			{
				//Get the Tiff parameters at the beginning of each text line
				//Tokenize with respect to '_'. //Convert the parameters to int
				//The format is "cutNumber_wavelengthIndex_tileIndexII_tileIndexJJ", e.g. "000_0_17_31"
				std::stringstream tiffParameters_ss(line.substr(0, line.find(".tif")));
				std::string isolatedParameter;
				getline(tiffParameters_ss, isolatedParameter, '_');			//cutNumber	
				const int cutNumber{ std::stoi(isolatedParameter) };		
				getline(tiffParameters_ss, isolatedParameter, '_');			//wavelengthIndex
				const int wavelengthIndex{ std::stoi(isolatedParameter) };	
				getline(tiffParameters_ss, isolatedParameter, '_');			//tileIndexII
				const int tileIndexII{ std::stoi(isolatedParameter) };		
				getline(tiffParameters_ss, isolatedParameter, '_');			//tileIndexJJ
				const int tileIndexJJ{ std::stoi(isolatedParameter) };		

				//Iterate over the vibratome cuts and laser wavelengths
				for (int iterCutNumber = firstCutNumber; iterCutNumber <= lastCutNumber; iterCutNumber++)
					for (std::vector<int>::size_type iterVec = 0; iterVec != vec_wavelengthIndex.size(); iterVec++)
						if (cutNumber == iterCutNumber && wavelengthIndex == vec_wavelengthIndex.at(iterVec))

							//Constrain the stacks
							//if (tileIndexII >= 25 && tileIndexII <= 26 && tileIndexJJ >= 29 && tileIndexJJ <= 30) 
							{
								image.loadTiffU8(inputPath + Util::zeroPadding(iterCutNumber, 3) + "\\", tiffParameters_ss.str());
								image.correctRSdistortionGPU(150. * um);
								image.flattenFieldGaussian(0.015);//0.010 for DAPI; 0.015 for TDT
								image.suppressCrosstalk(0.20);

								const std::string subFolderName{ Util::zeroPadding(iterCutNumber, 3) + "\\" };
								image.saveToFile(outputPath, subFolderName + "corrected_" + tiffParameters_ss.str(), TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);

								//Save the filename in the tileConfiguration text file
								vec_outputConfigTxt.at((iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec) << "corrected_" << line << "\n";

								//std::cout << "Cut number = " << cutNumber << "\twavelengthIndex = " << wavelengthIndex << "\t(II,JJ) = (" << tileIndexII << "," << tileIndexJJ << ")\n";//For debugging
								Util::pressESCforEarlyTermination();
							}
			}//if(line.front() != '#')
		
		//Close the text files
		for (int iterCutNumber = firstCutNumber; iterCutNumber <= lastCutNumber; iterCutNumber++)
			for (std::vector<int>::size_type iterVec = 0; iterVec != vec_wavelengthIndex.size(); iterVec++)
				vec_outputConfigTxt.at( (iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec ).close();

		inputConfigTxt.close();
		//Util::pressAnyKeyToCont();
	}
}//namespace

//Photobleach the sample with the resonant scanner to see how much the sample moves after slicing
//I bleach with the RS and not the galvo scanner or the stages because this way the RS is kept on the entire time while bleaching and imaging
namespace TestRoutines
{
	//Generate many short digital pulses and check the overall frameDuration with the oscilloscope
	void digitalLatency(const FPGA &fpga)
	{
		const double timeStep{ 4. * us };

		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 1);

		//Many short digital pulses to accumulate the error
		for (U32 ii = 0; ii < 99; ii++)
			realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 0);

		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 0);
	}

	//First calibrate the digital channels, then use it as a time reference
	void analogLatency(const FPGA &fpga)
	{
		const double delay{ 400. * us };
		const double timeStep{ 4. * us };

		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, timeStep, 10 * V);					//Initial pulse
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, timeStep, 0);
		realtimeSeq.pushLinearRamp(RTseq::RTCHAN::SCANNER, 4 * us, delay, 0, 5 * V, OVERRIDE::DIS);	//Linear ramp to accumulate the error
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, timeStep, 10 * V);					//Initial pulse
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, timeStep, 0);							//Final pulse

		//DO0
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 0);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, delay, 0);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 0);
	}

	void pixelclock(const FPGA &fpga)
	{
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam }; 		//Create the control sequence
		realtimeSeq.run();																			//Execute the control sequence
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGA &fpga)
	{
		const double timeStep{ 400. * us };

		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, timeStep, 0);
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGA &fpga)
	{
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		//DO
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, 4 * us, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, 4 * us, 0);

		//AO
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, 8 * us, 4 * V);
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, 4 * us, 2 * V);
		realtimeSeq.pushAnalogSinglet(RTseq::RTCHAN::SCANNER, 4 * us, 1 * V);

		realtimeSeq.mFpga.triggerControlSequence();	//Execute the control sequence
	}

	void analogRamp(const FPGA &fpga)
	{
		const double Vmax{ 5. * V };
		const double step{ 4. * us };

		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };
		realtimeSeq.pushLinearRamp(RTseq::RTCHAN::SCANNER, step, 2 * ms, 0, -Vmax, OVERRIDE::DIS);
		realtimeSeq.pushLinearRamp(RTseq::RTCHAN::SCANNER, step, 20 * ms, -Vmax, Vmax, OVERRIDE::DIS);
		realtimeSeq.pushLinearRamp(RTseq::RTCHAN::SCANNER, step, 2 * ms, Vmax, 0, OVERRIDE::DIS);

		const double pulsewidth(300. * us);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, pulsewidth, 1);
		realtimeSeq.pushDigitalSinglet(RTseq::RTCHAN::DODEBUG, 4 * us, 0);
	}

	//I think this is for matching the galvos forward and backward scans via imaging beads
	void fineTuneScanGalvo(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 400 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 30 };										//Number of frames for continuous acquisition
		const POSITION3 stagePosXYZ{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//CREATE THE CONTROL SEQUENCE
		const int wavelength_nm{ 1040 };
		const double laserPower{ 25. * mW };		//Laser power
		RTseq realtimeSeq{ fpga, LINECLOCK::RS, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::FIDELITY };
		mesoscope.configure(wavelength_nm);
		mesoscope.setPower(laserPower);

		//RS
		mesoscope.ResonantScanner::isRunning();		//To make sure that the RS is running

		//STAGES
		mesoscope.moveXYZ(stagePosXYZ);
		mesoscope.waitForMotionToStopAll();

		//SCANNERS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
		Galvo scanner{ realtimeSeq, FFOVslow / 2. };

		//CONTROL SEQUENCE
		mesoscope.openShutter();	//Open the uniblitz shutter. The destructor will close the shutter automatically
		realtimeSeq.run();
		Image image{ realtimeSeq };
		image.acquire();			//Execute the control sequence and acquire the image via continuous acquisition
		image.averageEvenOddFrames();
		image.save(g_imagingFolderPath, "Untitled", TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
	}

	void resonantScanner(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		ResonantScanner RScanner{ realtimeSeq };
		std::cout << "aaa = " << RScanner.downloadControlVoltage() << "\n";
		//RScanner.turnOnWithFOV(150 * um);
		//RScanner.turnOff();
	}

	//Use a single beamlet with the rescanner sync'ed to the scanner to keep the beam position fixed at the detector plane
	//Must manually open the laser and Uniblitz shutter and adjust the pockels power
	void galvosSync(const FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 10 };

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, heightPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };

		//SCANNERS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Scan duration in the slow axis
		const int wavelength_nm{ 750 };
		Galvo scanner{ realtimeSeq, FFOVslow / 2. };
		Galvo rescanner{ realtimeSeq, FFOVslow / 2.,  Laser::ID::VISION, wavelength_nm };

		//Execute the control sequence and acquire the image
		realtimeSeq.run();
		Util::pressAnyKeyToCont();
	}

	//Test the synchronization of the 2 galvos and the laser
	void galvosLaserSync(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 400 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet, selectPower;

		if (g_multibeam) //Multibeam
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<int>(FFOVslow / g_nChanPMT);
			selectPower = 800. * mW;
		}
		else             //Singlebeam
		{		
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOVslow;
			selectPower = 50. * mW;
		}

		//STACK
		const double pixelSizeZ{ 1.0 * um };
		//const double stackDepthZ{ 20. * um };	//Acquire a stack this deep in the Z-stage axis

		//CREATE THE CONTROL SEQUENCE
		const int wavelength_nm{ 1040 };
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::AUTO };
		mesoscope.configure(wavelength_nm);
		mesoscope.setPower(selectPower);

		//SCANNERS
		const Galvo scanner{ realtimeSeq, FFOVslowPerBeamlet / 2. };
		const Galvo rescanner{ realtimeSeq, FFOVslowPerBeamlet / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };
		//const Galvo rescanner{ realtimeSeq, 0, wavelengthIndex_s, mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };

		//EXECUTE THE CONTROL SEQUENCE
		realtimeSeq.run();
	}

	void stagePosition()
	{
		double duration;
		const POSITION3 stagePosXYZ{ 35.020 * mm, 19.808 * mm, 18.542 * mm };	//Stage initial position
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		std::cout << "Stages initial position:" << "\n";
		stage.printPosXYZ();

		auto t_start{ std::chrono::high_resolution_clock::now() };

		stage.moveXYZ(stagePosXYZ);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		stage.waitForMotionToStopSingle(AXIS::ZZ);

		std::cout << "Stages final position:" << "\n";
		stage.printPosXYZ();

		/*
		int input = 1;
		while (input)
		{
			std::cout << "Stage X position = " << stage.downloadPositionSingle_(X) << "\n";
			std::cout << "Stage Y position = " << stage.downloadPositionSingle_(Y) << "\n";
			std::cout << "Stage X position = " << stage.downloadPositionSingle_(Z) << "\n";

			std::cout << "Enter command: ";
			std::cin >> input;
			//input = 0;
		}
		*/
	}

	//Test configuring setDOtriggerEnabled and CTO for the stages
	void stageConfig()
	{
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };
		const Stage::DIOCHAN DOchan{ Stage::DIOCHAN::D1 };

		//std::cout << "Stages initial position: \n";
		//stage.printPosXYZ();
		std::cout << "Before vel: \n";
		stage.setVelSingle(AXIS::ZZ, 0.4 * mmps);

		//std::cout << "Stages initial vel: \n";
		//stage.printVelXYZ();

		//stage.isDOtriggerEnabled(Z, DOchan);
		//stage.setDOtriggerEnabled(Z, DOchan , true);

		//const Stage::DOPARAM triggerParam{ Stage::DOPARAM::TRIGSTEP };
		//stage.downloadDOtriggerParamSingle_(Z, DOchan , triggerParam);
		//std::cout << "X-stage vel: " << stage.downloadVelSingle_(X) / mmps << " mm/s" << "\n";
		//std::cout << "Y-stage vel: " << stage.downloadVelSingle_(Y) / mmps << " mm/s" << "\n";
		//std::cout << "Z-stage vel: " << stage.downloadVelSingle_(Z) / mmps << " mm/s" << "\n";
		//stage.printStageConfig(Z, DOchan);
	}

	void shutter(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		Pockels fidelity{ realtimeSeq, 1040, Laser::ID::FIDELITY };
		fidelity.setShutter(true);
		Sleep(5000);
		fidelity.setShutter(false);

		//Shutter shutterFidelity(fpga, FIDELITY);
		//shutterFidelity.open();
		//Sleep(5000);
		//shutterFidelity.close();
	}

//For keeping the pockels on to check the the laser power
//0. Make sure that the function generator feeds the lineclock
//1. Manually open the Vision shutter and Uniblitz shutter. The latter because the class destructor closes the shutter automatically
//2. Set g_pockelsAutoOff to 0 for holding on the last value
//3. Tune Vision's wavelength manually
	void pockels(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 350, 300, 1, g_multibeam };

		//DEFINE THE POCKELS
		//Pockels pockelsVision{ realtimeSeq, 750, Laser::ID::VISION };
		//Pockels pockels{ pockelsVision };

		Pockels pockelsFidelity{ realtimeSeq, 1040, Laser::ID::FIDELITY };
		Pockels pockels{ pockelsFidelity };

		pockels.pushVoltageSinglet(3.0 * V);
		//pockels.pushPowerSinglet(0. * mW);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		realtimeSeq.run();
		//pressAnyKeyToCont();
	}

	void semiAutoPockelsCalibration(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		//DEFINE THE POCKELS
		Pockels pockelsVision{ realtimeSeq, 1040, Laser::ID::VISION };
		Pockels pockels{ pockelsVision };

		//Pockels pockelsFidelity{ realtimeSeq, 1040, Laser::ID::FIDELITY };
		//Pockels pockels{ pockelsFidelity };

		for (double voltage_V = 0; voltage_V < 3.5; voltage_V +=0.2)
		{
			std::cout << "Current voltage = " << voltage_V << " V\n";
			pockels.pushVoltageSinglet(voltage_V * V);
			//realtimeSeq.run();
			//Sleep(30000);
		}
		//pockels.pushVoltageSinglet(8 * us, 0.1 * V, OVERRIDE::DIS);
		//realtimeSeq.run();
	
		Util::pressAnyKeyToCont();
	}

	void pockelsRamp(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int heightPerFrame_pix{ 35 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 100 };		//Number of frames for continuous acquisition

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, heightPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };

		//POCKELS
		const int wavelength_nm{ 1040 };
		Pockels pockels{ realtimeSeq, wavelength_nm, Laser::ID::FIDELITY};

		//Exponential ramp
		pockels.exponentialPowerRampAcrossFrames(1.0*960. * mW, 1. * um, -300. * um);

		//Linear ramp
		//const double Pi{ 500. * mW };
		//const double Pf{ 1000. * mW };
		//const SCANDIR scanDirZ{ SCANDIR::UPWARD };
		//const double laserPi = determineInitialLaserPower(Pi, Pf - Pi, scanDirZ);
		//const double laserPf = determineFinalLaserPower(Pi, Pf - Pi, scanDirZ);
		//std::cout << "Pi = " << laserPi << "\n";
		//std::cout << "Pf = " << laserPf << "\n";
		//pockels.linearPowerRampAcrossFrames(laserPi, laserPf);
		//pockels.linearPowerRampAcrossFrames(0.96 * Pf, Pi);

		//pockels.pushPowerSinglet(400 * us, Pi, OVERRIDE::EN);
		//pockels.pushVoltageSinglet(8* us, 0.0 * V, OVERRIDE::EN);
		//pockels.linearVoltageRampAcrossFrames(4. * V, 2. * V);	

		//EXECUTE THE CONTROL SEQUENCE
		realtimeSeq.run();



		Sleep(400);
		Pockels pockelsV{ realtimeSeq, 750, Laser::ID::VISION };
		pockelsV.exponentialPowerRampAcrossFrames(1.0*960. * mW, 1. * um, 300. * um);
		realtimeSeq.run();

		Util::pressAnyKeyToCont();
	}

	void lasers(const FPGA &fpga)
	{
		Laser laser{ Laser::ID::VISION };
		//Laser laser{ ID::FIDELITY };
		//std::cout << laser.isShutterOpen() << std::endl;
		//laser.setShutter(false);
		laser.setWavelength(1040);
		//laser.printWavelength_nm();
	}

	//Open the Uniblitz shutter manually
	void virtualLasers(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG , FIFOOUTfpga::DIS, 560, 300, 1, g_multibeam };

		const int wavelength_nm{ 1040 };
		const double laserPower{ 50. * mW };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::VISION };
		mesoscope.configure( wavelength_nm);
		mesoscope.setPower(laserPower);
	}

	//Photobleach a line along the fast axis (RS) on the sample
	void photobleach(const FPGA &fpga)
	{
		Laser laser{ Laser::ID::VISION };
		laser.setWavelength(920);

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::DIS, 400, 300, 100, g_multibeam };

		//RS
		ResonantScanner RScanner{ realtimeSeq };
		RScanner.isRunning();		//To make sure that the RS is running

		//SCANNERS. Keep the scanner fixed to bleach a line on the sample
		Galvo scanner{ realtimeSeq, 0 };

		//POCKELS
		Pockels pockels{ realtimeSeq, 920, Laser::ID::VISION };
		pockels.pushPowerSinglet(200 * mW);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		pockels.setShutter(true);
		realtimeSeq.run();

		//Wait until the sequence is over to close the shutter, otherwise this code will finish before the sequence
		Util::pressAnyKeyToCont();
		pockels.setShutter(false);
	}

	void convertI16toVolt()
	{
		std::cout << "volt to I16: " << FPGAfunc::convertVoltageToI16(-10) << "\n";
		std::cout << "int to volt: " << FPGAfunc::convertIntToVoltage(-32768) << "\n";
		std::cout << "volt to I16 to volt: " << FPGAfunc::convertIntToVoltage(FPGAfunc::convertVoltageToI16(0)) << "\n";
	}

	//To measure the saving speed of a Tiff file, either locally or remotely
	//Select a local or remote folder accordingly
	void ethernetSpeed()
	{
		std::string filename{ "testEthernetSpeed" };

		//The goal is to stream a stackDiffZ composed of 200 z-planes (100 um in 0.5 um-steps), where each frame has 300x560 pixels. Therefore, the stackDiffZ has 300x560x200 = 33.6 Mega pixels
		//The stackDiffZ size is 8 bits x 33.6M = 33.6 MB
		const int height{ 560 };
		const int width{ 300 };
		const int nFramesCont{ 200 };

		TiffU8 image{ height, width, nFramesCont };

		//Declare and start a stopwatch
		double duration;
		auto t_start{ std::chrono::high_resolution_clock::now() };

		//overriding the file saving has some travelOverhead
		//Splitting the stackDiffZ into a page structure (by assigning nFramesCont = 200 in saveToFile) gives a large travelOverhead
		image.saveToFile(g_imagingFolderPath, filename, TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";
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

		unsigned int n{ std::thread::hardware_concurrency() };
		std::cout << n << " concurrent threads are supported.\n";

		std::cout << "func1 and func2 will execute concurrently\n";

		FUNC x{ 1 };

		std::thread first{ &FUNC::func1, &x, 123 };
		std::thread second{ &FUNC::func2, &x, 314 };

		first.join();	//pauses until first finishes
		second.join();	//pauses until second finishes
	}

	void clipU8()
	{
		int input{ 260 };
		U8 output{ Util::clipU8top(input) };
		std::cout << (int)output << "\n";
		Util::pressAnyKeyToCont();
	}

	void dataLogger()
	{
		std::ofstream fileHandle;
		fileHandle.open(g_imagingFolderPath + "test.txt", std::ios_base::app);

		// Here system_clock is wall clock time from the system-wide realtime clock 
		std::time_t timenow{ std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) };
		std::tm buf;
		localtime_s(&buf, &timenow);
		auto timeStamp{ std::put_time(&buf, "%Y-%m-%d %X") };
		std::cout << timeStamp << std::endl;
		fileHandle << timeStamp << "\n";

		fileHandle.close();
		Util::pressAnyKeyToCont();
	}

	void createFolder()
	{
		for (int iterCutNumber = 0; iterCutNumber <= 52; iterCutNumber++)
		{
			const std::string newFolderFullPath{ "D:\\20200113_Liver20190812_03_lobe_corrected\\" + Util::zeroPadding(iterCutNumber, 3) };

			//Create a folder labeled by the cut number
			if (std::filesystem::exists(newFolderFullPath))
				std::cout << "WARNING: the file " << newFolderFullPath << " already exists. Folder creation skipped\n";
			else
				std::filesystem::create_directory(newFolderFullPath);
		}
		Util::pressAnyKeyToCont();
	}

	//Copy a subset of entire data to do some tests
	void filterTileConfigurationFile(const int firstCutNumber, const int lastCutNumber)
	{
		//Define the wavelengths to be processed
		const std::vector<int> vec_wavelengthIndex{ 0, 2 };


		const std::vector<int> tileIndexIIminMax{ 7, 36 };// { IImin, IImax }
		const std::vector<int> tileIndexJJminMax{ 11, 47 };// { JJmin, JJmax }

		const std::string sourcePath{ "D:\\20191129_Liver20190812_03_lobe_corrected\\" };
		const std::string destinationPath{ sourcePath };
		//const std::string destinationPath{ "D:\\20200113_Liver20190812_03_lobe_corrected\\" };

		if (firstCutNumber > lastCutNumber)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": The first cut number must be <= last cut number");

		//For Fiji's GridStitcher, generate a configuration text file for each vibratome cut and each laser wavelength
		//The file format is "_TileConfigurationCorrected_cutNumber_wavelengthIndex"
		std::vector<std::ofstream> vec_filteredConfigTxt((lastCutNumber - firstCutNumber + 1) * vec_wavelengthIndex.size());

		//Iterate over the vibratome cuts
		for (int iterCutNumber = firstCutNumber; iterCutNumber <= lastCutNumber; iterCutNumber++)
		{
			//Subforlder with the cut number
			const std::string subFolderName{ Util::zeroPadding(iterCutNumber, 3) + "\\" };

			//Iterate over the laser wavelengths
			for (std::vector<int>::size_type iterVec = 0; iterVec != vec_wavelengthIndex.size(); iterVec++)
			{
				//Generate the destination configuration textfile for GridStitcher
				const std::string filteredConfigTextFilename{ "_TileConfigurationFiltered_" + Util::zeroPadding(iterCutNumber, 3) + "_" + Util::toString(vec_wavelengthIndex.at(iterVec), 0) + ".txt" };
				//if (std::filesystem::exists(destinationPath + subFolderName + filteredConfigTextFilename))
				//{
				//	std::cout << "WARNING: the file " << filteredConfigTextFilename << " already exists. Files copying skipped\n";
				//}				
				//else
				{
					const unsigned int index{ (iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec };
					vec_filteredConfigTxt.at(index).open(destinationPath + subFolderName + filteredConfigTextFilename);
					vec_filteredConfigTxt.at(index) << "dim=3\n";	//Needed at the start of the txt for GridStitcher

					//Open the source configuration textfile
					std::ifstream sourceConfigTxt{ sourcePath + subFolderName + "_TileConfigurationCorrected_" + Util::zeroPadding(iterCutNumber, 3) + "_" + Util::toString(vec_wavelengthIndex.at(iterVec), 0) + ".txt" };
					if (!sourceConfigTxt)
						throw std::runtime_error((std::string)__FUNCTION__ + ": The file _TileConfiguration.txt failed to open");

					std::string line;
					getline(sourceConfigTxt, line);			//Skip the first line that contains "dim=3"
					while (getline(sourceConfigTxt, line))
						if (line.front() != '#')			//Skip all the lines that are commented out with #
						{
							//Get the Tiff parameters at the beginning of each text line
							//Tokenize with respect to '_'. //Convert the parameters to int
							//The format is "cutNumber_wavelengthIndex_tileIndexII_tileIndexJJ", e.g. "000_0_17_31"
							std::stringstream tiffParameters_ss(line.substr(0, line.find(".tif")));
							std::string isolatedParameter;
							getline(tiffParameters_ss, isolatedParameter, '_');			//Get rid of the prefix "corrected_"
							getline(tiffParameters_ss, isolatedParameter, '_');			//cutNumber	
							const int cutNumber{ std::stoi(isolatedParameter) };
							getline(tiffParameters_ss, isolatedParameter, '_');			//wavelengthIndex
							const int wavelengthIndex{ std::stoi(isolatedParameter) };
							getline(tiffParameters_ss, isolatedParameter, '_');			//tileIndexII
							const int tileIndexII{ std::stoi(isolatedParameter) };
							getline(tiffParameters_ss, isolatedParameter, '_');			//tileIndexJJ
							const int tileIndexJJ{ std::stoi(isolatedParameter) };

							//Constrain the stacks. //Copy the entry to the tileConfiguration textfile
							if (tileIndexII >= tileIndexIIminMax.at(0) && tileIndexII <= tileIndexIIminMax.at(1) && tileIndexJJ >= tileIndexJJminMax.at(0) && tileIndexJJ <= tileIndexJJminMax.at(1))
							{
								
								vec_filteredConfigTxt.at((iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec) << line << "\n";

								/*
								//Get the filename from the tileConfiguration textfile
								std::stringstream filteredTiffFilename{ line.substr(0, line.find(";")) };

								if (std::filesystem::exists(destinationPath + subFolderName + filteredTiffFilename.str()))
									std::cout << "WARNING: the file " << filteredTiffFilename.str() << " already exists. File copying skipped\n";
								else
								{
									std::filesystem::copy(sourcePath + subFolderName + filteredTiffFilename.str(), destinationPath + subFolderName + filteredTiffFilename.str());
									std::cout << "the file " << filteredTiffFilename.str() << " was copied\n";
								}
								*/
							}
						}//if(line.front() != '#')

					sourceConfigTxt.close();

					for (int tt = 0; tt < 2; tt++)
						for (int ss = 0; ss < 2; ss++)
						{
							const std::string testTiffFilename{ "corrected_" + Util::zeroPadding(iterCutNumber, 3) + "_" + Util::toString(vec_wavelengthIndex.at(iterVec), 0) + "_" +
								Util::zeroPadding(tileIndexIIminMax.at(tt),2) + "_" +
								Util::zeroPadding(tileIndexJJminMax.at(ss),2) };

							if (std::filesystem::exists(destinationPath + subFolderName + testTiffFilename + ".tif"))
								std::cout << "WARNING: the file " << testTiffFilename << " already exists. Dummy stack creation skipped\n";
							else
							{
								TiffU8 image{ 560, 300, 100 };
								image.saveToFile(destinationPath + subFolderName, testTiffFilename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);
								std::cout << tileIndexIIminMax.at(tt) << "_" << tileIndexJJminMax.at(ss) << std::endl;

								vec_filteredConfigTxt.at((iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec) << testTiffFilename << ".tif;;\t(" +
									Util::toString(270 * (tileIndexJJminMax.at(ss) - 29) - 49800, 0) + "," +
									Util::toString(476 * (tileIndexIIminMax.at(tt) - 32) - 86888, 0) + "," +
									Util::toString(19200 + 50 * iterCutNumber, 0) + ")\n";
							}
						}

					vec_filteredConfigTxt.at((iterCutNumber - firstCutNumber) * vec_wavelengthIndex.size() + iterVec).close();

				}//if(config file already exists)
			}//for(iterVec)
		}//for(iterCutNumber)
	
		Util::pressAnyKeyToCont();
	}

	void correctImage()
	{
		std::string inputFilename{ "000_2_00_01" };
		std::string outputFilename{ "output_" + inputFilename };
		TiffU8 image{ g_imagingFolderPath, inputFilename };
		image.correctRSdistortionGPU(150. * um);
		image.flattenFieldGaussian(0.015);
		image.suppressCrosstalk(0.20);
		image.saveToFile(g_imagingFolderPath, outputFilename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);

		//image.binFrames(5);
		//image.splitFrames(10);
		//image.mirrorOddFrames();
		//image.averageEvenOddFrames();
		
		//Declare and start a stopwatch
		double duration;
		auto t_start{ std::chrono::high_resolution_clock::now() };
		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";
		
		//pressAnyKeyToCont();
	}

	void correctImageBatch()
	{
		/*
		for (int ii = 0; ii < 4; ii++)
		{
			std::string inputTiff{ "000_0_000000" + Util::toString(ii,0) };
			std::string outputFilename{ "output_" + inputTiff };
			TiffU8 image{ inputTiff };
			image.correctRSdistortionGPU(150. * um);
			image.flattenFieldGaussian(0.009);
			image.suppressCrosstalk(0.20);
			image.saveToFile(outputFilename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);
		}*/
	}

	void quickStitcher()
	{
		//RTseq realtimeSeq{ fpga };
		//Image image{ realtimeSeq };
		//image.save("empty", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);
		//image.acquireVerticalStrip(SCANDIRX::LEFT);

		//TiffU8 asd{ image.data(), 300, 560 };
		//std::cout << image.tiff().heightPerFrame_pix() << "\n";
		//std::cout << image.tiff().widthPerFrame_pix() << "\n";


		/*
		std::string outputFilename{ "stitched" };
		const int height{ 8000 };
		const int width{ 300 };
		TiffU8 image00{ "01" };
		TiffU8 image01{ "02" };
		TiffU8 image03{ "03" };
		TiffU8 image04{ "04" };

		QuickStitcher stitched{ width,height, 1, 4 };
		stitched.push(image00, 0, 0);
		stitched.push(image01, 0, 1);
		stitched.push(image03, 0, 2);
		stitched.push(image04, 0, 3);
		stitched.saveToFile(outputFilename, OVERRIDE::DIS);
		*/
		std::string outputFilename{ "stitched" };
		const int height{ 8000 };
		const int width{ 300 };
		TiffU8 image00{ g_imagingFolderPath, "Tile_01" };
		TiffU8 image01{ g_imagingFolderPath, "Tile_02" };
		QuickStitcher stitched{ height, width, {2, 1}, {0, 0, 0} };
		stitched.push(image00.data(), { 0, 0 });
		stitched.push(image01.data(), { 1, 0 });
		stitched.saveToFile(g_imagingFolderPath, outputFilename, OVERRIDE::DIS);

		Util::pressAnyKeyToCont();
	}

	void boolmap()
	{
		//g_imagingFolderPath = "D:\\OwnCloud\\Data\\_Image processing\\For boolmap test\\"; //Override the global folder path
		std::string inputFilename{ "Panoramic_000" };
		std::string outputFilename{ "output" };
		TiffU8 image{ g_imagingFolderPath, inputFilename };

		//The tile array for the slow scan (overlay tile array) does not necessarily coincide with the tile array used in fast scanning
		const PIXDIM2 overlayTileSize_pix{ 280, 300 };//Note that 560/2=280 is used here because contX uses PANpixelSizeX=1.0 um for speed and not 0.5 um
		const TILEDIM2 overlayTileArraySizeIJ{ 40, 70 };
		const TILEOVERLAP3 overlayOverlapIJK_frac{ 0.15, 0.10, 0.5 };
		const double threshold{ 0.02 };
		
		Boolmap boolmap{ image, overlayTileArraySizeIJ, overlayTileSize_pix, overlayOverlapIJK_frac, threshold };
		boolmap.saveBoolmapToText(g_imagingFolderPath, "Boolmap", OVERRIDE::EN);
		boolmap.saveTiffWithBoolmapTileOverlay(g_imagingFolderPath, "TileMap", OVERRIDE::EN);
		boolmap.saveTiffWithBoolmapGridOverlay(g_imagingFolderPath, "GridOverlay", OVERRIDE::EN);
		boolmap.fillBoolmapHoles();
		boolmap.saveBoolmapToText(g_imagingFolderPath, "Boolmap_filled", OVERRIDE::EN);
		//boolmap.saveTiffWithBoolmapTileOverlay("TileMap_filled", OVERRIDE::EN);
		
		std::cout << "# bright stacks = " << boolmap.readNumberOfBrightStacks() << "\n";
		Util::pressAnyKeyToCont();
	}

	void vectorOfObjects()
	{
		class A
		{
		public:
			A(const int dummy)
			{
				mArray = new int[10];
			}
		private:
			int* mArray;
		};

		std::vector<A> aaa{ A{2} };
		A a{ 1 };
		aaa.push_back(a);

		Util::pressAnyKeyToCont();
	}
	
	void sequencerConcurrentTest()
	{
		class FUNC
		{
		public:
			void func1(const int x)
			{
				Sleep(10000);
				std::cout << "Thread saveFile finished\n";
			}
			void func2(const int x)
			{
				Sleep(5000);
				std::cout << "Thread moveStage finished\n";
			}
		};

		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560};
		const int widthPerFrame_pix{ 300 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };
		const int nFramesCont{ 100 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double pixelSizeZ{ 0.5 * um };									//Step size in the Z-stage axis
		const TILEOVERLAP3 stackOverlap_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };							//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };								//Sample thickness
		const double sampleSurfaceZ{ 18.471 * mm };

		Sample sample{ g_currentSample,  {g_stackCenterXYZ.XX, g_stackCenterXYZ.YY}, {10. * mm, 10. * mm, sampleLengthZ}, sampleSurfaceZ, cutAboveBottomOfStack };
		Stack stack{ FFOV, heightPerFrame_pix, widthPerFrame_pix, pixelSizeZ, nFramesCont, stackOverlap_frac };

		//Create a sequence
		Sequencer sequence{ sample, stack };
		sequence.generateCommandList();
		sequence.printToFile(g_imagingFolderPath, "_CommandlistLight", OVERRIDE::EN);

		if (1)
		{
			FUNC x;
			std::thread saveFile, moveStage;

			//Read the commands line by line
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.readNtotalCommands(); iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Sequencer::Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
				//commandline.printParameters();

				switch (commandline.mActionID)
				{
				case Action::ID::MOV:
					std::cout << "MOV " << "\n";

					//Skip the first MOV to position the stage
					if (iterCommandline != 0)
						moveStage = std::thread{ &FUNC::func2, &x, 314 };

					std::cout << "saveFile joinable? " << std::boolalpha << saveFile.joinable() << "\n";
					std::cout << "moveStage joinable? " << std::boolalpha << moveStage.joinable() << "\n";

					break;
				case Action::ID::ACQ:
					std::cout << "ACQ " << "\n";

					if (saveFile.joinable() && moveStage.joinable())
					{
						saveFile.join();
						moveStage.join();
					}
					break;
				case Action::ID::SAV:
					std::cout << "SAV" << "\n";
					saveFile = std::thread{ &FUNC::func1, &x, 123 };
					break;
				case Action::ID::CUT:
					std::cout << "CUT" << "\n";
					saveFile.join();

					break;
				default:
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
				}//switch
				//pressAnyKeyToCont();
			}//for

			std::cout << "saveFile joinable? " << std::boolalpha << saveFile.joinable() << "\n";
			std::cout << "moveStage joinable? " << std::boolalpha << moveStage.joinable() << "\n";

			if (moveStage.joinable())
				moveStage.join();

			if (saveFile.joinable())
				saveFile.join();

		}//if
	}
	/*
	void locationSequence()
	{
		//ACQUISITION SETTINGS
		const FFOV2 FFOV{ 200. * um, 150. * um };
		const int nDiffZ{ 100 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double pixelSizeZ{ 0.5 * um };								//Step size in the Z-stage axis
		const TILEOVERLAP3 stackOverlap_frac{ 0.05, 0.05, 0.05 };			//Stack overlap
		const Stack stack{ FFOV, pixelSizeZ, nDiffZ, stackOverlap_frac };

		//Create a sequence
		Sequencer sequence{ g_currentSample, stack, {g_stackCenterXYZ.XX, g_stackCenterXYZ.YY}, { 2, 2 } }; //Last 2 parameters: stack center and number of stacks
		std::vector<POSITION2> locationList{ sequence.generateLocationList() };

		for (std::vector<int>::size_type iterLocation = 0; iterLocation < locationList.size(); iterLocation++)
		{
			std::cout << "x = " << locationList.at(iterLocation).XX / mm << "\ty = " << locationList.at(iterLocation).YY / mm << "\n";
		}
		pressAnyKeyToCont();
	}
	*/

	//Generate a text file with the tile location for the BigStitcher
	void generateLocationsForBigStitcher()
	{
		// X is vertical and Y is horizontal, to match the directions of the XYZ-stage
		const TILEDIM2 tileSize{ 2, 2 };
		const int tileShiftX_pix{ 543 };
		const int tileShiftY_pix{ 291 };

		Logger datalog{ g_imagingFolderPath, g_currentSample.readName() + "_locations", OVERRIDE::EN };
		datalog.record("dim=3"); //Needed for the BigStitcher

		for (int tileNumber = 0; tileNumber < tileSize.II * tileSize.JJ; tileNumber++)
			//for (int tileNumber = 0; tileNumber < 180; tileNumber++)
		{
			TILEIJ tileIndicesIJ = tileNumberToIndicesIJ(tileNumber);
			int totalTileShiftX_pix{ -tileIndicesIJ.II * tileShiftX_pix };
			int TotalTileShiftY_pix{ -tileIndicesIJ.JJ * tileShiftY_pix };
			std::string line{ std::to_string(tileNumber) + ";;(" + std::to_string(TotalTileShiftY_pix) + "," + std::to_string(totalTileShiftX_pix) + ",0)" };	//In BigStitcher, X is horizontal and Y is vertical
			//std::string line{ std::to_string(tileNumber) + "\t" + std::to_string(tileNumberToIndicesIJ(tileNumber).at(X)) + "\t" + std::to_string(tileNumberToIndicesIJ(tileNumber).at(Y)) }; //For debugging
			datalog.record(line);
		}
	}

	//Snake pattern starting from the bottom right of the sample and going up
	TILEIJ tileNumberToIndicesIJ(const int tileNumber)
	{
		const TILEDIM2 tileSize{ 2, 2 };

		int II;
		int JJ{ tileNumber / tileSize.II };

		if (JJ % 2)	//JJ is odd
			II = tileSize.II - tileNumber % tileSize.II - 1;
		else		//JJ is even
			II = tileNumber % tileSize.II;

		return { II, JJ };
	}

	void PMT16Xconfig()
	{
		PMT16X PMT;

		//pmt.setSingleGain(PMT16XCHAN::CH00, 170);
		//PMT.setAllGains(255);
		//PMT.readTemp();
		PMT.readAllGains();
		
		//PMT.suppressGainsLinearly(0.4, RTseq::PMT16XCHAN::CH05, RTseq::PMT16XCHAN::CH10);
		
		/*
		//To make the count from all the channels similar,
		//rescan with frequency = 100 Hz and amplitude = 1.5V (which is larger than the size of the PMT16X
		//to use the linear part of the ramp). Set the refresh rate to 10 or 20 ms
		PMT.setAllGain({
			255,	//CH00
			255,	//CH01
			255,	//CH02
			255,	//CH03
			255,	//CH04
			255,	//CH05
			255,	//CH06
			255,	//CH07
			255,	//CH08
			255,	//CH09
			255,	//CH10
			255,	//CH11
			255,	//CH12
			255,	//CH13
			255,	//CH14
			255		//CH15
			});
			*/

		Util::pressAnyKeyToCont();
	}

	//Test reading different channels of the PMT16X
	//Must manually open the laser and Uniblitz shutter
	void PMT16Xdemultiplex(const FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 1 };										//Number of frames for continuous acquisition
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };		//Scan duration in the slow axis
		const int wavelength_nm{ 750 };
		const double laserPower{ 30. * mW };

		//CREATE THE CONTROL SEQUENCE
		RTseq realtimeSeq{ fpga, LINECLOCK::FG, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont, g_multibeam };
		Mesoscope mesoscope{ realtimeSeq, Laser::ID::VISION };
		mesoscope.configure( wavelength_nm);
		mesoscope.setPower(laserPower);

		//SCANNERS
		Galvo scanner{ realtimeSeq, FFOVslow / 2. };
		Galvo rescanner{ realtimeSeq, FFOVslow / 2., mesoscope.readCurrentLaser(), mesoscope.readCurrentWavelength_nm() };
		//Galvo scanner{ realtimeSeq, 0 };				//Keep the scanner fixed to see the emitted light swing across the PMT16X channels. The rescanner must be centered

		//EXECUTE THE CONTROL SEQUENCE
		realtimeSeq.run();
		Image image{ realtimeSeq };
		image.acquire();
		image.save(g_imagingFolderPath, "SingleChannel", TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);
	}

	void vibratome(const FPGA &fpga)
	{
		const double cutPlaneZ{ (20.700) * mm };

		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps , ContainerPosLimit };
		Vibratome vibratome{ fpga, stage };
		vibratome.cutTissue(cutPlaneZ);

		const POSITION3 samplePosXYZ{ 10. * mm, 23. * mm, cutPlaneZ };
		stage.moveXYZ(samplePosXYZ);
	}

	void filterwheel()
	{
		Filterwheel FWexcitation{ Filterwheel::ID::EXC };
		Filterwheel FWdetection{ Filterwheel::ID::DET };

		int wavelength_nm;
		if (1)
			wavelength_nm = 920;
		else
			wavelength_nm = 750;

		//FWexcitation.setWavelength(wavelengthIndex_s);
		//FWdetection.setWavelength(wavelengthIndex_s);

		if (1)//Multibeam. Turn both filterwheels concurrently
		{
			std::future<void> th1{ std::async(&Filterwheel::setWavelength, &FWexcitation, wavelength_nm) };
			std::future<void> th2{ std::async(&Filterwheel::setWavelength, &FWdetection, wavelength_nm) };
			th1.get();
			th2.get();
		}
		else//Singlebeam. Turn both filterwheels concurrently
		{
			std::future<void> th1{ std::async(&Filterwheel::setWavelength, &FWexcitation, wavelength_nm) };	//Leave the excitation filterwheel open
			std::future<void> th2{ std::async(&Filterwheel::setWavelength, &FWdetection, wavelength_nm) };
			th1.get();
			th2.get();
		}
	}

	void collectorLens()
	{
		CollectorLens collectorLens;
		//collectorLens.move(10.0 * mm);
		collectorLens.downloadConfig();
		//collectorLens.moveHome();
		Util::pressAnyKeyToCont();
	}

	void openCV()
	{
		//cv::Mat image;
		//image = cv::imread("", cv::IMREAD_COLOR); // Read the file

		//if (!image.data) // Check for invalid input
		//{
		//	std::cout << "Could not open or find the image" << std::endl;
		//}

		//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
		//cv::imshow("Display window", image); // Show our image inside it.
	}
}//namespace