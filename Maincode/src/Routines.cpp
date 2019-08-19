#include "Routines.h"

//SAMPLE PARAMETERS
POSITION3 stackCenterXYZ{ (51.000) * mm, (22.300 )* mm, (17.740) * mm };//Liver TDT
//double3 stackCenterXYZ{ (32.000) * mm, 19.100 * mm, (17.520 + 0.020) * mm };//Liver WT
const std::vector<LIMIT2> PetridishPosLimit{ { 27. * mm, 57. * mm}, { 0. * mm, 30. * mm}, { 15. * mm, 24. * mm} };		//Soft limit of the stage for the petridish

#if multibeam
//Sample beads4um{ "Beads4um16X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, multiply16X(50. * mW), multiply16X(0.0 * mWpum) }, { "GFP", 920, multiply16X(45. * mW), multiply16X(0. * mWpum) }, { "TDT", 1040, multiply16X(15. * mW), multiply16X(0. * mWpum) } }} };
Sample liver{ "Liver", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{ {"TDT", 1040, multiply16X(60. * mW), multiply16X(0.0 * mWpum) } , { "GFP", 920, multiply16X(40. * mW), multiply16X(0.0 * mWpum) } , { "DAPI", 750, multiply16X(12. * mW), multiply16X(0.09 * mWpum) } }} };
#else
Sample beads4um{ "Beads4um", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 35. * mW, 0. * mWpum }, { "GFP", 920, 30. * mW, 0. * mWpum }, { "TDT", 1040, 5. * mW, 0. * mWpum }}} };
Sample beads05um{ "Beads1um", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 40. * mW, 0. * mWpum }, { "GFP", 920, 40. * mW, 0. * mWpum }, { "TDT", 1040, 15. * mW, 0. * mWpum }}} };
Sample fluorSlide{ "fluorBlue", "SiliconeOil", "1.51", PetridishPosLimit, {{{ "DAPI", 750, 10. * mW, 0. * mWpum }}} };
Sample liver{ "Liver", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{{"TDT", 1040, 10. * mW, 0.0 * mWpum } , { "GFP", 920, 25. * mW, 0.0 * mWpum }, { "DAPI", 750, 7. * mW, 0.09 * mWpum }}} };
#endif
Sample currentSample{ liver };


namespace Routines
{
	//The "Swiss knife" of my routines
	void stepwiseScan(const FPGA &fpga)
	{
		//const RUNMODE acqMode{ RUNMODE::SINGLE };			//Single frame. The same location is imaged continuously if nFramesCont>1 (the galvo is scanned back and forth at the same location) and the average is returned
		//const RUNMODE acqMode{ RUNMODE::AVG };			//Single frame. The same location is imaged stepwise and the average is returned
		const RUNMODE acqMode{ RUNMODE::SCANZ };			//Scan in the axis STAGEZ stepwise with stackCenterXYZ.at(STAGEZ) the starting position
		//const RUNMODE acqMode{ RUNMODE::SCANZCENTERED };	//Scan in the axis STAGEZ stepwise with stackCenterXYZ.at(STAGEZ) the center of the stack
		//const RUNMODE acqMode{ RUNMODE::SCANXY };			//Scan in the axis STAGEX stepwise
		//const RUNMODE acqMode{ RUNMODE::COLLECTLENS };	//For optimizing the collector lens
		
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("DAPI") };	//Select a particular fluorescence channel
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const int nFramesCont{ 1 };	
		const double stackDepthZ{ 40. * um };	//Acquire a stack this deep in the axis STAGEZ
		const double stepSizeZ{ 1.0 * um };
	
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		//This is because the beads at 750 nm are chromatically shifted
		if (fluorLabel.mWavelength_nm == 750)
			stackCenterXYZ.ZZ -= 6 * um;
		//This is because FIDELITY is chromatically shifted wrt VISION
		if (fluorLabel.mWavelength_nm == 1040 && (whichLaser == Laser::ID::FIDELITY || whichLaser == Laser::ID::AUTO))
			stackCenterXYZ.ZZ -= 4 * um;

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (multibeam)
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<double>(FFOVslow / g_nChanPMT);
		}
		else//Singlebeam. When using a fluorescent slide, set selectScanFFOV = 0 and PMT16Xchan_int = PMT16XCHAN::CENTERED to let the laser scan over the PMT16X channels
		{
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOVslow;
		}

		std::vector<POSITION3> stagePositionXYZ;						//Vector with the sample locations to image
		int nSameLocation{ 1 };											//Number of frames at the same location
		double stepSizeX{ 0 };											//Step for lateral scanning
		bool saveAllPMT{ false };										//Save all PMT16X channels in separate pages in a Tiff
		double cLensPosIni{ 0 }, cLensPosFinal{ 0 }, cLensStep{ 0 };	//For debugging the collector lens
		switch (acqMode)
		{
		case RUNMODE::SINGLE:
			stagePositionXYZ.push_back(stackCenterXYZ);
			if (!multibeam) //For multibeam, no need for saving all the PMT channels
			{
				//saveAllPMT = true;
			}
			break;
		case RUNMODE::AVG:
			nSameLocation = 10;
			//Generate the discrete scan sequence for the stages
			for (int iterSameZ = 0; iterSameZ < nSameLocation; iterSameZ++)
				stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::SCANZ:
			//Generate the discrete scan sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < static_cast<int>(stackDepthZ / stepSizeZ); iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ + iterDiffZ * stepSizeZ });
			break;
		case RUNMODE::SCANZCENTERED:
			//Generate the discrete scan sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < static_cast<int>(stackDepthZ / stepSizeZ); iterDiffZ++)
			{
			const double halfStackLengthZ{ 0.5 * stackDepthZ };
			stagePositionXYZ.push_back({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ + iterDiffZ * stepSizeZ - halfStackLengthZ });
			}
				
			break;
		case RUNMODE::SCANXY:
			//saveAllPMT = true;
			//stagePositionXYZ.push_back({ stackCenterXYZ.at(Stage::X), stackCenterXYZ.at(Stage::Y), stackCenterXYZ.at(Stage::Z) });
			//stagePositionXYZ.push_back({ stackCenterXYZ.at(Stage::X) + 0.250 * mm, stackCenterXYZ.at(Stage::Y), stackCenterXYZ.at(Stage::Z) - 0.003 * mm});
			
			stepSizeX = 5. * um;
			//Generate the discrete scan sequence for the stages
			for (int iterDiffX = 0; iterDiffX < 60; iterDiffX++)
				stagePositionXYZ.push_back({ stackCenterXYZ.XX + iterDiffX * stepSizeX, stackCenterXYZ.YY, stackCenterXYZ.ZZ});
			break;
		case RUNMODE::COLLECTLENS:
			if(multibeam)
				throw std::invalid_argument((std::string)__FUNCTION__ + ": Collector-lens scanning available only for single beam");
			saveAllPMT = true;
			cLensPosIni = 8.0 * mm;
			cLensPosFinal = 12.0 * mm;
			cLensStep = 0.5 * mm;;
			nSameLocation = static_cast<int>( std::floor((cLensPosFinal - cLensPosIni)/ cLensStep) ) + 1;
			for (int iterSameZ = 0; iterSameZ < nSameLocation; iterSameZ++)
				stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerBeamletPerFrame_pix, FIFOOUTfpga::EN };

		//LASER
		VirtualLaser virtualLaser{ RTcontrol, fluorLabel.mWavelength_nm, whichLaser };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVOS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslowPerBeamlet / 2 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslowPerBeamlet / 2, &virtualLaser };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, 0, fluorLabel.mWavelength_nm };

		//DATALOG
		{
			Logger datalog(currentSample.mName);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", currentSample.mName);
			datalog.record("Immersion medium = ", currentSample.mImmersionMedium);
			datalog.record("Correction collar = ", currentSample.mObjectiveCollar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", g_tickPerUs);
			datalog.record("\nLASER--------------------------------------------------------");
			datalog.record("Laser used = ", virtualLaser.currentLaser_s());
			datalog.record("Laser wavelength (nm) = ", virtualLaser.currentWavelength_nm());
			datalog.record("Laser power first frame (mW) = ", fluorLabel.mScanPi / mW);
			datalog.record("Laser power increase (mW/um) = ", fluorLabel.mStackPinc / mWpum);
			datalog.record("Laser repetition period (us) = ", g_laserPulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * g_lineclockHalfPeriod / us);
			datalog.record("Pixel dwell time (us) = ", g_pixelDwellTime / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("Upscaling factor = ", RTcontrol.mUpscalingFactor);
			datalog.record("Width X (RS) (pix) = ", widthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", heightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", pixelSizeXY / um);
			datalog.record("\nSTAGE--------------------------------------------------------");
		}
		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
		stage.moveXYZ(stagePositionXYZ.front());	//Move the stage to the initial position
		Sleep(500);
		stage.waitForMotionToStopAll();

		//CREATE A STACK FOR STORING THE TIFFS
		const int nLocations{ static_cast<int>(stagePositionXYZ.size()) };
		TiffU8 output{ widthPerFrame_pix, heightPerFrame_pix, nLocations };
		std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm" };
		
		//OPEN THE UNIBLITZ SHUTTERS
		virtualLaser.openShutter();	//The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations  << "\n";
			stage.moveXYZ(stagePositionXYZ.at(iterLocation));
			stage.waitForMotionToStopAll();
			//stage.printPositionXYZ();		//Print the stage position	
			
			virtualLaser.setPower(fluorLabel.mScanPi + iterLocation * stepSizeZ * fluorLabel.mStackPinc);	//Update the laser power

			//Used to optimize the collector lens position
			if (acqMode == RUNMODE::COLLECTLENS)
				virtualLaser.moveCollectorLens(cLensPosIni + iterLocation * cLensStep);

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };
			image.acquire(saveAllPMT);				//Execute the RT control sequence and acquire the image
			image.averageFrames();					//Average the frames imaged via continuous acquisition
			//image.averageEvenOddFrames();
			image.correctImage(RScanner.mFFOV);

			if (acqMode == RUNMODE::SINGLE && !saveAllPMT)
			{
				//Save individual files
				filename.append("_P=" + toString(fluorLabel.mScanPi / mW, 1) + "mW" +
					"_x=" + toString(stagePositionXYZ.at(iterLocation).XX / mm, 3) + "_y=" + toString(stagePositionXYZ.at(iterLocation).YY / mm, 3) + "_z=" + toString(stagePositionXYZ.at(iterLocation).ZZ / mm, 4));
				std::cout << "Saving the stack...\n";
				image.save(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
			}
			output.pushImage(image.data(), iterLocation);
		}

		if (acqMode == RUNMODE::AVG || acqMode == RUNMODE::SCANZ || acqMode == RUNMODE::SCANZCENTERED)
		{	
			filename.append( "_Pi=" + toString(fluorLabel.mScanPi / mW, 1) + "mW_Pinc=" + toString(fluorLabel.mStackPinc / mWpum, 1) + "mWpum" +
				"_x=" + toString(stagePositionXYZ.front().XX / mm, 3) + "_y=" + toString(stagePositionXYZ.front().YY / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().ZZ / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().ZZ / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) );

			output.binFrames(nSameLocation);									//Divide the images in bins of nSameLocation frames each and return the average of each bin
			std::cout << "Saving the stack...\n";
			output.saveToFile(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);	//Save the scanZ to file
			pressESCforEarlyTermination();
		}

		if (acqMode == RUNMODE::SCANXY)
		{
			filename.append( "_P=" + toString(fluorLabel.mScanPi / mW, 1) + "mW" +
				"_xi=" + toString(stagePositionXYZ.front().XX / mm, 4) + "_xf=" + toString(stagePositionXYZ.back().XX / mm, 4) +
				"_y=" + toString(stagePositionXYZ.front().YY / mm, 3) +
				"_z=" + toString(stagePositionXYZ.front().ZZ / mm, 4) + "_Step=" + toString(stepSizeX / mm, 4) );

			output.binFrames(nSameLocation);							//Divide the images in bins of nSameLocation frames each and return the average of each bin
			std::cout << "Saving the stack...\n";
			output.saveToFile(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);	//Save the scanXY to file
			pressESCforEarlyTermination();
		}
	}

	//I triggered the stack acquisition using DO2 (stage motion) for both scanning directions: top-down and bottom-up. In both cases the bead z-position looks almost identical with a difference of maybe only 1 plane (0.5 um)
	//Remember that I do not use MACROS on the stages anymore*/
	void contScanZ(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("DAPI") };	//Select a particular laser
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const SCANDIR scanDirZ{ SCANDIR::UPWARD };			//Scan direction for imaging in z
		const int nFramesPerBin{ 1 };							//For averaging
		const double stackDepth{ 40. * um };
		const double pixelSizeZafterBinning{ 0.5 * um  };

		const int nFramesCont{ static_cast<int>(stackDepth / pixelSizeZafterBinning * nFramesPerBin) };		//Number of frames for continuous acquisition
		const double pixelSizeZbeforeBinning{ stackDepth / nFramesCont };									//pixel size per z frame
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis

		//This is because the beads at 750 nm are chromatically shifted wrt 920 nm and 1040 nm
		if (fluorLabel.mWavelength_nm == 750)
			stackCenterXYZ.ZZ -= 6 * um;
		//This is because FIDELITY is chromatically shifted wrt VISION
		if (fluorLabel.mWavelength_nm == 1040 && (whichLaser == Laser::ID::FIDELITY || whichLaser == Laser::ID::AUTO))
			stackCenterXYZ.ZZ -= 6 * um;

		//Center the stack
		////////////////////////////////////stackCenterXYZ.at(Stage::Z) -= nFramesCont * pixelSizeZbeforeBinning /2;

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (multibeam)
		{
			heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
			FFOVslowPerBeamlet = static_cast<double>(FFOVslow / g_nChanPMT);
		}
		else//Singlebeam
		{
			heightPerBeamletPerFrame_pix = heightPerFrame_pix;
			FFOVslowPerBeamlet = FFOVslow;
		}

		//CREATE THE REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEZ, nFramesCont, widthPerFrame_pix, heightPerBeamletPerFrame_pix, FIFOOUTfpga::EN };	//Note the STAGEZ flag

		//LASER
		const double laserPi = detInitialLaserPower(fluorLabel.mScanPi, stackDepth * fluorLabel.mStackPinc, scanDirZ);
		const double laserPf = detFinalLaserPower(fluorLabel.mScanPi, stackDepth * fluorLabel.mStackPinc, scanDirZ);
		const VirtualLaser virtualLaser{ RTcontrol, fluorLabel.mWavelength_nm, laserPi, laserPf, whichLaser };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVOS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslowPerBeamlet / 2 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslowPerBeamlet / 2, &virtualLaser };

		//STAGES
		const double stageZi = detInitialPos(stackCenterXYZ.ZZ, stackDepth, scanDirZ);
		const double stageZf = detFinalPos(stackCenterXYZ.ZZ, stackDepth, 0.010 * mm, scanDirZ);
		Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };							
		stage.moveXYZ({ stackCenterXYZ.XX, stackCenterXYZ.YY, stageZi });		//Move the stage to the initial position
		stage.waitForMotionToStopAll();
		Sleep(500);
		stage.setVelSingle(Stage::Axis::ZZ, pixelSizeZbeforeBinning / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));		//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix

		//EXECUTE THE RT CONTROL SEQUENCE
		virtualLaser.openShutter();	//Open the shutter. The destructor will close the shutter automatically
		Image image{ RTcontrol };
		image.initializeAcq(scanDirZ);
		std::cout << "Scanning the stack...\n";
		stage.moveSingle(Stage::ZZ, stageZf);	//Move the stage to trigger the ctl&acq sequence
		image.downloadData();

		virtualLaser.closeShutter();	//Close the shutter manually even though the destructor does it because the data processing could take a long time
		image.formImage();
		image.correctImage(RScanner.mFFOV);
		image.binFrames(nFramesPerBin);

		const std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm_P=" + toString((std::min)(laserPi, laserPf) / mW, 1) + "mW_Pinc=" + toString(fluorLabel.mStackPinc / mWpum, 1) +
			"mWpum_x=" + toString(stackCenterXYZ.XX / mm, 3) + "_y=" + toString(stackCenterXYZ.YY / mm, 3) +
			"_zi=" + toString(stageZi / mm, 4) + "_zf=" + toString(stageZf / mm, 4) + "_Step=" + toString(pixelSizeZafterBinning / mm, 4) };
		
		std::cout << "Saving the stack...\n";
		image.save(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
	}

	void contScanX(const FPGA &fpga)
	{
		if (multibeam)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Continuous x-stage scanning for single beam only");

		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("DAPI") };	//Select a particular laser
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		//SCANDIR iterScanDirX{ SCANDIR::LEFTWARD };
		SCANDIR iterScanDirX{ SCANDIR::RIGHTWARD };//Initial scan direction of stage x
		const int nCol{ 56 };//56
		const double FOVslow{ 4.0 * mm };
		const double pixelSizeX{ 0.5 * um };

		const int width_pix{ 300 };
		const int height_pix{ static_cast<int>(FOVslow / pixelSizeX) };

		//This is because the beads at 750 nm are chromatically shifted wrt 920 nm and 1040 nm
		if (fluorLabel.mWavelength_nm == 750)
			stackCenterXYZ.ZZ -= 6 * um;
		//This is because FIDELITY is chromatically shifted wrt VISION
		if (fluorLabel.mWavelength_nm == 1040 && (whichLaser == Laser::ID::FIDELITY || whichLaser == Laser::ID::AUTO))
			stackCenterXYZ.ZZ -= 6 * um;

		//LOCATIONS on the sample to image
		std::vector<double> stagePositionY;
		for (int iterLocation = 0; iterLocation < nCol; iterLocation++)
			stagePositionY.push_back( stackCenterXYZ.YY - iterLocation * 0.150 * mm );//for now, only pushing strip to the right works (moving the stage to the left)

		//CREATE THE REALTIME CONTROL SEQUENCE
		//The Image height is 2 (two galvo scanner swings) and nFrames is height_pix/2. Therefore, the total height of the final image is height_pix
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEX, height_pix/2, width_pix, 2, FIFOOUTfpga::EN };	//Note the STAGEX flag																								

		//LASER
		const VirtualLaser virtualLaser{ RTcontrol, fluorLabel.mWavelength_nm, fluorLabel.mScanPi, fluorLabel.mScanPi, whichLaser }; //Note that the initial and final laser powers are the same

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVOS. Keep them fixed at 0
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, 0 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, 0, &virtualLaser };

		//STAGES
		Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
		double stageXi = detInitialPos(stackCenterXYZ.XX - FOVslow / 2, FOVslow, iterScanDirX);		//Initial x position
		stage.moveXYZ({ stageXi, stackCenterXYZ.YY, stackCenterXYZ.ZZ });							//Move the stage to the initial position
		stage.waitForMotionToStopAll();
		Sleep(500);
		stage.setVelSingle(Stage::Axis::XX, pixelSizeX / g_lineclockHalfPeriod);					//Set the vel for imaging

		//EXECUTE THE RT CONTROL SEQUENCE
		virtualLaser.openShutter();	//Open the shutter. The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		const int nLocations{ static_cast<int>(stagePositionY.size()) };
		QuickStitcher stitchedImage{ width_pix, height_pix, 1, nCol };
		double stageXf;						//Stage final position
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			stageXi = detInitialPos(stackCenterXYZ.XX - FOVslow / 2, FOVslow, iterScanDirX);
			stageXf = detFinalPos(stackCenterXYZ.XX - FOVslow / 2, FOVslow, 0.100 * mm, iterScanDirX);

			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations << "\n";
			stage.moveXY({ stageXi, stagePositionY.at(iterLocation) });
			stage.waitForMotionToStopAll();
			Sleep(100);//IMPORTANT!!! Give more time to Image to upload the sequence

			Image image{ RTcontrol };
			image.initializeAcq();
			std::cout << "Scanning the stack...\n";
			stage.moveSingle(Stage::XX, stageXf);	//Move the stage to trigger the ctl&acq sequence
			image.downloadData();
			image.formImageVerticalStrip(iterScanDirX);

			//image.save("aaa", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::DIS);
			TiffU8 tmp{ image.data(), width_pix, height_pix };//I tried to access mTiff in image directly but it gives me an error
			stitchedImage.push(tmp, 0, iterLocation);//for now, only pushing strip to the right works

			reverseSCANDIR(iterScanDirX);
		}
			const std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm_P=" + toString(fluorLabel.mScanPi / mW, 1) +
				"mWpum_xi=" + toString(stageXi / mm, 3) + "_xf=" + toString(stageXf / mm, 3) +
				"_y=" + toString(stackCenterXYZ.YY / mm, 3) +
				"_z=" + toString(stackCenterXYZ.ZZ / mm, 4) + "_Step=" + toString(pixelSizeX / mm, 4) };
			std::cout << "Saving the stack...\n";
			stitchedImage.saveToFile(filename, OVERRIDE::DIS);
	}

	//Full sequence to image and cut an entire sample automatically
	//Note that the stack starts at stackCenterXYZ.at(Z) (i.e., the stack is not centered at stackCenterXYZ.at(Z))
	void sequencer(const FPGA &fpga, const bool run)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };			//Full FOV in the (slow axis, fast axis)
		const int nFramesCont{ 200 };																	//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double pixelSizeZ{ 0.5 * um };															//Step size in the axis STAGEZ
		const ROI4 roi{ 11.000 * mm, 34.825 * mm, 11.180 * mm, 35.025 * mm };							//Region of interest {ymin, xmin, ymax, xmax}
		const TILEOVERLAP4 stackOverlap_frac{ 0.05, 0.05, 0.05 };										//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };													//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };														//Sample thickness
		const double sampleSurfaceZ{ stackCenterXYZ.ZZ - nFramesCont * pixelSizeZ / 2 };				//For beads, center the stack around stackCenterXYZ.at(Z)

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet;
		if (multibeam)
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
		const Sample sample{ currentSample, roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		const Stack stack{ FFOV, pixelSizeZ, nFramesCont, stackOverlap_frac };
		//Sequence sequece(sample, stack);
		Sequence sequence{ sample, stack, {stackCenterXYZ.XX, stackCenterXYZ.YY}, { 10, 1 } }; //The last 2 parameters: stack center and number of stacks in axes {STAGEX, STAGEY}
		sequence.generateCommandList();
		sequence.printToFile("Commandlist");

		if (run)
		{
			//CREATE THE REALTIME CONTROL SEQUENCE
			RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEZ, nFramesCont, widthPerFrame_pix, heightPerBeamletPerFrame_pix, FIFOOUTfpga::EN };	//Note the STAGEZ flag

			//LASER
			VirtualLaser virtualLaser{ RTcontrol };

			//RS
			const ResonantScanner RScanner{ RTcontrol };
			RScanner.isRunning();		//Make sure that the RS is running

			//GALVOS
			const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslowPerBeamlet / 2 };
			Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslowPerBeamlet / 2 };

			//STAGES. Specify the velocity
			Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
			stage.moveSingle(Stage::ZZ, sample.mSurfaceZ);	//Move the z stage to the sample surface
			stage.waitForMotionToStopAll();
			stage.setVelSingle(Stage::Axis::ZZ, pixelSizeZ / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));	//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };

			//Read the commands line by line
			POSITION2 stackCenterXY;
			std::string longName;
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.size(); iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Sequence::Commandline commandline{ sequence.readCommandline(iterCommandline) };
				//commandline.printParameters();

				//Stopwatch
				//auto t_start{ std::chrono::high_resolution_clock::now() };

				switch (commandline.mAction)
				{
				case Action::ID::MOV://Move the x and y stages to mStackCenterXY
					stackCenterXY = commandline.mCommand.moveStage.mStackCenterXY;
					stage.moveXY(stackCenterXY);
					stage.waitForMotionToStopAll();
					break;
				case Action::ID::ACQ://Acquire a stack
					Action::AcqStack acqStack{ commandline.mCommand.acqStack };

					//Update the laser parameters if needed
					virtualLaser.configure(acqStack.mWavelength_nm);				//When switching pockels, the class destructor closes the uniblitz shutter
					virtualLaser.setPower(acqStack.mScanPi, acqStack.scanPf());
					virtualLaser.openShutter();										//Re-open the Uniblitz shutter if closed by the pockels destructor
					rescanner.reconfigure(&virtualLaser);							//The calibration of the rescanner depends on the laser and wavelength being used

					image.initializeAcq(static_cast<SCANDIR>(acqStack.mScanDirZ));
					std::cout << "Scanning the stack...\n";
					stage.moveSingle(Stage::ZZ, acqStack.scanZf());					//Move the stage to trigger the ctl&acq sequence
					image.downloadData();
					break;
				case Action::ID::SAV:
					longName = virtualLaser.currentLaser_s(true) + toString(acqStack.mWavelength_nm, 0) + "nm_Pi=" + toString(acqStack.mScanPi / mW, 1) + "mW_Pf=" + toString(acqStack.scanPf() / mW, 1) + "mW" +
						"_x=" + toString(stackCenterXY.XX / mm, 3) + "_y=" + toString(stackCenterXY.YY / mm, 3) +
						"_zi=" + toString(acqStack.mScanZi / mm, 4) + "_zf=" + toString(acqStack.scanZf() / mm, 4) + "_Step=" + toString(pixelSizeZ / mm, 4);

					image.formImage();
					//image.correctImage(RScanner.mFFOV);
					image.save(longName, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
					break;
				case Action::ID::CUT://Move the stage to the vibratome and then cut a slice off
					POSITION3 stagePositionXYZ{ commandline.mCommand.cutSlice.mBladePositionXY };
					//IMPLEMENT THE SLICING HERE!
					break;
				default:
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
				}//switch

				//Stop the stopwatch
				//double duration{ std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count() };
				//std::cout << "Elapsed time: " << duration << " ms" << "\n";

				//pressAnyKeyToCont();
			}//for
		}//if
	}

	//Image the sample non-stop. Use the PI program to move the stages manually
	void liveScan(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("TDT") };	//Select a particular fluorescence channel
		const int nFramesCont{ 1 };									//Number of frames for continuous acquisition
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUTfpga::EN };

		//LASER
		const VirtualLaser virtualLaser{ RTcontrol, fluorLabel.mWavelength_nm, Laser::ID::VISION };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVOS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslow / 2 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslow / 2, &virtualLaser };

		//OPEN THE UNIBLITZ SHUTTERS
		virtualLaser.openShutter();	//The destructor will close the shutter automatically

		while (true)
		{
			virtualLaser.setPower(fluorLabel.mScanPi);			//Set the laser power

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };
			image.acquire();									//Execute the RT control sequence and acquire the image
			image.averageFrames();								//Average the frames acquired via continuous acquisition
			image.correctImage(RScanner.mFFOV);
			image.save("Untitled", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);	//Save individual files
			Sleep(700);

			pressESCforEarlyTermination();						//Early termination if ESC is pressed
		}
	}

	/*
//Apply 'stepwiseScan' on a list of locations
void frameByFrameZscanTilingXY(const FPGA &fpga, const int nSlice)
{
	//ACQUISITION SETTINGS
	const FluorLabelList fluorLabelList{ {currentSample.findFluorLabel("DAPI")} };
	const INDICES2 nStacksIJ{ 2, 1 };
	const double pixelSizeXY{ 0.5 * um };
	const int widthPerFrame_pix{ 300 };
	const int heightPerFrame_pix{ 560 };
	const int nFramesCont{ 1 };											//Number of frames for continuous acquisition
	const double FFOVfast{ widthPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
	const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

	int heightPerBeamletPerFrame_pix;
	double FFOVslowPerBeamlet;
	if (multibeam)
	{
		heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / nChanPMT);
		FFOVslowPerBeamlet = static_cast<int>(FFOVslow / nChanPMT);
	}
	else //Singlebeam
	{
		heightPerBeamletPerFrame_pix = heightPerFrame_pix;
		FFOVslowPerBeamlet = FFOVslow;
	}

	//STACK
	const FFOV2 FFOV{ FFOVslow, FFOVfast };								//Full FOV in the (slow axis, fast axis)
	const double stepSizeZ{ 1.0 * um };									//Step size in the axis STAGEZ
	const double stackDepthZ{ 40. * um };								//Acquire a stack this deep in the axis STAGEZ
	const int nDiffZ{ static_cast<int>(stackDepthZ / stepSizeZ) };		//Number of frames at different Zs
	const double3 stackOverlap_frac{ 0.03, 0.03, 0.03 };				//Stack overlap
	const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

	//CREATE A REALTIME CONTROL SEQUENCE
	RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerBeamletPerFrame_pix, FIFOOUTfpga::EN };

	//RS
	const ResonantScanner RScanner{ RTcontrol };
	RScanner.isRunning();					//Make sure that the RS is running

	//LASER
	VirtualLaser virtualLaser{ RTcontrol, ID::VISION };

	//Create a location list
	Sequence sequence{ currentSample, stack, {stackCenterXYZ.at(X), stackCenterXYZ.at(Y)}, nStacksIJ };
	std::vector<POSITION2> locationListXY{ sequence.generateLocationList() };

	//STAGES
	Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

	//Iterate over the wavelengths
	for (std::vector<int>::size_type iterWL = 0; iterWL < fluorLabelList.size(); iterWL++)
	{
		//DATALOG
		Logger datalog("Slice" + std::to_string(nSlice) + "_" + fluorLabelList.at(iterWL).mName);
		datalog.record("SAMPLE-------------------------------------------------------");
		datalog.record("Sample = ", currentSample.mName);
		datalog.record("Immersion medium = ", currentSample.mImmersionMedium);
		datalog.record("Correction collar = ", currentSample.mObjectiveCollar);
		datalog.record("\nFPGA---------------------------------------------------------");
		datalog.record("FPGA clock (MHz) = ", g_tickPerUs);
		datalog.record("\nSCAN---------------------------------------------------------");
		datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
		datalog.record("RS period (us) = ", 2 * g_lineclockHalfPeriod / us);
		datalog.record("Pixel dwell time (us) = ", g_pixelDwellTime / us);
		datalog.record("RS fill factor = ", RScanner.mFillFactor);
		datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
		datalog.record("\nIMAGE--------------------------------------------------------");
		datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
		datalog.record("Upscaling factor = ", RTcontrol.mUpscalingFactor);
		datalog.record("Width X (RS) (pix) = ", widthPerFrame_pix);
		datalog.record("Height Y (galvo) (pix) = ", heightPerFrame_pix);
		datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
		datalog.record("Resolution Y (galvo) (um/pix) = ", pixelSizeXY / um);
		datalog.record("\nSTAGE--------------------------------------------------------");

		//Update the laser wavelength
		const int wavelength_nm{ fluorLabelList.at(iterWL).mWavelength_nm };
		virtualLaser.configure(wavelength_nm);		//When switching pockels, the class destructor closes the uniblitz shutter
		virtualLaser.openShutter();					//Re-open the Uniblitz shutter if closed by the pockels destructor

		//GALVOS
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslowPerBeamlet / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, FFOVslowPerBeamlet / 2, &virtualLaser };

		//Iterate over the locations
		for (std::vector<int>::size_type iterLocation = 0; iterLocation < locationListXY.size(); iterLocation++)
		{
			//Generate the discrete scan sequence for the stages
			std::vector<double3> stagePositionXYZ;
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			{
				//stagePositionXYZ.push_back({ locationListXY.at(iterLocation).at(X), locationListXY.at(iterLocation).at(Y), stackCenterXYZ.at(Z) + iterDiffZ * stepSizeZ });
				stagePositionXYZ.push_back({ locationListXY.at(iterLocation).at(X), locationListXY.at(iterLocation).at(Y), stackCenterXYZ.at(Z) - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
			}

			//CREATE A STACK FOR SAVING THE TIFFS
			TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, 1 };

			//ACQUIRE FRAMES AT DIFFERENT Zs
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
			{
				//Update the stages position
				stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
				stage.waitForMotionToStopAll();
				//stage.printPositionXYZ();		//Print the stage position

				std::cout << "Location: " << iterLocation + 1 << "/" << locationListXY.size() << "\tTotal frame: " << iterDiffZ + 1 << "/" << nDiffZ << "\n";

				//Update the laser power
				virtualLaser.setPower(fluorLabelList.at(iterWL).mScanPi + iterDiffZ * stepSizeZ * fluorLabelList.at(iterWL).mStackPinc);

				//EXECUTE THE RT CONTROL SEQUENCE
				Image image{ RTcontrol };
				image.acquire();							//Execute the RT control sequence and acquire the image
				image.averageFrames();						//Average the frames acquired via continuous acquisition
				image.correctImage(RScanner.mFFOV);
				tiffStack.pushSameZ(0, image.data());
				tiffStack.pushDiffZ(iterDiffZ);
				std::cout << "\n";

				pressESCforEarlyTermination();				//Early termination if ESC is pressed
			}

			//Save the stackDiffZ to file
			std::string shortName{ "Slice" + std::to_string(nSlice) + "_" + fluorLabelList.at(iterWL).mName + "_Tile" + std::to_string(iterLocation + 1) };
			std::string longName{ currentSample.mName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(fluorLabelList.at(iterWL).mScanPi / mW, 1) + "mW_Pinc=" + toString(fluorLabelList.at(iterWL).mStackPinc / mWpum, 1) + "mWpum" +
				"_x=" + toString(stagePositionXYZ.front().at(X) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(Y) / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().at(Z) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(Z) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };

			datalog.record(shortName + "\t" + longName);
			tiffStack.saveToFile(shortName, OVERRIDE::DIS);
		}//iterLocation
	}//iter_wv
}
*/	
}//namespace

//Photobleach the sample with the resonant scanner to see how much the sample moves after slicing
//I bleach with the RS and not the galvo or the stages because this way the RS is kept on the entire time while bleaching and imaging
namespace TestRoutines
{
	//Generate many short digital pulses and check the overall frameDuration with the oscilloscope
	void digitalLatency(const FPGA &fpga)
	{
		const double timeStep{ 4. * us };

		RTcontrol RTcontrol{ fpga };

		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);

		//Many short digital pulses to accumulate the error
		for (U32 ii = 0; ii < 99; ii++)
			RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);

		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
	}

	//First calibrate the digital channels, then use it as a time reference
	void analogLatency(const FPGA &fpga)
	{
		const double delay{ 400. * us };
		const double timeStep{ 4. * us };

		RTcontrol RTcontrol{ fpga };
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, timeStep, 10 * V);						//Initial pulse
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, timeStep, 0);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANGALVO, 4 * us, delay, 0, 5 * V, OVERRIDE::DIS);		//Linear ramp to accumulate the error
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, timeStep, 10 * V);						//Initial pulse
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, timeStep, 0);								//Final pulse

		//DO0
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, delay, 0);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
	}

	void pixelclock(const FPGA &fpga)
	{
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::STAGEX, 1, 300, 560, FIFOOUTfpga::DIS }; 	//Create a realtime control sequence
		Image image{ RTcontrol };
		image.initializeAcq();	//Execute the realtime control sequence and acquire the image
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGA &fpga)
	{
		const double timeStep{ 400. * us };

		RTcontrol RTcontrol{ fpga };
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGA &fpga)
	{
		RTcontrol RTcontrol{ fpga };

		//DO
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 0);

		//AO
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, 8 * us, 4 * V);
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, 4 * us, 2 * V);
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANGALVO, 4 * us, 1 * V);

		RTcontrol.triggerRT();	//Execute the realtime control sequence
	}

	void analogRamp(const FPGA &fpga)
	{
		const double Vmax{ 5. * V };
		const double step{ 4. * us };

		RTcontrol RTcontrol{ fpga };
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANGALVO, step, 2 * ms, 0, -Vmax, OVERRIDE::DIS);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANGALVO, step, 20 * ms, -Vmax, Vmax, OVERRIDE::DIS);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANGALVO, step, 2 * ms, Vmax, 0, OVERRIDE::DIS);

		const double pulsewidth(300. * us);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, pulsewidth, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 0);
	}

	//I think this is for matching the galvo forward and backward scans via imaging beads
	void fineTuneScanGalvo(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 30 };												//Number of frames for continuous acquisition
		const POSITION3 stagePositionXYZ{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUTfpga::EN };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVOS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslow / 2 };

		//LASER
		const int wavelength_nm{ 1040 };
		const double P{ 25. * mW };		//Laser power
		const VirtualLaser virtualLaser{ RTcontrol, wavelength_nm, P, P, Laser::ID::FIDELITY };

		//ACQUIRE FRAMES AT DIFFERENT Zs
		stage.moveXYZ(stagePositionXYZ);
		stage.waitForMotionToStopAll();

		//OPEN THE UNIBLITZ SHUTTERS
		virtualLaser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous acquisition
		image.averageEvenOddFrames();
		image.save("Untitled", TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
	}

	void resonantScanner(const FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga };

		ResonantScanner RScanner{ RTcontrol };
		std::cout << "aaa = " << RScanner.downloadControlVoltage() << "\n";
		//RScanner.turnOn(150 * um);
		//RScanner.turnOff();
	}

	//Use a single beamlet with the rescanner sync'ed to the scanner to keep the beam position fixed at the detector plane
	//Must manually open the laser and Uniblitz shutter and adjust the pockels power
	void galvosSync(const FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 35 };
		const int nFramesCont{ 1 };

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUTfpga::DIS };

		//GALVOS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };		//Scan duration in the slow axis
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslow / 2 };
		Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslow / 2 };

		//Execute the realtime control sequence and acquire the image
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence
		pressAnyKeyToCont();
	}

	//Test the synchronization of the 2 galvos and the laser
	void gavosLaserSync(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 2 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int heightPerBeamletPerFrame_pix;
		double FFOVslowPerBeamlet, selectPower;

#if multibeam
		//Multibeam
		heightPerBeamletPerFrame_pix = static_cast<int>(heightPerFrame_pix / g_nChanPMT);
		FFOVslowPerBeamlet = static_cast<int>(FFOVslow / g_nChanPMT);
		selectPower = 1400. * mW;
#else
		//Singlebeam
		heightPerBeamletPerFrame_pix = heightPerFrame_pix;
		FFOVslowPerBeamlet = FFOVslow;
		selectPower = 50. * mW;
#endif
		//STACK
		const double stepSizeZ{ 1.0 * um };
		const double stackDepthZ{ 20. * um };	//Acquire a stack this deep in the axis STAGEZ

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerBeamletPerFrame_pix, FIFOOUTfpga::EN };

		//LASER
		const int wavelength_nm{ 750 };
		const VirtualLaser virtualLaser{ RTcontrol, wavelength_nm, selectPower, selectPower, Laser::ID::VISION };

		//GALVOS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslowPerBeamlet / 2 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslowPerBeamlet / 2, &virtualLaser };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, 0, wavelength_nm };

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();			//Execute the RT control sequence and acquire the image
	}

	void stagePosition()
	{
		double duration;
		const POSITION3 stagePositionXYZ{ 35.020 * mm, 19.808 * mm, 18.542 * mm };	//Stage initial position
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		std::cout << "Stages initial position:" << "\n";
		stage.printPositionXYZ();

		auto t_start{ std::chrono::high_resolution_clock::now() };

		stage.moveXYZ(stagePositionXYZ);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		stage.waitForMotionToStopSingle(Stage::ZZ);

		std::cout << "Stages final position:" << "\n";
		stage.printPositionXYZ();

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

		//std::cout << "Stages initial position:" << "\n";
		//stage.printPositionXYZ();

		std::cout << "Stages initial vel:" << "\n";
		stage.printVelXYZ();

		//stage.isDOtriggerEnabled(Z, DOchan);
		//stage.setDOtriggerEnabled(Z, DOchan , true);

		//const Stage::DOPARAM triggerParam{ Stage::DOPARAM::TRIGSTEP };
		//stage.downloadDOtriggerParamSingle_(Z, DOchan , triggerParam);
		//std::cout << "x stage vel: " << stage.downloadVelSingle_(X) / mmps << " mm/s" << "\n";
		//std::cout << "y stage vel: " << stage.downloadVelSingle_(Y) / mmps << " mm/s" << "\n";
		//std::cout << "z stage vel: " << stage.downloadVelSingle_(Z) / mmps << " mm/s" << "\n";
		//stage.printStageConfig(Z, DOchan);
	}

	void shutter(const FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga };

		PockelsCell fidelity{ RTcontrol, 1040, Laser::ID::FIDELITY };
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
//2. Set pockelsAutoOff to 0 for holding on the last value
//3. Tune Vision's wavelength manually
	void pockels(const FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga };

		//DEFINE THE POCKELS CELLS
		PockelsCell pockelsVision{ RTcontrol, 750, Laser::ID::VISION };
		PockelsCell pockelsFidelity{ RTcontrol, 1040, Laser::ID::FIDELITY };

		PockelsCell pockels{ pockelsFidelity };
		//PockelsCell pockels{ pockelsFidelity };
		pockels.pushPowerSinglet(8 * us, 100. * mW, OVERRIDE::DIS);
		//pockels.pushPowerSinglet(8 * us, 0 * mW, OVERRIDE::DIS);
		//pockels.pushVoltageSinglet(8 * us, 2.0 * V, OVERRIDE::DIS);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		Image image{ RTcontrol };
		image.acquire();
		//pressAnyKeyToCont();
	}

	void pockelsRamp(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 35 };
		const int nFramesCont{ 200 };			//Number of frames for continuous acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUTfpga::DIS };

		//POCKELS CELL
		const int wavelength_nm{ 750 };
		PockelsCell pockels{ RTcontrol, wavelength_nm, Laser::ID::VISION };
		const double Pi{ 192. * mW }, Pf{ 336. * mW };
		//pockels.pushPowerSinglet(400 * us, Pf, OVERRIDE::EN);
		pockels.powerLinearScaling(0.96 * Pi, Pf);			//Linearly scale the laser power from the first to the last frame
		//pockels.powerLinearScaling(0.96 * Pf, Pi);

		//Test the voltage setpoint
		//pockels.pushVoltageSinglet(8* us, 0.5 * V);
		//pockels.voltageLinearRamp(0.5 * V, 1.0 * V);		//Linearly scale the pockels voltage from the first to the last frame

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous acquisition
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
		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga };

		const int wavelength_nm{ 1040 };
		const double laserPower{ 50. * mW };
		const VirtualLaser virtualLaser{ RTcontrol, wavelength_nm, laserPower, laserPower, Laser::ID::VISION };

		//EXECUTE THE RT CONTROL SEQUENCE
		//Image image{ RTcontrol };
		//image.acquire();					//Execute the RT control sequence
	}

	//Photobleach a line along the fast axis (RS) on the sample
	void photobleach(const FPGA &fpga)
	{
		Laser laser{ Laser::ID::VISION };
		laser.setWavelength(920);

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, 100, 300, 400, FIFOOUTfpga::DIS };

		//RS
		ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVO. Keep the galvo fixed to bleach a line on the sample
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, 0 };

		//POCKELS CELLS
		PockelsCell pockels{ RTcontrol, 920, Laser::ID::VISION };
		pockels.pushPowerSinglet(8 * us, 200 * mW, OVERRIDE::DIS);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		pockels.setShutter(true);
		Image image{ RTcontrol };
		image.acquire();

		//Wait until the sequence is over to close the shutter, otherwise this code will finish before the RT sequence
		pressAnyKeyToCont();
		pockels.setShutter(false);
	}

	void convertI16toVolt()
	{
		std::cout << "volt to I16: " << FPGAfunc::voltageToI16(-10) << "\n";
		std::cout << "int to volt: " << FPGAfunc::intToVoltage(-32768) << "\n";
		std::cout << "volt to I16 to volt: " << FPGAfunc::intToVoltage(FPGAfunc::voltageToI16(0)) << "\n";
	}

	//To measure the saving speed of a Tiff file, either locally or remotely
	//Select a local or remote folder accordingly
	void ethernetSpeed()
	{
		std::string filename{ "testEthernetSpeed" };

		//The goal is to stream a stackDiffZ composed of 200 z-planes (100 um in 0.5 um-steps), where each frame has 300x560 pixels. Therefore, the stackDiffZ has 300x560x200 = 33.6 Mega pixels
		//The stackDiffZ size is 8 bits x 33.6M = 33.6 MB
		const int width{ 300 };
		const int height{ 560 };
		const int nFramesCont{ 200 };

		TiffU8 image{ width, height, nFramesCont };

		//Declare and start a stopwatch
		double duration;
		auto t_start{ std::chrono::high_resolution_clock::now() };

		//overriding the file saving has some travelOverhead
		//Splitting the stackDiffZ into a page structure (by assigning nFramesCont = 200 in saveToFile) gives a large travelOverhead
		image.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);

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

		first.join();//pauses until first finishes
		second.join();//pauses until second finishes
	}

	void clipU8()
	{
		int input{ 260 };
		U8 output{ clipU8top(input) };
		std::cout << (int)output << "\n";
		pressAnyKeyToCont();
	}

	void correctImage()
	{

		std::string inputFilename{ "noCrosstalk_Liver_V750nm_P=192.0mW_x=51.500_y=18.300_z=17.7640" };
		std::string outputFilename{ "output_" + inputFilename };
		TiffU8 image{ inputFilename };
		image.correct16XFOVslow(1);
		//image.correctRSdistortionGPU(200. * um);	
		//image.flattenField(1.5);
		//image.suppressCrosstalk(0.2);
		image.saveToFile(outputFilename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);

		//image.binFrames(5);
		//image.splitIntoFrames(10);
		//image.mirrorOddFrames();
		//image.averageEvenOddFrames();

		/*
		//Declare and start a stopwatch
		double duration;
		auto t_start{ std::chrono::high_resolution_clock::now() };
		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";
		*/
		pressAnyKeyToCont();
	}

	void quickStitcher(const FPGA &fpga)
	{
		//RTcontrol RTcontrol{ fpga };
		//Image image{ RTcontrol };
		//image.save("empty", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);
		//image.formImageVerticalStrip(SCANDIRX::LEFT);

		//TiffU8 asd{ image.data(), 300, 560 };
		//std::cout << image.tiff().widthPerFrame_pix() << "\n";
		//std::cout << image.tiff().heightPerFrame_pix() << "\n";

		std::string outputFilename{ "stitched" };
		const int width{ 300 };
		const int height{ 8000 };
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

		pressAnyKeyToCont();
	}

	void locateSample()
	{
		std::string outputFilename{ "output" };
		TiffU8 image{ "stitched" };

		const int tileWidth_pix{ 300 };
		const int tileHeight_pix{ 400 };
		const int nTileCol{ image.widthPerFrame_pix() / tileWidth_pix };
		const int nTileRow{ image.heightPerFrame_pix() / tileHeight_pix };

		image.giveBoolMap(0.004, tileWidth_pix, tileHeight_pix);
		//image.isDark(0.01);

		pressAnyKeyToCont();
	}

	void isDark()
	{
		std::string inputFilename{ "Liver_V750nm_P=7.0mWpum_xi=53.500_xf=49.400_y=22.300_z=17.7840_Step=0.0005" };
		TiffU8 image{ inputFilename };
		image.giveBoolMap(0.004, 300, 400);

		pressAnyKeyToCont();
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

		pressAnyKeyToCont();
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
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };
		const int nFramesCont{ 80 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };										//Step size in the axis STAGEZ
		const ROI4 roi{ 9.950 * mm, 34.850 * mm, 10.150 * mm, 35.050 * mm };	//Region of interest {ymin, xmin, ymax, xmax}
		const TILEOVERLAP4 stackOverlap_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };							//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };								//Sample thickness
		const double sampleSurfaceZ{ 18.471 * mm };

		Sample sample{ currentSample, roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlap_frac };

		//Create a sequence
		Sequence sequence{ sample, stack };
		sequence.generateCommandList();
		sequence.printToFile("CommandlistLight");

		if (1)
		{
			FUNC x;
			std::thread saveFile, moveStage;

			//Read the commands line by line
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.size(); iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Sequence::Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
				//commandline.printParameters();

				switch (commandline.mAction)
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

	void locationSequence()
	{
		//ACQUISITION SETTINGS
		const FFOV2 FFOV{ 200. * um, 150. * um };
		const int nDiffZ{ 100 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };									//Step size in the axis STAGEZ
		const TILEOVERLAP4 stackOverlap_frac{ 0.05, 0.05, 0.05 };			//Stack overlap
		const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

		//Create a sequence
		Sequence sequence{ currentSample, stack, {stackCenterXYZ.XX, stackCenterXYZ.YY}, { 2, 2 } }; //Last 2 parameters: stack center and number of stacks
		std::vector<POSITION2> locationList{ sequence.generateLocationList() };

		for (std::vector<int>::size_type iterLocation = 0; iterLocation < locationList.size(); iterLocation++)
		{
			std::cout << "x = " << locationList.at(iterLocation).XX / mm << "\ty = " << locationList.at(iterLocation).YY / mm << "\n";
		}
		pressAnyKeyToCont();
	}

	//Generate a text file with the tile location for the BigStitcher
	void generateLocationsForBigStitcher()
	{
		// X is vertical and Y is horizontal, to match the directions of the XYZ stage
		const INDICES2 nStacksIJ{ 30, 28 };
		const int tileShiftX_pix{ 543 };
		const int tileShiftY_pix{ 291 };

		Logger datalog(currentSample.mName + "_locations");
		datalog.record("dim=3"); //Needed for the BigStitcher

		for (int nTile = 0; nTile < nStacksIJ.II * nStacksIJ.JJ; nTile++)
			//for (int nTile = 0; nTile < 180; nTile++)
		{
			INDICES2 nIJ = nTileToArrayIndices(nTile);
			int totalTileShiftX{ -nIJ.II * tileShiftX_pix };
			int TotalTileShiftY{ -nIJ.JJ * tileShiftY_pix };
			std::string line{ std::to_string(nTile) + ";;(" + std::to_string(TotalTileShiftY) + "," + std::to_string(totalTileShiftX) + ",0)" };	//In BigStitcher, X is horizontal and Y is vertical
			//std::string line{ std::to_string(nTile) + "\t" + std::to_string(nTileToArrayIndices(nTile).at(X)) + "\t" + std::to_string(nTileToArrayIndices(nTile).at(Y)) }; //For debugging
			datalog.record(line);
		}
	}

	//Snake pattern starting from the bottom right of the sample and going up
	INDICES2 nTileToArrayIndices(const int nTile)
	{
		const INDICES2 nStacksXY{ 30, 28 };

		int nx;
		int ny{ nTile / nStacksXY.II };

		if (ny % 2)	//ny is odd
			nx = nStacksXY.II - nTile % nStacksXY.II - 1;
		else		//ny is even
			nx = nTile % nStacksXY.II;

		return { nx,ny };
	}

	void PMT16Xconfig()
	{
		PMT16X PMT;

		//pmt.setSingleGain(PMT16XCHAN::CH00, 170);
	
		//PMT.setAllGain(255);
		//PMT.readTemp();
		PMT.readAllGain();
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
			255,	//CH08, this channel presents the lowest signal. For some reason 255 gives a lower signal than 200
			255,	//CH09
			255,	//CH10
			255,	//CH11
			255,	//CH12
			255,	//CH13
			255,	//CH14
			255		//CH15
			});
			*/

		pressAnyKeyToCont();
	}

	//Test reading different channels of the PMT16X
	//Must manually open the laser and Uniblitz shutter
	void PMT16Xdemultiplex(const FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 1 };										//Number of frames for continuous acquisition
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };		//Scan duration in the slow axis
		const int wavelength_nm{ 750 };

		//CREATE A REALTIME CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUTfpga::EN };

		//LASER
		const double laserPower{ 30. * mW };
		const VirtualLaser virtualLaser{ RTcontrol, wavelength_nm, laserPower, laserPower, Laser::ID::VISION };

		//GALVOS
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANGALVO, FFOVslow / 2 };
		Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANGALVO, FFOVslow / 2, &virtualLaser };
		//Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, 0 };				//Keep the scanner fixed to see the emitted light swing across the PMT16X channels. The rescanner must be centered

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();			//Execute the RT control sequence and acquire the image
		image.save("SingleChannel", TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);
	}

	void vibratome(const FPGA &fpga)
	{
		const double slicePlaneZ{ (23.640 + 0.050) * mm };

		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };
		Vibratome vibratome{ fpga, stage };
		vibratome.slice(slicePlaneZ);

		const POSITION3 samplePosition{ 0. * mm, 0. * mm, slicePlaneZ };
		stage.moveXYZ(stackCenterXYZ);
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

		//FWexcitation.setWavelength(wavelength_nm);
		//FWdetection.setWavelength(wavelength_nm);

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
		StepperActuator collectorLens{ "26000299" };
		collectorLens.move(10.0 * mm);
		collectorLens.downloadConfig();
		//collectorLens.home();
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