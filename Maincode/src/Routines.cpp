#include "Routines.h"
const std::vector<LIMIT2> PetridishPosLimit{ { 27. * mm, 57. * mm}, { 0. * mm, 30. * mm}, { 15. * mm, 24. * mm} };		//Soft limit of the stage for the petridish
const std::vector<LIMIT2> ContainerPosLimit{ { -65. * mm, 65. * mm}, { 1.99 * mm, 30. * mm}, { 10. * mm, 24. * mm} };		//Soft limit of the stage for the oil container

//SAMPLE PARAMETERS
POSITION3 stackCenterXYZ{ (44.600 ) * mm, (21.075)* mm, (16.915 + 0.050) * mm };

#if multibeam
//Sample beads4um{ "Beads4um16X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, multiply16X(50. * mW), multiply16X(0.0 * mWpum) }, { "GFP", 920, multiply16X(45. * mW), multiply16X(0. * mWpum) }, { "TDT", 1040, multiply16X(15. * mW), multiply16X(0. * mWpum) } }} };
//Sample liver{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{ {"TDT", 1040, multiply16X(50. * mW), multiply16X(0.0 * mWpum) } , { "GFP", 920, multiply16X(40. * mW), multiply16X(0.0 * mWpum) } , { "DAPI", 750, multiply16X(50. * mW), multiply16X(0.09 * mWpum) } }} };
Sample liverDAPITDT{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit,
					{{ {"TDT", 1040, multiply16X(50. * mW), multiply16X(0.1 * mWpum), 4 } ,
					   { "DAPI", 750, multiply16X(25. * mW), multiply16X(0.3 * mWpum), 2 } }} };

#else
Sample liverDAPITDT{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit,
					{{{"TDT", 1040, 30. * mW, 0.05 * mWpum, 4 } ,
					  { "DAPI", 750, 12. * mW, 0.2 * mWpum, 2 }}} };
Sample liverTDT{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", ContainerPosLimit,
					{{{"TDT", 1040, 30. * mW, 0.05 * mWpum, 4 }}} };

//Sample beads4um{ "Beads4um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 35. * mW, 0. * mWpum }, { "GFP", 920, 30. * mW, 0. * mWpum }, { "TDT", 1040, 5. * mW, 0. * mWpum }}} };
//Sample beads05um{ "Beads1um1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{"DAPI", 750, 40. * mW, 0. * mWpum }, { "GFP", 920, 40. * mW, 0. * mWpum }, { "TDT", 1040, 15. * mW, 0. * mWpum }}} };
//Sample fluorSlide{ "fluorBlue1X", "SiliconeOil", "1.51", PetridishPosLimit, {{{ "DAPI", 750, 10. * mW, 0. * mWpum }}} };
//Sample liver{ "Liver20190812_02", "SiliconeMineralOil5050", "1.49", PetridishPosLimit, {{{"TDT", 1040, 30. * mW, 0.0 * mWpum } , { "GFP", 920, 25. * mW, 0.0 * mWpum }, { "DAPI", 750, 40. * mW, 0.09 * mWpum }}} };
#endif
Sample currentSample{ liverDAPITDT };


double determineChromaticShift(const int wavelength_nm, const Laser::ID whichLaser)
{
	//This is because the beads at 750 nm are chromatically shifted
	if (wavelength_nm == 750)
		return 6 * um;

	//This is because FIDELITY is chromatically shifted wrt VISION
	if (wavelength_nm == 1040 && (whichLaser == Laser::ID::FIDELITY || whichLaser == Laser::ID::AUTO))
		return 4 * um;
}


namespace Routines
{
	//The "Swiss knife" of my routines
	void stepwiseScan(const FPGA &fpga)
	{
		const RUNMODE acqMode{ RUNMODE::SINGLE };			//Single frame. The same location is imaged continuously if nFramesCont>1 (the galvo is scanned back and forth at the same location) and the average is returned
		//const RUNMODE acqMode{ RUNMODE::AVG };			//Single frame. The same location is imaged stepwise and the average is returned
		//const RUNMODE acqMode{ RUNMODE::SCANZ };			//Scan in the Z-stage axis stepwise with stackCenterXYZ.at(STAGEZ) the starting position
		//const RUNMODE acqMode{ RUNMODE::SCANZCENTERED };	//Scan in the Z-stage axis stepwise with stackCenterXYZ.at(STAGEZ) the center of the stack
		//const RUNMODE acqMode{ RUNMODE::SCANXY };			//Scan in the X-stage axis stepwise
		//const RUNMODE acqMode{ RUNMODE::COLLECTLENS };	//For optimizing the collector lens
		
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("TDT") };	//Select a particular fluorescence channel
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const int nFramesCont{ 1 };	
		const double stackDepthZ{ 40. * um };								//Stack deepth in the Z-stage axis
		const double stepSizeZ{ 1.0 * um };
	
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		//stackCenterXYZ.ZZ -= determineChromaticShift(fluorLabel.mWavelength_nm, whichLaser);

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
				stagePositionXYZ.push_back({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
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

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//LASER
		VirtualLaser virtualLaser{ whichLaser };
		virtualLaser.configure(RTcontrol, fluorLabel.mWavelength_nm);

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();						//To make sure that the RS is running

		//SCANNERS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslowPerBeamlet / 2. };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslowPerBeamlet / 2., &virtualLaser };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANNER, 0, fluorLabel.mWavelength_nm };

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
		stage.moveXYZ(stagePositionXYZ.front());	//Move the stage to the initial position
		Sleep(500);									//Give the stages enough time to settle at the initial position
		stage.waitForMotionToStopAll();

		//CREATE A STACK FOR STORING THE TIFFS
		const int nLocations{ static_cast<int>(stagePositionXYZ.size()) };
		TiffU8 output{ heightPerFrame_pix, widthPerFrame_pix, nLocations };
		std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm" };
		
		//OPEN THE UNIBLITZ SHUTTERS
		virtualLaser.openShutter();					//The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations  << "\n";
			stage.moveXYZ(stagePositionXYZ.at(iterLocation));
			stage.waitForMotionToStopAll();
			//stage.printPositionXYZ();				//Print the stage position	
			
			virtualLaser.setPower(fluorLabel.mScanPmin + iterLocation * stepSizeZ * fluorLabel.mScanPinc);	//Update the laser power

			//Used to optimize the collector lens position
			if (acqMode == RUNMODE::COLLECTLENS)
				virtualLaser.moveCollectorLens(cLensPosIni + iterLocation * cLensStep);

			//EXECUTE THE CONTROL SEQUENCE
			RTcontrol.run();					//Execute the control sequence and acquire the image
			Image image{ RTcontrol };
			image.acquire(saveAllPMT);
			image.averageFrames();				//Average the frames imaged via continuous acquisition
			//image.averageEvenOddFrames();		//For debugging
			//image.correct(RScanner.mFFOV);

			if (acqMode == RUNMODE::SINGLE && !saveAllPMT)
			{
				//Save individual files
				filename.append("_P=" + toString(fluorLabel.mScanPmin / mW, 1) + "mW" +
					"_x=" + toString(stagePositionXYZ.at(iterLocation).XX / mm, 3) + "_y=" + toString(stagePositionXYZ.at(iterLocation).YY / mm, 3) + "_z=" + toString(stagePositionXYZ.at(iterLocation).ZZ / mm, 4) + 
					"_avg=" + toString(nFramesCont, 0));
				std::cout << "Saving the stack...\n";
				image.save(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
			}
			output.pushImage(image.data(), iterLocation);

			pressESCforEarlyTermination();
		}

		if (acqMode == RUNMODE::AVG || acqMode == RUNMODE::SCANZ || acqMode == RUNMODE::SCANZCENTERED)
		{	
			filename.append( "_P=" + toString(fluorLabel.mScanPmin / mW, 1) + "mW_Pinc=" + toString(fluorLabel.mScanPinc / mWpum, 2) + "mWpum" +
				"_x=" + toString(stagePositionXYZ.front().XX / mm, 3) + "_y=" + toString(stagePositionXYZ.front().YY / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().ZZ / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().ZZ / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) + 
				"_avg=" + toString(nFramesCont * nSameLocation, 0) );

			output.binFrames(nSameLocation);									//Divide the images in bins and return the binned image
			std::cout << "Saving the stack...\n";
			output.saveToFile(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);	//Save the scanZ to file
			pressESCforEarlyTermination();
		}

		if (acqMode == RUNMODE::SCANXY)
		{
			filename.append( "_P=" + toString(fluorLabel.mScanPmin / mW, 1) + "mW" +
				"_xi=" + toString(stagePositionXYZ.front().XX / mm, 4) + "_xf=" + toString(stagePositionXYZ.back().XX / mm, 4) +
				"_y=" + toString(stagePositionXYZ.front().YY / mm, 4) +
				"_z=" + toString(stagePositionXYZ.front().ZZ / mm, 4) + "_Step=" + toString(stepSizeX / mm, 4) +
				"_avg=" + toString(nFramesCont, 0) );

			output.binFrames(nSameLocation);									//Divide the images in bins and return the binned image
			std::cout << "Saving the stack...\n";
			output.saveToFile(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);	//Save the scanXY to file
			pressESCforEarlyTermination();
		}

		//DATALOG
		{
			Logger datalog(filename);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", currentSample.mName);
			datalog.record("Immersion medium = ", currentSample.mImmersionMedium);
			datalog.record("Correction collar = ", currentSample.mObjectiveCollar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", g_tickPerUs);
			datalog.record("\nLASER--------------------------------------------------------");
			datalog.record("Laser used = ", virtualLaser.currentLaser_s());
			datalog.record("Laser wavelength (nm) = ", virtualLaser.currentWavelength_nm());
			datalog.record("Min laser power (mW) = ", fluorLabel.mScanPmin / mW);
			datalog.record("Laser power increase (mW/um) = ", fluorLabel.mScanPinc / mWpum);
			datalog.record("Laser repetition period (us) = ", g_laserPulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * g_lineclockHalfPeriod / us);
			datalog.record("Pixel dwell time (us) = ", g_pixelDwellTime / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
			datalog.record("nFramesCont = ", nFramesCont);
			datalog.record("nSameLocation= ", nSameLocation);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", g_pulsesPerPix);
			datalog.record("Upscaling factor = ", g_upscalingFactor);
			datalog.record("Height Y (slow) (pix) = ", heightPerFrame_pix);
			datalog.record("Width X (fast) (pix) = ", widthPerFrame_pix);
			datalog.record("Resolution X (fast) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (slow) (um/pix) = ", pixelSizeXY / um);
			datalog.record("\nSTAGE--------------------------------------------------------");
			datalog.record("Stack center X (mm) = ", stackCenterXYZ.XX / mm);
			datalog.record("Stack center Y (mm) = ", stackCenterXYZ.YY / mm);
		}
		//pressAnyKeyToCont();
	}

	//I triggered the stack acquisition using DO2 (stage motion) for both scanning directions: top-down and bottom-up. In both cases the bead z-position looks almost identical with a difference of maybe only 1 plane (0.5 um)
	//Remember that I do not use MACROS on the stages anymore*/
	void contScanZ(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("TDT") };				//Select a particular laser
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		const SCANDIR scanDirZ{ SCANDIR::UPWARD };														//Scan direction for imaging in Z
		const int nFramesBinning{ 1 };																	//For binning
		const double stackDepth{ 100. * um };
		const double pixelSizeZafterBinning{ 1.0 * um  };

		const int nFrames{ static_cast<int>(nFramesBinning * stackDepth / pixelSizeZafterBinning) };	//Number of frames BEFORE binning for continuous acquisition
		const double pixelSizeZbeforeBinning{ pixelSizeZafterBinning / nFramesBinning };				//Pixel size per z frame
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };										//Full FOV in the slow axis

		//stackCenterXYZ.ZZ -= determineChromaticShift(fluorLabel.mWavelength_nm, whichLaser);

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

		//CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEZ, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFrames };	//Note the STAGEZ flag

		//LASER
		const double laserPi = determineInitialLaserPower(fluorLabel.mScanPmin, stackDepth * fluorLabel.mScanPinc, scanDirZ);
		const double laserPf = determineFinalLaserPower(fluorLabel.mScanPmin, stackDepth * fluorLabel.mScanPinc, scanDirZ);
		VirtualLaser virtualLaser{ whichLaser };
		virtualLaser.configure(RTcontrol, fluorLabel.mWavelength_nm);
		virtualLaser.setPower(laserPi, laserPf);

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//To make sure that the RS is running

		//SCANNERS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslowPerBeamlet / 2. };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslowPerBeamlet / 2., &virtualLaser };

		//STAGES
		const double stageZi = determineInitialScanPos(stackCenterXYZ.ZZ, stackDepth, 0. * mm, scanDirZ);
		const double stageZf = determineFinalScanPos(stackCenterXYZ.ZZ, stackDepth, 0.010 * mm, scanDirZ);
		Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };							
		stage.moveXYZ({ stackCenterXYZ.XX, stackCenterXYZ.YY, stageZi });		//Move the stage to the initial position
		stage.waitForMotionToStopAll();
		Sleep(500);																//Give the stages enough time to settle at the initial position
		stage.setVelSingle(Stage::Axis::ZZ, pixelSizeZbeforeBinning / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));		//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix

		//EXECUTE THE CONTROL SEQUENCE
		virtualLaser.openShutter();				//Open the shutter. The destructor will close the shutter automatically
		
		RTcontrol.initialize(scanDirZ);
		std::cout << "Scanning the stack...\n";
		stage.moveSingle(Stage::ZZ, stageZf);	//Move the stage to trigger the ctl&acq sequence
		RTcontrol.downloadData();

		virtualLaser.closeShutter();			//Close the shutter manually even though the destructor does it because the post-processing could take a long time
		Image image{ RTcontrol };
		image.acquire();
		image.binFrames(nFramesBinning);
		//image.correct(RScanner.mFFOV);

		const std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm_P=" + toString((std::min)(laserPi, laserPf) / mW, 1) + "mW_Pinc=" + toString(fluorLabel.mScanPinc / mWpum, 2) +
			"mWpum_x=" + toString(stackCenterXYZ.XX / mm, 3) + "_y=" + toString(stackCenterXYZ.YY / mm, 3) +
			"_zi=" + toString(stageZi / mm, 4) + "_zf=" + toString(stageZf / mm, 4) + "_Step=" + toString(pixelSizeZafterBinning / mm, 4) +
			"_bin=" + toString(nFramesBinning, 0) };
		
		std::cout << "Saving the stack...\n";
		image.save(filename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);

		//pressAnyKeyToCont();
	}

	void contScanX(const FPGA &fpga)
	{
		if (multibeam)
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Continuous X-stage scanning available for single beam only");

		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("TDT") };			//Select a particular laser
		const Laser::ID whichLaser{ Laser::ID::AUTO };
		//SCANDIR iterScanDirX{ SCANDIR::LEFTWARD };
		SCANDIR iterScanDirX{ SCANDIR::RIGHTWARD };													//Initial scan direction of stage 
		const double stitchedWidth{ 10.000 * mm };													//Total width of the tile array

		const double tileWidth{ 150. * um };														//Width of 1 strip (long vertical tile)
		const int nCol{ static_cast<int>(std::ceil(1. * stitchedWidth / tileWidth)) };				//Number of columns in the tile array
		const int tileWidth_pix{ 300 };																//Number of pixel width in a strip (long vertical tile)
		const double stitchedHeight{ 10.080 * mm };//= 36 * 0.280 * mm								//Total height of the tile array = height of the strip (long vertical tile). If changed, the X-stage timing must be recalibrated
		const double pixelSizeX{ 1.0 * um };														//WARNING: the image becomes distorted for pixelSizeX < 1 um
		const int stitchedHeight_pix{ static_cast<int>(stitchedHeight / pixelSizeX) };				//Total pixel height in the tile array

		//stackCenterXYZ.ZZ -= determineChromaticShift(fluorLabel.mWavelength_nm, whichLaser);

		//LOCATIONS on the sample to image
		std::vector<double> stagePositionY;
		const int nColHalf{ nCol / 2 };																//Make the center of the tile array coincide with stackCenterXYZ
		for (int iterLocation = 0; iterLocation < nCol; iterLocation++)
			stagePositionY.push_back( stackCenterXYZ.YY + tileWidth * (nColHalf - iterLocation) );	//for now, only allowed to stack strips to the right (i.e. only allowed to move the stage to the left)

		//CONTROL SEQUENCE
		//The Image height is 2 (two galvo swings) and nFrames is stitchedHeight_pix/2. The total height of the final image is therefore stitchedHeight_pix. Note the STAGEX flag
		const int nFrames{ 2 };
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEX, FIFOOUTfpga::EN, nFrames, tileWidth_pix, stitchedHeight_pix / 2 };

		//LASER
		VirtualLaser virtualLaser{ whichLaser };
		virtualLaser.configure(RTcontrol, fluorLabel.mWavelength_nm);
		virtualLaser.setPower(fluorLabel.mScanPmin);

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//To make sure that the RS is running

		//SCANNERS. Keep them fixed at 0
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, 0 };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, 0, &virtualLaser };

		//STAGES
		Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
		stage.moveXYZ({ stackCenterXYZ.XX, stackCenterXYZ.YY, stackCenterXYZ.ZZ });					//Move the stage to the initial position
		stage.waitForMotionToStopAll();
		Sleep(500);																					//Give the stages enough time to settle at the initial position
		stage.setVelSingle(Stage::Axis::XX, pixelSizeX / g_lineclockHalfPeriod);					//Set the vel for imaging

		virtualLaser.openShutter();	//Open the shutter. The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		const int nLocations{ static_cast<int>(stagePositionY.size()) };
		QuickStitcher stitchedImage{ stitchedHeight_pix, tileWidth_pix, {1, nCol} };
		double stageXi, stageXf;		//Stage final position
		for (int iterLocation = 0; iterLocation < nLocations; iterLocation++)
		{
			stageXi = determineInitialScanPos(stackCenterXYZ.XX - stitchedHeight / 2., stitchedHeight, 1. * mm, iterScanDirX);
			stageXf = determineFinalScanPos(stackCenterXYZ.XX - stitchedHeight / 2., stitchedHeight, 1. * mm, iterScanDirX);

			std::cout << "Frame: " << iterLocation + 1 << "/" << nLocations << "\n";
			stage.moveXY({ stageXi, stagePositionY.at(iterLocation) });
			stage.waitForMotionToStopAll();

			Sleep(300);					//Avoid iterations too close to each other, otherwise the X-stage will fail to trigger the ctl&acq sequence.
										//This might be because of g_postSequenceTimer

			RTcontrol.initialize();
			std::cout << "Scanning the stack...\n";
			stage.moveSingle(Stage::XX, stageXf);							//Move the stage to trigger the ctl&acq sequence
			RTcontrol.downloadData();

			Image image{ RTcontrol };
			image.acquireVerticalStrip(iterScanDirX);
			image.correctRSdistortion(tileWidth);							//Correct the image distortion induced by the nonlinear scanning of the RS

			TiffU8 tmp{ image.data(), stitchedHeight_pix, tileWidth_pix };	//I tried to access mTiff in image directly but it gives me an error
			stitchedImage.push(tmp, { 0, iterLocation });						//for now, only allowed to stack up strips to the right

			reverseSCANDIR(iterScanDirX);
			pressESCforEarlyTermination();
		}
			const std::string filename{ currentSample.mName + "_" + virtualLaser.currentLaser_s(true) + toString(fluorLabel.mWavelength_nm, 0) + "nm_P=" + toString(fluorLabel.mScanPmin / mW, 1) +
				"mWpum_xi=" + toString(stageXi / mm, 3) + "_xf=" + toString(stageXf / mm, 3) +
				"_yi=" + toString(stagePositionY.front() / mm, 3) + "_yf=" + toString(stagePositionY.back() / mm, 3) +
				"_z=" + toString(stackCenterXYZ.ZZ / mm, 4) + "_Step=" + toString(pixelSizeX / mm, 4) };
			std::cout << "Saving the stack...\n";
			stitchedImage.saveToFile(filename, OVERRIDE::DIS);
	}

	//Full sequence to image and cut an entire sample automatically
	//Note that the stack starts at stackCenterXYZ.at(Z) (i.e., the stack is not centered at stackCenterXYZ.at(Z))
	void sequencer(const FPGA &fpga, const bool run)
	{
		//for beads, center the stack around stackCenterXYZ.at(Z) -----> //const double sampleSurfaceZ{ stackCenterXYZ.ZZ - nFramesCont * pixelSizeZ / 2 };

		//ACQUISITION SETTINGS
		const double stackDepth{ 100. * um };
		const double pixelSizeZafterBinning{ 1.0 * um };												//Step size in the Z-stage axis
		
		const int nFramesAfterBinning{ static_cast<int>(stackDepth / pixelSizeZafterBinning) };
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };			//Full FOV in the (slow axis, fast axis)
		const SAMPLESIZE3 sampleSize{ 0.3 * mm, 0.2 * mm, 0.00 * mm };
		const TILEOVERLAP3 stackOverlap_frac{ 0.10, 0.05, 0.40 };										//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };													//height to cut above the bottom of the stack
		const double sampleSurfaceZ{ stackCenterXYZ.ZZ };

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
		const Sample sample{ currentSample, {stackCenterXYZ.XX, stackCenterXYZ.YY}, sampleSize, sampleSurfaceZ, cutAboveBottomOfStack };
		const Stack stack{ FFOV, pixelSizeZafterBinning, nFramesAfterBinning, stackOverlap_frac };
		Sequencer sequence{ sample, stack };
		sequence.generateCommandList();
		sequence.printToFile("Commandlist");

		if (run)
		{
			//STAGES
			Stage stage{ 5 * mmps, 5 * mmps, 0.5 * mmps, currentSample.mStageSoftPosLimXYZ };
			stage.moveSingle(Stage::ZZ, sample.mSurfaceZ);		//Move the Z-stage to the sample surface
			stage.waitForMotionToStopAll();

			//LASER
			VirtualLaser virtualLaser;

			//CONTROL SEQUENCE
			RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::STAGEZ, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix };

			//Read the commands line by line
			POSITION2 tileCenterXY;
			std::string longName;
			SCANDIR scanDirZ{ SCANDIR::UPWARD };
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.size(); iterCommandline++)
			{
				Sequencer::Commandline commandline{ sequence.readCommandline(iterCommandline) };

				//SCANNERS
				const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslowPerBeamlet / 2. };
				Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslowPerBeamlet / 2. };

				//These parameters must be accessible to all the cases
				int wavelength_nm, nFramesBinning;
				double scanZi, scanZf, scanPi, scanPf;
				switch (commandline.mAction)
				{
				case Action::ID::MOV://Move the X and Y-stages to mStackCenterXY
					tileCenterXY = commandline.mParam.moveStage.mTileCenterXY;
					stage.moveXY(tileCenterXY);
					stage.waitForMotionToStopAll();
					break;
				case Action::ID::ACQ://Acquire a stack
					Action::AcqStack acqStack{ commandline.mParam.acqStack };
					{
						//Set the number of frames considering that binning will be performed
						nFramesBinning = acqStack.nFrameBinning;
						const int nFramesBeforeBinning{ nFramesAfterBinning * nFramesBinning };													
						RTcontrol.setNumberOfFrames(nFramesBeforeBinning);

						//Set the vel for imaging. Frame duration (i.e., a galvo swing) = halfPeriodLineclock * heightPerBeamletPerFrame_pix	
						const double pixelSizeZbeforeBinning{ stackDepth / nFramesBeforeBinning };
						stage.setVelSingle(Stage::Axis::ZZ, pixelSizeZbeforeBinning / (g_lineclockHalfPeriod * heightPerBeamletPerFrame_pix));
					}

					//Save the parameters for saving the file
					wavelength_nm = acqStack.mWavelength_nm;
					scanPi = determineInitialLaserPower(acqStack.mScanPmin, stackDepth * acqStack.mScanPinc, scanDirZ);
					scanPf = determineFinalLaserPower(acqStack.mScanPmin, stackDepth * acqStack.mScanPinc, scanDirZ);
					scanZi = determineInitialScanPos(acqStack.mScanZmin, stackDepth, 0. * mm, scanDirZ);
					scanZf = determineFinalScanPos(acqStack.mScanZmin, stackDepth, 0. * mm, scanDirZ);

					//Update the laser parameters
					virtualLaser.configure(RTcontrol, wavelength_nm);		//The uniblitz shutter is closed by the pockels destructor when switching wavelengths
					virtualLaser.setPower(scanPi, scanPf);
					virtualLaser.openShutter();								//Re-open the Uniblitz shutter if closed by the pockels destructor
					rescanner.reconfigure(&virtualLaser);					//The calibration of the rescanner depends on the laser and wavelength being used

					RTcontrol.initialize(scanDirZ);							//Use the scan direction determined dynamically
					std::cout << "Scanning the stack...\n";
					stage.moveSingle(Stage::ZZ, scanZf);					//Move the stage to trigger the ctl&acq sequence
					RTcontrol.downloadData();

					reverseSCANDIR(scanDirZ);
					break;
				case Action::ID::SAV:
					{
					longName = virtualLaser.currentLaser_s(true) + toString(wavelength_nm, 0) + "nm_Pi=" + toString(scanPi / mW, 1) + "mW_Pf=" + toString(scanPf / mW, 1) + "mW" +
						"_x=" + toString(tileCenterXY.XX / mm, 3) +
						"_y=" + toString(tileCenterXY.YY / mm, 3) +
						"_zi=" + toString(scanZi / mm, 4) + "_zf=" + toString(scanZf / mm, 4) +
						"_Step=" + toString(pixelSizeZafterBinning / mm, 4) + "_bin=" + toString(nFramesBinning, 0);

						Image image{ RTcontrol };
						image.acquire();
						image.binFrames(nFramesBinning);
						image.save(longName, TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
						std::cout << "\n";
					}
					break;
				case Action::ID::CUT://Move the stage to the vibratome and then cut a slice off
					POSITION3 stagePositionXYZ{ commandline.mParam.cutSlice.mBladePositionXY };
					//IMPLEMENT THE SLICING HERE!
					break;
				default:
					throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected action invalid");
				}//switch

				//Stopwatch
				//auto t_start{ std::chrono::high_resolution_clock::now() };
				//double duration{ std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count() };
				//std::cout << "Elapsed time: " << duration << " ms" << "\n";				
			}//for
		}//if
		pressAnyKeyToCont();
	}

	//Image the sample non-stop. Use the PI program to move the stages manually
	void liveScan(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const FluorLabelList::FluorLabel fluorLabel{ currentSample.findFluorLabel("TDT") };	//Select a particular fluorescence channel
		const int nFramesCont{ 1 };															//Number of frames for continuous acquisition
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };							//Full FOV in the slow axis

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//LASER
		VirtualLaser virtualLaser{ Laser::ID::VISION };
		virtualLaser.configure(RTcontrol, fluorLabel.mWavelength_nm);

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//To make sure that the RS is running

		//SCANNERS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslow / 2. };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslow / 2., &virtualLaser };

		//OPEN THE UNIBLITZ SHUTTERS
		virtualLaser.openShutter();				//The destructor will close the shutter automatically

		while (true)
		{
			virtualLaser.setPower(fluorLabel.mScanPmin);					//Set the laser power

			RTcontrol.run();
			Image image{ RTcontrol };
			image.acquire();												//Execute the control sequence
			image.averageFrames();											//Average the frames acquired via continuous acquisition
			//image.correct(RScanner.mFFOV);
			image.save("Untitled", TIFFSTRUCT::SINGLEPAGE, OVERRIDE::EN);	//Save individual files
			Sleep(700);

			pressESCforEarlyTermination();									//Early termination if ESC is pressed
		}
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

		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

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

		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, timeStep, 10 * V);						//Initial pulse
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, timeStep, 0);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANNER, 4 * us, delay, 0, 5 * V, OVERRIDE::DIS);	//Linear ramp to accumulate the error
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, timeStep, 10 * V);						//Initial pulse
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, timeStep, 0);							//Final pulse

		//DO0
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, delay, 0);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
	}

	void pixelclock(const FPGA &fpga)
	{
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::STAGEX, FIFOOUTfpga::DIS, 560, 300, 1 }; 	//Create the control sequence
		RTcontrol.run();																				//Execute the control sequence
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGA &fpga)
	{
		const double timeStep{ 400. * us };

		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, timeStep, 0);
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGA &fpga)
	{
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

		//DO
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 0);

		//AO
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, 8 * us, 4 * V);
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, 4 * us, 2 * V);
		RTcontrol.pushAnalogSinglet(RTcontrol::RTCHAN::SCANNER, 4 * us, 1 * V);

		RTcontrol.mFpga.triggerControlSequence();	//Execute the control sequence
	}

	void analogRamp(const FPGA &fpga)
	{
		const double Vmax{ 5. * V };
		const double step{ 4. * us };

		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANNER, step, 2 * ms, 0, -Vmax, OVERRIDE::DIS);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANNER, step, 20 * ms, -Vmax, Vmax, OVERRIDE::DIS);
		RTcontrol.pushLinearRamp(RTcontrol::RTCHAN::SCANNER, step, 2 * ms, Vmax, 0, OVERRIDE::DIS);

		const double pulsewidth(300. * us);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, pulsewidth, 1);
		RTcontrol.pushDigitalSinglet(RTcontrol::RTCHAN::DODEBUG, 4 * us, 0);
	}

	//I think this is for matching the galvos forward and backward scans via imaging beads
	void fineTuneScanGalvo(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 400 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 30 };												//Number of frames for continuous acquisition
		const POSITION3 stagePositionXYZ{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };
		stage.moveXYZ(stagePositionXYZ);
		stage.waitForMotionToStopAll();

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//To make sure that the RS is running

		//SCANNERS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };					//Full FOV in the slow axis
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslow / 2. };

		//LASER
		const int wavelength_nm{ 1040 };
		const double laserPower{ 25. * mW };		//Laser power
		VirtualLaser virtualLaser{ Laser::ID::FIDELITY };
		virtualLaser.configure(RTcontrol, wavelength_nm);
		virtualLaser.setPower(laserPower);

		//CONTROL SEQUENCE
		virtualLaser.openShutter();	//Open the uniblitz shutter. The destructor will close the shutter automatically
		RTcontrol.run();
		Image image{ RTcontrol };
		image.acquire();			//Execute the control sequence and acquire the image via continuous acquisition
		image.averageEvenOddFrames();
		image.save("Untitled", TIFFSTRUCT::MULTIPAGE, OVERRIDE::DIS);
	}

	void resonantScanner(const FPGA &fpga)
	{
		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

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
		const int heightPerFrame_pix{ 35 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 1 };

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::DIS, heightPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//SCANNERS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Scan duration in the slow axis
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslow / 2. };
		Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslow / 2. };

		//Execute the control sequence and acquire the image
		RTcontrol.run();
		pressAnyKeyToCont();
	}

	//Test the synchronization of the 2 galvos and the laser
	void gavosLaserSync(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int heightPerFrame_pix{ 560 };
		const int widthPerFrame_pix{ 300 };
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
		const double stackDepthZ{ 20. * um };	//Acquire a stack this deep in the Z-stage axis

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::EN, heightPerBeamletPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//LASER
		const int wavelength_nm{ 750 };
		VirtualLaser virtualLaser{ Laser::ID::VISION };
		virtualLaser.configure(RTcontrol, wavelength_nm);
		virtualLaser.setPower(selectPower);

		//SCANNERS
		const Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslowPerBeamlet / 2. };
		const Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslowPerBeamlet / 2., &virtualLaser };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANNER, 0, wavelength_nm };

		//EXECUTE THE CONTROL SEQUENCE
		RTcontrol.run();
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

		//std::cout << "Stages initial position: \n";
		//stage.printPositionXYZ();
		std::cout << "Before vel: \n";
		stage.setVelSingle(Stage::Axis::ZZ, 0.4 * mmps);

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
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

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
		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

		//DEFINE THE POCKELS CELLS
		PockelsCell pockelsVision{ RTcontrol, 750, Laser::ID::VISION };
		PockelsCell pockelsFidelity{ RTcontrol, 1040, Laser::ID::FIDELITY };

		PockelsCell pockels{ pockelsFidelity };
		//PockelsCell pockels{ pockelsFidelity };
		pockels.pushPowerSinglet(8 * us, 100. * mW, OVERRIDE::DIS);
		//pockels.pushPowerSinglet(8 * us, 0 * mW, OVERRIDE::DIS);
		//pockels.pushVoltageSinglet(8 * us, 2.0 * V, OVERRIDE::DIS);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		RTcontrol.run();
		//pressAnyKeyToCont();
	}

	void pockelsRamp(const FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int heightPerFrame_pix{ 35 };
		const int widthPerFrame_pix{ 300 };
		const int nFramesCont{ 200 };						//Number of frames for continuous acquisition

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::DIS, heightPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//POCKELS CELL
		const int wavelength_nm{ 1040 };
		PockelsCell pockels{ RTcontrol, wavelength_nm, Laser::ID::FIDELITY };
		const double Pi{ 192. * mW }, Pf{ 336. * mW };
		//const double Pi{ 30. * mW }, Pf{ 60. * mW };
		//pockels.pushPowerSinglet(400 * us, Pf, OVERRIDE::EN);
		pockels.powerLinearScaling(Pi, Pf);					//Linearly scale the laser power from the first to the last frame
		//pockels.powerLinearScaling(0.96 * Pf, Pi);

		//Test the voltage setpoint
		//pockels.pushVoltageSinglet(8* us, 0.5 * V);
		//pockels.voltageLinearRamp(0.5 * V, 1.0 * V);		//Linearly scale the pockels voltage from the first to the last frame

		//EXECUTE THE CONTROL SEQUENCE
		RTcontrol.run();
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
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG , MAINTRIG::PC, FIFOOUTfpga::DIS, 560, 300, 1 };

		const int wavelength_nm{ 1040 };
		const double laserPower{ 50. * mW };
		VirtualLaser virtualLaser{ Laser::ID::VISION };
		virtualLaser.configure(RTcontrol, wavelength_nm);
		virtualLaser.setPower(laserPower);
	}

	//Photobleach a line along the fast axis (RS) on the sample
	void photobleach(const FPGA &fpga)
	{
		Laser laser{ Laser::ID::VISION };
		laser.setWavelength(920);

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::DIS, 400, 300, 100 };

		//RS
		ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//To make sure that the RS is running

		//SCANNERS. Keep the scanner fixed to bleach a line on the sample
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, 0 };

		//POCKELS CELLS
		PockelsCell pockels{ RTcontrol, 920, Laser::ID::VISION };
		pockels.pushPowerSinglet(8 * us, 200 * mW, OVERRIDE::DIS);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		pockels.setShutter(true);
		RTcontrol.run();

		//Wait until the sequence is over to close the shutter, otherwise this code will finish before the sequence
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
		const int height{ 560 };
		const int width{ 300 };
		const int nFramesCont{ 200 };

		TiffU8 image{ height, width, nFramesCont };

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

		std::string inputFilename{ "Liver20190812_02_F1040nm_P=800.0mW_Pinc=1.60mWpum_x=44.000_y=21.000_zi=16.6200_zf=16.7300_Step=0.0010_bin=4 (1)" };
		std::string outputFilename{ "output_" + inputFilename };
		TiffU8 image{ inputFilename };
		//image.correctFOVslowCPU(1);
		//image.correctRSdistortionGPU(200. * um);	
		image.flattenField(2.0, 4, 11);
		image.suppressCrosstalk(0.2);
		image.saveToFile(outputFilename, TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);

		//image.binFrames(5);
		//image.splitFrames(10);
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
		//pressAnyKeyToCont();
	}

	void quickStitcher()
	{
		//RTcontrol RTcontrol{ fpga };
		//Image image{ RTcontrol };
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
		TiffU8 image00{ "Tile_01" };
		TiffU8 image01{ "Tile_02" };
		QuickStitcher stitched{ height, width, {2, 1} };
		stitched.push(image00, { 0, 0 });
		stitched.push(image01, { 1, 0 });
		stitched.saveToFile(outputFilename, OVERRIDE::DIS);

		pressAnyKeyToCont();
	}

	void thresholdSample()
	{
		std::string inputFilename{ "aaa" };
		std::string outputFilename{ "output" };

		const double threshold{ 0.03 };
		const int tileHeight_pix{ 560 };
		const int tileWidth_pix{ 300 };
		TiffU8 image{ inputFilename };

		const TileArray tileArray{ {image.heightPerFrame_pix() / 2, image.widthPerFrame_pix() / 2}, tileHeight_pix, tileWidth_pix, { 36, 67 }, {0.0, 0.0, 0.0} };

		BoolMap boolmap{ image, tileArray, threshold };
		//boolmap.saveTileMapToText("Boolmap");
		//boolmap.saveTileMap("TileMap", OVERRIDE::EN);
		boolmap.SaveTileGridOverlap("TileGrid", OVERRIDE::EN);
		//std::cout << boolmap.isTileBright({ 0, 30 }) << "\n";

		//PIXELS2 positionXY_pix = boolmap.tileIndicesIJtoPixelPosition_pix({0.0, 0.0, 0.0},  { 35, 66 });
		//std::cout << positionXY_pix.ii << "\t" << positionXY_pix.jj<< "\n";

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
		const int heightPerFrame_pix{ 400 };
		const int widthPerFrame_pix{ 300 };
		const FFOV2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };
		const int nFramesCont{ 80 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };										//Step size in the Z-stage axis
		const TILEOVERLAP3 stackOverlap_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };							//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };								//Sample thickness
		const double sampleSurfaceZ{ 18.471 * mm };

		Sample sample{ currentSample,  {stackCenterXYZ.XX, stackCenterXYZ.YY}, {10. * mm, 10. * mm, sampleLengthZ}, sampleSurfaceZ, cutAboveBottomOfStack };
		Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlap_frac };

		//Create a sequence
		Sequencer sequence{ sample, stack };
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
				Sequencer::Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
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
	/*
	void locationSequence()
	{
		//ACQUISITION SETTINGS
		const FFOV2 FFOV{ 200. * um, 150. * um };
		const int nDiffZ{ 100 };											//Number of frames for continuous acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };									//Step size in the Z-stage axis
		const TILEOVERLAP3 stackOverlap_frac{ 0.05, 0.05, 0.05 };			//Stack overlap
		const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

		//Create a sequence
		Sequencer sequence{ currentSample, stack, {stackCenterXYZ.XX, stackCenterXYZ.YY}, { 2, 2 } }; //Last 2 parameters: stack center and number of stacks
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
		const INDICES2 tileSizeIJ{ 2, 2 };
		const int tileShiftX_pix{ 543 };
		const int tileShiftY_pix{ 291 };

		Logger datalog(currentSample.mName + "_locations");
		datalog.record("dim=3"); //Needed for the BigStitcher

		for (int tileNumber = 0; tileNumber < tileSizeIJ.II * tileSizeIJ.JJ; tileNumber++)
			//for (int tileNumber = 0; tileNumber < 180; tileNumber++)
		{
			INDICES2 tileIndicesIJ = tileNumberToIndicesIJ(tileNumber);
			int totalTileShiftX_pix{ -tileIndicesIJ.II * tileShiftX_pix };
			int TotalTileShiftY_pix{ -tileIndicesIJ.JJ * tileShiftY_pix };
			std::string line{ std::to_string(tileNumber) + ";;(" + std::to_string(TotalTileShiftY_pix) + "," + std::to_string(totalTileShiftX_pix) + ",0)" };	//In BigStitcher, X is horizontal and Y is vertical
			//std::string line{ std::to_string(tileNumber) + "\t" + std::to_string(tileNumberToIndicesIJ(tileNumber).at(X)) + "\t" + std::to_string(tileNumberToIndicesIJ(tileNumber).at(Y)) }; //For debugging
			datalog.record(line);
		}
	}

	//Snake pattern starting from the bottom right of the sample and going up
	INDICES2 tileNumberToIndicesIJ(const int tileNumber)
	{
		const INDICES2 tileSizeIJ{ 2, 2 };

		int II;
		int JJ{ tileNumber / tileSizeIJ.II };

		if (JJ % 2)	//JJ is odd
			II = tileSizeIJ.II - tileNumber % tileSizeIJ.II - 1;
		else		//JJ is even
			II = tileNumber % tileSizeIJ.II;

		return { II, JJ };
	}

	void PMT16Xconfig()
	{
		PMT16X PMT;

		//pmt.setSingleGain(PMT16XCHAN::CH00, 170);
		//PMT.setAllGains(255);
		//PMT.readTemp();
		PMT.readAllGains();
		
		//PMT.suppressGainsLinearly(0.4, RTcontrol::PMT16XCHAN::CH05, RTcontrol::PMT16XCHAN::CH10);
		
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

		pressAnyKeyToCont();
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

		//CREATE THE CONTROL SEQUENCE
		RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, FIFOOUTfpga::EN, heightPerFrame_pix, widthPerFrame_pix, nFramesCont };

		//LASER
		const double laserPower{ 30. * mW };
		VirtualLaser virtualLaser{ Laser::ID::VISION };
		virtualLaser.configure(RTcontrol, wavelength_nm);
		virtualLaser.setPower(laserPower);

		//SCANNERS
		Galvo scanner{ RTcontrol, RTcontrol::RTCHAN::SCANNER, FFOVslow / 2. };
		Galvo rescanner{ RTcontrol, RTcontrol::RTCHAN::RESCANNER, FFOVslow / 2., &virtualLaser };
		//Galvo scanner{ RTcontrol, RTCHAN::SCANNER, 0 };				//Keep the scanner fixed to see the emitted light swing across the PMT16X channels. The rescanner must be centered

		//EXECUTE THE CONTROL SEQUENCE
		RTcontrol.run();
		Image image{ RTcontrol };
		image.acquire();
		image.save("SingleChannel", TIFFSTRUCT::MULTIPAGE, OVERRIDE::EN);
	}

	void vibratome(const FPGA &fpga)
	{
		const double slicePlaneZ{ (18.000) * mm };

		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps , ContainerPosLimit };
		Vibratome vibratome{ fpga, stage };
		vibratome.slice(slicePlaneZ);

		const POSITION3 samplePosition{ 10. * mm, 23. * mm, slicePlaneZ };
		stage.moveXYZ(samplePosition);
		//stage.moveXYZ(stackCenterXYZ);
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