#include "Routines.h"

//SAMPLE PARAMETERS
const double3 stackCenterXYZ{ 52.870 * mm, 17.000 * mm, 18.074 * mm };//Beads 68, 74
//const double3 stackCenterXYZ{ 50.000 * mm, -7.000 * mm, 18.110 * mm };//Fluorescent slide
const std::string sampleName{ "Beads4um" };
const std::string immersionMedium{ "SiliconeOil" };
const std::string collar{ "1.51" };
const ChannelList channelListBeads{ {{ "DAPI", 750, 45. * mW, 0. * mWpum }, { "GFP", 920, 45. * mW, 0. * mWpum }, { "TDT", 1040, 15. * mW, 0. * mWpum }} };	//4um beads
//const ChannelList channelListBeads{ {{ "DAPI", 750, 40. * mW, 0. * mWpum }, { "GFP", 920, 40. * mW, 0. * mWpum }, { "TDT", 1040, 15. * mW, 0. * mWpum }} };	//0.5um beads
//const ChannelList channelListLiver{ {{ "TDT", 1040, 80. * mW, 0.0 * mWpum } , { "GFP", 920, 80. * mW, 0.4 * mWpum }, { "DAPI", 750, 7. * mW, 0.15 * mWpum }} };
const ChannelList channelListFluorSlide { { { "DAPI", 750, 10. * mW, 0. * mWpum }} };	//Fluorescent slide
const ChannelList channelList{ channelListBeads };
//const ChannelList channelList{ channelListFluorSlide };

namespace PMT1XRoutines
{
	//The "Swiss knife" of my routines
	void frameByFrameScan(const FPGAns::FPGA &fpga)
	{
		//Each of the following modes can be used under 'continuous XY acquisition' by setting nFramesCont > 1, meaning that the galvo is scanned back and
		//forth on the same z plane. The images the can be averaged
		//const RUNMODE acqMode{ RUNMODE::SINGLE };			//Single shot. Image the same z plane continuosly 'nFramesCont' times and average the images
		//const RUNMODE acqMode{ RUNMODE::AVG };			//Image the same z plane frame by frame 'nSameZ' times and average the images
		//const RUNMODE acqMode{ RUNMODE::STACK };			//Image a stack frame by frame from the initial z position
		const RUNMODE acqMode{ RUNMODE::STACKCENTERED };	//Image a stack frame by frame centered at the initial z position

		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("DAPI") };	//Select a particular fluorescence channel
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 5 };				//Number of frames for continuous XY acquisition

		//STACK
		const double stepSizeZ{ 1.0 * um };
		const double stackDepthZ{ 100. * um };	//Acquire a stack this deep in Z

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps};
		std::vector<double3> stagePositionXYZ;

		int nDiffZ;		//Number of frames at different Zs
		int nSameZ;		//Number of frames at the same Z
		OVERRIDE override;
		switch (acqMode)
		{
		case RUNMODE::SINGLE:
			nSameZ = 1;
			nDiffZ = 1; //Do not change this
			override = OVERRIDE::DIS;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::AVG:
			nSameZ = 10;
			nDiffZ = 1; //Do not change this
			override = OVERRIDE::DIS;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::STACK:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			override = OVERRIDE::DIS;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) + iterDiffZ * stepSizeZ });
			break;
		case RUNMODE::STACKCENTERED:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			override = OVERRIDE::DIS;
			//Generate the discrete scan sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };

		//LASER
		const VirtualLaser laser{ RTcontrol, singleChannel.mWavelength_nm, LASER::AUTO };

		//DATALOG
		{
			Logger datalog("datalog_" + sampleName);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", sampleName);
			datalog.record("Immersion medium = ", immersionMedium);
			datalog.record("Correction collar = ", collar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", tickPerUs);
			datalog.record("\nLASER--------------------------------------------------------");
			datalog.record("Laser wavelength (nm) = ", singleChannel.mWavelength_nm);
			datalog.record("Laser power first frame (mW) = ", singleChannel.mScanPi / mW);
			datalog.record("Laser power increase (mW/um) = ", singleChannel.mStackPinc / mWpum);
			datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleFactorU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", RTcontrol.mHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (FFOVslow / um) / RTcontrol.mHeightPerFrame_pix);
			datalog.record("\nSTAGE--------------------------------------------------------");
		}

		//CREATE A STACK FOR STORING THE TIFFS
		TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, nSameZ };

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
		{
			stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
			stage.waitForMotionToStopAll();
			stage.printPositionXYZ();		//Print the stage position		

			//Acquire many frames at the same Z via discontinuous acquisition
			for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
			{
				std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
					"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << "\n";

				laser.setPower(singleChannel.mScanPi + iterDiffZ * stepSizeZ * singleChannel.mStackPinc);	//Update the laser power

				//EXECUTE THE RT CONTROL SEQUENCE
				Image image{ RTcontrol };
				image.acquire();			//Execute the RT control sequence and acquire the image
				image.averageFrames();		//Average the frames acquired via continuous XY acquisition
				tiffStack.pushSameZ(iterSameZ, image.pointerToTiff());

				if (acqMode == RUNMODE::SINGLE)
				{
					//Save individual files
					std::string singleFilename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_P=" + toString(singleChannel.mScanPi / mW, 1) + "mW" +
						"_x=" + toString(stagePositionXYZ.at(iterDiffZ).at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.at(iterDiffZ).at(YY) / mm, 3) + "_z=" + toString(stagePositionXYZ.at(iterDiffZ).at(ZZ) / mm, 4) };
					image.saveTiffSinglePage(singleFilename, override);
					Sleep(700);
				}
			}
			tiffStack.pushDiffZ(iterDiffZ);

			std::cout << "\n";
		}

		if (acqMode == RUNMODE::AVG || acqMode == RUNMODE::STACK || acqMode == RUNMODE::STACKCENTERED)
		{
			//Save the stackDiffZ to file
			std::string stackFilename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_Pi=" + toString(singleChannel.mScanPi / mW, 1) + "mW_Pinc=" + toString(singleChannel.mStackPinc / mWpum, 1) + "mWpum" +
				"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
			tiffStack.saveToFile(stackFilename, override);
		}
	}

	//Apply 'frameByFrameScan' on a list of locations. I don't use continuous z-scan because of its limited reach (160 planes)
	void frameByFrameScanTiling(const FPGAns::FPGA &fpga, const int nSlice)
	{
		//ACQUISITION SETTINGS
		const ChannelList channelList{ channelList };
		//const ChannelList channelList{ {channelListLiver.findChannel("DAPI")} };
		const int2 nStacksXY{ 3, 4 };
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 1 };				//Number of frames for continuous XY acquisition

		//STACK
		const double2 FFOV{ heightPerFrame_pix * pixelSizeXY , widthPerFrame_pix * pixelSizeXY };							//Full FOV in the (slow axis, fast axis)
		const double stepSizeZ{ 0.5 * um };									//Step size in z
		const double stackDepthZ{ 100. * um };								//Acquire a stack this deep in Z
		const int nDiffZ{ static_cast<int>(stackDepthZ / stepSizeZ) };		//Number of frames at different Zs
		const double3 stackOverlap_frac{ 0.03, 0.03, 0.03 };				//Stack overlap
		const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOV.at(XX) / 2 };

		//LASER
		VirtualLaser laser{ RTcontrol, channelList.front().mWavelength_nm };

		//Create a location list
		Sequencer sequence{ channelList, Sample(sampleName, immersionMedium, collar), stack, stackCenterXYZ, nStacksXY };
		std::vector<double2> locationXYList{ sequence.generateLocationList() };

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//Iterate over the wavelengths
		for (std::vector<int>::size_type iter_wv = 0; iter_wv < channelList.size(); iter_wv++)
		{
			//DATALOG
			Logger datalog("datalog_Slice" + std::to_string(nSlice) + "_" + channelList.at(iter_wv).mName);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", sampleName);
			datalog.record("Immersion medium = ", immersionMedium);
			datalog.record("Correction collar = ", collar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", tickPerUs);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOV.at(XX) / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleFactorU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", RTcontrol.mHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (FFOV.at(XX) / um) / RTcontrol.mHeightPerFrame_pix);
			datalog.record("\n");

			//Update the laser wavelength
			const int wavelength_nm = channelList.at(iter_wv).mWavelength_nm;
			laser.setWavelength(wavelength_nm);	//When switching pockels, the pockels destructor closes the uniblitz shutter
			laser.openShutter();				//Re-open the Uniblitz shutter if closed

			//Iterate over the locations
			for (std::vector<int>::size_type iter_loc = 0; iter_loc < locationXYList.size(); iter_loc++)
			{
				//Generate the discrete scan sequence for the stages
				std::vector<double3> stagePositionXYZ;
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
					stagePositionXYZ.push_back({ locationXYList.at(iter_loc).at(XX), locationXYList.at(iter_loc).at(YY), stackCenterXYZ.at(ZZ) + iterDiffZ * stepSizeZ });
				
				//CREATE A STACK FOR STORING THE TIFFS
				TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, 1 };

				//ACQUIRE FRAMES AT DIFFERENT Zs
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				{
					//Update the stages position
					stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
					stage.waitForMotionToStopAll();
					//stage.printPositionXYZ();		//Print the stage position		

					std::cout << "Location: " << iter_loc + 1 << "/" << locationXYList.size() << "\tTotal frame: " << iterDiffZ + 1 << "/" << nDiffZ << "\n";

					//Update the laser power
					laser.setPower(channelList.at(iter_wv).mScanPi + iterDiffZ * stepSizeZ * channelList.at(iter_wv).mStackPinc);

					//EXECUTE THE RT CONTROL SEQUENCE
					Image image{ RTcontrol };
					image.acquire();			//Execute the RT control sequence and acquire the image
					image.averageFrames();		//Average the frames acquired via continuous XY acquisition
					tiffStack.pushSameZ(0, image.pointerToTiff());
					tiffStack.pushDiffZ(iterDiffZ);
					std::cout << "\n";
					
					pressESCforEarlyTermination();		//Early termination if ESC is pressed
				}

				//Save the stackDiffZ to file
				std::string shortName{ "Slice" + std::to_string(nSlice) + "_" + channelList.at(iter_wv).mName + "_Tile" + std::to_string(iter_loc + 1) };
				std::string longName{ sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(channelList.at(iter_wv).mScanPi / mW, 1) + "mW_Pinc=" + toString(channelList.at(iter_wv).mStackPinc / mWpum, 1) + "mWpum" +
					"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
					"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };

				datalog.record(shortName + "\t" + longName);
				tiffStack.saveToFile(shortName, OVERRIDE::DIS);
			}//iter_loc
		}//iter_wv
	}

	//Image the sample non-stop. Use the PI program to move the stages around manually
	void liveScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("DAPI") };	//Select a particular fluorescence channel
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 1 };				//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };

		//LASER
		const VirtualLaser laser{ RTcontrol, singleChannel.mWavelength_nm, LASER::AUTO };

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		while (true)
		{
			laser.setPower(singleChannel.mScanPi);	//Set the laser power

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };
			image.acquire();								//Execute the RT control sequence and acquire the image
			image.averageFrames();							//Average the frames acquired via continuous XY acquisition
			image.saveTiffSinglePage("Untitled", OVERRIDE::EN);	//Save individual files
			Sleep(700);

			pressESCforEarlyTermination();		//Early termination if ESC is pressed
		}
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
	void continuousScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("GFP") };	//Select a particular laser
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 160 };				//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };
		const ZSCAN scanDirZ{ ZSCAN::TOPDOWN };		//Scan direction in z
		const double stackDepth{ nFramesCont * stepSizeZ };

		double stageZi, stageZf, laserPi, laserPf;
		switch (scanDirZ)
		{
		case ZSCAN::TOPDOWN:
			stageZi = stackCenterXYZ.at(ZZ);
			stageZf = stackCenterXYZ.at(ZZ) + stackDepth;
			laserPi = singleChannel.mScanPi;
			laserPf = singleChannel.mScanPi + stackDepth * singleChannel.mStackPinc;
			break;
		case ZSCAN::BOTTOMUP:
			stageZi = stackCenterXYZ.at(ZZ) + stackDepth;
			stageZf = stackCenterXYZ.at(ZZ);
			laserPi = singleChannel.mScanPi + stackDepth * singleChannel.mStackPinc;
			laserPf = singleChannel.mScanPi;
			break;
		}

		//STAGES
		const double3 initialStageXYZ{ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stageZi };		//Initial position of the stages. The sign of stackDepth determines the scanning direction					
		Stage stage{ 5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix) };	//Specify the vel. Duration of a frame = a galvo swing = halfPeriodLineclock * heightPerFrame_pix
		stage.moveXYZ(initialStageXYZ);
		stage.waitForMotionToStopAll();

		//CREATE THE REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::ZSTAGE, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };	//Notice the ZSTAGE flag

		//LASER
		const VirtualLaser laser{ RTcontrol, singleChannel.mWavelength_nm, laserPi, LASER::VISION };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVO RT linear scan
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };

		//OPEN THE SHUTTER
		laser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.initialize(scanDirZ);
		std::cout << "Scanning the stack...\n";
		stage.moveSingle(ZZ, stageZf);	//Move the stage to trigger the control sequence and data acquisition
		image.downloadData();
		image.postprocess();

		const std::string filename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_P=" + toString((std::min)(laserPi, laserPf) / mW, 1) + "mW_Pinc=" + toString(singleChannel.mStackPinc / mWpum, 1) +
			"mWpum_x=" + toString(initialStageXYZ.at(XX) / mm, 3) + "_y=" + toString(initialStageXYZ.at(YY) / mm, 3) +
			"_zi=" + toString(stageZi / mm, 4) + "_zf=" + toString(stageZf / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
		image.saveTiffMultiPage(filename, OVERRIDE::DIS);
	}

	//Full sequence to image and cut an entire sample automatically
	//Note that the stack starts at stackCenterXYZ.at(ZZ). Therefore, the stack is not centered around stackCenterXYZ.at(ZZ).
	void sequencer(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const double2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };		//Full FOV in the (slow axis, fast axis)
		const int nFramesCont{ 160 };																	//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };																//Step size in z
		const ROI roi{ 11.000 * mm, 34.825 * mm, 11.180 * mm, 35.025 * mm };							//Region of interest {ymin, xmin, ymax, xmax}
		const double3 stackOverlap_frac{ 0.05, 0.05, 0.05 };											//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };													//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };														//Sample thickness
		const double sampleSurfaceZ{ stackCenterXYZ.at(ZZ) };

		//const ChannelList channelList{ channelList };
		const ChannelList channelList{ {channelList.findChannel("GFP")} };
		const Sample sample{ sampleName, immersionMedium, collar, roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		const Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlap_frac };

		//Create a sequence
		//Sequencer sequence{ channelList, sample, stack };
		Sequencer sequence{ channelList, Sample(sampleName, immersionMedium, collar), stack, stackCenterXYZ, { 2, 1 } }; //Last 2 parameters: stack center and number of stacks
		sequence.generateCommandList();
		sequence.printToFile("Commandlist");


		if (1)
		{
			//STAGES. Specify the velocity
			Stage stage{ 5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix) };
			stage.moveSingle(ZZ, sample.mSurfaceZ);	//Move the z stage to the sample surface

			//CREATE THE REALTIME CONTROL SEQUENCE
			FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::ZSTAGE, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };	//Notice the ZSTAGE flag

			//LASER: wavelength_nm, laserPower, whichLaser
			VirtualLaser laser{ RTcontrol, channelList.front().mWavelength_nm };

			//RS
			const ResonantScanner RScanner{ RTcontrol };
			RScanner.isRunning();		//Make sure that the RS is running

			//GALVO RT linear ramp	
			const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOV.at(XX) / 2 };

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };

			//Read the commands line by line
			double scanZi, scanZf, scanPi, stackPinc;
			double2 stackCenterXY;
			int wavelength_nm;
			ZSCAN scanDirZ;
			std::string longName;
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.mCommandCounter; iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
				//commandline.printParameters();

				//Stopwatch
				//auto t_start{ std::chrono::high_resolution_clock::now() };

				switch (commandline.mAction)
				{
				case ACTION::MOV:
					//Move the x and y stages to mStackCenterXY
					stackCenterXY = commandline.mCommand.moveStage.mStackCenterXY;
					stage.moveXY(stackCenterXY);
					stage.waitForMotionToStopAll();
					break;
				case ACTION::ACQ:
					//Acquire a stack using the parameters:
					AcqStack acqStack{ commandline.mCommand.acqStack };

					wavelength_nm = acqStack.mWavelength_nm;
					scanDirZ = static_cast<ZSCAN>(acqStack.mScanDirZ);
					scanZi = acqStack.mScanZi;
					scanZf = scanZi + static_cast<int>(scanDirZ) * acqStack.mStackDepth;
					scanPi = acqStack.mScanPi;
					stackPinc = acqStack.mStackPinc;

					//Update the laser parameters if needed
					laser.setWavelength(wavelength_nm);	//When switching pockels, the pockels destructor closes the uniblitz shutter
					laser.setPower(scanPi, static_cast<int>(scanDirZ) * stackPinc);
					laser.openShutter();	//Re-open the Uniblitz shutter if closed

					image.initialize(scanDirZ);
					std::cout << "Scanning the stack...\n";
					stage.moveSingle(ZZ, scanZf);		//Move the stage to trigger the control sequence and data acquisition
					image.downloadData();
					break;
				case ACTION::SAV:
					longName = toString(wavelength_nm, 0) + "nm_Pi=" + toString(scanPi / mW, 1) + "mW_Pf=" + toString((scanPi + static_cast<int>(scanDirZ) * stackPinc) / mW, 1) + "mW" +
						"_x=" + toString(stackCenterXY.at(XX) / mm, 3) + "_y=" + toString(stackCenterXY.at(YY) / mm, 3) +
						"_zi=" + toString(scanZi / mm, 4) + "_zf=" + toString(scanZf / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4);

					image.postprocess();
					image.saveTiffMultiPage(longName, OVERRIDE::DIS);
					break;
				case ACTION::CUT:
					//Move the stage to and then cut a slice off
					double3 stagePositionXYZ{ commandline.mCommand.cutSlice.mBladePositionXY };
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


}//namespace


namespace PMT16XRoutines
{
	//Copy of frameByFrameScan() with sync'ed rescanner
	void PMT16XframeByFrameScan(const FPGAns::FPGA &fpga)
	{
		//Each of the following modes can be used under 'continuous XY acquisition' by setting nFramesCont > 1, meaning that the galvo is scanned back and
		//forth on the same z plane. The images the can be averaged
		const RUNMODE acqMode{ RUNMODE::SINGLE };			//Single shot. Image the same z plane continuosly 'nFramesCont' times and average the images
		//const RUNMODE acqMode{ RUNMODE::AVG };			//Image the same z plane frame by frame 'nSameZ' times and average the images
		//const RUNMODE acqMode{ RUNMODE::STACK };			//Image a stack frame by frame from the initial z position
		//const RUNMODE acqMode{ RUNMODE::STACKCENTERED };	//Image a stack frame by frame centered at the initial z position

		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("TDT") };	//Select a particular fluorescence channel
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };	//35 for PMT16X
		const int nFramesCont{ 1 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int selectHeightPerFrame_pix;
		double selectScanFFOV, selectRescanFFOV, selectPower, selectPowerInc;
		PMT16XCHAN PMT16Xchan;

#if multibeam
			//Multibeam
			selectHeightPerFrame_pix = static_cast<int>(heightPerFrame_pix / 16);
			selectScanFFOV = FFOVslow / 16;
			selectRescanFFOV = FFOVslow / 16;
			PMT16Xchan = PMT16XCHAN::CH00;
			selectPower = 1000. * mW;
			selectPowerInc = 0;
#else
			//Singlebeam
			//For fluorescent slide, set selectScanFFOV = 0.0 and PMT16Xchan = PMT16XCHAN::CH00 to let the laser scan through the PMT16X channels
			selectHeightPerFrame_pix = heightPerFrame_pix;
			selectScanFFOV = FFOVslow;
			selectRescanFFOV = FFOVslow;
			PMT16Xchan = PMT16XCHAN::CH02;
			selectPower = singleChannel.mScanPi;
			selectPowerInc = singleChannel.mStackPinc;
#endif
		//STACK
		const double stepSizeZ{ 1.0 * um };
		const double stackDepthZ{ 20. * um };	//Acquire a stack this deep in Z

		//STAGES
		std::vector<double3> stagePositionXYZ;		
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		int nDiffZ;		//Number of frames at different Zs
		int nSameZ;		//Number of frames at the same Z
		OVERRIDE override;
		switch (acqMode)
		{
		case RUNMODE::SINGLE:
			nSameZ = 1;
			nDiffZ = 1; //Do not change this
			override = OVERRIDE::DIS;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::AVG:
			nSameZ = 10;
			nDiffZ = 1; //Do not change this
			override = OVERRIDE::DIS;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case RUNMODE::STACK:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			override = OVERRIDE::DIS;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) + iterDiffZ * stepSizeZ });
			break;
		case RUNMODE::STACKCENTERED:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			override = OVERRIDE::DIS;
			//Generate the discrete scan sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, selectHeightPerFrame_pix, FIFOOUT::EN, PMT16Xchan };

		//LASER
		const VirtualLaser laser{ RTcontrol,  singleChannel.mWavelength_nm, LASER::VISION };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, selectScanFFOV / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, selectRescanFFOV / 2, singleChannel.mWavelength_nm };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, 0, singleChannel.mWavelength_nm };

		//DATALOG
		{
			Logger datalog("datalog_" + sampleName);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", sampleName);
			datalog.record("Immersion medium = ", immersionMedium);
			datalog.record("Correction collar = ", collar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", tickPerUs);
			datalog.record("\nLASER--------------------------------------------------------");
			datalog.record("Laser wavelength (nm) = ", singleChannel.mWavelength_nm);
			datalog.record("Laser power first frame (mW) = ", selectPower / mW);
			datalog.record("Laser power increase (mW/um) = ", selectPowerInc / mWpum);
			datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOVslow / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleFactorU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", selectHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (selectScanFFOV / um) / selectHeightPerFrame_pix);
			datalog.record("\nSTAGE--------------------------------------------------------");
		}

		//CREATE A STACK FOR STORING THE TIFFS
		TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, nSameZ };

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		//ACQUIRE FRAMES AT DIFFERENT Zs
		for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
		{
			stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
			stage.waitForMotionToStopAll();
			stage.printPositionXYZ();		//Print the stage position		

			//Acquire many frames at the same Z via discontinuous acquisition
			for (int iterSameZ = 0; iterSameZ < nSameZ; iterSameZ++)
			{
				std::cout << "Frame # (diff Z): " << (iterDiffZ + 1) << "/" << nDiffZ << "\tFrame # (same Z): " << (iterSameZ + 1) << "/" << nSameZ <<
					"\tTotal frame: " << iterDiffZ * nSameZ + (iterSameZ + 1) << "/" << nDiffZ * nSameZ << "\n";

				laser.setPower(selectPower + iterDiffZ * stepSizeZ * selectPowerInc);	//Update the laser power

				//EXECUTE THE RT CONTROL SEQUENCE
				Image image{ RTcontrol };
				image.acquire();			//Execute the RT control sequence and acquire the image
				image.averageFrames();		//Average the frames acquired via continuous XY acquisition
				//image.averageEvenOddFrames();
				tiffStack.pushSameZ(iterSameZ, image.pointerToTiff());

				if (acqMode == RUNMODE::SINGLE)
				{
					//Save individual files
					std::string singleFilename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_P=" + toString(selectPower / mW, 1) + "mW" +
						"_x=" + toString(stagePositionXYZ.at(iterDiffZ).at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.at(iterDiffZ).at(YY) / mm, 3) + "_z=" + toString(stagePositionXYZ.at(iterDiffZ).at(ZZ) / mm, 4) };
					image.saveTiffMultiPage(singleFilename, override);
				}
			}
			tiffStack.pushDiffZ(iterDiffZ);
			std::cout << "\n";
		}

		if (acqMode == RUNMODE::AVG || acqMode == RUNMODE::STACK || acqMode == RUNMODE::STACKCENTERED)
		{
			//Save the stackDiffZ to file
			std::string stackFilename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_Pi=" + toString(selectPower / mW, 1) + "mW_Pinc=" + toString(selectPowerInc / mWpum, 1) + "mWpum" +
				"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
			tiffStack.saveToFile(stackFilename, override);

			pressESCforEarlyTermination();
		}
	}

	//Copy of PMT1XRoutines::frameByFrameScanTiling() with sync'ed rescanner
	void frameByFrameScanTiling(const FPGAns::FPGA &fpga, const int nSlice)
	{
		//ACQUISITION SETTINGS
		const ChannelList channelList{ {channelList.findChannel("GFP")} }; //Override
		const int2 nStacksXY{ 30, 28 };
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 1 };											//Number of frames for continuous XY acquisition
		const double FFOVfast{ widthPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int selectHeightPerFrame_pix;
		double selectScanFFOV, selectRescanFFOV;
		PMT16XCHAN PMT16Xchan;

#if multibeam
			//Multibeam
			selectHeightPerFrame_pix = static_cast<int>(heightPerFrame_pix / 16);
			selectScanFFOV = FFOVslow / 16;
			selectRescanFFOV = FFOVslow / 16;
			PMT16Xchan = PMT16XCHAN::CH00;
#else
			//Singlebeam
			selectHeightPerFrame_pix = heightPerFrame_pix;
			selectScanFFOV = FFOVslow;
			selectRescanFFOV = FFOVslow;
			PMT16Xchan = PMT16XCHAN::CH08;
#endif

		//STACK
		const double2 FFOV{ FFOVslow, FFOVfast };							//Full FOV in the (slow axis, fast axis)
		const double stepSizeZ{ 1.0 * um };									//Step size in z
		const double stackDepthZ{ 10. * um };								//Acquire a stack this deep in Z
		const int nDiffZ{ static_cast<int>(stackDepthZ / stepSizeZ) };		//Number of frames at different Zs
		const double3 stackOverlap_frac{ 0.03, 0.03, 0.03 };				//Stack overlap
		const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16Xchan };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, selectScanFFOV / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, selectRescanFFOV / 2, channelList.front().mWavelength_nm };

		//LASER
		VirtualLaser laser{ RTcontrol, channelList.front().mWavelength_nm, LASER::VISION };

		//Create a location list
		Sequencer sequence{ channelList, Sample(sampleName, immersionMedium, collar), stack, stackCenterXYZ, nStacksXY };
		std::vector<double2> locationXYList{ sequence.generateLocationList() };

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//Iterate over the wavelengths
		for (std::vector<int>::size_type iter_wv = 0; iter_wv < channelList.size(); iter_wv++)
		{
			//DATALOG
			Logger datalog("datalog_Slice" + std::to_string(nSlice) + "_" + channelList.at(iter_wv).mName);
			datalog.record("SAMPLE-------------------------------------------------------");
			datalog.record("Sample = ", sampleName);
			datalog.record("Immersion medium = ", immersionMedium);
			datalog.record("Correction collar = ", collar);
			datalog.record("\nFPGA---------------------------------------------------------");
			datalog.record("FPGA clock (MHz) = ", tickPerUs);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Slow axis FFOV (um) = ", FFOV.at(XX) / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleFactorU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", RTcontrol.mHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (FFOV.at(XX) / um) / RTcontrol.mHeightPerFrame_pix);
			datalog.record("\n");

			//Update the laser wavelength
			const int wavelength_nm = channelList.at(iter_wv).mWavelength_nm;
			laser.setWavelength(wavelength_nm);	//When switching pockels, the pockels destructor closes the uniblitz shutter
			laser.openShutter();				//Re-open the Uniblitz shutter if closed

			//Iterate over the locations
			for (std::vector<int>::size_type iter_loc = 0; iter_loc < locationXYList.size(); iter_loc++)
			{
				//Generate the discrete scan sequence for the stages
				std::vector<double3> stagePositionXYZ;
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
					stagePositionXYZ.push_back({ locationXYList.at(iter_loc).at(XX), locationXYList.at(iter_loc).at(YY), stackCenterXYZ.at(ZZ) + iterDiffZ * stepSizeZ });

				//CREATE A STACK FOR STORING THE TIFFS
				TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, 1 };

				//ACQUIRE FRAMES AT DIFFERENT Zs
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				{
					//Update the stages position
					stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
					stage.waitForMotionToStopAll();
					//stage.printPositionXYZ();		//Print the stage position		

					std::cout << "Location: " << iter_loc + 1 << "/" << locationXYList.size() << "\tTotal frame: " << iterDiffZ + 1 << "/" << nDiffZ << "\n";

					//Update the laser power
					laser.setPower(channelList.at(iter_wv).mScanPi + iterDiffZ * stepSizeZ * channelList.at(iter_wv).mStackPinc);

					//EXECUTE THE RT CONTROL SEQUENCE
					Image image{ RTcontrol };
					image.acquire();			//Execute the RT control sequence and acquire the image
					image.averageFrames();		//Average the frames acquired via continuous XY acquisition
					tiffStack.pushSameZ(0, image.pointerToTiff());
					tiffStack.pushDiffZ(iterDiffZ);
					std::cout << "\n";

					pressESCforEarlyTermination();		//Early termination if ESC is pressed
				}

				//Save the stackDiffZ to file
				std::string shortName{ "Slice" + std::to_string(nSlice) + "_" + channelList.at(iter_wv).mName + "_Tile" + std::to_string(iter_loc + 1) };
				std::string longName{ sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(channelList.at(iter_wv).mScanPi / mW, 1) + "mW_Pinc=" + toString(channelList.at(iter_wv).mStackPinc / mWpum, 1) + "mWpum" +
					"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
					"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };

				datalog.record(shortName + "\t" + longName);
				tiffStack.saveToFile(shortName, OVERRIDE::DIS);
			}//iter_loc
		}//iter_wv
	}

	//Image the sample non-stop. Use the PI program to move the stages around manually
	void liveScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("TDT") };	//Select a particular fluorescence channel
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis
		const int nFramesCont{ 1 };				//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//LASER
		const VirtualLaser laser{ RTcontrol, singleChannel.mWavelength_nm, LASER::VISION };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();					//Make sure that the RS is running

		//GALVO RT linear scan
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, FFOVslow / 2, singleChannel.mWavelength_nm };

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		while (true)
		{
			laser.setPower(singleChannel.mScanPi);	//Set the laser power

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };
			image.acquire();									//Execute the RT control sequence and acquire the image
			image.averageFrames();								//Average the frames acquired via continuous XY acquisition
			image.saveTiffSinglePage("Untitled", OVERRIDE::EN);	//Save individual files
			Sleep(700);

			pressESCforEarlyTermination();		//Early termination if ESC is pressed
		}
	}

	//Copy of PMT1XRoutines::continuousScan() with sync'ed rescanner
	void continuousScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const ChannelList::SingleChannel singleChannel{ channelList.findChannel("DAPI") };	//Select a particular laser
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 200 };				//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };
		const ZSCAN scanDirZ{ ZSCAN::TOPDOWN };		//Scan direction in z
		const double stackDepth{ nFramesCont * stepSizeZ };

		//Override the global stage position
		//const double3 stackCenterXYZ = { 50.983 * mm, 16.460 * mm, 18.059 * mm - nFramesCont * stepSizeZ /2 };

		double stageZi, stageZf, laserPi, laserPf;
		switch (scanDirZ)
		{
		case ZSCAN::TOPDOWN:
			stageZi = stackCenterXYZ.at(ZZ);
			stageZf = stackCenterXYZ.at(ZZ) + stackDepth + 20 * stepSizeZ; //Notice I have set a longer range
			laserPi = singleChannel.mScanPi;
			laserPf = singleChannel.mScanPi + stackDepth * singleChannel.mStackPinc;
			break;
		case ZSCAN::BOTTOMUP:
			stageZi = stackCenterXYZ.at(ZZ) + stackDepth;
			stageZf = stackCenterXYZ.at(ZZ) - 20 * stepSizeZ;				//Notice I have set a longer range
			laserPi = singleChannel.mScanPi + stackDepth * singleChannel.mStackPinc;
			laserPf = singleChannel.mScanPi;
			break;
		}

		//STAGES
		const double3 initialStageXYZ{ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stageZi};		//Initial position of the stages. The sign of stackDepth determines the scanning direction					
		Stage stage{ 5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix) };	//Specify the vel. Duration of a frame = a galvo swing = halfPeriodLineclock * heightPerFrame_pix
		stage.moveXYZ(initialStageXYZ);
		stage.waitForMotionToStopAll();

		//CREATE THE REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::ZSTAGE, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };	//Notice the ZSTAGE flag

		//LASER
		const VirtualLaser laser{ RTcontrol, singleChannel.mWavelength_nm, laserPi, LASER::VISION };
		//laser.powerLinearRamp(laserPi, laserPf);

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVO RT linear scan
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };	//Full FOV in the slow axis
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, FFOVslow / 2, singleChannel.mWavelength_nm };

		//OPEN THE SHUTTER
		laser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.initialize(scanDirZ);
		std::cout << "Scanning the stack...\n";
		stage.moveSingle(ZZ, stageZf);	//Move the stage to trigger the control sequence and data acquisition
		image.downloadData();
		image.postprocess();

		const std::string filename{ sampleName + "_" + toString(singleChannel.mWavelength_nm, 0) + "nm_P=" + toString((std::min)(laserPi, laserPf) / mW, 1) + "mW_Pinc=" + toString(singleChannel.mStackPinc / mWpum, 1) +
			"mWpum_x=" + toString(initialStageXYZ.at(XX) / mm, 3) + "_y=" + toString(initialStageXYZ.at(YY) / mm, 3) +
			"_zi=" + toString(stageZi / mm, 4) + "_zf=" + toString(stageZf / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
		image.saveTiffMultiPage(filename, OVERRIDE::DIS);
	}

}//namespace

//Photobleach the sample with the resonant scanner to see how much the sample moves after slicing
//I bleach with the RS and not the galvo or the stages because this way the RS is kept on the entire time while bleaching and imaging
namespace TestRoutines
{
	//Generate many short digital pulses and check the overall frameDuration with the oscilloscope
	void digitalLatency(const FPGAns::FPGA &fpga)
	{
		const double timeStep{ 4. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };

		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 1);

		//Many short digital pulses to accumulate the error
		for (U32 ii = 0; ii < 99; ii++)
			RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 0);

		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 0);
	}

	//First calibrate the digital channels, then use it as a time reference
	void analogLatency(const FPGAns::FPGA &fpga)
	{
		const double delay{ 400. * us };
		const double timeStep{ 4. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, timeStep, 10 * V);				//Initial pulse
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, timeStep, 0);
		RTcontrol.pushLinearRamp(RTCHAN::SCANGALVO, 4 * us, delay, 0, 5 * V);		//Linear ramp to accumulate the error
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, timeStep, 10 * V);				//Initial pulse
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, timeStep, 0);					//Final pulse

		//DO0
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 0);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, delay, 0);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 0);
	}

	void pixelclock(const FPGAns::FPGA &fpga)
	{
		std::vector<U8> stackOfAverages;

		FPGAns::RTcontrol RTcontrol{ fpga }; 		//Create a realtime control sequence
		Image image{ RTcontrol };
		image.acquire();					//Execute the realtime control sequence and acquire the image
		//image.pushToVector(stackOfAverages);
		//std::cout << "size: " << stackOfAverages.size() << "\n";
		//TiffU8 acqParam{ stackOfAverages, 300, 400 };
		//acqParam.saveTiff("Untitled");
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGAns::FPGA &fpga)
	{
		const double timeStep{ 400. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, timeStep, 0);
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGAns::FPGA &fpga)
	{
		FPGAns::RTcontrol RTcontrol{ fpga };

		//DO
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, 4 * us, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, 4 * us, 0);

		//AO
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, 8 * us, 4 * V);
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, 4 * us, 2 * V);
		RTcontrol.pushAnalogSinglet(RTCHAN::SCANGALVO, 4 * us, 1 * V);

		RTcontrol.triggerRT();	//Execute the realtime control sequence
	}

	void analogRamp(const FPGAns::FPGA &fpga)
	{
		const double Vmax{ 5. * V };
		const double step{ 4. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };
		RTcontrol.pushLinearRamp(RTCHAN::SCANGALVO, step, 2 * ms, 0, -Vmax);
		RTcontrol.pushLinearRamp(RTCHAN::SCANGALVO, step, 20 * ms, -Vmax, Vmax);
		RTcontrol.pushLinearRamp(RTCHAN::SCANGALVO, step, 2 * ms, Vmax, 0);

		const double pulsewidth(300. * us);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, pulsewidth, 1);
		RTcontrol.pushDigitalSinglet(RTCHAN::DODEBUG, 4 * us, 0);
	}

	//I think this is for matching the galvo forward and backward scans via imaging beads
	void fineTuneScanGalvo(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 30 };											//Number of frames for continuous XY acquisition
		const double3 stagePositionXYZ{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::RS, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//RS
		const ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVO
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis
		Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };

		//LASER
		const int wavelength_nm{ 1040 };
		const double P{ 25. * mW };		//Laser power
		VirtualLaser laser{ RTcontrol, wavelength_nm, P, LASER::FIDELITY };

		//ACQUIRE FRAMES AT DIFFERENT Zs
		stage.moveXYZ(stagePositionXYZ);
		stage.waitForMotionToStopAll();

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition
		image.averageEvenOddFrames();
		image.saveTiffMultiPage("Untitled", OVERRIDE::DIS);
	}

	void resonantScanner(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		ResonantScanner RScanner{ RTcontrol };
		std::cout << "aaa = " << RScanner.downloadControlVoltage() << "\n";
		//RScanner.turnOn(150 * um);
		//RScanner.turnOff();
	}

	//Use a single beamlet with the rescanner sync'ed to the scanner to keep the beam position fixed at the detector plane
	//Must manually open the laser and Uniblitz shutter and adjust the pockels power
	void galvosSync(const FPGAns::FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 2 };
		const int wavelength_nm = 750;			//The rescanner calib depends on the laser wavelength

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::DIS, PMT16XCHAN::CH08 };

		//GALVOS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };		//Scan duration in the slow axis
		Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };
		Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, FFOVslow / 2, wavelength_nm };

		//Execute the realtime control sequence and acquire the image
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence
	}

	void stagePosition()
	{
		double duration;
		const double3 stagePositionXYZ{ 35.020 * mm, 19.808 * mm, 18.542 * mm };	//Stage initial position
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		std::cout << "Stages initial position:" << "\n";
		stage.printPositionXYZ();

		auto t_start{ std::chrono::high_resolution_clock::now() };

		stage.moveXYZ(stagePositionXYZ);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		stage.waitForMotionToStopSingle(ZZ);

		std::cout << "Stages final position:" << "\n";
		stage.printPositionXYZ();

		/*
		int input = 1;
		while (input)
		{
			std::cout << "Stage X position = " << stage.downloadPositionSingle_(XX) << "\n";
			std::cout << "Stage Y position = " << stage.downloadPositionSingle_(YY) << "\n";
			std::cout << "Stage X position = " << stage.downloadPositionSingle_(ZZ) << "\n";

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
		const int DOchan{ 1 };

		//std::cout << "Stages initial position:" << "\n";
		//stage.printPositionXYZ();

		std::cout << "Stages initial vel:" << "\n";
		stage.printVelXYZ();

		//stage.isDOtriggerEnabled(ZZ, DOchannel);
		//stage.setDOtriggerEnabled(ZZ, DOchannel , true);

		//const int triggerParam = 1;
		//stage.downloadDOtriggerParamSingle_(ZZ, DOchannel , triggerParam);
		//std::cout << "x stage vel: " << stage.downloadVelSingle_(XX) / mmps << " mm/s" << "\n";
		//std::cout << "y stage vel: " << stage.downloadVelSingle_(YY) / mmps << " mm/s" << "\n";
		//std::cout << "z stage vel: " << stage.downloadVelSingle_(ZZ) / mmps << " mm/s" << "\n";
		//stage.printStageConfig(ZZ, DOchan);
	}

	void shutter(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		PockelsCell fidelity{ RTcontrol, 1040, LASER::FIDELITY };
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
	void pockels(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		//DEFINE THE POCKELS CELLS
		PockelsCell pockelsVision{ RTcontrol, 750, LASER::VISION };			//Vision
		PockelsCell pockelsFidelity{ RTcontrol, 1040, LASER::FIDELITY };		//Fidelity

		PockelsCell pockels{ pockelsVision };
		//PockelsCell pockels{ pockelsFidelity };
		pockels.pushPowerSinglet(8 * us, 300. * mW);
		//pockels.pushPowerSinglet(8 * us, 0 * mW);
		//pockels.pushVoltageSinglet(8 * us, 1.0 * V);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		Image image{ RTcontrol };
		image.acquire();
	}

	void pockelsRamp(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 10 };			//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::DIS, PMT16XCHAN::CH00 };

		//POCKELS CELL
		const int wavelength_nm{ 750 };
		PockelsCell pockels{ RTcontrol, wavelength_nm, LASER::VISION };
		//pockels.pushPowerSinglet(400 * us, 50. * mW, OVERRIDE);
		pockels.powerLinearRamp(10. * mW, 10. * mW);		//Linearly scale the laser power from the first to the last frame

		//Test the voltage setpoint
		//pockels.pushVoltageSinglet(8* us, 0.5 * V);
		//pockels.voltageLinearRamp(0.5 * V, 1.0 * V);		//Linearly scale the pockels voltage from the first to the last frame


		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition
	}

	void lasers(const FPGAns::FPGA &fpga)
	{
		Laser laser{ LASER::VISION };
		//Laser laser{ LASER::FIDELITY };
		//std::cout << laser.isShutterOpen() << std::endl;
		//laser.setShutter(false);
		laser.setWavelength(1040);
		//laser.printWavelength_nm();
	}

	//Open the Uniblitz shutter manually
	void virtualLasers(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		const int wavelength_nm{ 920 };
		const double laserPower{ 50. * mW };		//Laser power
		VirtualLaser laser{ RTcontrol, wavelength_nm, laserPower };

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();					//Execute the RT control sequence
	}

	void convertI16toVolt()
	{
		std::cout << "volt to I16: " << FPGAns::voltageToI16(1) << "\n";
		std::cout << "I16 to colt: " << FPGAns::I16toVoltage(32767) << "\n";
		std::cout << "volt to I16 to volt: " << FPGAns::I16toVoltage(FPGAns::voltageToI16(0)) << "\n";
	}

	void tiffU8()
	{
		std::string inputFilename{ "Beads_4um_750nm_50mW_x=35.120_y=19.808_z=18.4610" };
		std::string outputFilename{ "test" };

		const int nFramesCont{ 10 };
		TiffU8 image{ inputFilename, nFramesCont };

		image.mirrorOddFrames();
		//image.averageFrames();
		image.averageEvenOddFrames();
		image.saveToFile(outputFilename, MULTIPAGE::EN, OVERRIDE::EN);

		//image.saveToFile(outputFilename, 2);

		//image.mirrorOddFrames(nFramesCont);
		//image.averageEvenOddFrames(nFramesCont);
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

		//overriding the file saving has some overhead
		//Splitting the stackDiffZ into a page structure (by assigning nFramesCont = 200 in saveToFile) gives a large overhead
		image.saveToFile(filename, MULTIPAGE::DIS, OVERRIDE::EN);

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
		const double2 FFOV{ heightPerFrame_pix * pixelSizeXY, widthPerFrame_pix * pixelSizeXY };
		const int nFramesCont{ 80 };										//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };									//Step size in z
		const ROI roi{ 9.950 * mm, 34.850 * mm, 10.150 * mm, 35.050 * mm }; //Region of interest {ymin, xmin, ymax, xmax}
		const double3 stackOverlapXYZ_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };						//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };							//Sample thickness
		const double sampleSurfaceZ{ 18.471 * mm };

		const ChannelList channelList{ channelList };
		//const ChannelList channelList{ channelList.findChannel("DAPI") };
		Sample sample{ "Beads4um", "Grycerol", "1.47", roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlapXYZ_frac };

		//Create a sequence
		Sequencer sequence{ channelList, sample, stack };
		sequence.generateCommandList();
		sequence.printToFile("CommandlistLight");


		if (1)
		{
			FUNC x;
			std::thread saveFile, moveStage;

			//Read the commands line by line
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.mCommandCounter; iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
				//commandline.printParameters();

				switch (commandline.mAction)
				{
				case ACTION::MOV:
					std::cout << "MOV " << "\n";

					//Skip the first MOV to position the stage
					if (iterCommandline != 0)
						moveStage = std::thread{ &FUNC::func2, &x, 314 };

					std::cout << "saveFile joinable? " << std::boolalpha << saveFile.joinable() << "\n";
					std::cout << "moveStage joinable? " << std::boolalpha << moveStage.joinable() << "\n";

					break;
				case ACTION::ACQ:
					std::cout << "ACQ " << "\n";

					if (saveFile.joinable() && moveStage.joinable())
					{
						saveFile.join();
						moveStage.join();
					}
					break;
				case ACTION::SAV:
					std::cout << "SAV" << "\n";
					saveFile = std::thread{ &FUNC::func1, &x, 123 };
					break;
				case ACTION::CUT:
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

	void locationSequencer()
	{
		//ACQUISITION SETTINGS
		const double2 FFOV{ 200. * um, 150. * um };
		const int nDiffZ{ 100 };											//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };									//Step size in z
		const double3 stackOverlap_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const Stack stack{ FFOV, stepSizeZ, nDiffZ, stackOverlap_frac };

		//Create a sequence
		Sequencer sequence{ channelList, Sample(sampleName, immersionMedium, collar), stack, stackCenterXYZ, { 2, 2 } }; //Last 2 parameters: stack center and number of stacks
		std::vector<double2> locationList{ sequence.generateLocationList() };

		for (std::vector<int>::size_type iter_loc = 0; iter_loc < locationList.size(); iter_loc++)
		{
			std::cout << "x = " << locationList.at(iter_loc).at(XX) / mm << "\ty = " << locationList.at(iter_loc).at(YY) / mm << "\n";
		}
	}

	void PMT16Xconfig()
	{
		PMT16X pmt;
		//pmt.readAllGain();
		//pmt.setSingleGain(12, 170);
		//pmt.setAllGain(255);
		//pmt.readTemp();

		//To make the count from all the channels similar,
		//rescan with frequency = 100 Hz and amplitude = 1.5V (which is larger than the size of the PMT16X
		//to use the linear part of the ramp). Set the refresh rate to 10 or 20 ms
		pmt.setAllGain({
			255,	//CH01
			255,	//CH02
			255,	//CH03
			255,	//CH04
			255,	//CH05
			255,	//CH06
			255,	//CH07
			255,	//CH08
			255,	//CH09, this channel presents the lowest signal. For some reason 255 gives a lower signal than 200
			255,	//CH10
			255,	//CH11
			255,	//CH12
			255,	//CH13
			255,	//CH14
			255,	//CH15
			255		//CH16
			});
	}

	//Test reading different channels of the PMT16X
	//Must manually open the laser and Uniblitz shutter
	void PMT16Xdemultiplex(const FPGAns::FPGA &fpga)
	{
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 1 };						//Number of frames for continuous XY acquisition
		const int wavelength_nm = 750;

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, FIFOOUT::EN, PMT16XCHAN::CH08 };

		//GALVOS
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };				//Scan duration in the slow axis
		Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, FFOVslow / 2 };
		//Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, 0 };		//Keep the scanner fixed to see the emitted light swing across the PMT16X channels. The rescanner must be centered
		Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, FFOVslow / 2, wavelength_nm };

		//LASER
		VirtualLaser laser{ RTcontrol, 750, 30. * mW, LASER::VISION };

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();			//Execute the RT control sequence and acquire the image
		image.saveTiffMultiPage("SingleChannel", OVERRIDE::EN);
	}

	//Test the synchronization of the 2 galvos and the laser
	void PMT16XgavosSyncAndLaser(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const double pixelSizeXY{ 0.5 * um };
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 560 };
		const int nFramesCont{ 2 };
		const double FFOVslow{ heightPerFrame_pix * pixelSizeXY };			//Full FOV in the slow axis

		int selectHeightPerFrame_pix;
		double selectScanFFOV, selectPower;
		PMT16XCHAN PMT16Xchan;

#if multibeam
			//Multibeam
			selectHeightPerFrame_pix = static_cast<int>(heightPerFrame_pix / 16);
			selectScanFFOV = FFOVslow / 16;
			PMT16Xchan = PMT16XCHAN::CH00;
			selectPower = 1400. * mW;
#else
			//Singlebeam
			selectHeightPerFrame_pix = heightPerFrame_pix;
			selectScanFFOV = FFOVslow;
			PMT16Xchan = PMT16XCHAN::CH02;
			selectPower = 50. * mW;
#endif
		//STACK
		const double stepSizeZ{ 1.0 * um };
		const double stackDepthZ{ 20. * um };	//Acquire a stack this deep in Z

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, nFramesCont, widthPerFrame_pix, selectHeightPerFrame_pix, FIFOOUT::EN, PMT16Xchan };

		//LASER
		const int wavelength_nm = 750;
		const VirtualLaser laser{ RTcontrol, wavelength_nm, selectPower, LASER::VISION };

		//GALVO RT linear scan
		const Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, selectScanFFOV / 2 };
		const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, selectScanFFOV / 2, wavelength_nm };
		//const Galvo rescanner{ RTcontrol, RTCHAN::RESCANGALVO, 0, wavelength_nm };

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();			//Execute the RT control sequence and acquire the image
	}


	void vibratome(const FPGAns::FPGA &fpga)
	{
		const double slicePlaneZ = (23.640 + 0.050) * mm;

		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };
		Vibratome vibratome{ fpga, stage };
		vibratome.slice(slicePlaneZ);

		const double3 samplePosition{ 0. * mm, 0. * mm, slicePlaneZ };
		stage.moveXYZ(stackCenterXYZ);
	}

	void filterwheel()
	{
		Filterwheel FWexcitation{ FILTERWHEEL::EXC };
		Filterwheel FWdetection{ FILTERWHEEL::DET };

		int wavelength_nm;
		if (1)
			wavelength_nm = 920;
		else
			wavelength_nm = 750;

		//FWexcitation.setWavelength(wavelength_nm);
		//FWdetection.setWavelength(wavelength_nm);

#if multibeam
		//Multiplex
		//Turn both filterwheels concurrently
		std::thread th1{ &Filterwheel::setWavelength, &FWexcitation, wavelength_nm };
		std::thread th2{ &Filterwheel::setWavelength, &FWdetection, wavelength_nm };
		th1.join();
		th2.join();
#else
		//Single beam
		//Turn both filterwheels concurrently
		std::thread th1{ &Filterwheel::setPosition, &FWexcitation, FILTERCOLOR::OPEN };				//Leave the excitation filterwheel open
		std::thread th2{ &Filterwheel::setWavelength, &FWdetection, wavelength_nm };
		th1.join();
		th2.join();
#endif
	}


	void stepper()
	{
		Stepper CollectorLens1;
		CollectorLens1.move(4.0 * mm);
		//CollectorLens1.downloadPosition();
		//CollectorLens1.home();
	}



	//Photobleach a line along the fast axis (RS) on the sample
	void photobleach(const FPGAns::FPGA &fpga)
	{
		Laser laser{ LASER::VISION };
		laser.setWavelength(920);

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, LINECLOCK::FG, MAINTRIG::PC, 100, 300, 400, FIFOOUT::DIS, PMT16XCHAN::CH00 };

		//RS
		ResonantScanner RScanner{ RTcontrol };
		RScanner.isRunning();		//Make sure that the RS is running

		//GALVO. Keep the galvo fixed to bleach a line on the sample
		Galvo scanner{ RTcontrol, RTCHAN::SCANGALVO, 0 };

		//POCKELS CELLS
		PockelsCell pockels{ RTcontrol, 920, LASER::VISION };
		pockels.pushPowerSinglet(8 * us, 200 * mW);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		pockels.setShutter(true);
		Image image{ RTcontrol };
		image.acquire();

		//Wait until the sequence is over to close the shutter, otherwise this code will finish before the RT sequence
		pressAnyKeyToCont();
		pockels.setShutter(false);
	}

	//Generate a text file with the tile location for the BigStitcher
	void generateLocationsForBigStitcher()
	{
		// X is vertical and Y is horizontal, to match the directions of the XYZ stage
		const int2 nStacksXY{ 30, 28 };	
		const int2 tileShiftXY_pix{ 543, 291 };

		Logger datalog(sampleName + "_locations");
		datalog.record("dim=3"); //Needed in BigStitcher
	
		for (int nTile = 0; nTile < nStacksXY.at(XX) * nStacksXY.at(YY); nTile++)
		//for (int nTile = 0; nTile < 180; nTile++)
		{
			int2 nXY = nTileToArrayIndices(nTile);
			int tileShiftX{ -nXY.at(XX) * tileShiftXY_pix.at(XX) };
			int tileShiftY{ -nXY.at(YY) * tileShiftXY_pix.at(YY) };
			std::string line{ std::to_string(nTile) + ";;(" + std::to_string(tileShiftY) + "," + std::to_string(tileShiftX) + ",0)" };	//In BigStitcher, X is horizontal and Y is vertical
			//std::string line{ std::to_string(nTile) + "\t" + std::to_string(nTileToArrayIndices(nTile).at(XX)) + "\t" + std::to_string(nTileToArrayIndices(nTile).at(YY)) }; //For debugging
			datalog.record(line);
		}

	}

	//Snake pattern starting from the bottom right of the sample and going up
	int2 nTileToArrayIndices(const int nTile)
	{
		const int2 nStacksXY{ 30, 28 };

		int nx;
		int ny{ nTile / nStacksXY.at(XX) };

		if (ny % 2)	//ny is odd
			nx = nStacksXY.at(XX) - nTile % nStacksXY.at(XX) - 1;
		else		//ny is even
			nx = nTile % nStacksXY.at(XX);
	
		return {nx,ny};
	}
}//namespace