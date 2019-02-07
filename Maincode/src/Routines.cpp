#include "Routines.h"

//SAMPLE PARAMETERS
const double3 stackCenterXYZ{ 43.800 * mm, 17.300 * mm, 20.400 * mm };
const std::string sampleName{ "Liver" };
const std::string immersionMedium{ "SiliconMineralOil5050" };
const std::string collar{ "1.49" };
const std::vector<LaserList::SingleLaser> laserListLiver{ { 920, 80. * mW, 40. * mW } , { 750, 25. * mW, 25. * mW }, { 1040, 120. * mW, 40. * mW } };	//Define the wavelengths and laser powers for liver


namespace MainRoutines
{
	//The "swiss knife" of my routines
	void discreteZstageScan(const FPGAns::FPGA &fpga)
	{
		//Each of the following modes can be used under 'continuous XY acquisition' by setting nFramesCont > 1, meaning that the galvo is scanned back and
		//forth on the same z plane. The images the can be averaged
		const RunMode acqMode{ SINGLEMODE };			//Single shot
		//const RunMode acqMode{ LIVEMODE };			//Image the same z plane many times as single shots. Used it for adjusting the microscope live
		//const RunMode acqMode{ AVGMODE };				//Image the same z plane many times and average the images
		//const RunMode acqMode{ STACKMODE };			//Stack volume from the initial z position
		//const RunMode acqMode{ STACKCENTEREDMODE };		//Stack volume centered at the initial z position

		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 1 };				//Number of frames for continuous XY acquisition

		//RS
		const ResonantScanner RScanner{ fpga };
		RScanner.isRunning();					//Make sure that the RS is running

		//STACK
		const double stepSizeZ{ 0.5 * um };
		const double stackDepthZ{ 50. * um };			//Acquire a stack of this depth or thickness in Z

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps};
		std::vector<double3> stagePositionXYZ;

		int nDiffZ;		//Number of frames at different Zs
		int nSameZ;		//Number of frames at the same Z
		OverrideFileSelector overrideFlag;
		switch (acqMode)
		{
		case SINGLEMODE:
			nSameZ = 1;
			nDiffZ = 1; //Do not change this
			overrideFlag = NOOVERRIDE;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case LIVEMODE:
			nSameZ = 500;
			nDiffZ = 1; //Do not change this
			overrideFlag = OVERRIDE;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case AVGMODE:
			nSameZ = 10;
			nDiffZ = 1; //Do not change this
			overrideFlag = NOOVERRIDE;
			stagePositionXYZ.push_back(stackCenterXYZ);
			break;
		case STACKMODE:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			overrideFlag = NOOVERRIDE;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) + iterDiffZ * stepSizeZ });
			break;
		case STACKCENTEREDMODE:
			nSameZ = 1;
			nDiffZ = static_cast<int>(stackDepthZ / stepSizeZ);
			overrideFlag = NOOVERRIDE;
			//Generate the control sequence for the stages
			for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				stagePositionXYZ.push_back({ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) - 0.5 * stackDepthZ + iterDiffZ * stepSizeZ });
			break;
		default:
			throw std::invalid_argument((std::string)__FUNCTION__ + ": Selected acquisition mode not available");
		}

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		//GALVO RT linear scan
		const double FFOVgalvo{ 200. * um };			//Full FOV in the slow axis
		const Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		//LASER
		const LaserList::SingleLaser laserParams{ laserListLiver.at(1) };	//Choose a particular wavelength
		double laserPower{ laserParams.mScanPi };							//Initialize the laser power
		const VirtualLaser laser{ RTcontrol, laserParams.mWavelength_nm, AUTO };

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
			datalog.record("Laser wavelength (nm) = ", laserParams.mWavelength_nm);
			datalog.record("Laser power first frame (mW) = ", laserParams.mScanPi / mW);
			datalog.record("Laser power last frame (mW) = ", (laserParams.mScanPi + laserParams.mStackPinc) / mW);
			datalog.record("Laser repetition period (us) = ", VISIONpulsePeriod / us);
			datalog.record("\nSCAN---------------------------------------------------------");
			datalog.record("RS FFOV (um) = ", RScanner.mFFOV / um);
			datalog.record("RS period (us) = ", 2 * halfPeriodLineclock / us);
			datalog.record("Pixel dwell time (us) = ", RTcontrol.mDwell / us);
			datalog.record("RS fill factor = ", RScanner.mFillFactor);
			datalog.record("Galvo FFOV (um) = ", FFOVgalvo / um);
			datalog.record("\nIMAGE--------------------------------------------------------");
			datalog.record("Max count per pixel = ", RTcontrol.mPulsesPerPix);
			datalog.record("8-bit upscaling factor = ", RTcontrol.mUpscaleFactorU8);
			datalog.record("Width X (RS) (pix) = ", RTcontrol.mWidthPerFrame_pix);
			datalog.record("Height Y (galvo) (pix) = ", RTcontrol.mHeightPerFrame_pix);
			datalog.record("Resolution X (RS) (um/pix) = ", RScanner.mSampRes / um);
			datalog.record("Resolution Y (galvo) (um/pix) = ", (FFOVgalvo / um) / RTcontrol.mHeightPerFrame_pix);
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

				laser.setPower(laserPower);	//Update the laser power

				//EXECUTE THE RT CONTROL SEQUENCE
				Image image{ RTcontrol };
				image.acquire();			//Execute the RT control sequence and acquire the image
				image.averageFrames();		//Average the frames acquired via continuous XY acquisition
				tiffStack.pushSameZ(iterSameZ, image.pointerToTiff());

				if (acqMode == SINGLEMODE || acqMode == LIVEMODE)
				{
					//Save individual files
					std::string singleFilename{ sampleName + "_" + toString(laserParams.mWavelength_nm, 0) + "nm_" + toString(laserPower / mW, 1) + "mW" +
						"_x=" + toString(stagePositionXYZ.at(iterDiffZ).at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.at(iterDiffZ).at(YY) / mm, 3) + "_z=" + toString(stagePositionXYZ.at(iterDiffZ).at(ZZ) / mm, 4) };
					image.saveTiffSinglePage(singleFilename, overrideFlag);
					Sleep(700);
				}
			}
			tiffStack.pushDiffZ(iterDiffZ);

			std::cout << "\n";
			laserPower += laserParams.mStackPinc / nDiffZ;		//Increase the laser power for the next Z plane
		}

		if (acqMode == AVGMODE || acqMode == STACKMODE || acqMode == STACKCENTEREDMODE)
		{
			//Save the stackDiffZ to file
			std::string stackFilename{ sampleName + "_" + toString(laserParams.mWavelength_nm, 0) + "nm_Pi=" + toString(laserParams.mScanPi / mW, 1) + "mW_Pf=" + toString((laserParams.mScanPi + laserParams.mStackPinc) / mW, 1) + "mW" +
				"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
				"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
			tiffStack.saveToFile(stackFilename, overrideFlag);
		}

		laser.closeShutter();
		//pressAnyKeyToCont();
	}

	//Apply 'discreteZstageScan' on a list of locations
	void discreteZstageScanForCARE(const FPGAns::FPGA &fpga)
	{
		//Location list
		const std::vector<double3> locationList = {
			{43.800 * mm, 17.300 * mm, 20.400 * mm},
			{44.800 * mm, 17.100 * mm, 20.400 * mm},
			{43.900 * mm, 17.100 * mm, 20.400 * mm},
			{43.400 * mm, 17.100 * mm, 20.400 * mm},
			{42.400 * mm, 17.100 * mm, 20.400 * mm},
			{45.300 * mm, 17.000 * mm, 20.400 * mm},
			{44.500 * mm, 16.900 * mm, 20.400 * mm},
			{45.200 * mm, 16.800 * mm, 20.400 * mm},
			{45.000 * mm, 16.800 * mm, 20.400 * mm},
			{44.800 * mm, 16.800 * mm, 20.400 * mm} };

		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 10 };				//Number of frames for continuous XY acquisition

		//RS
		const ResonantScanner RScanner{ fpga };
		RScanner.isRunning();					//Make sure that the RS is running

		//STACK
		const double stepSizeZ{ 0.5 * um };
		double stackDepthZ{ 100. * um };									//Acquire a stack of this depth or thickness in Z
		const int nDiffZ{ static_cast<int>(stackDepthZ / stepSizeZ) };		//Number of frames at different Zs

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		//GALVO RT linear scan
		const double FFOVgalvo{ 200. * um };			//Full FOV in the slow axis
		const Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//Iterate over the wavelengths
		for (std::vector<int>::size_type wv_iter = 0; wv_iter < laserListLiver.size(); wv_iter++)
			//for (std::vector<int>::size_type wv_iter = 2; wv_iter < wavelengthList.size(); wv_iter++)		//For debugging
		{
			//Iterate over the locations
			for (std::vector<int>::size_type loc_iter = 0; loc_iter < locationList.size(); loc_iter++)
			{
				//Generate the control sequence for the stages
				std::vector<double3> stagePositionXYZ;
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
					stagePositionXYZ.push_back({ locationList.at(loc_iter).at(XX), locationList.at(loc_iter).at(YY), locationList.at(loc_iter).at(ZZ) + iterDiffZ * stepSizeZ });

				//LASER
				const int wavelength_nm = laserListLiver.at(wv_iter).mWavelength_nm;
				double laserPower{ laserListLiver.at(wv_iter).mScanPi };
				const VirtualLaser laser{ RTcontrol, wavelength_nm, AUTO };

				//CREATE A STACK FOR STORING THE TIFFS
				TiffStack tiffStack{ widthPerFrame_pix, heightPerFrame_pix, nDiffZ, 1 };

				//OPEN THE UNIBLITZ SHUTTERS
				laser.openShutter();	//The destructor will close the shutter automatically

				//ACQUIRE FRAMES AT DIFFERENT Zs
				for (int iterDiffZ = 0; iterDiffZ < nDiffZ; iterDiffZ++)
				{
					//Update the stages position
					stage.moveXYZ(stagePositionXYZ.at(iterDiffZ));
					stage.waitForMotionToStopAll();
					stage.printPositionXYZ();		//Print the stage position		

					std::cout << "Total frame: " << iterDiffZ + 1 << "/" << nDiffZ << "\n";

					//Update the laser power
					laser.setPower(laserPower);	

					//EXECUTE THE RT CONTROL SEQUENCE
					Image image{ RTcontrol };
					image.acquire();			//Execute the RT control sequence and acquire the image
					image.averageFrames();		//Average the frames acquired via continuous XY acquisition
					tiffStack.pushSameZ(0, image.pointerToTiff());
					tiffStack.pushDiffZ(iterDiffZ);
					std::cout << "\n";
					laserPower += laserListLiver.at(wv_iter).mStackPinc / nDiffZ;		//calculate the new laser power
				}

				//Save the stackDiffZ to file
				std::string stackFilename{ sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(laserListLiver.at(wv_iter).mScanPi / mW, 1) +
					"mW_Pf=" + toString((laserListLiver.at(wv_iter).mScanPi + laserListLiver.at(wv_iter).mStackPinc) / mW, 1) + "mW" +
					"_x=" + toString(stagePositionXYZ.front().at(XX) / mm, 3) + "_y=" + toString(stagePositionXYZ.front().at(YY) / mm, 3) +
					"_zi=" + toString(stagePositionXYZ.front().at(ZZ) / mm, 4) + "_zf=" + toString(stagePositionXYZ.back().at(ZZ) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
				tiffStack.saveToFile(stackFilename, NOOVERRIDE);

				laser.closeShutter();

				//Break if the ESC is pressed for an early termination
				if (GetAsyncKeyState(VK_ESCAPE) & 0x0001)
					throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");

			}//loc_iter
		}//wv_iter
	}

	//Image the sample non-stop. Use the PI program to move the stages manually
	void liveScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 1 };				//Number of frames for continuous XY acquisition

		//RS
		const ResonantScanner RScanner{ fpga };
		RScanner.isRunning();					//Make sure that the RS is running

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		//GALVO RT linear scan
		const double FFOVgalvo{ 200. * um };			//Full FOV in the slow axis
		const Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		//LASER
		const LaserList::SingleLaser laserParams{ laserListLiver.at(0) };	//Choose a particular wavelength
		const VirtualLaser laser{ RTcontrol, laserParams.mWavelength_nm, AUTO };

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		while (true)
		{
			laser.setPower(laserParams.mScanPi);	//Set the laser power

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };
			image.acquire();								//Execute the RT control sequence and acquire the image
			image.averageFrames();							//Average the frames acquired via continuous XY acquisition
			image.saveTiffSinglePage("Untitled", OVERRIDE);	//Save individual files
			Sleep(500);

			//Break if the ESC is pressed for an early termination
			if (GetAsyncKeyState(VK_ESCAPE) & 0x0001)
				break;
		}
		laser.closeShutter();
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
	void contZstageScan(const FPGAns::FPGA &fpga)
	{
		const int centeredStackFlag = 0; //0 for centered

		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 160 };				//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };

		//STAGES
		const ScanDirection stackScanDirZ{ TOPDOWN };		//Scan direction in z
		const double stackDepth{ stackScanDirZ * nFramesCont * stepSizeZ };
		const double3 stageXYZi{ stackCenterXYZ.at(XX), stackCenterXYZ.at(YY), stackCenterXYZ.at(ZZ) - centeredStackFlag * stackDepth / 2 };	//Initial position of the stages. The sign of stackDepth determines the scanning direction					
		Stage stage{ 5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix) };												//Specify the vel. Duration of a frame = a galvo swing = halfPeriodLineclock * heightPerFrame_pix
		stage.moveXYZ(stageXYZi);
		stage.waitForMotionToStopAll();

		//RS
		const ResonantScanner RScanner{ fpga };
		RScanner.isRunning();		//Make sure that the RS is running

		//CREATE THE REALTIME CONTROL SEQUENCE. Notice the STAGETRIG flag to enable triggering the RT sequence with the stage
		FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, STAGETRIG };	

		//LASER: wavelength_nm, laserPower, whichLaser
		const int wavelength_nm{ 750 };
		const double laserPower{ 30. * mW };
		const VirtualLaser laser{ RTcontrol, wavelength_nm, laserPower, VISION };
		//VirtualLaser laser{ RTcontrol, 1040, 25. * mW, AUTO };

		//GALVO RT linear scan
		const double FFOVgalvo{ 200. * um };	//Full FOV in the slow axis
		const Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		//OPEN THE SHUTTER
		laser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.initialize();
		std::cout << "Scanning the stack...\n";
		stage.moveSingle(ZZ, stageXYZi.at(ZZ) + stackDepth);	//Move the stage to trigger the control sequence and data acquisition
		image.downloadData();
		image.postprocess();

		const std::string filename{ sampleName + "_" + toString(wavelength_nm, 0) + "nm_Pi=" + toString(laserPower / mW, 1) +
			"_x=" + toString(stageXYZi.at(XX) / mm, 3) + "_y=" + toString(stageXYZi.at(YY) / mm, 3) +
			"_zi=" + toString(stageXYZi.at(ZZ) / mm, 4) + "_zf=" + toString((stageXYZi.at(ZZ) + stackDepth) / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4) };
		image.saveTiffMultiPage(filename, NOOVERRIDE, stackScanDirZ);

		//Disable ZstageAsTrigger to be able to move the z-stage without triggering the acquisition sequence
		//RTcontrol.setZstageTriggerEnabled(false);

		laser.closeShutter();
		//pressAnyKeyToCont();
	}

	//Full sequence to image and cut an entire sample automatically
	void sequencer(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const double2 FFOV{ 200. * um, 150. * um };
		const int nFramesCont{ 80 };											//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };										//Step size in z
		const ROI roi{ 11.000 * mm, 34.825 * mm, 11.180 * mm, 35.025 * mm };	//Region of interest {ymin, xmin, ymax, xmax}
		const double3 stackOverlap_frac{ 0.05, 0.05, 0.05 };					//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };							//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };								//Sample thickness
		const double sampleSurfaceZ{ 18.521 * mm };

		const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW }, { 1040, 30. * mW, 0. * mW } };
		//const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW } };
		//const std::vector<LaserList::SingleLaser> laserList{{ 1040, 25. * mW, 0. * mW } };
		const Sample sample{ sampleName, immersionMedium, collar, roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		const Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlap_frac };

		//Create a sequence
		//Sequencer sequence{ laserList, sample, stack };
		Sequencer sequence{ laserList, Sample("Beads4um", "Grycerol", "1.47"), stack, stackCenterXYZ, { 2, 2 } }; //Last 2 parameters: stack center and nGavoSwings of stacks
		sequence.generateCommandList();
		sequence.printToFile("Commandlist");


		if (1)
		{
			//STAGES. Specify the velocity
			Stage stage{ 5 * mmps, 5 * mmps, stepSizeZ / (halfPeriodLineclock * heightPerFrame_pix) };
			stage.moveSingle(ZZ, sample.mSurfaceZ);	//Move the z stage to the sample surface

			//RS
			const ResonantScanner RScanner{ fpga };
			RScanner.isRunning();		//Make sure that the RS is running

			//CREATE THE REALTIME CONTROL SEQUENCE
			FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix, STAGETRIG };	//Notice the STAGETRIG flag

			//LASER: wavelength_nm, laserPower, whichLaser
			VirtualLaser laser{ RTcontrol, laserList.front().mWavelength_nm };

			//GALVO RT linear ramp	
			const Galvo galvo{ RTcontrol, RTGALVO1, FFOV.at(XX) / 2 };

			//EXECUTE THE RT CONTROL SEQUENCE
			Image image{ RTcontrol };

			//Read the commands line by line
			double scanZi, scanZf, scanPi, stackPinc;
			double2 stackCenterXY;
			int wavelength_nm;
			ScanDirection scanDirZ;
			std::string filename;
			for (std::vector<int>::size_type iterCommandline = 0; iterCommandline != sequence.mCommandCounter; iterCommandline++)
				//for (std::vector<int>::size_type iterCommandline = 0; iterCommandline < 2; iterCommandline++) //For debugging
			{
				Commandline commandline{ sequence.readCommandline(iterCommandline) }; //Implement read-from-file?
				//commandline.printParameters();

				//Stopwatch
				//auto t_start{ std::chrono::high_resolution_clock::now() };

				switch (commandline.mAction)
				{
				case MOV:
					//Move the x and y stages to mStackCenterXY
					stackCenterXY = commandline.mCommand.moveStage.mStackCenterXY;
					stage.moveXY(stackCenterXY);
					stage.waitForMotionToStopAll();
					break;
				case ACQ:
					//Acquire a stack using the parameters:
					AcqStack acqStack{ commandline.mCommand.acqStack };

					wavelength_nm = acqStack.mWavelength_nm;
					scanDirZ = static_cast<ScanDirection>(acqStack.mScanDirZ);
					scanZi = acqStack.mScanZi;
					scanZf = scanZi + scanDirZ * acqStack.mStackDepth;
					scanPi = acqStack.mScanPi;
					stackPinc = acqStack.mStackPinc;

					//Update the laser parameters
					laser.setWavelength(wavelength_nm);
					laser.setPower(scanPi, stackPinc);

					//OPEN THE SHUTTER
					laser.openShutter();	//The destructor will close the shutter automatically

					image.initialize();
					std::cout << "Scanning the stack...\n";
					stage.moveSingle(ZZ, scanZf);		//Move the stage to trigger the control sequence and data acquisition
					image.downloadData();
					break;
				case SAV:
					filename = toString(wavelength_nm, 0) + "nm_Pi=" + toString(scanPi / mW, 1) + "mW_Pf=" + toString((scanPi + scanDirZ * stackPinc) / mW, 1) + "mW" +
						"_x=" + toString(stackCenterXY.at(XX) / mm, 3) + "_y=" + toString(stackCenterXY.at(YY) / mm, 3) +
						"_zi=" + toString(scanZi / mm, 4) + "_zf=" + toString(scanZf / mm, 4) + "_Step=" + toString(stepSizeZ / mm, 4);

					image.postprocess();
					image.saveTiffMultiPage(filename, NOOVERRIDE, scanDirZ);
					break;
				case CUT:
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

		pressAnyKeyToCont();
	}

}//namespace


namespace TestRoutines
{
	void fineTuneGalvoScan(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 30 };											//Number of frames for continuous XY acquisition
		const double3 stagePositionXYZ{ 35.05 * mm, 10.40 * mm, 18.204 * mm };	//Stage initial position

		//RS
		const ResonantScanner RScanner{ fpga };
		RScanner.isRunning();		//Make sure that the RS is running

		//STAGES
		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, RS, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		//GALVO
		const double FFOVgalvo{ 200. * um };			//Full FOV in the slow axis
		Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		//LASER
		const int wavelength_nm{ 1040 };
		const double P{ 25. * mW };		//Laser power
		VirtualLaser laser{ RTcontrol, wavelength_nm, P, FIDELITY };

		//ACQUIRE FRAMES AT DIFFERENT Zs
		stage.moveXYZ(stagePositionXYZ);
		stage.waitForMotionToStopAll();

		//OPEN THE UNIBLITZ SHUTTERS
		laser.openShutter();	//The destructor will close the shutter automatically

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{RTcontrol};
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition
		image.averageEvenOddFrames();
		image.saveTiffMultiPage("Untitled", NOOVERRIDE);

		pressAnyKeyToCont();
	}

	//Generate many short digital pulses and check the overall frameDuration with the oscilloscope
	void digitalLatency(const FPGAns::FPGA &fpga)
	{
		const double step{ 4. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };

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
		const double delay{ 400. * us };
		const double step{ 4. * us };

		FPGAns::RTcontrol RTcontrol(fpga);
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 10 * V);				//Initial pulse
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 0);
		RTcontrol.pushLinearRamp(RTGALVO1, 4 * us, delay, 0, 5 * V);		//Linear ramp to accumulate the error
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 10 * V);				//Initial pulse
		RTcontrol.pushAnalogSinglet(RTGALVO1, step, 0);						//Final pulse

		//DO0
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, delay, 0);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
	}

//For keeping the pockels on to check the the laser power
//0. Make sure that the function generator feeds the lineclock
//1. Manually open the Vision shutter and Uniblitz shutter. The latter because the class destructor closes the shutter automatically
//2. Set pockelsAutoOff = DISABLE for holding the last value
//3. Tune Vision's wavelength manually
	void pockels(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		//DEFINE THE POCKELS CELLS
		PockelsCell pockelsVision{ RTcontrol, 750, VISION };			//Vision
		PockelsCell pockelsFidelity{ RTcontrol, 1040, FIDELITY };		//Fidelity

		PockelsCell pockels{ pockelsVision };
		//PockelsCell pockels{ pockelsFidelity };
		pockels.pushPowerSinglet(8 * us, 20 * mW);
		//pockels.pushPowerSinglet(8 * us, 0 * mW);
		//pockels.pushVoltageSinglet(8 * us, 1.0 * V);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		Image image{ RTcontrol };
		image.acquire();

		//pressAnyKeyToCont();
	}

	void photobleach(const FPGAns::FPGA &fpga)
	{
		//RS
		ResonantScanner RScanner{ fpga };
		RScanner.isRunning();		//Make sure that the RS is running

		Laser laser{ VISION };
		laser.setWavelength(920);

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, FG, 50 };

		//GALVO
		Galvo galvo{ RTcontrol, RTGALVO1, 0 };	//Keep the galvo fixed to photobleach a line on the sample

		//POCKELS CELLS
		PockelsCell pockels{ RTcontrol, 920, VISION };
		pockels.pushPowerSinglet(8 * us, 200 * mW);

		//LOAD AND EXECUTE THE CONTROL SEQUENCE ON THE FPGA
		pockels.setShutter(true);
		Image image{ RTcontrol };
		image.acquire();
		pockels.setShutter(false);

		pressAnyKeyToCont();
	}

	void pockelsRamp(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 10 };			//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, FG, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		//POCKELS CELL
		const int wavelength_nm{ 750 };
		const double laserPower{ 25. * mW };		//Laser power
		PockelsCell pockels{ RTcontrol, wavelength_nm, VISION };

		//Test the voltage setpoint
		//pockels.pushVoltageSinglet(8 * us, 0.375 * V);
		//pockels.voltageLinearRamp(0.25 * V, 0.5 * V);		//Linearly scale the pockels voltage from the first to the last frame

		//Test the laser power setpoint
		//pockels.pushPowerSinglet(8 * us, laserPower);
		//pockels.powerLinearRamp(P, 2 * laserPower);		//Linearly scale the laser power from the first to the last frame

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition

		pressAnyKeyToCont();
	}


	void galvo(const FPGAns::FPGA &fpga)
	{
		const int width_pix{ 300 };
		const int height_pix{ 400 };
		const int nFramesDiscont{ 1 };
		const int nFramesCont{ 10 };

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, FG, nFramesCont, width_pix, height_pix };

		//GALVO
		const double FFOVgalvo{ 200. * um };				//Full FOV in the slow axis
		Galvo galvo{ RTcontrol, RTGALVO1, FFOVgalvo / 2 };

		for (int iter = 0; iter < nFramesDiscont; iter++)
		{
			std::cout << "Iteration: " << iter + 1 << "\n";

			//Execute the realtime control sequence and acquire the image
			Image image{ RTcontrol };
			image.acquire(); //Execute the RT control sequence and acquire the image
			image.saveTiffSinglePage("Untitled", OVERRIDE);
		}
	}

	void pixelclock(const FPGAns::FPGA &fpga)
	{
		std::vector<U8> stackOfAverages;

		FPGAns::RTcontrol RTcontrol{ fpga }; 		//Create a realtime control sequence
		Image image{ RTcontrol };
		image.acquire();							//Execute the realtime control sequence and acquire the image
		//image.pushToVector(stackOfAverages);
		//std::cout << "size: " << stackOfAverages.size() << "\n";
		//TiffU8 acqParam{ stackOfAverages, 300, 400 };
		//acqParam.saveTiff("Untitled");

		pressAnyKeyToCont();
	}

	//Test the analog and digital output and the relative timing wrt the pixel clock
	void analogAndDigitalOut(const FPGAns::FPGA &fpga)
	{
		FPGAns::RTcontrol RTcontrol{ fpga };

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
		const double Vmax{ 5. * V };
		const double step{ 4. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };
		RTcontrol.pushLinearRamp(RTGALVO1, step, 2 * ms, 0, -Vmax);
		RTcontrol.pushLinearRamp(RTGALVO1, step, 20 * ms, -Vmax, Vmax);
		RTcontrol.pushLinearRamp(RTGALVO1, step, 2 * ms, Vmax, 0);

		const double pulsewidth(300. * us);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, pulsewidth, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, 4 * us, 0);
	}

	//Generate a long digital pulse and check the frameDuration with the oscilloscope
	void digitalTiming(const FPGAns::FPGA &fpga)
	{
		const double step{ 400. * us };

		FPGAns::RTcontrol RTcontrol{ fpga };
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 1);
		RTcontrol.pushDigitalSinglet(RTDODEBUG, step, 0);
	}

	void filterwheel()
	{
		Filterwheel FWexcitation{ FWEXC };
		Filterwheel FWdetection{ FWDET };

		int wavelength_nm;
		if (0)
			wavelength_nm = 1040;
		else
			wavelength_nm = 750;

		//FWexcitation.setWavelength(wavelength);
		//FWdetection.setWavelength(wavelength);
		std::thread th1{ &Filterwheel::setWavelength, &FWexcitation, wavelength_nm };
		std::thread th2{ &Filterwheel::setWavelength, &FWdetection, wavelength_nm };
		th1.join();
		th2.join();

		pressAnyKeyToCont();
	}

	void shutter(const FPGAns::FPGA &fpga)
	{
		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga };

		PockelsCell fidelity{ RTcontrol, 1040, FIDELITY };
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
		Laser laser{ VISION };
		//Laser laser{FIDELITY};
		//std::cout << laser.isShutterOpen() << std::endl;
		//laser.setShutter(false);
		laser.setWavelength(1040);
		//laser.printWavelength_nm();

		pressAnyKeyToCont();
	}

	void virtualLasers(const FPGAns::FPGA &fpga)
	{
		//ACQUISITION SETTINGS
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const int nFramesCont{ 1 };			//Number of frames for continuous XY acquisition

		//CREATE A REALTIME CONTROL SEQUENCE
		FPGAns::RTcontrol RTcontrol{ fpga, FG, nFramesCont, widthPerFrame_pix, heightPerFrame_pix };

		const int wavelength_nm{ 920 };
		const double laserPower{ 50. * mW };		//Laser power
		VirtualLaser laser{ RTcontrol, wavelength_nm, laserPower };

		//EXECUTE THE RT CONTROL SEQUENCE
		Image image{ RTcontrol };
		image.acquire();		//Execute the RT control sequence and acquire the image via continuous XY acquisition

		//laser.openShutter();
		//Sleep(3000);
		//laser.closeShutter();

		//pressAnyKeyToCont();
	}

	void resonantScanner(const FPGAns::FPGA &fpga)
	{
		ResonantScanner RScanner{ fpga };
		std::cout << "aaa = " << RScanner.downloadControlVoltage() << "\n";
		//RScanner.turnOn(150 * um);
		//RScanner.turnOff();
	}

	void convertI16toVolt()
	{
		std::cout << "volt to I16: " << FPGAns::voltageToI16(1) << "\n";
		std::cout << "I16 to colt: " << FPGAns::I16toVoltage(32767) << "\n";
		std::cout << "volt to I16 to volt: " << FPGAns::I16toVoltage(FPGAns::voltageToI16(0)) << "\n";
		pressAnyKeyToCont();
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
		image.saveToFile(outputFilename, MULTIPAGE, OVERRIDE);

		//image.saveToFile(outputFilename, 2);

		//image.mirrorOddFrames(nFramesCont);
		//image.averageEvenOddFrames(nFramesCont);

		pressAnyKeyToCont();
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
		image.saveToFile(filename, SINGLEPAGE, OVERRIDE);

		//Stop the stopwatch
		duration = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
		std::cout << "Elapsed time: " << duration << " ms" << "\n";

		pressAnyKeyToCont();
	}

	void vibratome(const FPGAns::FPGA &fpga)
	{
		const double slicePlaneZ = 22.900 * mm;

		Stage stage{ 5. * mmps, 5. * mmps, 0.5 * mmps };
		Vibratome vibratome{ fpga, stage };
		vibratome.slice(slicePlaneZ);

		const double3 samplePosition{ 0. * mm, 0. * mm, slicePlaneZ };
		stage.moveXYZ(stackCenterXYZ);

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

		unsigned int n{ std::thread::hardware_concurrency() };
		std::cout << n << " concurrent threads are supported.\n";

		std::cout << "func1 and func2 will execute concurrently\n";

		FUNC x{ 1 };

		std::thread first{ &FUNC::func1, &x, 123 };
		std::thread second{ &FUNC::func2, &x, 314 };

		first.join();//pauses until first finishes
		second.join();//pauses until second finishes

		pressAnyKeyToCont();
	}

	void sequencerSim()
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
		const int widthPerFrame_pix{ 300 };
		const int heightPerFrame_pix{ 400 };
		const double2 FFOV{ 200. * um, 150. * um };
		const int nFramesCont{ 80 };										//Number of frames for continuous XYZ acquisition. If too big, the FPGA FIFO will overflow and the data transfer will fail
		const double stepSizeZ{ 0.5 * um };									//Step size in z
		const ROI roi{ 9.950 * mm, 34.850 * mm, 10.150 * mm, 35.050 * mm }; //Region of interest {ymin, xmin, ymax, xmax}
		const double3 stackOverlapXYZ_frac{ 0.05, 0.05, 0.05 };				//Stack overlap
		const double cutAboveBottomOfStack{ 15. * um };						//height to cut above the bottom of the stack
		const double sampleLengthZ{ 0.01 * mm };								//Sample thickness
		const double sampleSurfaceZ{ 18.471 * mm };

		//const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW }, { 1040, 30. * mW, 0. * mW } };
		const std::vector<LaserList::SingleLaser> laserList{ { 750, 60. * mW, 0. * mW } };
		//const std::vector<LaserList::SingleLaser> laserList{{ 1040, 25. * mW, 0. * mW } };
		Sample sample{ "Beads4um", "Grycerol", "1.47", roi, sampleLengthZ, sampleSurfaceZ, cutAboveBottomOfStack };
		Stack stack{ FFOV, stepSizeZ, nFramesCont, stackOverlapXYZ_frac };

		//Create a sequence
		Sequencer sequence{ laserList, sample, stack };
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
				case MOV:
					std::cout << "MOV " << "\n";

					//Skip the first MOV to position the stage
					if (iterCommandline != 0)
						moveStage = std::thread{ &FUNC::func2, &x, 314 };

					std::cout << "saveFile joinable? " << std::boolalpha << saveFile.joinable() << "\n";
					std::cout << "moveStage joinable? " << std::boolalpha << moveStage.joinable() << "\n";

					break;
				case ACQ:
					std::cout << "ACQ " << "\n";

					if (saveFile.joinable() && moveStage.joinable())
					{
						saveFile.join();
						moveStage.join();
					}
					break;
				case SAV:
					std::cout << "SAV" << "\n";
					saveFile = std::thread{ &FUNC::func1, &x, 123 };
					break;
				case CUT:
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
		pressAnyKeyToCont();
	}
}//namespace