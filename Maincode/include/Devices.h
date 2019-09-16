#pragma once
#include <future>
#include <fstream>										//file management
#include <ctime>										//Clock()
#include <algorithm>									//std::max and std::min
#include "FPGAapi.h"
#include "PI_GCS2_DLL.h"
#include "serial/serial.h"
#include <memory>										//For smart pointers
#include "Thorlabs.MotionControl.KCube.StepperMotor.h"	//For the Thorlabs stepper
#include "SampleConfig.h"

class Image final
{
public:
	Image(const RTseq &realtimeSeq);
	~Image();
	Image(const Image&) = delete;				//Disable copy-constructor
	Image& operator=(const Image&) = delete;	//Disable assignment-constructor
	Image(Image&&) = delete;					//Disable move constructor
	Image& operator=(Image&&) = delete;			//Disable move-assignment constructor

	U8* const data() const;
	void acquire(const bool saveAllPMT = false);
	void acquireVerticalStrip(const SCANDIR scanDirX);
	void correct(const double FFOVfast);
	void correctRSdistortion(const double FFOVfast);
	void averageFrames();
	void averageEvenOddFrames();
	void binFrames(const int nFramesPerBin);
	void save(std::string filename, const TIFFSTRUCT pageStructure, const OVERRIDE override) const;
private:
	const RTseq &mRTcontrol;					//Const because the variables referenced by mRTcontrol are not changed by the methods in this class
	TiffU8 mTiff;								//Tiff that stores the content of mBufferA and mBufferB

	void demultiplex_(const bool saveAllPMT);
	void demuxSingleChannel_();
	void demuxAllChannels_(const bool saveAllPMT);
};

class ResonantScanner final
{
public:
	double mFillFactor;		//Fill factor: how much of an RS swing is covered by the pixels
	double mFFOV;			//Current FFOV
	double mSampRes;		//Spatial sampling resolution (length/pixel)

	ResonantScanner(const RTseq &realtimeSeq);
	ResonantScanner(const ResonantScanner&) = delete;				//Disable copy-constructor
	ResonantScanner& operator=(const ResonantScanner&) = delete;	//Disable assignment-constructor
	ResonantScanner(ResonantScanner&&) = delete;					//Disable move constructor
	ResonantScanner& operator=(ResonantScanner&&) = delete;			//Disable move-assignment constructor

	void setFFOV(const double FFOV);
	void turnOn(const double FFOV);
	void turnOnUsingVoltage(const double controlVoltage);
	void turnOff();
	double downloadControlVoltage() const;
	void isRunning() const;
private:
	const RTseq &mRTcontrol;								//Needed to retrieve 'mRTcontrol.mWidthPerFrame_pix' to calculate the fill factor
	const double mVMAX{ 5. * V };							//Max control voltage allowed
	const double mDelay{ 10. * ms };
	const double mVoltagePerDistance{ 0.00595 * V / um };	//Calibration factor. Last calibrated 
	double mFullScan;										//Full scan = distance between turning points
	double mControlVoltage;									//Control voltage 0-5V

	void setVoltage_(const double controlVoltage);
};

class PMT16X final
{
public:
	PMT16X();
	~PMT16X();
	PMT16X(const PMT16X&) = delete;				//Disable copy-constructor
	PMT16X& operator=(const PMT16X&) = delete;	//Disable assignment-constructor
	PMT16X(PMT16X&&) = delete;					//Disable move constructor
	PMT16X& operator=(PMT16X&&) = delete;		//Disable move-assignment constructor

	void readAllGains() const;
	void setSingleGain(const RTseq::PMT16XCHAN chan, const int gain) const;
	void setAllGainToZero() const;
	void setAllGains(const int gain) const;
	void setAllGains(std::vector<uint8_t> gains) const;
	void suppressGainsLinearly(const double scaleFactor, const RTseq::PMT16XCHAN lowerChan, const RTseq::PMT16XCHAN higherChan) const;
	void readTemp() const;
private:
	std::unique_ptr<serial::Serial> mSerial;
	const COM mPort{ COM::PMT16X };
	const int mBaud{ 9600 };
	const int mTimeout{ 300 * ms };
	const int mRxBufferSize{ 256 };				//Serial buffer size

	std::vector<uint8_t> sendCommand_(std::vector<uint8_t> command) const;
	int PMT16XCHANtoInt_(const RTseq::PMT16XCHAN chan) const;
	uint8_t sumCheck_(const std::vector<uint8_t> input, const int index) const;		//The PMT requires a sumcheck. Refer to the manual
};

class Stage final
{
public:
	enum class DOPARAM { TRIGSTEP = 1, AXISNUMBER = 2, TRIGMODE = 3, POLARITY = 7, STARTTHRES = 8, STOPTHRES = 9, TRIGPOS = 10 };		//*cast
	enum class DOTRIGMODE { POSDIST = 0, ONTARGET = 2, INMOTION = 6, POSOFFSET = 7 };
	enum class DIOCHAN { D1 = 1, D2 = 2 };
	const std::vector<LIMIT2> mTravelRangeXYZ{ { -65. * mm, 65. * mm }, { -30. * mm, 30. * mm }, { 0. * mm, 26. * mm } };				//Travel range set by the physical limits of the stage

	Stage(const double velX, const double velY, const double velZ, const std::vector<LIMIT2> stageSoftPosLimXYZ = { {0,0},{0,0},{0,0} });
	~Stage();
	Stage(const Stage&) = delete;				//Disable copy-constructor
	Stage& operator=(const Stage&) = delete;	//Disable assignment-constructor
	Stage(Stage&&) = delete;					//Disable move constructor
	Stage& operator=(Stage&&) = delete;			//Disable move-assignment constructor

	POSITION3 readPosXYZ() const;
	void printPosXYZ() const;
	void moveSingle(const AXIS stage, const double position);
	void moveXY(const POSITION2 posXY);
	void moveXYZ(const POSITION3 posXYZ);
	bool isMoving(const AXIS axis) const;
	void waitForMotionToStopSingle(const AXIS axis) const;
	void waitForMotionToStopAll() const;
	void stopAll() const;
	void setVelSingle(const AXIS axis, const double vel);
	void setVelXYZ(const VELOCITY3 vel);
	void printVelXYZ() const;
	void setDOtriggerParamSingle(const AXIS axis, const DIOCHAN DIOchan, const DOPARAM triggerParamID, const double value) const;
	void setDOtriggerParamAll(const AXIS axis, const DIOCHAN DOchan, const double triggerStep, const DOTRIGMODE triggerMode, const double startThreshold, const double stopThreshold) const;
	bool isDOtriggerEnabled(const AXIS axis, const DIOCHAN DOchan) const;
	void setDOtriggerEnabled(const AXIS axis, const DIOCHAN DOchan, const BOOL triggerState) const;
	void printStageConfig(const AXIS axis, const DIOCHAN chan) const;
private:
	const int mPort_z{ 4 };										//COM port
	const int mBaud_z{ 38400 };
	std::array<int, 3> mHandleXYZ;								//Stage handler

	const char mNstagesPerController[2]{ "1" };					//Number of stages per controller (currently 1)
	POSITION3 mPosXYZ;											//Absolute position of the stages
	VELOCITY3 mVelXYZ;											//Velocity of the stages
	std::vector<LIMIT2> mSoftPosLimXYZ{ {0,0},{0,0},{0,0} };	//Travel soft limits (may differ from the hard limits stored in the internal memory of the stages)
																//Initialized with invalid values (lower limit = upper limit) for safety. It must be overridden by the constructor
	double readCurrentPosition_(const AXIS axis) const;
	void setCurrentPosition_(const AXIS axis, const double position);
	double readCurrentVelocity_(const AXIS axis) const;
	void setCurrentVelocity_(const AXIS axis, const double velocity);
	double downloadPositionSingle_(const AXIS axis);
	double downloadVelSingle_(const AXIS axis) const;
	double downloadDOtriggerParamSingle_(const AXIS axis, const DIOCHAN DOchan, const DOPARAM triggerParamID) const;
	void configDOtriggers_() const;
	std::string convertAxisToString_(const AXIS axis) const;
};

class Vibratome
{
public:
	const POSITION2 mStageInitialSlicePosXY{ -53. * mm, 2. * mm };	//Position the stages in front oh the vibratome's blade
	const double mStageFinalSlicePosY{ 20. * mm };					//Final position of the Y-stage after slicing

	Vibratome(const FPGA &fpga, Stage &stage);
	Vibratome(const Vibratome&) = delete;							//Disable copy-constructor
	Vibratome& operator=(const Vibratome&) = delete;				//Disable assignment-constructor
	Vibratome(Vibratome&&) = delete;								//Disable move constructor
	Vibratome& operator=(Vibratome&&) = delete;						//Disable move-assignment constructor

	void pushStartStopButton() const;
	void sliceTissue(const double planeZtoCut);
private:
	const FPGA &mFpga;
	Stage &mStage;

	const double mSlicingVel{ 0.5 * mmps };											//Move the Y-stage at this velocity for slicing
	const VELOCITY3 mStageConveyingVelXYZ{ 10. * mmps, 10.  *mmps, 0.5 * mmps };	//Transport the sample between the objective and vibratome at this velocity
	//enum MotionDir { BACKWARD = -1, FORWARD = 1 };
	//double mCuttingSpeed{ 0.5 * mmps };	//Speed of the vibratome for cutting (manual setting)
	//double mMovingSpeed{ 2.495 * mmps };	//Measured moving speed of the head: 52.4 mm in 21 seconds = 2.495 mm/s. Set by hardware. Cannot be changed
	//double mTravelRange{ 52.4 * mm };		//(horizontal) travel range of the head. I measured 104.8 seconds at 0.5 mm/s = 52.4 mm
	//void moveHead_(const double duration, const MotionDir motionDir) const;
	//void cutAndRetractDistance(const double distance) const;
	//void retractDistance(const double distance) const;
};

class Filterwheel final
{
public:
	enum class ID { DET, EXC };
	enum class COLOR { BLUE, GREEN, RED, OPEN, CLOSED };
	Filterwheel(const ID whichFilterwheel);
	~Filterwheel();
	Filterwheel(const Filterwheel&) = delete;				//Disable copy-constructor
	Filterwheel& operator=(const Filterwheel&) = delete;	//Disable assignment-constructor
	Filterwheel(Filterwheel&&) = delete;					//Disable move constructor
	Filterwheel& operator=(Filterwheel&&) = delete;			//Disable move-assignment constructor

	void setColor(const COLOR color);
	void setWavelength(const int wavelength_nm);
private:
	/* Excitation wheel with the beamsplitters (as of Feb 2019)
	position #1, 750 nm beamsplitter
	position #2, open (no beamsplitter)
	position #3, 920 nm beamsplitter
	position #4, open (no beamsplitter)
	position #5, 1040 nm beamsplitter
	position #6, open (no beamsplitter)
	*/
	const std::vector<COLOR> mExcConfig{ COLOR::BLUE, COLOR::OPEN, COLOR::GREEN, COLOR::OPEN, COLOR::RED, COLOR::OPEN };

	/* Detection wheel with the emission filters (as of Feb 2019)
	position #1, FF01-492/sp (blue)
	position #2, FF01-520/60 (green)
	position #3, BLP01-532R (red)
	position #4, beam block
	position #5, FF01-514/44 (green)
	position #6, open (no filter)
	*/
	const std::vector<COLOR> mDetConfig{ COLOR::BLUE, COLOR::OPEN, COLOR::RED, COLOR::CLOSED, COLOR::GREEN, COLOR::OPEN };
	//Currently, there are 2 green filters set up in the wheel (pos #2 and #5). I write  COLOR::GREEN at the second position to use FF01-520/60 or at the fifth position to use FF01-514/44
	//Leave the unused filter as COLOR::OPEN
		
	ID mWhichFilterwheel;							//Device ID = 1, 2, ...
	std::string mFilterwheelName;					//Device given name
	std::vector<COLOR> mFWconfig;					//Store the filterwheel configuration for excitation or detection
	COLOR mColor;									//Current filterwheel color
	int mPosition;									//Current filterwheel position
	std::unique_ptr<serial::Serial> mSerial;
	COM mPort;
	const int mBaud{ 115200 };
	const int mTimeout{ 150 * ms };
	const int mNpos{ 6 };							//Number of filter positions
	const double mTurningSpeed{ 0.8 / seconds };	//The measured filterwheel turning speed is ~ 1 position/s. Choose a slightly smaller value
	const int mRxBufSize{ 256 };					//Serial buffer size

	int downloadPosition_() const;
	int convertColorToPosition_(const COLOR color) const;
	COLOR convertPositionToColor_(const int position) const;
	std::string convertColorToString_(const COLOR color) const;
};

class CombinedFilterwheel
{
public:
	CombinedFilterwheel();
	void turnFilterwheels(const int wavelength_nm);
private:
	Filterwheel mFWexcitation;
	Filterwheel mFWdetection;
};

class Laser final
{
public:
	enum class ID { VISION, FIDELITY, AUTO };
	std::string laserName;

	Laser(const ID whichLaser);
	~Laser();
	Laser(const Laser&) = delete;				//Disable copy-constructor
	Laser& operator=(const Laser&) = delete;	//Disable assignment-constructor
	Laser(Laser&&) = delete;					//Disable move constructor
	Laser& operator=(Laser&&) = delete;			//Disable move-assignment constructor

	void printWavelength_nm() const;
	void setWavelength(const int wavelength_nm);
	void setShutter(const bool state) const;
	bool isShutterOpen() const;
	int readCurrentWavelength_nm() const;
private:
	ID mWhichLaser;
	int mWavelength_nm;
	std::unique_ptr<serial::Serial> mSerial;
	COM  mPort;
	int mBaud;
	const int mTimeout{ 100 * ms };
	const double mTuningSpeed{ 35. / seconds };	//in nm per second. The measured laser tuning speed is ~ 40 nm/s. Choose a slightly smaller value
	const int mRxBufSize{ 256 };				//Serial buffer size

	int downloadWavelength_nm_() const;
};

class Shutter final
{
public:
	Shutter(const FPGA &fpga, const Laser::ID whichLaser);
	~Shutter();

	void setState(const bool state) const;
	void pulse(const double pulsewidth) const;
private:
	const FPGA &mFpga;
	NiFpga_FPGAvi_ControlBool mWhichShutter;	//Device ID
};

class Pockels final
{
public:
	Pockels(RTseq &realtimeSeq, const int wavelength_nm, const Laser::ID laserSelector);	//Do not set the output to 0 through the destructor to allow latching the last value

	void pushVoltageSinglet(const double timeStep, const double AO, const OVERRIDE override) const;
	void pushPowerSinglet(const double timeStep, const double P, const OVERRIDE override) const;
	void setVoltageToZero() const;
	//void voltageLinearScaling(const double Vi, const double Vf) const;
	void pushPowerLinearScaling(const double Pi, const double Pf) const;
	void pushPowerExponentialScaling(const double Pmin, const double interframeDistance, const double decayLengthZ) const;
	void setShutter(const bool state) const;
private:
	RTseq &mRTcontrol;							//Non-const because the laser power is pushed into the queues in RTseq						
	RTseq::RTCHAN mPockelsRTchan;
	RTseq::RTCHAN mScalingRTchan;
	int mWavelength_nm;							//Laser wavelength
	const double timeStep{ 8. * us };
	double mMaxPower;							//Softlimit for the laser power
	const Shutter mShutter;

	double convertPowerToVolt_(const double power) const;
};

//Integrate Laser, Shutter, and Pockels classes in a single class
class VirtualLaser
{
public:
	VirtualLaser(const Laser::ID whichLaser = Laser::ID::AUTO);
	Laser::ID readCurrentLaser() const;
	std::string readCurrentLaser_s(const bool justTheNameInitials) const;
	int readCurrentWavelength_nm() const;
	void isLaserInternalShutterOpen() const;
	void setWavelength(RTseq &realtimeSeq, const int wavelength_nm);
	void setPowerLinearScaling(const double Pi, const double Pf) const;
	void setPowerExponentialScaling(const double Pmin, const double distancePerFrame, const double decayLengthZ) const;
	void openShutter() const;
	void closeShutter() const;
private:
	Laser::ID mWhichLaser;							//use VISION, FIDELITY, or AUTO (let the code decide)
	Laser::ID mCurrentLaser;						//Current laser in use: VISION or FIDELITY
	Laser mVision;
	Laser mFidelity;
	std::unique_ptr <Pockels> mPockelsPtr;			//Create a pockels handle dynamically. Alternatively, I could create a fixed handle for each wavelength used
	const double mPockelTimeStep{ 8. * us };		//Time step for the pockels sequence

	std::string convertLaserNameToString_(const Laser::ID whichLaser) const;
	Laser::ID autoSelectLaser_(const int wavelength_nm) const;
};

class StepperActuator
{
public:
	StepperActuator(const char* serialNumber);
	~StepperActuator();
	StepperActuator(const StepperActuator&) = delete;				//Disable copy-constructor
	StepperActuator& operator=(const StepperActuator&) = delete;	//Disable assignment-constructor
	StepperActuator(StepperActuator&&) = delete;					//Disable move constructor
	StepperActuator& operator=(StepperActuator&&) = delete;			//Disable move-assignment constructor

	void move(const double position);
	void downloadConfig() const;
	void moveHome();
private:
	//To obtain the calibration, use Thorlabs APT software to set the position in mm, then read the position in internal-units via downloadConfig() implemented in this class
	double mPosition;

	const char* mSerialNumber;									//Each Thorlabs actuator has a unique serial number
	const double mCalib{ 26000000. / (12.9442 * mm) };			//Calibration factor to convert mm to the actuator's internal units
	const std::vector<double> mPosLimit{ 0. * mm, 13. * mm };
	const int mVel_iu{ 323449856 };								//Equivalent to 3 mm/s
	const int mAcc_iu{ 11041 };									//Equivalent to 0.5 mm/s^2
};

class CollectorLens final: public StepperActuator
{
public:
	CollectorLens();
	void set(const int wavelength_nm);
};

class Galvo final
{
public:
	Galvo::Galvo(RTseq &realtimeSeq, const double posMax);
	Galvo(RTseq &realtimeSeq, const double posMax, const Laser::ID whichLaser, const int wavelength_nm);
	Galvo(const Galvo&) = delete;							//Disable copy-constructor
	Galvo& operator=(const Galvo&) = delete;				//Disable assignment-constructor
	Galvo(Galvo&&) = delete;								//Disable move constructor
	Galvo& operator=(Galvo&&) = delete;						//Disable move-assignment constructor

	void setVoltageToZero() const;
	void pushVoltageSinglet(const double timeStep, const double AO) const;
	void pushVoltageLinearRamp(const double timeStep, const double rampLength, const double Vi, const double Vf, const OVERRIDE override) const;
	void pushPositionLinearRamp(const double posInitial, const double posFinal, const double posOffset, const OVERRIDE override) const;
private:
	const double mRampDurationFineTuning{ -30. * us };		//Slightly decrease the ramp duration, otherwise the ramp overflow in each frame accumulates over a continuous scan (e.g. over 200 frames)
															//Ideally, the ramp duration of the galvo is exactly g_lineclockHalfPeriod * mRTcontrol.mHeightPerBeamletPerFrame_pix
															//However, in practice g_lineclockHalfPeriod  is not fixed but seems to depend on the RS amplitude
															//If mRampDurationFineTuning is changed, then g_scannerDelay has to be readjusted to match the galvo's forward and backward scans

	const double mInterBeamletDistance{ 17.5 * um };		//Set by the beamsplitter specs
	RTseq &mRTcontrol;										//Non-const because some methods in this class change the variables referenced by mRTcontrol	
	RTseq::RTCHAN mWhichGalvo;
	double mVoltagePerDistance;
	double mVoltageOffset;
	double mPosMax;
	double readSinglebeamVoltageOffset_() const;
};

//Integrate VirtualLaser, CombinedFilterwheel, and CollectorLens classes in a single class
class Mesoscope : public VirtualLaser, public Vibratome
{
public:
	Mesoscope(RTseq &rtseq, const Laser::ID whichLaser = Laser::ID::AUTO);
	Mesoscope(const Mesoscope&) = delete;				//Disable copy-constructor
	Mesoscope& operator=(const Mesoscope&) = delete;	//Disable assignment-constructor
	Mesoscope(Mesoscope&&) = delete;					//Disable move constructor
	Mesoscope& operator=(Mesoscope&&) = delete;			//Disable move-assignment constructor

	void configure(const int wavelength_nm);
	void setPower(const double laserPower) const;
	void openShutter() const;
	void moveCollectorLens(const double position);

	void waitForMotionToStopAll();
	void setVelSingle(const AXIS axis, const double vel);
	void moveSingle(const AXIS stage, const double position);
	void moveXY(const POSITION2 posXY);
	void moveXYZ(const POSITION3 posXYZ);
private:
	RTseq &mRTseq;
	CombinedFilterwheel mVirtualFilterWheel;
	CollectorLens mCollectorLens;
	Stage mStage;

	POSITION3 determineChromaticShiftXYZ_();
};