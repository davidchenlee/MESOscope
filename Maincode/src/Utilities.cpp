#include "Utilities.h"

//Convert an int to hex and print it out
void printHex(const int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << "\n";
}

//Convert an unsigned char to hex and print it out
void printHex(const std::vector<uint8_t>  input)
{
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << static_cast<int>(input[ii]);
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << "\n";
}

//Convert a string to hex and print it out
void printHex(const std::string input)
{
	const char *cstr{ input.c_str() };
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << static_cast<int>(cstr[ii]);
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << "\n";
}

void printBinary16(const int input)
{
	std::cout << std::bitset<16>(input) << "\n";
}

//Convert a double to a fixed 2p14
U16 doubleToFx2p14(double n)
{
	const int FIXED_BIT{ 14 }; //Number of decimal digits. It MUST match the LV implementation: currently fx2.14 (U16 is split into 2 integer digits + 14 decimal digits)
	U16 int_part{ 0 }, frac_part{ 0 };
	double t;
	int_part = static_cast<U16>(floor(n)) << FIXED_BIT;
	n -= static_cast<U16>(floor(n));

	t = 0.5;
	for (int i = 0; i < FIXED_BIT; i++) {
		if ((n - t) >= 0) {
			n -= t;
			frac_part += (1 << (FIXED_BIT - 1 - i));
		}
		t = t / 2.;
	}

	return int_part + frac_part;
}

int SCANDIRtoInt(const SCANDIR scanDir)
{
	switch (scanDir)
	{
	case SCANDIR::RIGHTWARD:
	case SCANDIR::INWARD:
	case SCANDIR::UPWARD:
		return +1;
	case SCANDIR::LEFTWARD:
	case SCANDIR::OUTWARD:
	case SCANDIR::DOWNWARD:
		return -1;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}
}

//Clip x so that lower <= x <= upper
template<class T> inline T clip(T x, T lower, T upper)
{
	return (std::min)(upper, (std::max)(x, lower));
}

//Clip so that x <= 0xFF
template<class T> inline U8 clipU8top(const T x)
{
	return static_cast<U8>( (std::min)(x, static_cast<T>(255)) );
}
template U8 clipU8top(const int x);	//Allow calling the function from a different .cpp file

//Clip so that 0x00 <= x <= 0xFF
template<class T> inline U8 clipU8dual(const T x)
{
	return static_cast<U8>( (std::max)(static_cast<T>(0),(std::min)(x, static_cast<T>(255))) );
}

//Convert a double to a string with decimal places
std::string toString(const double number, const int nDecimalPlaces)
{
	std::ostringstream str;
	str << std::fixed << std::setprecision(nDecimalPlaces);
	str << number;
	return str.str();
}

//Check if the file already exists
std::string doesFileExist(const std::string filename)
{
	std::string suffix{ "" };

	for (int ii = 1; std::experimental::filesystem::exists(folderPath + filename + suffix + ".tif"); ii++)
		suffix = " (" + std::to_string(ii) + ")";

	return filename + suffix;
}

//Pause the sequence until any key is pressed
void pressAnyKeyToCont()
{
	std::cout << "\nPress any key to continue...\n";
	getchar();
}

//Early termination if ESC is pressed
void pressESCforEarlyTermination()
{
	if (GetAsyncKeyState(VK_ESCAPE) & 0x0001)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
}

void pressAnyKeyToContOrESCtoExit()
{
	char input_char;
	while (true)
	{
		std::cout << "\nPress any key to continue or ESC to exit\n";
		input_char = _getch();

		if (input_char == 27)
			throw std::runtime_error((std::string)__FUNCTION__ + ": Control sequence terminated");
		else
			break;//Break the while loop
	}
}

#pragma region "Logger"
Logger::Logger(const std::string filename)
{
	mFileHandle.open(folderPath + filename + ".txt");
};

Logger::~Logger()
{
	mFileHandle.close();
};

void Logger::record(const std::string description)
{
	mFileHandle << description + "\n";
}

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << "\n";
}

void Logger::record(const std::string description, const std::string input)
{
	mFileHandle << description << input << "\n";
}
#pragma endregion "Logger"

#pragma region "TiffU8"
//Construct a tiff from a file
TiffU8::TiffU8(const std::string filename) :
	mNframes{ 1 }
{
	TIFF *tiffHandle{ TIFFOpen((folderPath + filename + ".tif").c_str(), "r") };

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failed opening the Tiff file");

	//Read the Tiff tags
	int samplesPerPixel{ 0 }, bitsPerSample{ 0 };
	if (!TIFFGetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_SAMPLESPERPIXEL");
	if (!TIFFGetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, &bitsPerSample))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_BITSPERSAMPLE,");
	if(!TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &mHeightPerFrame_pix))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_IMAGELENGTH");
	if (!TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &mWidthPerFrame_pix))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_IMAGEWIDTH");

	//Reject unsupported file formats
	if (samplesPerPixel != 1 || bitsPerSample != 8)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Only 8-bit grayscale Tiff supported");

	if (mHeightPerFrame_pix % 2)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Odd number of rows not supported");

	//Read the number of frames from the ImageJ tags
	char* TIFFTAG_ImageJ;
	if (TIFFGetField(tiffHandle, TIFFTAG_IMAGEDESCRIPTION, &TIFFTAG_ImageJ))
	{
		std::string tiffTag{ TIFFTAG_ImageJ };
		//std::cout << tiffTag << "\n";	//For debugging

		std::string  keyword{ "slices=" };
		std::string::size_type keywordPos{ tiffTag.find(keyword) };	//Find the keyword in the string
		if (keywordPos != std::string::npos)
		{
			//std::cout << "found at: " << keywordPos << '\n';	//For debugging
			tiffTag.erase(tiffTag.begin(), tiffTag.begin() + keywordPos + keyword.length());	//Delete the beginning of the string until the end of the keyword
			std::string::size_type keywordPos{ tiffTag.find("\n") };							//Find the first ocurrence of '\n' in the remaining string
			if (keywordPos != std::string::npos)
			{
				//std::cout << "found at: " << keywordPos << '\n';								//For debugging
				tiffTag.erase(tiffTag.begin() + keywordPos, tiffTag.end());						//Delete the end of the string starting from the found '\n'
			}

			//std::cout << std::stoi(tiffTag) << "\n";	//For debugging
			mNframes = std::stoi(tiffTag);
		}
	}	
	//The pointer TIFFTAG_ImageJ can't be cleaned up with delete because it was passed by reference to TIFFGetField()

	//For debugging
	//std::cout << "Image pixel width = " << mWidthPerFrame_pix << "\n";
	//std::cout << "Image pixel height = " << mHeightPerFrame_pix << "\n";
	//std::cout << "Number of frames = " << mNframes << "\n";

	if (mHeightPerFrame_pix <= 0 || mWidthPerFrame_pix <= 0 || mNframes <= 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	mNpixPerFrame = mHeightPerFrame_pix * mWidthPerFrame_pix;
	mNpixAllFrames = mNpixPerFrame * mNframes;

	//Length in memory of one row of pixel in the image. Targeting 'U8' only. Alternatively, mBytesPerLine = TIFFScanlineSize(tiffHandle);
	mBytesPerLine = mWidthPerFrame_pix * sizeof(U8);	

	U8* buffer{ (U8*)_TIFFmalloc(mBytesPerLine) };

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	mArray = new U8[mNpixAllFrames];	//Allocate memory for the image

	for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
	{
		//Read the tiff one strip at a time
		for (int iterRow_pix = 0; iterRow_pix < mHeightPerFrame_pix; iterRow_pix++)
		{
			if (TIFFReadScanline(tiffHandle, buffer, iterRow_pix, 0) < 0)
				break;
			std::memcpy(&mArray[(iterFrame * mHeightPerFrame_pix + iterRow_pix) * mBytesPerLine], buffer, mBytesPerLine);
		}
		TIFFReadDirectory(tiffHandle);
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file. I hope the pointer TIFFTAG_ImageJ is cleaned up here
}

//Construct a Tiff from an array
TiffU8::TiffU8(const U8* inputImage, const int heightPerFrame_pix, const int widthPerFrame_pix, const int nFrames) :
	mHeightPerFrame_pix{ heightPerFrame_pix },
	mWidthPerFrame_pix{ widthPerFrame_pix },
	mNframes{ nFrames },
	mBytesPerLine{ static_cast<int>(widthPerFrame_pix * sizeof(U8)) },
	mNpixPerFrame{ heightPerFrame_pix * widthPerFrame_pix },
	mNpixAllFrames{ mNpixPerFrame * nFrames }
{
	if (mHeightPerFrame_pix <= 0 || mWidthPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	mArray = new U8[mNpixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, inputImage, mNpixAllFrames * sizeof(U8));
}

//Construct a Tiff from a vector
TiffU8::TiffU8(const std::vector<U8> &inputImage, const int heightPerFrame_pix, const int widthPerFrame_pix, const int nFrames) :
	mHeightPerFrame_pix{ heightPerFrame_pix },
	mWidthPerFrame_pix{ widthPerFrame_pix },
	mNframes{ nFrames },
	mBytesPerLine{ static_cast<int>(widthPerFrame_pix * sizeof(U8)) },
	mNpixPerFrame{ heightPerFrame_pix * widthPerFrame_pix },
	mNpixAllFrames{ mNpixPerFrame * nFrames }
{
	if (mHeightPerFrame_pix <= 0 || mWidthPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	mArray = new U8[mNpixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, &inputImage[0], mNpixAllFrames * sizeof(U8));
}

//Construct a new Tiff by allocating memory and initialize it to zero for safety
TiffU8::TiffU8(const int heightPerFrame_pix, const int widthPerFrame_pix, const int nFrames) :
	mHeightPerFrame_pix{ heightPerFrame_pix },
	mWidthPerFrame_pix{ widthPerFrame_pix },
	mNframes{ nFrames },
	mBytesPerLine{ static_cast<int>(widthPerFrame_pix * sizeof(U8)) },
	mNpixPerFrame{ heightPerFrame_pix * widthPerFrame_pix },
	mNpixAllFrames{ mNpixPerFrame * nFrames }
{
	if (mHeightPerFrame_pix <= 0 || mWidthPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	mArray = new U8[mNpixAllFrames]();
}

TiffU8::~TiffU8()
{
	delete[] mArray;
}

//Access the Tiff data in the TiffU8 object
U8* const TiffU8::data() const
{
	return mArray;
}

int TiffU8::heightPerFrame_pix() const
{
	return mHeightPerFrame_pix;
}

int TiffU8::widthPerFrame_pix() const
{
	return mWidthPerFrame_pix;
}

int TiffU8::nFrames() const
{
	return mNframes;
}

//Push a frame into mArray. The frame index starts from 0
void TiffU8::pushImage(const U8* inputArray, const int frameIndex) const
{
	if (frameIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be >= 0");
	if (frameIndex > mNframes)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be smaller than or equal to the number of frames " + std::to_string(mNframes));

	std::memcpy(&mArray[frameIndex * mHeightPerFrame_pix * mBytesPerLine], inputArray, mHeightPerFrame_pix * mBytesPerLine);
}

//Push a frame interval. The frame index starts from 0
void TiffU8::pushImage(const U8* inputArray, const int firstFrameIndex, const int lastFrameIndex) const
{
	if (firstFrameIndex < 0 || lastFrameIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be >= 0");
	if (firstFrameIndex > mNframes || lastFrameIndex > mNframes)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be smaller than or equal to the number of frames");
	if (lastFrameIndex < firstFrameIndex)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The last frame index must be >= the first frame index");

	std::memcpy(&mArray[firstFrameIndex * mHeightPerFrame_pix * mBytesPerLine], inputArray, (lastFrameIndex - firstFrameIndex + 1) * mHeightPerFrame_pix * mBytesPerLine);
}

//Divide the concatenated images in a stack of nFrames
void TiffU8::splitFrames(const int nFrames)
{
	if (nFrames <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame number must be >0");

	mNframes = nFrames;
	mHeightPerFrame_pix = mHeightPerFrame_pix / nFrames;
}

//Merge all the frames in a single image
void TiffU8::mergeFrames()
{
	mHeightPerFrame_pix = mNframes * mHeightPerFrame_pix;
	mNframes = 1;
}

//Re-order the input arrays so that channels belonging to the same frame are grouped together
void TiffU8::mergePMT16Xchan(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const
{
	/*
	The input arrays have the structure:
	inputArrayA = |CH00 f1|
				  |  .	  |
				  |CH00 fN|
				  |  .	  |
				  |  .	  |
				  |  .	  |
				  |CH07 f1|
				  |  .	  |
				  |CH07 fN|

	inputArrayB = |CH08 f1|
				  |  .	  |
				  |CH08 fN|
				  |  .	  |
				  |  .	  |
				  |  .	  |
				  |CH15 f1|
				  |  .	  |
				  |CH15 fN|

	"Merging" puts the channels belonging to the same frame together. The resulting structure is:
	mArray = |CH00 f1|
			 |  .	 |
			 |CH15 f1|
			 |CH15 f2|
			 |  .	 |
			 |  .	 |
			 |  .	 |
			 |CH00 f2|
			 |CH00 fN|
			 |  .	 |
			 |CH15 fN|
	Note in the figure above that the channel ordering within each frame is reversed wrt the next frame because of the bidirectionality of the scan galvo
	*/
	if (heightPerChannelPerFrame < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel height per channel must be >= 0");

	const int heightAllChannelsPerFrame{ g_nChanPMT * heightPerChannelPerFrame };
	const int heightPerChannelAllFrames{ heightPerChannelPerFrame * mNframes };
	const int nChanPMThalf{ g_nChanPMT / 2 };

	//Even 'iterFrame' (Raster scan the sample from the positive to the negative direction of the X-stage)
	for (int iterFrame = 0; iterFrame < mNframes; iterFrame += 2)
		for (int chanIndex = 0; chanIndex < nChanPMThalf; chanIndex++)
		{
			//CH00-CH07
			std::memcpy(&mArray[((15 - chanIndex) * heightPerChannelPerFrame + iterFrame * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(iterFrame * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			//CH08-CH15
			std::memcpy(&mArray[((7 - chanIndex) * heightPerChannelPerFrame + iterFrame * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(iterFrame * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}
	//Odd 'iterFrame' (Raster scan the sample from the negative to the positive direction of the X-stage)
	for (int iterFrame = 1; iterFrame < mNframes; iterFrame += 2)
		for (int chanIndex = 0; chanIndex < nChanPMThalf; chanIndex++)
		{
			//CH00-CH07
			std::memcpy(&mArray[(chanIndex * heightPerChannelPerFrame + iterFrame * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(iterFrame * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			//CH08-CH15
			std::memcpy(&mArray[((chanIndex + nChanPMThalf) * heightPerChannelPerFrame + iterFrame * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(iterFrame * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}
}

//Divide the concatenated images in a stack of nFrames and save it (the microscope concatenates all the images and hands over a long image that has to be resized into individual images)
void TiffU8::saveToFile(std::string filename, const TIFFSTRUCT tiffStruct, const OVERRIDE override, const SCANDIR scanDirZ) const
{
	int height_pix, width_pix, nFrames;

	//Multi page structure
	if (tiffStruct == TIFFSTRUCT::MULTIPAGE)
	{
		height_pix = mHeightPerFrame_pix;
		width_pix = mWidthPerFrame_pix;
		nFrames = mNframes;
	}
	//Single page
	else
	{
		height_pix = mHeightPerFrame_pix * mNframes;
		width_pix = mWidthPerFrame_pix;
		nFrames = 1;
	}

	/*For debugging
	std::cout << nFrames << "\n";
	std::cout << Pixel height << "\n";
	std::cout << Pixel width << "\n";
	*/

	if (override == OVERRIDE::DIS)
		filename = doesFileExist(filename);	//Check if the file exits. It gives some overhead

	TIFF *tiffHandle{ TIFFOpen((folderPath + filename + ".tif").c_str(), "w") };

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving " + filename + ".tif failed");

	U8 *buffer{ (U8*)_TIFFmalloc(mBytesPerLine) };	//Buffer used to store the row of pixel information for writing to file

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	//Choose whether to save the first frame at the top or bottom of the stack
	int frameIndex, lastFrame;
	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:	//Forward saving: the first frame is at the top of the stack
		frameIndex = 0;
		lastFrame = nFrames - 1;
		break;
	case SCANDIR::DOWNWARD:	//Reverse saving: the first frame is at the bottom of the stack
		frameIndex = nFrames - 1;
		lastFrame = 0;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}

	do
	{
		//TAGS
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, height_pix);										//Set the pixel height of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, width_pix);										//Set the pixel width of the image
		//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);							//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
		TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);											//Set number of channels per pixel
		TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);												//Set the size of the channels
		TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);								//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
		TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);							//Single channel with min as black				
		TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, width_pix));	//Set the strip size of the file to be size of one row of pixels
		//TIFFSetField(tiffHandle, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);									//Specify that it's a frame within the multipage file
		//TIFFSetField(tiffHandle, TIFFTAG_PAGENUMBER, frameIndex, nFrames);							//Specify the frame number

		//IMAGEJ TAG FOR USING HYPERSTACKS
		std::string TIFFTAG_ImageJ = "ImageJ=1.52e\nimages=" + std::to_string(nFrames) + "\nchannels=1\nslices=" + std::to_string(nFrames) + "\nhyperstack=true\nmode=grayscale\nunit=\\u00B5m\nloop=false ";
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEDESCRIPTION, TIFFTAG_ImageJ.c_str());

		//Write a frame to the file one strip at a time
		for (int iterRow_pix = 0; iterRow_pix < height_pix; iterRow_pix++)
		{
			std::memcpy(buffer, &mArray[(frameIndex * height_pix + iterRow_pix) * mBytesPerLine], mBytesPerLine);
			if (TIFFWriteScanline(tiffHandle, buffer, iterRow_pix, 0) < 0)
				break;
		}
		TIFFWriteDirectory(tiffHandle); //Create a page structure. This gives a large overhead

		if (frameIndex == lastFrame)
			break;

		frameIndex += SCANDIRtoInt(scanDirZ); //Increasing iterator for UPWARD. Decreasing for DOWNWARD
	} while (true);

	_TIFFfree(buffer);		//Destroy the buffer
	TIFFClose(tiffHandle);	//Close the output tiff file

	std::cout << "Successfully saved: " << filename << ".tif\n";
}

//Save mArray as a text file
void TiffU8::saveToTxt(const std::string filename) const
{
	std::ofstream fileHandle;									//Create output file
	fileHandle.open(folderPath + filename + ".txt");			//Open the file

	for (int iterPix = 0; iterPix < mNpixAllFrames; iterPix++)
		fileHandle << mArray[iterPix] << "\n";					//Write each element

	fileHandle.close();											//Close the txt file
}

//Mirror the odd frames (the frame index starts from 0) vertically because the galvo (vectical axis of the image) performs bi-directional scanning and the images from different frames are concatenated
void TiffU8::mirrorOddFrames()
{
	if (mNframes > 1)
	{
		U8 *buffer = new U8[mBytesPerLine];		//Buffer used to store a row of pixels

		for (int iterFrame = 1; iterFrame < mNframes; iterFrame += 2)
		{
			//Swap the first and last rows of the sub-image, then do the second and second last rows, etc
			for (int iterRow_pix = 0; iterRow_pix < mHeightPerFrame_pix / 2; iterRow_pix++)
			{
				const int eneTene{ iterFrame * mHeightPerFrame_pix + iterRow_pix };				//Swap this row
				const int moneMei{ (iterFrame + 1) * mHeightPerFrame_pix - iterRow_pix - 1 };		//With this one
				std::memcpy(buffer, &mArray[eneTene*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[eneTene*mBytesPerLine], &mArray[moneMei*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[moneMei*mBytesPerLine], buffer, mBytesPerLine);
			}
		}
		delete[] buffer;	//Release the memory
	}
}

//Mirror the entire array mArray vertically
void TiffU8::mirror()
{
	U8 *buffer = new U8[mBytesPerLine];		//Buffer used to store a row of pixels

	//Swap the first and last rows of the sub-image, then do the second and second last rows, etc
	for (int IterRow = 0; IterRow < mHeightPerFrame_pix / 2; IterRow++)
	{
		const int eneTene{ IterRow };									//Swap this row
		const int moneMei{ mHeightPerFrame_pix - IterRow - 1 };		//With this one
		std::memcpy(buffer, &mArray[eneTene*mBytesPerLine], mBytesPerLine);
		std::memcpy(&mArray[eneTene*mBytesPerLine], &mArray[moneMei*mBytesPerLine], mBytesPerLine);
		std::memcpy(&mArray[moneMei*mBytesPerLine], buffer, mBytesPerLine);
	}
	delete[] buffer;	//Release the memory
}

//The galvo (vectical axis of the image) performs bi-directional scanning and the data is saved in a long image (vertical strip)
//Divide the concatenated images in a stack of nFrames, average the even and odd frames separately, and return the averages in different pages
void TiffU8::averageEvenOddFrames()
{
	if (mNframes > 2)
	{
		//Calculate the average of the even and odd frames separately
		unsigned int* avg{ new unsigned int[2 * mNpixPerFrame]() };
		for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
			for (int iterPix = 0; iterPix < mNpixPerFrame; iterPix++)
			{
				if (iterFrame % 2)
					avg[iterPix] += mArray[iterFrame * mNpixPerFrame + iterPix];				//Odd frames
				else
					avg[mNpixPerFrame + iterPix] += mArray[iterFrame * mNpixPerFrame + iterPix];	//Even frames
			}

		//Put 'evenImage' and 'oddImage' back into mArray. Concatenate 'oddImage' after 'evenImage'. Ignore the rest of the data in mArray
		const int nFramesHalf{ mNframes / 2 };
		for (int iterPix = 0; iterPix < 2 * mNpixPerFrame; iterPix++)
		{
			if (mNframes % 2)	//Odd number of frames: 1, 3, 5, etc
				mArray[iterPix] = static_cast<U8>(1. * avg[iterPix] / (nFramesHalf + 1));
			else				//Even number of frames: 0, 2, 4, etc
				mArray[iterPix] = static_cast<U8>(1. * avg[iterPix] / nFramesHalf);
		}

		mNframes = 2;	//Keep the odd and even averages in separate pages
		delete[] avg;
	}
}

//Divide the concatenated images in a stack of nFrames and return the average over all the frames
void TiffU8::averageFrames()
{
	if (mNframes > 1)
	{
		unsigned int* sum{ new unsigned int[mNpixPerFrame]() };

		//For each pixel, calculate the sum intensity over all the frames
		for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
			for (int iterPix = 0; iterPix < mNpixPerFrame; iterPix++)
				sum[iterPix] += mArray[iterFrame * mNpixPerFrame + iterPix];

		//Calculate the average intensity and assign it to mArray
		for (int iterPix = 0; iterPix < mNpixPerFrame; iterPix++)
			mArray[iterPix] = static_cast<U8>(1. * sum[iterPix] / mNframes);

		//Update the number of frames in the stack to 1
		mNframes = 1;
		delete[] sum;
	}
}

//Take every nFramesPerBin frames in mArray and bin them. To be used by continuous scanning
void TiffU8::binFrames(const int nFramesPerBin)
{
	//nFramesPerBin must be a divisor of mNframes
	if (nFramesPerBin <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The bin size must be >0");

	//nFramesPerBin must be a divisor of mNframes
	if (mNframes%nFramesPerBin != 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The bin size must be a divisor of the total number of frames " + std::to_string(mNframes));

	//If mNframes = 1, there is only a single image. If nFramesPerBin = 1, no average is performed and the stack remains the same
	if (mNframes > 1 && nFramesPerBin > 1)
	{
		const int nBins{ mNframes / nFramesPerBin };	//Number of bins in the stack
		const int nPixPerBin{ mNpixPerFrame * nFramesPerBin };
		unsigned int* sum{ new unsigned int[nBins * mNpixPerFrame]() };

		//Take the first nFramesPerBin frames and average them. Then continue averaging every nFramesPerBin frames until the end of the stack
		for (int binIndex = 0; binIndex < nBins; binIndex++)
			for (int iterFrame = 0; iterFrame < nFramesPerBin; iterFrame++)		//Frame index within a bin
				for (int iterPix = 0; iterPix < mNpixPerFrame; iterPix++)			//Read the individual pixels
					sum[binIndex * mNpixPerFrame + iterPix] += mArray[binIndex * nPixPerBin + iterFrame * mNpixPerFrame + iterPix];

		//Calculate the average intensity in a bin and assign it to mArray
		for (int binIndex = 0; binIndex < nBins; binIndex++)
			for (int iterPix = 0; iterPix < mNpixPerFrame; iterPix++)
				mArray[binIndex * mNpixPerFrame + iterPix] = static_cast<U8>(1. * sum[binIndex * mNpixPerFrame + iterPix] / nFramesPerBin);

		//Update the number of frames in the stack
		mNframes = nBins;
		delete[] sum;
	}
}

//Correct the image distortion induced by the nonlinear scanning of the RS
//Code based on Martin's algorithm, https://github.com/mpicbg-csbd/scancorrect, mweigert@mpi-cbg.de
//OpenCL code based on http://simpleopencl.blogspot.com/2013/06/tutorial-simple-start-with-opencl-and-c.html
void TiffU8::correctRSdistortionGPU(const double FFOVfast)
{
	if (FFOVfast <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": FFOV must be >0");

	const int heightAllFrames{ mHeightPerFrame_pix * mNframes };

	//Start and stop time of the RS scan that define FFOVfast
	const double t1{ 0.5 * (g_lineclockHalfPeriod - mWidthPerFrame_pix * g_pixelDwellTime) };
	const double t2{ g_lineclockHalfPeriod - t1 };

	//The full amplitude of the RS (from turning point to turning point) in um. It is assumed that the laser scans the sample following x(t) = 0.5 * fullScan ( 1 - cos (2 * PI * f * t) )
	const double fullScan{ 2. * FFOVfast / (std::cos(PI * t1 / g_lineclockHalfPeriod) - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

	//Start and stop positions of the RS that define FFOVfast
	const double x1{ 0.5 * fullScan * (1. - std::cos(PI * t1 / g_lineclockHalfPeriod)) };
	const double x2{ 0.5 * fullScan * (1. - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

	/*//For debugging
	std::cout << "t1 (us): " << t1 / us << "\n";
	std::cout << "t2 (us): " << t2 / us << "\n";
	std::cout << "x1 (um): " << x1 / um << "\n";
	std::cout << "x2 (um): " << x2 / um << "\n";
	*/

	//Normalized variables
	const float xbar1{ static_cast<float>(x1 / fullScan) };
	const float xbar2{ static_cast<float>(x2 / fullScan) };
	const float tbar1{ static_cast<float>(t1 / g_lineclockHalfPeriod) };
	const float tbar2{ static_cast<float>(t2 / g_lineclockHalfPeriod) };
	const float PI_float{ static_cast<float>(PI) };

	//Precompute the mapping of the fast coordinate (k)
	float *kPrecomputed{ new float[mWidthPerFrame_pix] };
	for (int k = 0; k < mWidthPerFrame_pix; k++) {
		const float x = 1.f * k / (mWidthPerFrame_pix - 1.f);
		const float a = 1.f - 2 * xbar1 - 2 * (xbar2 - xbar1) * x;
		const float t = (std::acos(a) / PI_float - tbar1) / (tbar2 - tbar1);
		kPrecomputed[k] = t * (mWidthPerFrame_pix - 1.f);
		//std::cout << kk_floats_precomputed[k] << "\n";	//For debugging
	}

	std::vector<cl::Platform> all_platforms;
	cl::Platform::get(&all_platforms);
	if (all_platforms.size() == 0) {
		throw std::runtime_error((std::string)__FUNCTION__ + ": No platforms found. Check OpenCL installation!");
	}
	cl::Platform default_platform{ all_platforms[0] };

	//Get default device of the default platform
	std::vector<cl::Device> all_devices;
	default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
	if (all_devices.size() == 0) {
		throw std::runtime_error((std::string)__FUNCTION__ + ": No devices found. Check OpenCL installation!");
	}
	cl::Device default_device{ all_devices[0] };

	/*//For debugging
	std::cout << "Using platform: " << default_platform.getInfo<CL_PLATFORM_NAME>() << "\n";
	std::cout << "Using device: " << default_device.getInfo<CL_DEVICE_NAME>() << "\n";
	std::cout << "Device version: " << default_device.getInfo<CL_DEVICE_VERSION>() << "\n";
	std::cout << "Device global mem size: " << default_device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>() << "\n";
	std::cout << "Device max mem alloc size: " << default_device.getInfo<CL_DEVICE_MAX_MEM_ALLOC_SIZE>() << "\n";
	std::cout << "Device max compute units: " << default_device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>() << "\n"; //7
	std::cout << "Device max work group sizes: " << default_device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() << "\n";//1024
	std::cout << "Device max work item sizes: ( " << default_device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>().at(0) << " " <<
		default_device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>().at(1) << " " <<
		default_device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>().at(2)  <<" )\n"; //(1024 1024 64)
	*/

	//Build a string with the openCl kernel
	std::string openclFilename{ "openclKernel.cl" };
	std::ifstream openclKernelCode{ openclFilePath + openclFilename };
	if (!openclKernelCode.is_open())
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Failed opening the file " + openclFilename);
	std::string sourceCode{ std::istreambuf_iterator<char>(openclKernelCode), std::istreambuf_iterator<char>() };	//Create a string from the beginning to the end of the file
	cl::Program::Sources sources{ 1, std::make_pair(sourceCode.c_str(), sourceCode.length()) };

	cl::Context context{ { default_device } };
	cl::Program program{ context, sources };
	if (program.build({ default_device }) != CL_SUCCESS) {
		throw std::runtime_error((std::string)__FUNCTION__);
		std::cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << "\n";
	}

	//Create buffers on the device
	cl::Buffer buffer_kPrecomputed{ context, CL_MEM_READ_WRITE, sizeof(float) * mWidthPerFrame_pix };
	cl::Buffer buffer_uncorrectedArray{ context, CL_MEM_READ_WRITE, sizeof(unsigned char) * mNpixAllFrames };
	cl::Buffer buffer_correctedArray{ context, CL_MEM_READ_WRITE, sizeof(unsigned char) * mNpixAllFrames };
	cl::Buffer buffer_debugger{ context, CL_MEM_READ_WRITE, sizeof(double) };

	//Create queue to which we will push commands for the device.
	cl::CommandQueue queue{ context, default_device };

	//Write arrays to the device
	queue.enqueueWriteBuffer(buffer_kPrecomputed, CL_TRUE, 0, sizeof(float) * mWidthPerFrame_pix, kPrecomputed);
	queue.enqueueWriteBuffer(buffer_uncorrectedArray, CL_TRUE, 0, sizeof(unsigned char) * mNpixAllFrames, mArray);

	//Run the kernel
	cl::Kernel kernel_add{ cl::Kernel{program,"correctRSdistortion"} };
	kernel_add.setArg(0, buffer_kPrecomputed);
	kernel_add.setArg(1, buffer_uncorrectedArray);
	kernel_add.setArg(2, buffer_correctedArray);
	kernel_add.setArg(3, mWidthPerFrame_pix);
	kernel_add.setArg(4, buffer_debugger);
	queue.enqueueNDRangeKernel(kernel_add, cl::NullRange, cl::NDRange(mWidthPerFrame_pix, heightAllFrames), cl::NullRange);
	queue.finish();

	//Read correctedArray from the device
	unsigned char* correctedArray = new unsigned char[mNpixAllFrames];
	queue.enqueueReadBuffer(buffer_correctedArray, CL_TRUE, 0, sizeof(unsigned char) * mNpixAllFrames, correctedArray);

	double debugger;
	queue.enqueueReadBuffer(buffer_debugger, CL_TRUE, 0, sizeof(double), &debugger);

	/*
	std::cout << "correctedArray: \n";
	for (int i = 0; i < 10; i++) {
		std::cout << (int) correctedArray[i] << " ";
	}
	std::cout << "\n";

	std::cout << std::fixed;
	std::cout << std::setprecision(10);
	std::cout << "Debugger: " << debugger << "\n";
	*/

	delete[] kPrecomputed;
	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//Called by TiffU8::correctRSdistortionCPU()
inline U8 interpolateU8(float lam, const U8  &val1, const U8 &val2)
{
	//Old way with clipping
	//int res = static_cast<int>(std::round( (1 - lam) * val1 + lam * val2) );
	//return static_cast<U8>(clip(res, (std::numeric_limits<U8>::min)(), (std::numeric_limits<U8>::max)()));

	//New way without clipping
	return static_cast<U8>(std::round((1.f - lam) * val1 + lam * val2));
}

//Correct the image distortion induced by the nonlinear scanning of the RS
//Code based on Martin's algorithm, https://github.com/mpicbg-csbd/scancorrect, mweigert@mpi-cbg.de
void TiffU8::correctRSdistortionCPU(const double FFOVfast)
{
	if (FFOVfast <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": FFOV must be >0");

	U8* correctedArray = new U8[mNpixAllFrames];

	//Start and stop time of the RS scan that define FFOVfast
	const double t1{ 0.5 * (g_lineclockHalfPeriod - mWidthPerFrame_pix * g_pixelDwellTime) };
	const double t2{ g_lineclockHalfPeriod - t1 };

	//The full amplitude of the RS (from turning point to turning point) in um
	const double fullScan{ 2. * FFOVfast / (std::cos(PI * t1 / g_lineclockHalfPeriod) - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

	//Start and stop positions of the RS that define the FFOVfast
	const double x1{ 0.5 * fullScan * (1 - std::cos(PI * t1 / g_lineclockHalfPeriod)) };
	const double x2{ 0.5 * fullScan * (1 - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

	/*//For debugging
	std::cout << "t1 (us): " << t1 / us << "\n";
	std::cout << "t2 (us): " << t2 / us << "\n";
	std::cout << "x1 (um): " << x1 / um << "\n";
	std::cout << "x2 (um): " << x2 / um << "\n";
	*/

	//Normalized variables
	const float xbar1{ static_cast<float>(x1 / fullScan) };
	const float xbar2{ static_cast<float>(x2 / fullScan) };
	const float tbar1{ static_cast<float>(t1 / g_lineclockHalfPeriod) };
	const float tbar2{ static_cast<float>(t2 / g_lineclockHalfPeriod) };
	const float PI_float{ static_cast<float>(PI) };

	// precompute the mapping of the fast coordinate (k)
	float *kk_precomputed = new float[mWidthPerFrame_pix];
	for (int k = 0; k < mWidthPerFrame_pix; k++) {
		const float x{ 1.f * k / (mWidthPerFrame_pix - 1.f) };
		const float a{ 1.f - 2 * xbar1 - 2 * (xbar2 - xbar1) * x };
		const float t{ (std::acos(a) / PI_float - tbar1) / (tbar2 - tbar1) };
		kk_precomputed[k] = t * (mWidthPerFrame_pix - 1.f);
		//std::cout << kk_floats_precomputed[k] << "\n";
	}

# pragma omp parallel for schedule(dynamic)
	for (int iterRow_pix = 0; iterRow_pix < mHeightPerFrame_pix * mNframes; iterRow_pix++) {
		for (int k = 0; k < mWidthPerFrame_pix; k++) {
			const float kk_float{ kk_precomputed[k] };
			const int kk{ static_cast<int>(std::floor(kk_float)) };
			const int kk1{ clip(kk, 0, mWidthPerFrame_pix - 1) };
			const int kk2{ clip(kk + 1, 0, mWidthPerFrame_pix - 1) };
			const U8 value1{ mArray[iterRow_pix * mWidthPerFrame_pix + kk1] };	//Read from the input array
			const U8 value2{ mArray[iterRow_pix * mWidthPerFrame_pix + kk2] };	//Read from the input array
			correctedArray[iterRow_pix * mWidthPerFrame_pix + k] = interpolateU8(kk_float - kk1, value1, value2);	//Interpolate and save to the output array
		}
	}

	delete[] kk_precomputed;
	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//When using 16X beamlets, the slow axis is slightly stretched out (3 pixels on the edges, 0 pixels at the center)
//I was trying to correct the slow axis with this function. However, I realized that it's better to apply an affine transformation instead
void TiffU8::correctFOVslowCPU(const double FFOVslow)
{
	if (FFOVslow <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": FFOV must be >0");

	U8* correctedArray = new U8[mNpixAllFrames];

	//Normalized variables
	const float xbar2{ 1.007f };
	const float xbar1{ 1.f - xbar2 };

	// precompute the mapping of the slow coordinate (k)
	float *kk_precomputed = new float[mHeightPerFrame_pix];
	for (int k = 0; k < mHeightPerFrame_pix; k++) {
		const float x{ 1.f * k / (mHeightPerFrame_pix - 1.f) };
		const float xnew{ xbar1 + (xbar2 - xbar1) * x };
		kk_precomputed[k] = xnew * (mHeightPerFrame_pix - 1.f);
	}

	//# pragma omp parallel for schedule(dynamic)
	for (int iterCol_pix = 0; iterCol_pix < mWidthPerFrame_pix; iterCol_pix++) {
		for (int k = 0; k < mHeightPerFrame_pix; k++) {
			const float kk_float{ kk_precomputed[k] };
			const int kk{ static_cast<int>(std::floor(kk_float)) };
			const int kk1{ clip(kk, 0, mHeightPerFrame_pix - 1) };
			const int kk2{ clip(kk + 1, 0, mHeightPerFrame_pix - 1) };
			const U8 value1{ mArray[kk1 * mWidthPerFrame_pix + iterCol_pix] };	//Read from the input array
			const U8 value2{ mArray[kk2 * mWidthPerFrame_pix + iterCol_pix] };	//Read from the input array
			correctedArray[k * mWidthPerFrame_pix + iterCol_pix] = interpolateU8(kk_float - kk1, value1, value2);	//Interpolate and save to the output array
		}
	}

	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//The PMT16X channels have some crosstalk. The image in every strip corresponding to a PMT16X channel is ghost-imaged on the neighbor top and bottom strips
//To correct for this, substract a lineThicknessFactor of the neighbor top and neighbor bottom strips
void TiffU8::suppressCrosstalk(const double crosstalkRatio)
{
	if (crosstalkRatio < 0 || crosstalkRatio > 1.0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The crosstalk ratio must be in the range [0, 1.0]");

	const int nPixPerFramePerBeamlet{ mNpixPerFrame / g_nChanPMT };	//Number of pixels in a strip
	U8* correctedArray{ new U8[mNpixAllFrames] };

	for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
		for (int iterPix = 0; iterPix < nPixPerFramePerBeamlet; iterPix++)
		{
			//First channel
			correctedArray[iterFrame * mNpixPerFrame + iterPix] = clipU8dual(
				mArray[iterFrame * mNpixPerFrame + iterPix] - crosstalkRatio * mArray[iterFrame * mNpixPerFrame + nPixPerFramePerBeamlet + iterPix]);

			//Last channel
			correctedArray[iterFrame * mNpixPerFrame + (g_nChanPMT - 1) * nPixPerFramePerBeamlet + iterPix] = clipU8dual(
				mArray[iterFrame * mNpixPerFrame + (g_nChanPMT - 1) * nPixPerFramePerBeamlet + iterPix] - crosstalkRatio * mArray[iterFrame * mNpixPerFrame + (g_nChanPMT - 2) * nPixPerFramePerBeamlet + iterPix]);

			//All channels in between
			for (int chanIndex = 1; chanIndex < g_nChanPMT - 1; chanIndex++)
				correctedArray[iterFrame * mNpixPerFrame + chanIndex * nPixPerFramePerBeamlet + iterPix] = clipU8dual(
					mArray[iterFrame * mNpixPerFrame + chanIndex * nPixPerFramePerBeamlet + iterPix]
					- crosstalkRatio * (mArray[iterFrame * mNpixPerFrame + (chanIndex - 1) * nPixPerFramePerBeamlet + iterPix] + mArray[iterFrame * mNpixPerFrame + (chanIndex + 1) * nPixPerFramePerBeamlet + iterPix]));
		}
	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//Upscale the pixel counts for the lower and higher channels of the PMT16X. The channel indices go from 0 to g_nChanPMT-1
//The upscaling factor follows a linear interpolation
void TiffU8::flattenField(const double scaleFactor, const int lowerChan, const int higherChan)
{
	if (scaleFactor < 1.0 || scaleFactor > 3.0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The scale factor must be in the range [1.0-3.0]");

	if (lowerChan > higherChan)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The lower channel index must be < than the higher channel index");

	if (lowerChan >= g_nChanPMT || higherChan >= g_nChanPMT)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The lower channel index must be < than the higher channel index");

	const double lowerSlope{ (scaleFactor - 1.0) / lowerChan };							//Interpolation slope for the lower channels
	const double higherSlope{ (scaleFactor - 1.0) / ((g_nChanPMT - 1) - higherChan) };	//Interpolation slope for the higer channels

	std::vector<double> vec_upscalingFactors(g_nChanPMT, 1.0);							//Vector of gains

	//Lower channels: from CH00 to lowerChan
	for (int chanIndex = 0; chanIndex <= lowerChan; chanIndex++)
		vec_upscalingFactors.at(chanIndex) = -lowerSlope * chanIndex + scaleFactor;

	//Higher channels: from higherChan to CH15
	for (int chanIndex = higherChan; chanIndex < g_nChanPMT; chanIndex++)
		vec_upscalingFactors.at(chanIndex) = higherSlope * (chanIndex - (g_nChanPMT - 1)) + scaleFactor;

	//For debugging
	//for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
		//std::cout << "upscaling " << chanIndex << " = " << vec_upscalingFactors.at(chanIndex) << "\n";

	//Upscale mArray
	const int nPixPerFramePerBeamlet{ mNpixPerFrame / g_nChanPMT };	//Number of pixels in a strip
	for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
		for (int iterPix = 0; iterPix < nPixPerFramePerBeamlet; iterPix++)
			for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
				mArray[iterFrame * mNpixPerFrame + chanIndex * nPixPerFramePerBeamlet + iterPix] = clipU8dual(vec_upscalingFactors.at(chanIndex) * mArray[iterFrame * mNpixPerFrame + chanIndex * nPixPerFramePerBeamlet + iterPix]);
}

/*Old way of doing the field flattening
//Upscale the channel n = 0, 1, ..., 15 by (1 + (sqrt(factor) - 1) * |n/7.5 - 1|)^2
void TiffU8::flattenField(const double maxScaleFactor)
{
	if (maxScaleFactor < 1.0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The scale factor must be >= 1.0");

	const int nPixPerFramePerBeamlet{ mNpixPerFrame / g_nChanPMT };	//Number of pixels in a strip
	std::vector<double> vec_upscalingFactors(g_nChanPMT);

	for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
	{
		const double aux{ 1 +  (std::sqrt(maxScaleFactor) - 1) * std::abs(chanIndex/7.5 - 1)};
		vec_upscalingFactors.at(chanIndex) = aux * aux;
	}

	//For debugging
	//for (int chanIndex = 0; chanIndex < nChanPMT; chanIndex++)
	//	std::cout << vec_upscalingFactors.at(chanIndex) << "\n";

	for (int iterFrame = 0; iterFrame < mNframes; iterFrame++)
		for (int iterPix = 0; iterPix < mNpixPerFramePerBeamlet; iterPix++)
			for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
				mArray[iterFrame * mNpixPerFrame + chanIndex * mNpixPerFramePerBeamlet + iterPix] = clipU8dual(vec_upscalingFactors.at(chanIndex) * mArray[iterFrame * mNpixPerFrame + chanIndex * mNpixPerFramePerBeamlet + iterPix]);
}
*/
#pragma endregion "TiffU8"

#pragma region "QuickStitcher"
//tileHeight_pix = tile height, tileWidth_pix = tile width, tileArraySize = { number of tiles as rows, number of tiles as columns}
//II is the row index (along the image height) and JJ is the column index (along the image width) of the tile wrt the tile array. II and JJ start from 0
QuickStitcher::QuickStitcher(const int tileHeight_pix, const int tileWidth_pix, const INDICES2 tileArraySize) :
	mTileArraySize{ tileArraySize },
	mStitchedTiff{ tileHeight_pix * tileArraySize.II, tileWidth_pix * tileArraySize.JJ, 1 }
{
	if (tileHeight_pix <= 0 || tileWidth_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile height and width must be > 0");

	if(mTileArraySize.II <= 0 || mTileArraySize.JJ <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The array dimensions must be > 0");
}

//II is the row index (along the image height) and JJ is the column index (along the image width) of the tile wrt the tile array. II and JJ start from 0
void QuickStitcher::push(const TiffU8 &tile, const INDICES2 tileIndicesIJ)
{
	if (tileIndicesIJ.II < 0 || tileIndicesIJ.II >= mTileArraySize.II)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile row index II must be in the range [0-" + std::to_string(mTileArraySize.II - 1) + "]");
	if (tileIndicesIJ.JJ < 0 || tileIndicesIJ.JJ >= mTileArraySize.JJ)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile column index JJ must be in the range [0-" + std::to_string(mTileArraySize.JJ - 1) + "]");

	const int tileHeight_pix{ tile.heightPerFrame_pix() };													//Height of the tile
	const int tileWidth_pix{ tile.widthPerFrame_pix() };													//Width of the tile
	const int rowShift_pix{ tileIndicesIJ.II *  tileHeight_pix };											//In the mTiff image, shift the tile down by this many pixels
	const int colShift_pix{ tileIndicesIJ.JJ *  tileWidth_pix };											//In the mTiff image, shift the tile to the right by this many pixels
	const int tileBytesPerRow{ static_cast<int>(tileWidth_pix * sizeof(U8)) };								//Bytes per row of the input tile
	const int stitchedTiffBytesPerRow{ static_cast<int>(mStitchedTiff.widthPerFrame_pix() * sizeof(U8)) };	//Bytes per row of the tiled image

	/*
	//Transfer the data from the input tile to mStitchedTiff. Old way: copy pixel by pixel
	for (int iterCol_pix = 0; iterCol_pix < tileWidth_pix; iterCol_pix++)
		for (int iterRow_pix = 0; iterRow_pix < tileHeight_pix; iterRow_pix++)
			mStitchedTiff.data()[(rowShift_pix + iterRow_pix) * mStitchedTiff.widthPerFrame_pix() + colShift_pix + iterCol_pix] = tile.data()[iterRow_pix * tileWidth_pix + iterCol_pix];*/

	//Transfer the data from the input tile to mStitchedTiff. New way: copy row by row
	for (int iterRow_pix = 0; iterRow_pix < tileHeight_pix; iterRow_pix++)
		std::memcpy(&mStitchedTiff.data()[(rowShift_pix + iterRow_pix) * stitchedTiffBytesPerRow + colShift_pix * sizeof(U8)], &tile.data()[iterRow_pix * tileBytesPerRow], tileBytesPerRow);
}

void QuickStitcher::saveToFile(std::string filename, const OVERRIDE override) const
{
	mStitchedTiff.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override, SCANDIR::UPWARD);
}
#pragma endregion "QuickStitcher"