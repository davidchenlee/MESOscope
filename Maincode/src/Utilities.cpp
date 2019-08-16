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
		t = t / 2;
	}

	return int_part + frac_part;
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

//Used to increase the laser power 16 times
double multiply16X(const double input)
{
	return 16 * input;
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

//Switch the scan direction. It is important to pass scanDir by reference to be able to modify it
void reverseSCANDIR(SCANDIR &scanDir)
{
	switch (scanDir)
	{
	//STAGEX
	case SCANDIR::LEFTWARD:
		scanDir =  SCANDIR::RIGHTWARD;
		break;
	case SCANDIR::RIGHTWARD:
		scanDir =  SCANDIR::LEFTWARD;
		break;
	//STAGEY
	case SCANDIR::OUTWARD:
		scanDir =  SCANDIR::INWARD;
		break;
	case SCANDIR::INWARD:
		scanDir =  SCANDIR::OUTWARD;
		break;
	//STAGEZ
	case SCANDIR::DOWNWARD:
		scanDir =  SCANDIR::UPWARD;
		break;
	case SCANDIR::UPWARD:
		scanDir =  SCANDIR::DOWNWARD;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

double detInitialPos(const double posMin, const double travel, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The travel range must be >0");

	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin + travel;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

//Travel overhead to avoid the nonlinearity at the end of the stage scan
double detFinalPos(const double posMin, const double travel, const double travelOverhead, const SCANDIR scanDir)
{
	if (travel <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The travel range must be >0");
	
	switch (scanDir)
	{
	case SCANDIR::UPWARD:
	case SCANDIR::RIGHTWARD:
		return posMin + travel + travelOverhead;
	case SCANDIR::DOWNWARD:
	case SCANDIR::LEFTWARD:
		return posMin - travelOverhead;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

double detInitialLaserPower(const double powerMin, const double powerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0 || powerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The laser power and power increase must be >=0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin;
	case SCANDIR::DOWNWARD:
		return powerMin + powerInc;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
	}
}

double detFinalLaserPower(const double powerMin, const double powerInc, const SCANDIR scanDirZ)
{
	if (powerMin < 0 || powerInc < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + "The laser power and power increase must be >=0");

	switch (scanDirZ)
	{
	case SCANDIR::UPWARD:
		return powerMin + powerInc;
	case SCANDIR::DOWNWARD:
		return powerMin;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + "Invalid scan direction");
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
TiffU8::TiffU8(const std::string filename) : mNframes{ 1 }
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
	if (!TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &mWidthPerFrame_pix))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_IMAGEWIDTH");
	if(!TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &mHeightPerFrame_pix))
		throw std::runtime_error((std::string)__FUNCTION__ + ": TIFFGetField failed reading TIFFTAG_IMAGELENGTH");

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

	if (mWidthPerFrame_pix <= 0 || mHeightPerFrame_pix <= 0 || mNframes <= 0)
		throw std::runtime_error((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	//Length in memory of one row of pixel in the image. Targeting 'U8' only. Alternatively, mBytesPerLine = TIFFScanlineSize(tiffHandle);
	mBytesPerLine = mWidthPerFrame_pix * sizeof(U8);	

	U8* buffer{ (U8*)_TIFFmalloc(mBytesPerLine) };

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	mArray = new U8[mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes];	//Allocate memory for the image

	for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
	{
		//Read the tiff one strip at a time
		for (int rowIndex = 0; rowIndex < mHeightPerFrame_pix; rowIndex++)
		{
			if (TIFFReadScanline(tiffHandle, buffer, rowIndex, 0) < 0)
				break;
			std::memcpy(&mArray[(frameIndex * mHeightPerFrame_pix + rowIndex) * mBytesPerLine], buffer, mBytesPerLine);
		}
		TIFFReadDirectory(tiffHandle);
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file. I hope the pointer TIFFTAG_ImageJ is cleaned up here
}

//Construct a Tiff from an array
TiffU8::TiffU8(const U8* inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames) :
	mWidthPerFrame_pix{ widthPerFrame }, mHeightPerFrame_pix{ heightPerFrame }, mNframes{ nFrames }, mBytesPerLine{ static_cast<int>(widthPerFrame * sizeof(U8)) }
{
	if (mWidthPerFrame_pix <= 0 || mHeightPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	const int nPixAllFrames{ mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes };
	mArray = new U8[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, inputImage, nPixAllFrames * sizeof(U8));
}

//Construct a Tiff from a vector
TiffU8::TiffU8(const std::vector<U8> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nFrames) :
	mWidthPerFrame_pix{ widthPerFrame }, mHeightPerFrame_pix{ heightPerFrame }, mNframes{ nFrames }, mBytesPerLine{ static_cast<int>(widthPerFrame * sizeof(U8)) }
{
	if (mWidthPerFrame_pix <= 0 || mHeightPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	const int nPixAllFrames{ mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes };
	mArray = new U8[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, &inputImage[0], nPixAllFrames * sizeof(U8));
}

//Construct a new Tiff by allocating memory and initialize it to zero for safety
TiffU8::TiffU8(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nFrames) :
	mWidthPerFrame_pix{ widthPerFrame_pix }, mHeightPerFrame_pix{ heightPerFrame_pix }, mNframes{ nFrames }, mBytesPerLine{ static_cast<int>(widthPerFrame_pix * sizeof(U8)) }
{
	if (mWidthPerFrame_pix <= 0 || mHeightPerFrame_pix <= 0 || mNframes <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The image pixel width, pixel height, and number of frames must be >0");

	mArray = new U8[mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes]();
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

int TiffU8::widthPerFrame_pix() const
{
	return mWidthPerFrame_pix;
}

int TiffU8::heightPerFrame_pix() const
{
	return mHeightPerFrame_pix;
}

int TiffU8::nFrames() const
{
	return mNframes;
}

//Divide the concatenated images in a stack of nFrames
void TiffU8::splitIntoFrames(const int nFrames)
{
	if (nFrames <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame number must be >0");

	mNframes = nFrames;
	mHeightPerFrame_pix = mHeightPerFrame_pix / nFrames;
}



//Divide the concatenated images in a stack of nFrames and save it (the microscope concatenates all the images and hands over a long image that has to be resized into individual images)
void TiffU8::saveToFile(std::string filename, const TIFFSTRUCT tiffStruct, const OVERRIDE override, const SCANDIR scanDirZ) const
{
	int width_pix, height_pix, nFrames;

	//Multi page structure
	if (tiffStruct == TIFFSTRUCT::MULTIPAGE)
	{
		nFrames = mNframes;
		width_pix = mWidthPerFrame_pix;
		height_pix = mHeightPerFrame_pix;
	}
	//Single page
	else
	{
		nFrames = 1;
		width_pix = mWidthPerFrame_pix;
		height_pix = mHeightPerFrame_pix * mNframes;
	}

	/*For debugging
	std::cout << nFrames << "\n";
	std::cout << Pixel width << "\n";
	std::cout << Pixel height << "\n";
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
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, width_pix);										//Set the pixel width of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, height_pix);										//Set the pixel height of the image
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
		for (int rowIndex = 0; rowIndex < height_pix; rowIndex++)
		{
			std::memcpy(buffer, &mArray[(frameIndex * height_pix + rowIndex) * mBytesPerLine], mBytesPerLine);
			if (TIFFWriteScanline(tiffHandle, buffer, rowIndex, 0) < 0)
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

//Mirror the odd frames (the frame index starts from 0) vertically because the galvo (vectical axis of the image) performs bi-directional scanning and the images from different frames are concatenated
void TiffU8::mirrorOddFrames()
{
	if (mNframes > 1)
	{
		U8 *buffer = new U8[mBytesPerLine];		//Buffer used to store a row of pixels

		for (int frameIndex = 1; frameIndex < mNframes; frameIndex += 2)
		{
			//Swap the first and last rows of the sub-image, then do the second and second last rows, etc
			for (int rowIndex = 0; rowIndex < mHeightPerFrame_pix / 2; rowIndex++)
			{
				const int eneTene{ frameIndex * mHeightPerFrame_pix + rowIndex };				//Swap this row
				const int moneMei{ (frameIndex + 1) * mHeightPerFrame_pix - rowIndex - 1 };		//With this one
				std::memcpy(buffer, &mArray[eneTene*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[eneTene*mBytesPerLine], &mArray[moneMei*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[moneMei*mBytesPerLine], buffer, mBytesPerLine);
			}
		}
		delete[] buffer;	//Release the memory
	}
}

//Merge all the frames in a single image
void TiffU8::mergeFrames()
{
	mHeightPerFrame_pix = mNframes * mHeightPerFrame_pix;
	mNframes = 1;
}

//Mirror the entire array mArray vertically
void TiffU8::mirror()
{
	U8 *buffer = new U8[mBytesPerLine];		//Buffer used to store a row of pixels

	//Swap the first and last rows of the sub-image, then do the second and second last rows, etc
	for (int rowIndex = 0; rowIndex < mHeightPerFrame_pix / 2; rowIndex++)
	{
		const int eneTene{ rowIndex };									//Swap this row
		const int moneMei{ mHeightPerFrame_pix - rowIndex - 1 };		//With this one
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
		const int nPixPerFrame{ mWidthPerFrame_pix * mHeightPerFrame_pix };

		//Calculate the average of the even and odd frames separately
		unsigned int* avg{ new unsigned int[2 * nPixPerFrame]() };
		for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
			{
				if (frameIndex % 2)
					avg[pixIndex] += mArray[frameIndex * nPixPerFrame + pixIndex];					//Odd frames
				else
					avg[nPixPerFrame + pixIndex] += mArray[frameIndex * nPixPerFrame + pixIndex];	//Even frames
			}

		//Put 'evenImage' and 'oddImage' back into mArray. Concatenate 'oddImage' after 'evenImage'. Ignore the rest of the data in mArray
		int	nFramesHalf{ mNframes / 2 };
		for (int pixIndex = 0; pixIndex < 2 * nPixPerFrame; pixIndex++)
		{
			if (mNframes % 2)	//Odd number of frames: 1, 3, 5, etc
				mArray[pixIndex] = static_cast<U8>(1. * avg[pixIndex] / (nFramesHalf + 1));
			else				//Even number of frames: 0, 2, 4, etc
				mArray[pixIndex] = static_cast<U8>(1. * avg[pixIndex] / nFramesHalf);
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
		const int nPixPerFrame{ mWidthPerFrame_pix * mHeightPerFrame_pix };
		unsigned int* sum{ new unsigned int[nPixPerFrame]() };

		//For each pixel, calculate the sum intensity over all the frames
		for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
				sum[pixIndex] += mArray[frameIndex * nPixPerFrame + pixIndex];

		//Calculate the average intensity and assign it to mArray
		for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
			mArray[pixIndex] = static_cast<U8>(1. * sum[pixIndex] / mNframes);

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
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The bin size  must be a divisor of the total number of frames " + std::to_string(mNframes));

	//If mNframes = 1, there is only a single image. If nFramesPerBin = 1, no average is performed and the stack remains the same
	if (mNframes > 1 && nFramesPerBin > 1)
	{
		const int nBins{ mNframes / nFramesPerBin };	//Number of bins in the stack
		const int nPixPerFrame{ mWidthPerFrame_pix * mHeightPerFrame_pix };
		const int nPixPerBin{ nPixPerFrame * nFramesPerBin };
		unsigned int* sum{ new unsigned int[nBins * nPixPerFrame]() };

		//Take the first nFramesPerBin frames and average them. Then continue averaging every nFramesPerBin frames until the end of the stack
		for (int binIndex = 0; binIndex < nBins; binIndex++)
			for (int frameIndex = 0; frameIndex < nFramesPerBin; frameIndex++)		//Frame index within a bin
				for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)			//Read the individual pixels
					sum[binIndex * nPixPerFrame + pixIndex] += mArray[binIndex * nPixPerBin + frameIndex * nPixPerFrame + pixIndex];

		//Calculate the average intensity in a bin and assign it to mArray
		for (int binIndex = 0; binIndex < nBins; binIndex++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
				mArray[binIndex * nPixPerFrame + pixIndex] = static_cast<U8>(1. * sum[binIndex * nPixPerFrame + pixIndex] / nFramesPerBin);

		//Update the number of frames in the stack
		mNframes = nBins;
		delete[] sum;
	}
}

/*
//Calculate the max and min tile coordinates in the image to obtain the sample contour
std::vector<bool> TiffU8::maxMin_(std::vector<bool> input, const int nRow, const int nCol)
{
	mWidthPerFrame_pix;
	std::vector<int> vec_indexMin;
	std::vector<int> vec_indexMax;
	for (std::vector<int>::size_type iter = 0; iter != input.size(); iter++)
		;
}
*/

//Flood fill the sample contour


//Return the average
double TiffU8::giveTileAverage_(const int tileWidth_pix, const int tileHeight_pix, const int tileRowIndex, const int tileColIndex) const
{
	if (tileWidth_pix <= 0 || tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The width and height pixel number must be >0");

	if (tileWidth_pix < 0 || tileHeight_pix < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The row and column indices must be >=0");

	const int nPixPerTile{ tileWidth_pix * tileHeight_pix };		//Number of pixels in a tile

	int sum{ 0 };
	for (int rowIndex = tileRowIndex * tileHeight_pix; rowIndex < (tileRowIndex + 1) * tileHeight_pix; rowIndex++)
		for (int colIndex = tileColIndex * tileWidth_pix; colIndex < (tileColIndex + 1) * tileWidth_pix; colIndex++)
			sum += mArray[rowIndex * mWidthPerFrame_pix + colIndex];
	return sum / nPixPerTile;
}

//Divide the large image into tiles of size tileWidth_pix * tileHeight_pix and return an array of tiles indicating if the tile NOT dark dark
std::vector<bool> TiffU8::giveBoolMap(const double threshold, const int tileWidth_pix, const int tileHeight_pix) const
{
	if (threshold < 0 || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");

	if (tileWidth_pix <= 0 || tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The width and height pixel number must be >0");

	const int nTileCol{ mWidthPerFrame_pix / tileWidth_pix };		//Number of tile-colums
	const int nTileRow{ mHeightPerFrame_pix / tileHeight_pix };		//Number of tile-rows

	std::vector<double> vec_avg;
	//Start scanning the tiles from the top-left corner of the image. Scan the first row from left to right. Go back and scan the second row from left to right. Etc...
	for (int iterTileRow = 0; iterTileRow < nTileRow; iterTileRow++)
		for (int iterTileCol = 0; iterTileCol < nTileCol; iterTileCol++)
			vec_avg.push_back(giveTileAverage_(tileWidth_pix, tileHeight_pix, iterTileRow, iterTileCol));

	const int threshold_255{ static_cast<int>(threshold * 255) };	//Threshold in the range [0-255]
	std::vector<bool> vec_isBright;
	for (std::vector<int>::size_type iter = 0; iter != vec_avg.size(); iter++)
		vec_isBright.push_back(vec_avg.at(iter) >= threshold_255);

	//For debugging
	//for (std::vector<int>::size_type iter = 0; iter != nTileCol; iter++)
		//std::cout << "avg count: " << vec_avg.at(iter) << "\tis dark?: " << vec_isBright.at(iter) << "\n";

	std::ofstream fileHandle;									//Create output file
	fileHandle.open(folderPath + "ArrayOfTiles.txt");			//Open the file
	for (int iterTileRow = 0; iterTileRow < nTileRow; iterTileRow++)
	{
		//Iterate over the colums
		for (int iterTileCol = 0; iterTileCol < nTileCol; iterTileCol++)
			fileHandle << vec_isBright.at(iterTileRow * nTileCol + iterTileCol);
		fileHandle << "\n";	//End the row
	}
	fileHandle.close();											//Close the txt file

	return vec_isBright;
}

//Save mArray as a text file
void TiffU8::saveToTxt(const std::string filename) const
{
	std::ofstream fileHandle;									//Create output file
	fileHandle.open(folderPath + filename + ".txt");			//Open the file

	for (int pixIndex = 0; pixIndex < mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes; pixIndex++)
		fileHandle << mArray[pixIndex] << "\n";					//Write each element

	fileHandle.close();											//Close the txt file
}

//Specify the frame to push. The frame index starts from 0
void TiffU8::pushImage(const U8* inputArray, const int frameIndex) const
{
	if (frameIndex < 0 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be >= 0");
	if (frameIndex > mNframes)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be smaller than or equal to the number of frames " + std::to_string(mNframes));

	std::memcpy(&mArray[frameIndex * mHeightPerFrame_pix * mBytesPerLine], inputArray, mHeightPerFrame_pix * mBytesPerLine);
}

//Specify the frame interval to push. The frame index starts from 0
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

"Merging" places channels belonging to the same frame together. The resulting structure is:
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
void TiffU8::mergePMT16Xchan(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const
{
	if (heightPerChannelPerFrame < 0 )
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The pixel height per channel must be >= 0");

	const int heightAllChannelsPerFrame{ g_nChanPMT * heightPerChannelPerFrame };
	const int heightPerChannelAllFrames{ heightPerChannelPerFrame * mNframes };

	//Even 'frameIndex' (Raster scan the sample from the positive to the negative direction of the x-stage)
	for (int frameIndex = 0; frameIndex < mNframes; frameIndex += 2)
		for (int chanIndex = 0; chanIndex < 8; chanIndex++)
		{
			//CH00-CH07
			std::memcpy(&mArray[((15 - chanIndex) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			//CH08-CH15
			std::memcpy(&mArray[((7 - chanIndex) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}
	//Odd 'frameIndex' (Raster scan the sample from the negative to the positive direction of the x-stage)
	for (int frameIndex = 1; frameIndex < mNframes; frameIndex += 2)
		for (int chanIndex = 0; chanIndex < 8; chanIndex++)
		{
			//CH00-CH07
			std::memcpy(&mArray[(chanIndex * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			//CH08-CH15
			std::memcpy(&mArray[((chanIndex + 8) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}
}

//Correct the image distortion induced by the nonlinear scanning of the RS
//Correction code based on Martin's algorithm, https://github.com/mpicbg-csbd/scancorrect, mweigert@mpi-cbg.de
//OpenCL code based on http://simpleopencl.blogspot.com/2013/06/tutorial-simple-start-with-opencl-and-c.html
void TiffU8::correctRSdistortionGPU(const double FFOVfast)
{
	if (FFOVfast <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": FFOV must be >0");

	const int nPixAllFrames{ mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes };
	const int heightAllFrames{ mHeightPerFrame_pix * mNframes };

	//Start and stop time of the RS scan that define FFOVfast
	const double t1{ 0.5 * (g_lineclockHalfPeriod - mWidthPerFrame_pix * g_pixelDwellTime) };
	const double t2{ g_lineclockHalfPeriod - t1 };

	//The full amplitude of the RS (from turning point to turning point) in um. It is assumed that the laser scans the sample following x(t) = 0.5 * fullScan ( 1 - cos (2 * PI * f * t) )
	const double fullScan{ 2 * FFOVfast / (std::cos(PI * t1 / g_lineclockHalfPeriod) - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

	//Start and stop positions of the RS that define FFOVfast
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
	if(!openclKernelCode.is_open())
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
	cl::Buffer buffer_uncorrectedArray{ context, CL_MEM_READ_WRITE, sizeof(unsigned char) * nPixAllFrames };
	cl::Buffer buffer_correctedArray{ context, CL_MEM_READ_WRITE, sizeof(unsigned char) * nPixAllFrames };
	cl::Buffer buffer_debugger{ context, CL_MEM_READ_WRITE, sizeof(double) };

	//Create queue to which we will push commands for the device.
	cl::CommandQueue queue{ context, default_device };

	//Write arrays to the device
	queue.enqueueWriteBuffer(buffer_kPrecomputed, CL_TRUE, 0, sizeof(float) * mWidthPerFrame_pix, kPrecomputed);
	queue.enqueueWriteBuffer(buffer_uncorrectedArray, CL_TRUE, 0, sizeof(unsigned char) * nPixAllFrames, mArray);

	//Run the kernel
	cl::Kernel kernel_add{ cl::Kernel{program,"correctRSdistortion"} };
	kernel_add.setArg(0, buffer_kPrecomputed);
	kernel_add.setArg(1, buffer_uncorrectedArray);
	kernel_add.setArg(2, buffer_correctedArray);
	kernel_add.setArg(3, mWidthPerFrame_pix);
	kernel_add.setArg(4, buffer_debugger);
	queue.enqueueNDRangeKernel(kernel_add,cl::NullRange,cl::NDRange(mWidthPerFrame_pix, heightAllFrames),cl::NullRange);
	queue.finish();

	//Read correctedArray from the device
	unsigned char* correctedArray{ new unsigned char[nPixAllFrames] };
	queue.enqueueReadBuffer(buffer_correctedArray, CL_TRUE, 0, sizeof(unsigned char) * nPixAllFrames, correctedArray);

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
//Correction code based on Martin's algorithm, https://github.com/mpicbg-csbd/scancorrect, mweigert@mpi-cbg.de
void TiffU8::correctRSdistortionCPU(const double FFOVfast)
{
	if (FFOVfast <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": FFOV must be >0");

	const int nPixAllFrames{ mWidthPerFrame_pix * mHeightPerFrame_pix * mNframes };
	U8* correctedArray{ new U8[nPixAllFrames] };

	//Start and stop time of the RS scan that define FFOVfast
	const double t1{ 0.5 * (g_lineclockHalfPeriod - mWidthPerFrame_pix * g_pixelDwellTime) };
	const double t2{ g_lineclockHalfPeriod - t1 };

	//The full amplitude of the RS (from turning point to turning point) in um
	const double fullScan{ 2 * FFOVfast / (std::cos(PI * t1 / g_lineclockHalfPeriod) - std::cos(PI * t2 / g_lineclockHalfPeriod)) };

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
	float *kk_precomputed{ new float[mWidthPerFrame_pix] };
	for (int k = 0; k < mWidthPerFrame_pix; k++) {
		const float x{ 1.f * k / (mWidthPerFrame_pix - 1.f) };
		const float a{ 1.f - 2 * xbar1 - 2 * (xbar2 - xbar1) * x };
		const float t{ (std::acos(a) / PI_float - tbar1) / (tbar2 - tbar1) };
		kk_precomputed[k] = t * (mWidthPerFrame_pix - 1.f);
		//std::cout << kk_floats_precomputed[k] << "\n";
	}
	
# pragma omp parallel for schedule(dynamic)
	for (int rowIndex = 0; rowIndex < mHeightPerFrame_pix * mNframes; rowIndex++) {
		for (int k = 0; k < mWidthPerFrame_pix; k++) {
			const float kk_float{ kk_precomputed[k] };
			const int kk{ static_cast<int>(std::floor(kk_float)) };
			const int kk1{ clip(kk, 0, mWidthPerFrame_pix - 1) };
			const int kk2{ clip(kk + 1, 0, mWidthPerFrame_pix - 1) };
			const U8 value1{ mArray[rowIndex * mWidthPerFrame_pix + kk1] };	//Read from the input array
			const U8 value2{ mArray[rowIndex * mWidthPerFrame_pix + kk2] };	//Read from the input array
			correctedArray[rowIndex * mWidthPerFrame_pix + k] = interpolateU8(kk_float - kk1, value1, value2);	//Interpolate and save to the output array
		}
	}

	delete[] kk_precomputed;
	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//The PMT16X channels have some crosstalk. The image in every strip corresponding to a PMT16X channel is ghost-imaged on the neighbor top and bottom strips
//To correct for this, substract a fraction of the neighbor top and neighbor bottom strips
void TiffU8::suppressCrosstalk(const double crosstalkRatio)
{
	if (crosstalkRatio < 0 || crosstalkRatio > 1.0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The crosstalk ratio must be in the range [0, 1.0]");

	const int nPixPerFrame{ mWidthPerFrame_pix * mHeightPerFrame_pix };			//Number of pixels in a single frame
	const int nPixStrip{ mWidthPerFrame_pix * mHeightPerFrame_pix / g_nChanPMT };	//Number of pixels in a strip
	U8* correctedArray{ new U8[nPixPerFrame * mNframes] };

	for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
		for (int pixIndex = 0; pixIndex < nPixStrip; pixIndex++)
		{
			//First channel
			correctedArray[frameIndex * nPixPerFrame + pixIndex] = clipU8dual(
				mArray[frameIndex * nPixPerFrame + pixIndex] - crosstalkRatio * mArray[frameIndex * nPixPerFrame + nPixStrip + pixIndex]);

			//Last channel
			correctedArray[frameIndex * nPixPerFrame + (g_nChanPMT - 1) * nPixStrip + pixIndex] = clipU8dual(
				mArray[frameIndex * nPixPerFrame + (g_nChanPMT - 1) * nPixStrip + pixIndex] - crosstalkRatio * mArray[frameIndex * nPixPerFrame + (g_nChanPMT - 2) * nPixStrip + pixIndex]);

			//All channels in between
			for (int chanIndex = 1; chanIndex < g_nChanPMT - 1; chanIndex++)
				correctedArray[frameIndex * nPixPerFrame + chanIndex * nPixStrip + pixIndex] = clipU8dual(
					mArray[frameIndex * nPixPerFrame + chanIndex * nPixStrip + pixIndex]
					- crosstalkRatio * ( mArray[frameIndex * nPixPerFrame + (chanIndex - 1) * nPixStrip + pixIndex] + mArray[frameIndex * nPixPerFrame + (chanIndex + 1) * nPixStrip + pixIndex] ));
		}
	delete[] mArray;			//Free the memory-block containing the old, uncorrected array
	mArray = correctedArray;	//Reassign the pointer mArray to the newly corrected array
}

//Upscale the channel n = 0, 1, ..., 15 by (1 + (sqrt(factor) - 1) * |n/7.5 - 1|)^2
void TiffU8::flattenField(const double maxScaleFactor)
{
	if (maxScaleFactor < 1.0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The scale factor must be >= 1.0");

	const int nPixPerFrame{ mWidthPerFrame_pix * mHeightPerFrame_pix };				//Number of pixels in a single frame
	const int nPixStrip{ mWidthPerFrame_pix * mHeightPerFrame_pix / g_nChanPMT };	//Number of pixels in a strip
	std::vector<double> vec_upscalingFactors(g_nChanPMT);

	for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
	{
		const double aux{ 1 +  (std::sqrt(maxScaleFactor) - 1) * std::abs(chanIndex/7.5 - 1)};
		vec_upscalingFactors.at(chanIndex) = aux * aux;
	}

	//For debugging
	//for (int chanIndex = 0; chanIndex < nChanPMT; chanIndex++)
	//	std::cout << vec_upscalingFactors.at(chanIndex) << "\n";

	for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
		for (int pixIndex = 0; pixIndex < nPixStrip; pixIndex++)
			for (int chanIndex = 0; chanIndex < g_nChanPMT; chanIndex++)
				mArray[frameIndex * nPixPerFrame + chanIndex * nPixStrip + pixIndex] = clipU8dual(vec_upscalingFactors.at(chanIndex) * mArray[frameIndex * nPixPerFrame + chanIndex * nPixStrip + pixIndex]);
}
#pragma endregion "TiffU8"

//tileWidth_pix: tile width, tileHeight_pix: tile height, nTileRow: number of tile-rows in the stitched image, nTileCol: number of tile-columns in the stitched image
QuickStitcher::QuickStitcher(const int tileWidth_pix, const int tileHeight_pix, const int nTileRow, const int nTileCol) : mStitchedTiff{ tileWidth_pix * nTileCol, tileHeight_pix * nTileRow, 1 }, mNrow{ nTileRow }, mNcol{ nTileCol }
{
	if (tileWidth_pix <= 0 || tileHeight_pix <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The tile width and height must be > 0");

	if(mNrow <= 0 || mNcol <= 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The array dimensions must be > 0");
}

//rowIndex = 0, 1, 2, ... from TOP to BOTTOM and colIndex = 0, 1, 2, ... from LEFT to RIGHT of the mTiff image.
void QuickStitcher::push(const TiffU8 &tile, const int rowIndex, const int colIndex)
{
	if (colIndex < 0 || rowIndex < 0)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The array indices must be >= 0");
	if (colIndex >= mNcol)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The width array index must be <" + std::to_string(mNcol));
	if (rowIndex >= mNrow)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The height array index must be <" + std::to_string(mNrow));

	const int tileWidth_pix{ tile.widthPerFrame_pix() };	//Width of the tile
	const int tileHeight_pix{ tile.heightPerFrame_pix() };	//Height of the tile
	const int colShift_pix{ colIndex *  tileWidth_pix };	//In the mTiff image, shift the tile to the right by this many pixels
	const int rowShift_pix{ rowIndex *  tileHeight_pix };	//In the mTiff image, shift the tile down by this many pixels
	const int tileBytesPerRow{ static_cast<int>(tileWidth_pix * sizeof(U8)) };										//Bytes per row of the input tile
	const int stitchedTiffBytesPerRow{ static_cast<int>(mStitchedTiff.widthPerFrame_pix() * sizeof(U8)) };			//Bytes per row of the stitched image

	/*
	//Transfer the data from the input tile to mStitchedTiff. Old way: copy pixel by pixel
	for (int iterCol = 0; iterCol < tileWidth_pix; iterCol++)
		for (int iterRow = 0; iterRow < tileHeight_pix; iterRow++)
			mStitchedTiff.data()[(rowShift_pix + iterRow) * mStitchedTiff.widthPerFrame_pix() + colShift_pix + iterCol] = tile.data()[iterRow * tileWidth_pix + iterCol];*/

	//Transfer the data from the input tile to mStitchedTiff. New way: copy row by row
	for (int iterRow = 0; iterRow < tileHeight_pix; iterRow++)
		std::memcpy(&mStitchedTiff.data()[(rowShift_pix + iterRow) * stitchedTiffBytesPerRow + colShift_pix * sizeof(U8)], &tile.data()[iterRow * tileBytesPerRow], tileBytesPerRow);
}

void QuickStitcher::saveToFile(std::string filename, const OVERRIDE override) const
{
	mStitchedTiff.saveToFile(filename, TIFFSTRUCT::SINGLEPAGE, override, SCANDIR::UPWARD);
}

/*Obsolete
#pragma region "Stack"
TiffStack::TiffStack(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nDiffZ, const int nSameZ) :
	mArrayDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ), mArraySameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ) {}

void TiffStack::pushSameZ(const int indexSameZ, const U8* data)
{
	mArraySameZ.pushImage(indexSameZ, data);
}

//I want get the average of the stack mArraySameZ and store it in a single frame of mArrayDiffZ
//However, if I apply averageFrames() on mArraySameZ, it collapses mArraySameZ to a single image (containing the average), and therefore, the next iterations won't be able to use mArraySameZ container anymore
//Temporary hack: make a duplicate of mArraySameZ and calculate the average on it
void TiffStack::pushDiffZ(const int indexDiffZ)
{
	TiffU8 avgTiff{ mArraySameZ.data(), mArraySameZ.widthPerTile(), mArraySameZ.heightPerTile(), mArraySameZ.nFrames() }; //Make a copy of mArraySameZ
	avgTiff.averageFrames();																								//Average the images with the same Z
	mArrayDiffZ.pushImage(indexDiffZ, avgTiff.data());
}

void TiffStack::saveToFile(const std::string filename, OVERRIDE override) const
{
	mArrayDiffZ.saveToFile(filename, TIFFSTRUCT::EN, override);

	//For debugging. Save mArraySameZ
	//mArraySameZ.saveToFile(filename, TIFFSTRUCT::EN, override);
}
#pragma endregion "Stack"
*/

/*
//Take the top frame of the stack and return true if it's dark. Divide the image in quadrants for a better sensitivity
bool TiffU8::isDark(const double threshold) const
{
	if (threshold < 0  || threshold > 1)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The threshold must be in the range [0-1]");

	const int threshold_255{ static_cast<int>(threshold * 255) };	//Threshold in the range [0-255]
	const int nPix{ mWidthPerFrame_pix * mHeightPerFrame_pix };		//Number of pixels in the entire image
	const double nPixQuad{ 1. * nPix / 4 };							//Number of pixels in a quadrant

	//Divide the image in 4 quadrants
	const int halfwidth{ mWidthPerFrame_pix / 2 };
	const int halfHeight{ mHeightPerFrame_pix / 2 };

	std::vector<int> vec_sum;										//Vector of the sum for each quadrant
	//Start scanning the tiles from the top-left corner of the image
	//Scan the first row from left to right. Go back and scan the second row from left to right. Etc...
	for (int iterTileRow = 0; iterTileRow < 2; iterTileRow++)
		for (int iterTileCol = 0; iterTileCol < 2; iterTileCol++)
		{
			int sum{ 0 };
			for (int rowIndex = iterTileRow * halfHeight; rowIndex < (iterTileRow + 1) * halfHeight; rowIndex++)
				for (int colIndex = iterTileCol * halfwidth; colIndex < (iterTileCol + 1) * halfwidth; colIndex++)
					sum += mArray[rowIndex * mWidthPerFrame_pix + colIndex];
			vec_sum.push_back(sum);
		}

	const double sumTL{ 1. * vec_sum.at(0) / nPixQuad };	//Average count top-left
	const double sumTR{ 1. * vec_sum.at(1) / nPixQuad };	//Average count top-right
	const double sumBL{ 1. * vec_sum.at(2) / nPixQuad };	//Average count bottom-left
	const double sumBR{ 1. * vec_sum.at(3) / nPixQuad };	//Average count bottom-right

	//For debuging
	std::cout << "Average count TL = " << sumTL << "\n";
	std::cout << "Average count TR = " << sumTR << "\n";
	std::cout << "Average count BL = " << sumBL << "\n";
	std::cout << "Average count BR = " << sumBR << "\n";

	return (sumTL < threshold_255 && sumTR < threshold_255 && sumBL < threshold_255 && sumBR < threshold_255);
}
*/