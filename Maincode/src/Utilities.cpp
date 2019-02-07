#include "Utilities.h"

//Convert an int to hex and print it out
void printHex(const int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << "\n";
}

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
TiffU8::TiffU8(const std::string filename, const int nframes): mNframes(nframes)
{
	TIFF *tiffHandle{ TIFFOpen((folderPath + filename + ".tif").c_str(), "r") };

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Opening Tiff failed");

	int widthAllFrames, heightAllFrames;
	TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &widthAllFrames);
	TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &heightAllFrames);
	//TIFFGetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, &mStripSize);

	if (heightAllFrames % 2)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Odd number of rows not supported");

	std::cout << "Width all frames = " << widthAllFrames << "\n";
	std::cout << "Height all frames = " << heightAllFrames << "\n";
	//std::cout << "Strip size = " << mStripSize << "\n";

	//Assign the width and height of a single frame
	mWidthPerFrame = widthAllFrames;
	mHeightPerFrame = heightAllFrames / nframes;
	mBytesPerLine = mWidthPerFrame * sizeof(unsigned char);	//Length in memory of one row of pixel in the image. Targeting 'unsigned char' only
															//alternatively, mBytesPerLine = TIFFScanlineSize(tiffHandle);

	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failed assigning mBytesPerLine");

	unsigned char* buffer{ (unsigned char *)_TIFFmalloc(mBytesPerLine) };

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	mArray = new unsigned char[mWidthPerFrame * heightAllFrames];	//Allocate memory for the image

	//Read the tiff one strip at a time
	for (int rowIndex = 0; rowIndex < heightAllFrames; rowIndex++)
	{
		if (TIFFReadScanline(tiffHandle, buffer, rowIndex, 0) < 0)
			break;
		std::memcpy(&mArray[(heightAllFrames - rowIndex - 1)*mBytesPerLine], buffer, mBytesPerLine);
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file
}

//Construct a Tiff from an array
TiffU8::TiffU8(const unsigned char* inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(unsigned char))
{
	const int nPixAllFrames{ widthPerFrame * heightPerFrame * nframes };
	mArray = new unsigned char[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, inputImage, nPixAllFrames * sizeof(unsigned char));
}

//Construct a Tiff from a vector
TiffU8::TiffU8(const std::vector<unsigned char> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(unsigned char))
{
	const int nPixAllFrames{ widthPerFrame * heightPerFrame * nframes };
	mArray = new unsigned char[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, &inputImage[0], nPixAllFrames * sizeof(unsigned char));
}

//Construct a Tiff by allocating memory and initialize it
TiffU8::TiffU8(const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(unsigned char))
{
	mArray = new unsigned char[widthPerFrame * heightPerFrame * nframes]();
}

TiffU8::~TiffU8()
{
	delete[] mArray;
}

//Access the Tiff data in the TiffU8 object
unsigned char* const TiffU8::pointerToTiff() const
{
	return mArray;
}

//Split mArray into sub-images (or "frames")
//Purpose: the microscope concatenates each plane in a stack and hands over a vertically long image which has to be resized into sub-images
void TiffU8::saveToFile(std::string filename, const TiffPageStructSelector pageStructFlag, const OverrideFileSelector overrideFlag, const ScanDirection scanDir) const
{
	int width, height, nFrames;

	//Multi page structure
	if (pageStructFlag)
	{
		nFrames = mNframes;
		width = mWidthPerFrame;
		height = mHeightPerFrame;
	}
	//Single page
	else
	{
		nFrames = 1;
		width = mWidthPerFrame;
		height = mHeightPerFrame * mNframes;
	}
	   	  
	/*For debugging
	std::cout << nFrames << "\n";
	std::cout << width << "\n";
	std::cout << height << "\n";
	*/

	if (!overrideFlag)
		filename = doesFileExist(filename);	//Check if the file exits. It gives some overhead
	
	TIFF *tiffHandle{ TIFFOpen((folderPath + filename + ".tif").c_str(), "w") };

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving Tiff failed");

	unsigned char *buffer{ (unsigned char *)_TIFFmalloc(mBytesPerLine) };	//Buffer used to store the row of pixel information for writing to file

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	//Choose whether to save the first frame at the top or bottom of the stack
	int iterFrame, lastFrame;
	switch (scanDir)
	{
	case TOPDOWN:		//Forward saving: first frame at the top of the stack
		iterFrame = 0;
		lastFrame = nFrames - 1;
		break;
	case BOTTOMUP:	//Reverse saving: first frame at the bottom of the stack
		iterFrame = nFrames - 1;
		lastFrame = 0;
		break;
	default:
		throw std::invalid_argument((std::string)__FUNCTION__ + ": Invalid scan direction");
	}

	do
	{
		//TAGS
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, width);											//Set the width of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, height);											//Set the height of the image
		//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);							//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
		TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);											//Set number of channels per pixel
		TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);												//Set the size of the channels
		TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);								//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
		TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);							//Single channel with min as black				
		TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, width));		//Set the strip size of the file to be size of one row of pixels
		//TIFFSetField(tiffHandle, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);									//Specify that it's a frame within the multipage file
		//TIFFSetField(tiffHandle, TIFFTAG_PAGENUMBER, iterFrame, nFrames);								//Specify the frame number

		//IMAGEJ TAG FOR USING HYPERSTACKS
		std::string TIFFTAG_ImageJ = "ImageJ=1.52e\nimages=" + std::to_string(nFrames) + "\nchannels=1\nslices=" + std::to_string(nFrames) + "\nhyperstack=true\nmode=grayscale\nunit=\\u00B5m\nloop=false ";
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEDESCRIPTION, TIFFTAG_ImageJ);								

		//Write the sub-image to the file one strip at a time
		for (int rowIndex = 0; rowIndex < height; rowIndex++)
		{	
			std::memcpy(buffer, &mArray[(iterFrame * height + height - rowIndex - 1)*mBytesPerLine], mBytesPerLine);
			if (TIFFWriteScanline(tiffHandle, buffer, rowIndex, 0) < 0)
				break;
		}
		TIFFWriteDirectory(tiffHandle); //Create a page structure. This gives a large overhead

		if (iterFrame == lastFrame)
			break;

		iterFrame += scanDir; //Increasing iterator for TOPDOWN. Decreasing for BOTTOMUP
	}
	while (true);

	_TIFFfree(buffer);		//Destroy the buffer
	TIFFClose(tiffHandle);	//Close the output tiff file

	std::cout << "Tiff successfully saved\n";
}

//The galvo (vectical axis of the image) performs bi-directional scanning and the data is saved in a long image (vertical stripe)
//Divide the long image in nFrames and vertically mirror the odd frames
void TiffU8::mirrorOddFrames()
{
	if (mNframes > 1)
	{
		unsigned char *buffer{ (unsigned char *)_TIFFmalloc(mBytesPerLine) };		//Buffer used to store the row of pixel information for writing to file

		if (buffer == NULL) //Check that the buffer memory was allocated
			throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory");

		for (int frame = 1; frame < mNframes; frame += 2)
		{
			//Swap the first and last rows of the sub-image, then do to the second first and second last rows, etc
			for (int rowIndex = 0; rowIndex < mHeightPerFrame / 2; rowIndex++)
			{
				int eneTene = frame * mHeightPerFrame + rowIndex;				//Swap this row
				int moneMei = (frame + 1) * mHeightPerFrame - rowIndex - 1;		//With this one
				std::memcpy(buffer, &mArray[eneTene*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[eneTene*mBytesPerLine], &mArray[moneMei*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[moneMei*mBytesPerLine], buffer, mBytesPerLine);
			}
		}
		_TIFFfree(buffer);	//Release the memory
	}

}

//The galvo (vectical axis of the image) performs bi-directional scanning and the data is saved in a long image (vertical stripe)
//Divide the long image in nFrames, average the even and odd frames separately, and return the averages in different pages
void TiffU8::averageEvenOddFrames()
{
	if (mNframes > 2)
	{
		const int nPixPerFrame = mWidthPerFrame * mHeightPerFrame;

		//Calculate the average of the even and odd frames separately
		unsigned int* avg{ new unsigned int[2 * nPixPerFrame]() };
		for (int frame = 0; frame < mNframes; frame++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
			{
				if (frame % 2)
					avg[pixIndex] += mArray[frame * nPixPerFrame + pixIndex];					//Odd frames
				else
					avg[nPixPerFrame + pixIndex] += mArray[frame * nPixPerFrame + pixIndex];	//Even frames
			}

		//Put 'evenImage' and 'oddImage' back into mArray. Concatenate 'oddImage' after 'evenImage'. Ignore the rest of the data in mArray
		int	nFramesHalf{ mNframes / 2 };
		for (int pixIndex = 0; pixIndex < 2 * nPixPerFrame; pixIndex++)
		{
			if (mNframes % 2)	//Odd number of frames: 1, 3, 5, etc
				mArray[pixIndex] = static_cast<unsigned char>(1. * avg[pixIndex] / (nFramesHalf + 1));
			else				//Even number of frames: 0, 2, 4, etc
				mArray[pixIndex] = static_cast<unsigned char>(1. * avg[pixIndex] / nFramesHalf);
		}

		mNframes = 2;	//Keep the averages in separate pages
		delete[] avg;
	}

}

//Split the vertically long image into nFrames and return the average
void TiffU8::averageFrames()
{
	if (mNframes > 1)
	{
		const int nPixPerFrame{ mWidthPerFrame * mHeightPerFrame };

		unsigned int* avg{ new unsigned int[nPixPerFrame]() };
		for (int frame = 0; frame < mNframes; frame++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
				avg[pixIndex] += mArray[frame * nPixPerFrame + pixIndex];

		for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
			mArray[pixIndex] = static_cast<unsigned char>(1. * avg[pixIndex] / mNframes);

		mNframes = 1;
		delete[] avg;
	}
}

void TiffU8::analyze() const
{
	double totalCount{ 0 };
	for (int index = 0; index < mWidthPerFrame * mHeightPerFrame; index++)
		totalCount += mArray[index];

	//std::cout << "Total count = " << totalCount << "\n";
}


void TiffU8::saveTxt(const std::string filename) const
{
	std::ofstream fileHandle;									//Create output file
	fileHandle.open(folderPath + filename + ".txt");			//Open the file

	for (int pixIndex = 0; pixIndex < mWidthPerFrame * mHeightPerFrame * mNframes; pixIndex++)
		fileHandle << mArray[pixIndex] << "\n";					//Write each element

	fileHandle.close();											//Close the txt file
}

void TiffU8::pushImage(const int frame, const unsigned char* inputArray) const
{
	std::memcpy(&mArray[frame * mHeightPerFrame * mBytesPerLine], inputArray, mHeightPerFrame * mBytesPerLine);
}

#pragma endregion "TiffU8"


#pragma region "Stack"
TiffStack::TiffStack(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nDiffZ, const int nSameZ) :
	mDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ), mSameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ) {}

void TiffStack::pushSameZ(const int indexSameZ, unsigned char* const pointerToTiff)
{
	mSameZ.pushImage(indexSameZ, pointerToTiff);
}

void TiffStack::pushDiffZ(const int indexDiffZ)
{
	mSameZ.averageFrames();	//Average the images with the same Z
	mDiffZ.pushImage(indexDiffZ, mSameZ.pointerToTiff());
}

void TiffStack::saveToFile(const std::string filename, OverrideFileSelector overrideFlag) const
{
	mDiffZ.saveToFile(filename, MULTIPAGE, overrideFlag);
}
#pragma endregion "Stack"