#include "Utilities.h"

//Convert an int to hex and print it out
void printHex(int input)
{
	std::cout << std::hex << std::uppercase << input << std::nouppercase << std::dec << std::endl;
}

void printHex(std::vector<uint8_t>  input)
{
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << (int)input[ii];
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << std::endl;
}

//Convert a string to hex and print it out
void printHex(std::string input)
{
	const char *cstr = input.c_str();
	for (size_t ii = 0; ii < input.size(); ii++)
	{
		std::cout << std::hex << std::uppercase << (int)cstr[ii];
		std::cout << " ";
	}
	std::cout << std::nouppercase << std::dec << std::endl;
}

void printBinary16(int input)
{
	std::cout << std::bitset<16>(input) << std::endl;
}

//Convert a double to a fixed 2p14
U16 convertDoubleToFx2p14(double n)
{
	const int FIXED_BIT = 14; //Number of decimal digits. It MUST match the LV implementation: currently fx2.14 (U16 is split into 2 integer digits + 14 decimal digits)
	U16 int_part = 0, frac_part = 0;
	double t;
	int_part = (U16)floor(n) << FIXED_BIT;
	n -= (U16)floor(n);

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
std::string file_exists(const std::string filename)
{
	std::string suffix("");

	for (int ii = 1; std::experimental::filesystem::exists(folderPath + filename + suffix + ".tif"); ii++)
		suffix = " (" + std::to_string(ii) + ")";

	return filename + suffix;
}

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
	mFileHandle << "\n";
	mFileHandle << description << std::endl;
}

void Logger::record(const std::string description, const double input)
{
	mFileHandle << description << input << std::endl;
}

void Logger::record(const std::string description, const std::string input)
{
	mFileHandle << description << input << std::endl;
}

#pragma region "TiffU8"

//Construct a tiff from a file
TiffU8::TiffU8(const std::string filename)
{
	TIFF *tiffHandle = TIFFOpen((folderPath + filename + ".tif").c_str(), "r");

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + "Opening Tiff failed");

	TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &mWidth);
	TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &mHeight);
	//TIFFGetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, &mStripSize);

	if (mHeight % 2)
		throw std::runtime_error((std::string)__FUNCTION__ + "Odd number of rows not supported");

	std::cout << "Width = " << mWidth << "\n";
	std::cout << "Height = " << mHeight << "\n";
	//std::cout << "Strip size = " << mStripSize << "\n";

	mBytesPerLine = mWidth * sizeof(unsigned char);	//Length in memory of one row of pixel in the image. Targeting 'unsigned char' only
													//alternatively, mBytesPerLine = TIFFScanlineSize(tiffHandle);

	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + "Failed assigning mBytesPerLine");

	unsigned char* buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory for raster of TIFF image");
	}

	mArray = new unsigned char[mWidth * mHeight];	//Allocate memory for the image

	//Read the tiff one strip at a time
	for (int rowIndex = 0; rowIndex < mHeight; rowIndex++)
	{
		if (TIFFReadScanline(tiffHandle, buffer, rowIndex, 0) < 0)
			break;
		std::memcpy(&mArray[(mHeight - rowIndex - 1)*mBytesPerLine], buffer, mBytesPerLine);
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file
}

//Construct a Tiff from an array
TiffU8::TiffU8(const unsigned char* inputImage, const int width, const int height) : mWidth(width), mHeight(height), mBytesPerLine(width * sizeof(unsigned char))
{
	mArray = new unsigned char[width * height];

	//CopyinputImage onto mArray
	std::memcpy(mArray, inputImage, mWidth * mHeight * sizeof(unsigned char));
}

//Construct a Tiff from a vector
TiffU8::TiffU8(const std::vector<unsigned char> &inputImage, const int width, const int height) : mWidth(width), mHeight(height), mBytesPerLine(width * sizeof(unsigned char))
{
	mArray = new unsigned char[width * height];

	//Copy inputImage onto mArray
	std::memcpy(mArray, &inputImage[0], mWidth * mHeight * sizeof(unsigned char));
}

//Construct a Tiff by allocating memory
TiffU8::TiffU8(const int width, const int height) : mWidth(width), mHeight(height), mBytesPerLine(width * sizeof(unsigned char))
{
	mArray = new unsigned char[width * height]();
}

TiffU8::~TiffU8()
{
	delete[] mArray;
}

//Access the Tiff data in the TiffU8 object
unsigned char* const TiffU8::accessTiffArray() const
{
	return mArray;
}

//Split mArray into sub-images (or "frames")
//Purpose: the microscope concatenates each plane in a stack and hands over a vertically long image which has to be resized into sub-images
void TiffU8::saveTiff(std::string filename, const int nFrames, const bool overrideFile) const
{
	if (!overrideFile)
		filename = file_exists(filename);

	TIFF *tiffHandle = TIFFOpen((folderPath + filename + ".tif").c_str(), "w");

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving Tiff failed");

	unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);	//Buffer used to store the row of pixel information for writing to file

		if (buffer == NULL) //Check that the buffer memory was allocated
		{
			TIFFClose(tiffHandle);
			std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory for raster of TIFF image");
		}

	const int heightSingle_pix = mHeight / nFrames; //Divide the total height

	std::string TIFFTAG_ImageJ = "ImageJ=1.52e\nimages=" + std::to_string(nFrames) + "\nchannels=1\nslices=" + std::to_string(nFrames) + "\nhyperstack=true\nmode=grayscale\nunit=\\u00B5m\nloop=false ";

	for (int frame = 0; frame < nFrames; frame++)
	{
		//TAGS
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, mWidth);											//Set the width of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, heightSingle_pix);								//Set the height of the image
		//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);							//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
		TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);											//Set number of channels per pixel
		TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);												//Set the size of the channels
		TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);								//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
		TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);							//Single channel with min as black				
		TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, mWidth));		//Set the strip size of the file to be size of one row of pixels
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEDESCRIPTION, TIFFTAG_ImageJ);								//ImageJ tag hyperstack
		//TIFFSetField(tiffHandle, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);									//Specify that it's a frame within the multipage file
		//TIFFSetField(tiffHandle, TIFFTAG_PAGENUMBER, frame, nFrames);									//Specify the frame number

		//Write the sub-image to the file one strip at a time
		for (int rowIndex = 0; rowIndex < heightSingle_pix; rowIndex++)
		{
			std::memcpy(buffer, &mArray[(frame * heightSingle_pix + heightSingle_pix - rowIndex - 1)*mBytesPerLine], mBytesPerLine);
			if (TIFFWriteScanline(tiffHandle, buffer, rowIndex, 0) < 0)
				break;
		}
		TIFFWriteDirectory(tiffHandle); //Create a page structure
	}

	_TIFFfree(buffer);		//Destroy the buffer
	TIFFClose(tiffHandle);	//Close the output tiff file

	std::cout << "Tiff successfully saved" << std::endl;
}

//The galvo (vectical axis of the image) performs bi-directional scanning
//Divide the long image (vertical stripe) in nFrames and vertically mirror the odd frames
void TiffU8::mirrorVertical(const int nFrames)
{
	if (nFrames > 1)
	{
		unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);		//Buffer used to store the row of pixel information for writing to file

		if (buffer == NULL) //Check that the buffer memory was allocated
			std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory");

		const int heightSingle_pix = mHeight / nFrames; //Divide the total height

		for (int frame = 1; frame < nFrames; frame += 2)
		{
			//Swap the first and last rows of the sub-image, then do to the second first and second last rows, etc
			for (int rowIndex = 0; rowIndex < heightSingle_pix / 2; rowIndex++)
			{
				int eneTene = frame * heightSingle_pix + rowIndex;				//Swap this row
				int moneMei = (frame + 1) * heightSingle_pix - rowIndex - 1;	//With this one
				std::memcpy(buffer, &mArray[eneTene*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[eneTene*mBytesPerLine], &mArray[moneMei*mBytesPerLine], mBytesPerLine);
				std::memcpy(&mArray[moneMei*mBytesPerLine], buffer, mBytesPerLine);
			}
		}
		_TIFFfree(buffer);//Release the memory
	}

}

void TiffU8::averageSeparately(const int nFrames)
{
	if (nFrames > 2)
	{
		const int heightSingle_pix = mHeight / nFrames; //Divide the total height
		const int nPixSingleFrame = mWidth * heightSingle_pix;

		//Calculate the average of the even and odd frames separately
		unsigned int* avg = new unsigned int[2 * nPixSingleFrame]();
		for (int frame = 0; frame < nFrames; frame++)
			for (int pixIndex = 0; pixIndex < nPixSingleFrame; pixIndex++)
			{
				if (frame % 2)
					avg[pixIndex] += mArray[frame * nPixSingleFrame + pixIndex];					//Odd frames
				else
					avg[nPixSingleFrame + pixIndex] += mArray[frame * nPixSingleFrame + pixIndex];	//Even frames
			}

		//Put 'evenImage' and 'oddImage' back into mArray. Concatenate 'oddImage' after 'evenImage'. Ignore the rest of the data in mArray
		int	nFramesHalf = nFrames / 2;
		for (int pixIndex = 0; pixIndex < 2 * nPixSingleFrame; pixIndex++)
		{
			if (nFrames % 2)	//Odd number of frames: 1, 3, 5, etc
				mArray[pixIndex] = static_cast<unsigned char>(1.0 * avg[pixIndex] / (nFramesHalf + 1));
			else				//Even number of frames: 0, 2, 4, etc
				mArray[pixIndex] = static_cast<unsigned char>(1.0 * avg[pixIndex] / nFramesHalf);
		}

		mHeight = 2 * heightSingle_pix;
		delete[] avg;
	}

}

//Split the vertically long image into nFrames and calculate the average
void TiffU8::average(const int nFrames)
{
	if (nFrames > 1)
	{
		mHeight = mHeight / nFrames; //Update the total height to that of a single frame
		const int nPixSingleFrame = mWidth * mHeight;

		unsigned int* avg = new unsigned int[nPixSingleFrame]();
		for (int frame = 0; frame < nFrames; frame++)
			for (int pixIndex = 0; pixIndex < nPixSingleFrame; pixIndex++)
				avg[pixIndex] += mArray[frame * nPixSingleFrame + pixIndex];

		for (int pixIndex = 0; pixIndex < nPixSingleFrame; pixIndex++)
			mArray[pixIndex] = static_cast<unsigned char>(1.0 * avg[pixIndex] / nFrames);

		delete[] avg;
	}
}

void TiffU8::analyze() const
{
	double totalCount = 0;
	for (int index = 0; index < mWidth * mHeight; index++)
		totalCount += mArray[index];

	//std::cout << "Total count = " << totalCount << std::endl;
}


void TiffU8::saveTxt(const std::string filename) const
{
	std::ofstream fileHandle;									//Create output file
	fileHandle.open(folderPath + filename + ".txt");			//Open the file

	for (int pixIndex = 0; pixIndex < mWidth * mHeight; pixIndex++)
		fileHandle << mArray[pixIndex] << std::endl;					//Write each element

	fileHandle.close();											//Close the txt file
}

void TiffU8::pushImage(const int frame, const int nFrames, const unsigned char* inputArray) const
{
	const int height_pix = mHeight / nFrames;
	std::memcpy(&mArray[frame * height_pix * mBytesPerLine], inputArray, height_pix * mBytesPerLine);
}

#pragma endregion "TiffU8"