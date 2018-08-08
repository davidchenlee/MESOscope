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
	std::string suffix = "";

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


//Open a Tiff in the constructor
Tiffer::Tiffer(std::string filename)
{
	TIFF *tiffHandle = TIFFOpen((folderPath + filename + ".tif").c_str(), "r");

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + "Opening Tiff failed");

	TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &mWidth_pix);
	TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &mHeight_pix);
	TIFFGetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, &mStripSize_pix);

	if (mHeight_pix % 2)
		throw std::runtime_error((std::string)__FUNCTION__ + "Odd number of rows not supported");

	std::cout << "Width = " << mWidth_pix << "\n";
	std::cout << "Height = " << mHeight_pix << "\n";
	std::cout << "Strip size = " << mStripSize_pix << "\n";

	mBytesPerLine = TIFFScanlineSize(tiffHandle);	//Length in memory of one row of pixel in the image.

	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + "Assigning mBytesPerLine failed");

	unsigned char* buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory for raster of TIFF image");
	}

	mImage.resize(mWidth_pix * mHeight_pix);	//Allocate memory for the image

	//Read the tiff one strip at a time
	for (int rowIndex = 0; rowIndex < mHeight_pix; rowIndex++)
	{
		if (TIFFReadScanline(tiffHandle, buffer, rowIndex, 0) < 0)
			break;
		std::memcpy(&mImage[(mHeight_pix - rowIndex - 1)*mBytesPerLine], buffer, mBytesPerLine);
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file
}

Tiffer::~Tiffer()
{
}

//Split mImage into sub-images (or "segment")
//Purpose: the microscope concatenates each plane in a stack and hands over a vertically long image, which has to be re-structured into sub-images
void Tiffer::saveTiff(std::string filename, const int nSegments) const
{
	TIFF *tiffHandle = TIFFOpen((folderPath + filename + ".tif").c_str(), "w");

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving Tiff failed");

	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + "mBytesPerLine is NULL");

	unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);	//Buffer used to store the row of pixel information for writing to file

	const int heightSingle_pix = mHeight_pix / nSegments; //Divide the total height
	for (int segment = 0; segment < nSegments; segment++)
	{
		//TAGS
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, mWidth_pix);										//Set the width of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, heightSingle_pix);								//Set the height of the image
		TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);											//Set number of channels per pixel
		TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);												//Set the size of the channels
		TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);								//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
		//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);							//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
		TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);							//Single channel with min as black				
		TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, mWidth_pix));	//Set the strip size of the file to be size of one row of pixels
		//TIFFSetField(tiffHandle, TIFFTAG_SUBFILETYPE, 3);												//Specify that it's a segment within the multipage file
		//TIFFSetField(tiffHandle, TIFFTAG_PAGENUMBER, segment, nSegments);									//Specify the segment number

		if (buffer == NULL) //Check that the buffer memory was allocated
		{
			TIFFClose(tiffHandle);
			std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory for raster of TIFF image");
		}

		//Write the sub-image to the file one strip at a time
		for (int rowIndex = 0; rowIndex < heightSingle_pix; rowIndex++)
		{
			std::memcpy(buffer, &mImage[(segment * heightSingle_pix + heightSingle_pix - rowIndex - 1)*mBytesPerLine], mBytesPerLine);
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
//Divide the vertically long image in nSegments and vertically flip the odd segments
void Tiffer::verticalFlip(const int nSegments)
{
	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + "mBytesPerLine is NULL");

	unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);		//Buffer used to store the row of pixel information for writing to file

	if (buffer == NULL) //Check that the buffer memory was allocated
		std::runtime_error((std::string)__FUNCTION__ + "Could not allocate memory");

	const int heightSingle_pix = mHeight_pix / nSegments; //Divide the total height

	for (int segment = 1; segment < nSegments; segment+=2)
	{
		//Swap the first and last rows of the sub-image, then do to the second first and second last rows, etc
		for (int rowIndex = 0; rowIndex < heightSingle_pix / 2; rowIndex++)
		{
			int eneTene = segment * heightSingle_pix + rowIndex;			//Swap this row
			int moneMei = (segment + 1) * heightSingle_pix - rowIndex - 1;	//With this one
			std::memcpy(buffer, &mImage[eneTene*mBytesPerLine], mBytesPerLine);
			std::memcpy(&mImage[eneTene*mBytesPerLine], &mImage[moneMei*mBytesPerLine], mBytesPerLine);
			std::memcpy(&mImage[moneMei*mBytesPerLine], buffer, mBytesPerLine);
		}
	}
	_TIFFfree(buffer);//Release the memory
}

//Split the vertically long image into nSegments and calculate the average
void Tiffer::average(const int nSegments)
{
	mHeight_pix = mHeight_pix / nSegments; //Divide the total height
	const int nPix = mWidth_pix * mHeight_pix;

	std::vector<double> avg(nPix);

	for (int pixIndex = 0; pixIndex < nPix; pixIndex++)
		for (int segment = 0; segment < nSegments; segment++)
			//avg.at(pixIndex) += 1.0 * mImage.at(segment * nPix + pixIndex);
			avg.at(pixIndex) += 1.0 * mImage.at(segment * nPix + pixIndex) / nSegments;

	mImage.resize(nPix);

	for (int pixIndex = 0; pixIndex < nPix; pixIndex++)
		mImage.at(pixIndex) = (unsigned char)avg.at(pixIndex);
}