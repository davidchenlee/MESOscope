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

	for (int ii = 1; std::experimental::filesystem::exists(foldername + filename + suffix + ".tif"); ii++)
		suffix = " (" + std::to_string(ii) + ")";

	return filename + suffix;
}

Logger::Logger(const std::string filename)
{
	mFileHandle.open(foldername + filename + ".txt");
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


//open a Tiff in the constructor
Tiffer::Tiffer(std::string filename)
{
	TIFF *tiffHandle = TIFFOpen((foldername + filename + ".tif").c_str(), "r");

	if (tiffHandle == nullptr)
		throw std::runtime_error("Opening Tiff failed");

	TIFFGetField(tiffHandle, TIFFTAG_IMAGEWIDTH, &mWidth_pix);
	TIFFGetField(tiffHandle, TIFFTAG_IMAGELENGTH, &mHeight_pix);
	TIFFGetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, &mStripSize_pix);

	if (mHeight_pix % 2)
		throw std::runtime_error("Odd number of rows not supported");

	std::cout << "Width = " << mWidth_pix << "\n";
	std::cout << "Height = " << mHeight_pix << "\n";
	std::cout << "Strip size = " << mStripSize_pix << "\n";

	mBytesPerLine = TIFFScanlineSize(tiffHandle);				//Length in memory of one row of pixel in the image.

	if (mBytesPerLine == NULL)
		throw std::runtime_error("Assigning mBytesPerLine failed");

	unsigned char* buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);

	if (buffer == NULL) //Check the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		std::runtime_error("Could not allocate memory for raster of TIFF image");
	}

	mImage.resize(mWidth_pix * mHeight_pix);	//Allocate memory for the image

	//Now writing image to the file one strip at a time
	for (int rowIndex = 0; rowIndex < mHeight_pix; rowIndex++)
	{
		if (TIFFReadScanline(tiffHandle, buffer, rowIndex, 0) < 0)
			break;
		std::memcpy(&mImage[(mHeight_pix - rowIndex - 1)*mBytesPerLine], buffer, mBytesPerLine);    // check the index here, and figure why not using h*bytesPerLine
	}

	_TIFFfree(buffer);		//Release the memory
	TIFFClose(tiffHandle);	//Close the tif file
}

Tiffer::~Tiffer()
{

}

//Split the image into nPages
void Tiffer::saveToTiff(std::string filename, const int nPages)
{
	TIFF *tiffHandle = TIFFOpen((foldername + filename + ".tif").c_str(), "w");

	mHeight_pix = mHeight_pix / nPages; //Divide the large image into nPages

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving Tiff failed");

	if (mBytesPerLine == NULL)
		throw std::runtime_error("mBytesPerLine is NULL");

	unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);							//Buffer used to store the row of pixel information for writing to file

	for (int page = 0; page < nPages; page++)
	{
		//TAGS
		TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, mWidth_pix);										//Set the width of the image
		TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, mHeight_pix);										//Set the height of the image
		TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);											//Set number of channels per pixel
		TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);												//Set the size of the channels
		TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);								//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
																										//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);							//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
		TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);							//Single channel with min as black				
		TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, mWidth_pix));	//Set the strip size of the file to be size of one row of pixels
		//TIFFSetField(tiffHandle, TIFFTAG_SUBFILETYPE, 3);												//Specify that it's a page within the multipage file
		//TIFFSetField(tiffHandle, TIFFTAG_PAGENUMBER, page, nPages);									//Specify the page number


		if (buffer == NULL) //Check the buffer memory was allocated
		{
			TIFFClose(tiffHandle);
			std::runtime_error("Could not allocate memory for raster of TIFF image");
		}

		//Now writing image to the file one strip at a time
		for (int rowIndex = 0; rowIndex < mHeight_pix; rowIndex++)
		{
			std::memcpy(buffer, &mImage[(page * mHeight_pix + mHeight_pix - rowIndex - 1)*mBytesPerLine], mBytesPerLine);    // check the index here, and figure out why not using h*bytesPerLine
			if (TIFFWriteScanline(tiffHandle, buffer, rowIndex, 0) < 0)
				break;
		}
		TIFFWriteDirectory(tiffHandle); //Create a page structure
	}

	_TIFFfree(buffer);//Destroy the buffer
	TIFFClose(tiffHandle);//Close the output file

	std::cout << "File successfully saved" << std::endl;
}

//Vertically flip a particular page = 0, 1, 2, ...
void Tiffer::verticalFlip(const int page)
{
	if (mBytesPerLine == NULL)
		throw std::runtime_error("mBytesPerLine is NULL");

	unsigned char *buffer = (unsigned char *)_TIFFmalloc(mBytesPerLine);		//Buffer used to store the row of pixel information for writing to file

	if (buffer == NULL) //Check the buffer memory was allocated
		std::runtime_error("Could not allocate memory");

	//Now writing image to the file one strip at a time
	int halfHeight_pix = mHeight_pix / 2;
	for (int rowIndex = 0; rowIndex < halfHeight_pix / 2; rowIndex++)
	{
		std::memcpy(buffer, &mImage[(page*halfHeight_pix + rowIndex)*mBytesPerLine], mBytesPerLine);
		std::memcpy(&mImage[(page*halfHeight_pix + rowIndex)*mBytesPerLine], &mImage[(page*halfHeight_pix + halfHeight_pix - rowIndex - 1)*mBytesPerLine], mBytesPerLine);
		std::memcpy(&mImage[(page*halfHeight_pix + halfHeight_pix - rowIndex - 1)*mBytesPerLine], buffer, mBytesPerLine);
	}
	_TIFFfree(buffer);//Release the memory
}

void Tiffer::average()
{

}