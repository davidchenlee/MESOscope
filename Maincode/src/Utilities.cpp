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

	//Initialize the member variables for width and height of a single frame
	mWidthPerFrame = widthAllFrames;
	mHeightPerFrame = heightAllFrames / nframes;
	mBytesPerLine = mWidthPerFrame * sizeof(U8);	//Length in memory of one row of pixel in the image. Targeting 'U8' only
															//alternatively, mBytesPerLine = TIFFScanlineSize(tiffHandle);

	if (mBytesPerLine == NULL)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Failed assigning mBytesPerLine");

	U8* buffer{ (U8*)_TIFFmalloc(mBytesPerLine) };

	if (buffer == NULL) //Check that the buffer memory was allocated
	{
		TIFFClose(tiffHandle);
		throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory for raster of TIFF image");
	}

	mArray = new U8[mWidthPerFrame * heightAllFrames];	//Allocate memory for the image

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
TiffU8::TiffU8(const U8* inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(U8))
{
	const int nPixAllFrames{ widthPerFrame * heightPerFrame * nframes };
	mArray = new U8[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, inputImage, nPixAllFrames * sizeof(U8));
}

//Construct a Tiff from a vector
TiffU8::TiffU8(const std::vector<U8> &inputImage, const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(U8))
{
	const int nPixAllFrames{ widthPerFrame * heightPerFrame * nframes };
	mArray = new U8[nPixAllFrames];

	//Copy input image onto mArray
	std::memcpy(mArray, &inputImage[0], nPixAllFrames * sizeof(U8));
}

//Construct a new Tiff by allocating memory and initialize it
TiffU8::TiffU8(const int widthPerFrame, const int heightPerFrame, const int nframes) :
	mWidthPerFrame(widthPerFrame), mHeightPerFrame(heightPerFrame), mNframes(nframes), mBytesPerLine(widthPerFrame * sizeof(U8))
{
	mArray = new U8[widthPerFrame * heightPerFrame * nframes]();
}

TiffU8::~TiffU8()
{
	delete[] mArray;
}

//Access the Tiff data in the TiffU8 object
U8* const TiffU8::pointerToTiff() const
{
	return mArray;
}

int TiffU8::widthPerFrame() const
{
	return mWidthPerFrame;
}

int TiffU8::heightPerFrame() const
{
	return mHeightPerFrame;
}

int TiffU8::nFrames() const
{
	return mNframes;
}

//Split mArray into sub-images (or "frames")
//Purpose: the microscope concatenates all the planes in a scanned stack and hands over a vertically-concatenated image which has to be resized into sub-images
void TiffU8::saveToFile(std::string filename, const MULTIPAGE multipage, const OVERRIDE override, const ZSCAN scanDir) const
{
	int width, height, nFrames;

	//Multi page structure
	if (static_cast<bool>(multipage))
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

	if (!static_cast<bool>(override))
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
	int iterFrame, lastFrame;
	switch (scanDir)
	{
	case ZSCAN::TOPDOWN:		//Forward saving: first frame at the top of the stack
		iterFrame = 0;
		lastFrame = nFrames - 1;
		break;
	case ZSCAN::BOTTOMUP:	//Reverse saving: first frame at the bottom of the stack
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
		//I think many readers ignore the 'TIFFTAG_ORIENTATION' tag and consider the origin of the image at the TOP-LEFT
		for (int rowIndex = 0; rowIndex < height; rowIndex++)
		{
			std::memcpy(buffer, &mArray[(iterFrame * height + rowIndex)*mBytesPerLine], mBytesPerLine);
			if (TIFFWriteScanline(tiffHandle, buffer, rowIndex, 0) < 0)
				break;
		}
		TIFFWriteDirectory(tiffHandle); //Create a page structure. This gives a large overhead

		if (iterFrame == lastFrame)
			break;

		iterFrame += static_cast<int>(scanDir); //Increasing iterator for TOPDOWN. Decreasing for BOTTOMUP
	} while (true);

	_TIFFfree(buffer);		//Destroy the buffer
	TIFFClose(tiffHandle);	//Close the output tiff file

	std::cout << "Successfully saved: " << filename << ".tif\n";
}

//The galvo (vectical axis of the image) performs bi-directional scanning and the data is saved in a vertically-concatenated image
//Divide the long image in nFrames and mirror the odd frames vertically
void TiffU8::mirrorOddFrames()
{
	if (mNframes > 1)
	{
		U8 *buffer{ (U8*)_TIFFmalloc(mBytesPerLine) };		//Buffer used to store the row of pixel information for writing to file

		if (buffer == NULL) //Check that the buffer memory was allocated
			throw std::runtime_error((std::string)__FUNCTION__ + ": Could not allocate memory");

		for (int frame = 1; frame < mNframes; frame += 2)
		{
			//Swap the first and last rows of the sub-image, then do the second and second last rows, etc
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
				mArray[pixIndex] = static_cast<U8>(1. * avg[pixIndex] / (nFramesHalf + 1));
			else				//Even number of frames: 0, 2, 4, etc
				mArray[pixIndex] = static_cast<U8>(1. * avg[pixIndex] / nFramesHalf);
		}

		mNframes = 2;	//Keep the averages in separate pages
		delete[] avg;
	}
}

//Divide the vertically-concatenated image into 'nFrames' frames and return the average over all the frames
void TiffU8::averageFrames()
{
	if (mNframes > 1)
	{
		const int nPixPerFrame{ mWidthPerFrame * mHeightPerFrame };
		unsigned int* avg{ new unsigned int[nPixPerFrame]() };

		//For each pixel, calculate the sum intensity over all the frames
		for (int frameIndex = 0; frameIndex < mNframes; frameIndex++)
			for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
				avg[pixIndex] += mArray[frameIndex * nPixPerFrame + pixIndex];

		//Calculate the average intensity and reassign  it to mArray
		for (int pixIndex = 0; pixIndex < nPixPerFrame; pixIndex++)
			mArray[pixIndex] = static_cast<U8>(1. * avg[pixIndex] / mNframes);

		//Update the number of frames to 1
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

//Specify the frame to push. The frame index starts from 0
void TiffU8::pushImage(const int frameIndex, const U8* inputArray) const
{
	if (frameIndex > mNframes)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be smaller than or equal to the number of frames");

	std::memcpy(&mArray[frameIndex * mHeightPerFrame * mBytesPerLine], inputArray, mHeightPerFrame * mBytesPerLine);
}

//Specify the frame interval to push. The frame index starts from 0
void TiffU8::pushImage(const int firstFrameIndex, const int lastFrameIndex, const U8* inputArray) const
{
	if (firstFrameIndex > mNframes || lastFrameIndex > mNframes)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": The frame index must be smaller than or equal to the number of frames");

	if (lastFrameIndex < firstFrameIndex)
		throw std::invalid_argument((std::string)__FUNCTION__ + ": lastFrameIndex must be greater than or equal to firstFrameIndex");

	std::memcpy(&mArray[firstFrameIndex * mHeightPerFrame * mBytesPerLine], inputArray, (lastFrameIndex - firstFrameIndex + 1) * mHeightPerFrame * mBytesPerLine);
}


/*
The input arrays have the structure:
inputArrayA = |CH01 f1|
			  |  .	  |
			  |CH01 fN|
			  |  .	  |
			  |  .	  |
			  |  .	  |
			  |CH08 f1|
			  |  .	  |
			  |CH08 fN|

inputArrayB = |CH09 f1|
			  |  .	  |
			  |CH09 fN|
			  |  .	  |
			  |  .	  |
			  |  .	  |
			  |CH16 f1|
			  |  .	  |
			  |CH16 fN|

"Merging" places channels belonging to the same frame together. The resulting structure is:
mArray = |CH01 f1|
		 |  .	 |
		 |CH16 f1|
		 |CH16 f2|
		 |  .	 |
		 |  .	 |
		 |  .	 |
		 |CH01 f2|
		 |CH01 fN|
		 |  .	 |
		 |CH16 fN|
Note in the figure above that the channel ordering within each frame is reversed wrt the next frame because of the bidirectionality of the scan galvo
*/
void TiffU8::mergePMT16Xchannels(const int heightPerChannelPerFrame, const U8* inputArrayA, const U8* inputArrayB) const
{
	//old way
	//std::memcpy(mArray, inputArrayA, 8 * heightPerChannelAllFrames * mBytesPerLine);
	//std::memcpy(&mArray[8 * heightPerChannelAllFrames * mBytesPerLine], inputArrayB, 8 * heightPerChannelAllFrames * mBytesPerLine);

	const int heightAllChannelsPerFrame = 16 * heightPerChannelPerFrame;
	const int heightPerChannelAllFrames = heightPerChannelPerFrame * mNframes;

	//Note that CH01 corresponds to chanIndex = 0,  CH02 corresponds to chanIndex = 1, etc
	//Even 'frameIndex' (forward scan)
	for (int frameIndex = 0; frameIndex < mNframes; frameIndex += 2)
		for (int chanIndex = 0; chanIndex < 8; chanIndex++)
		{
			std::memcpy(&mArray[(chanIndex * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			std::memcpy(&mArray[((chanIndex + 8) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}

	//Odd 'frameIndex' (backward scan)
	for (int frameIndex = 1; frameIndex < mNframes; frameIndex += 2)
		for (int chanIndex = 0; chanIndex < 8; chanIndex++)
		{
			std::memcpy(&mArray[((15 - chanIndex) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayA[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
			std::memcpy(&mArray[((7 - chanIndex) * heightPerChannelPerFrame + frameIndex * heightAllChannelsPerFrame) * mBytesPerLine], &inputArrayB[(frameIndex * heightPerChannelPerFrame + chanIndex * heightPerChannelAllFrames) * mBytesPerLine], heightPerChannelPerFrame * mBytesPerLine);
		}
}




inline int round_to_int(float r) {
	return (int)lrint(r);
}

inline int clip(int x, int lower, int upper)
{
	return min(upper, max(x, lower));

}

template<class T> inline T interpolate(float lam, const T  &val1, const T &val2)
{
	int res = round_to_int((1.f - lam)*val1 + lam * val2);

	return (T)(clip(res, (std::numeric_limits<U8>::min)(), (std::numeric_limits<U8>::max)()));
}



//Correct the image distortion induced by the nonlinear scanning of the RS
void TiffU8::correctRSdistortion()
{
	const int nPixAllFrames{ mWidthPerFrame * mHeightPerFrame * mNframes };
	U8* correctedArray = new U8[nPixAllFrames];



	/*
		# time per half mirror scan in us
	T0 = .5 / f * 1e6

	Nx = x.shape[-1]

	# calculate start and stop time in us, assuming centered stage
	if t1 is None:
		t1 = .5 * (T0 - Nx * dt)
	t2 = T0 - t1


	# the full width in um
	w0 = 2 * w / (np.cos(2 * np.pi * f * t1 * 1e-6) - np.cos(2 * np.pi * f * t2 * 1e-6))

	# start and stop positions
	x1 = w0 / 2. * (1 - np.cos(2 * np.pi * f * t1 * 1e-6))
	x2 = w0 / 2. * (1 - np.cos(2 * np.pi * f * t2 * 1e-6))

	# calculate the new output shape

	Nx_out = int(np.round((x2 - x1) / dx))

	xbar1 = x1 / w0
	xbar2 = x2 / w0
	tbar1 = t1 / T0
	tbar2 = t2 / T0
	*/



	// precompute the mapping of the fast coordinate (k)
	float *kk_floats_precomputed = new float[mWidthPerFrame];
	for (int k = 0; k < mWidthPerFrame; k++) {
		const float x = k / (mWidthPerFrame - 1.f);
		//const float a = 1.f - 2.f*x1 - 2.f*(x2 - x1)*x;
		//const float t = (acos(a) / PI - t1) / (t2 - t1);
		//kk_floats_precomputed[k] = t * (mWidthPerFrame - 1.f);
	}

//# pragma omp parallel for schedule(dynamic)
	for (int j = 0; j < mHeightPerFrame; j++) {
		for (int k = 0; k < mWidthPerFrame; k++) {

			const float kk_float = kk_floats_precomputed[k];
			const int kk = round_to_int(floor(kk_float));
			const int kk1 = clip(kk, 0, mWidthPerFrame - 1);
			const int kk2 = clip(kk + 1, 0, mWidthPerFrame - 1);

			const U8 value1 = mArray[j, kk1];	//Read from the input array
			const U8 value2 = mArray[j, kk2];	//Read from the input array
			correctedArray[j, k] = interpolate<U8>(kk_float - kk1, value1, value2);	//Interpolate and save to the output array
		}
	}

	delete[] kk_floats_precomputed;
	


	//Apply correction to mArray. For now, I just copy all the pixels
	std::memcpy(correctedArray, mArray, nPixAllFrames * sizeof(U8));

	delete[] mArray;			//Free the memory-block containing the old, uncorrected array

	mArray = correctedArray;	//Reassign the pointer mArray to the new, corrected array
}

#pragma endregion "TiffU8"


#pragma region "Stack"
TiffStack::TiffStack(const int widthPerFrame_pix, const int heightPerFrame_pix, const int nDiffZ, const int nSameZ) :
	mDiffZ(widthPerFrame_pix, heightPerFrame_pix, nDiffZ), mSameZ(widthPerFrame_pix, heightPerFrame_pix, nSameZ) {}

void TiffStack::pushSameZ(const int indexSameZ, U8* const pointerToTiff)
{
	mSameZ.pushImage(indexSameZ, pointerToTiff);
}

void TiffStack::pushDiffZ(const int indexDiffZ)
{
	//Temporary hack
	//I want to average all the stacks in mSameZ and then move to the next plane and repeat
	//However, averageFrames() collapses mSameZ to a single image containing the average
	//Solution: make a temporary copy of mSameZ and calculate the average over it
	TiffU8 auxTiff{ mSameZ.pointerToTiff(), mSameZ.widthPerFrame(), mSameZ.heightPerFrame(), mSameZ.nFrames() }; //Make a copy of mSameZ
	auxTiff.averageFrames();	//Average the images with the same Z
	mDiffZ.pushImage(indexDiffZ, auxTiff.pointerToTiff());
}

void TiffStack::saveToFile(const std::string filename, OVERRIDE override) const
{
	mDiffZ.saveToFile(filename, MULTIPAGE::EN, override);
}
#pragma endregion "Stack"