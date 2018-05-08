/* The following example is from http://research.cs.wisc.edu/graphics/Courses/638-f1999/libtiff_tutorial.htm */

#include "Tiffscope.h"

int writeFrametoTiff(unsigned char *imageIn, std::string fileName)
{
	TIFF *tiffHandle = TIFFOpen(fileName.c_str(), "w");

	if (tiffHandle == nullptr)
		throw std::runtime_error((std::string)__FUNCTION__ + ": Saving Tiff failed");

	unsigned char *image = new unsigned char[nPixPerFrame];			//Create an 1D array representing the image

	for (int ii = 0; ii < nPixPerFrame; ii++)
		image[ii] = 0;																	//Initialize the array

	
	for (int ii = 0; ii < nPixPerFrame; ii++)
		image[ii] =  imageIn[ii];

	//TAGS
	TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, widthPerFrame_pix);					//Set the width of the image
	TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, heightPerFrame_pix);					//Set the height of the image
	TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, 1);					//Set number of channels per pixel
	TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);									//Set the size of the channels
	TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);					//Set the origin of the image. Many readers ignore this tag (ImageJ, Windows preview, etc...)
	//TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);				//PLANARCONFIG_CONTIG (for example, RGBRGBRGB) or PLANARCONFIG_SEPARATE (R, G, and B separate)
	TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);				//Single channel with min as black


	tsize_t bytesPerLine = widthPerFrame_pix;			//Length in memory of one row of pixel in the image.
	unsigned char *buffer = nullptr;									//Buffer used to store the row of pixel information for writing to file

	//Allocating memory to store pixels of current row
	if (TIFFScanlineSize(tiffHandle))
		buffer = (unsigned char *)_TIFFmalloc(bytesPerLine);
	else
		buffer = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(tiffHandle));

	//Set the strip size of the file to be size of one row of pixels
	TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, widthPerFrame_pix));

	//Now writing image to the file one strip at a time. CURRENTLY ONLE THE FIRST FRAME IS SAVED!!!!
	for (int row = 0; row < heightPerFrame_pix; row++)
	{
		memcpy(buffer, &image[(heightPerFrame_pix - row - 1)*bytesPerLine], bytesPerLine);    // check the index here, and figure tiffHandle why not using h*bytesPerLine
		if (TIFFWriteScanline(tiffHandle, buffer, row, 0) < 0)
			break;
	}

	//Close the output file
	(void)TIFFClose(tiffHandle);

	//Destroy the buffer
	if (buffer)
		_TIFFfree(buffer);

	return 0;
}



/*
//READ FILE EXAMPLE
int readTiff(void)
{
	TIFF *tif = TIFFOpen("D:\\OwnCloud\\Codes\\Cpp playground\\Playground\\marbles.tif", "r");

	//define uint32 unsigned long
	uint32 width, height;
	TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
	TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);        // uint32 height;

	std::cout << "Width = " << width << "\n";
	std::cout << "Height = " << height << "\n";

	uint32 npixels = width * height;
	uint32* raster = (uint32 *)_TIFFmalloc(npixels * sizeof(uint32));

	if (raster == nullptr) // check the raster's memory was allocaed
	{
		TIFFClose(tif);
		std::cerr << "Could not allocate memory for raster of TIFF image" << std::endl;
		return -1;
	}

	// Check the tif read to the raster correctly
	if (!TIFFReadRGBAImage(tif, width, height, raster, 0))
	{
		TIFFClose(tif);
		std::cerr << "Could not read raster of TIFF image" << std::endl;
		return -1;
	}

	//read a pixel. The channels are RGBA
	unsigned char X = (unsigned char)TIFFGetA(raster[0]);

	std::cout << +X << "\n";

	//release the memory
	_TIFFfree(raster);

	//close the tif file
	TIFFClose(tif);

	return 0;
}


//WRITE FILE EXAMPLE
int writeSyntheticTiff(void)
{
	uint32 width = 400;
	uint32 height = 400;

	TIFF *tiffHandle = TIFFOpen("tiffExample.tif", "w");

	//1 channel
	int samplePerPixel = 1;

	//create an 1D array representing the image
	unsigned char *image = new unsigned char[width*height*samplePerPixel];

	//initialize the array
	for (int ii = 0; ii < width*height*samplePerPixel; ii++)
		image[ii] = 0;

	//create a color gradient as an exercise
	for (int ii = 0; ii < width*height*samplePerPixel; ii++)
	{
		image[ii] = 255. / (width*height)*ii ;
	}

	//color gradient
	//for (int ii = 0; ii < width*height*samplePerPixel; ii += 4)
	{
		//image[ii] = 255. / (width*height)*ii / 4; //Red
		//image[ii+1] = 255. / (width*height)*ii / 4; //Green
		//image[ii+2] = 255. / (width*height)*ii / 4; //Blue
	}

	//TAGS
	TIFFSetField(tiffHandle, TIFFTAG_IMAGEWIDTH, width);					// set the width of the image
	TIFFSetField(tiffHandle, TIFFTAG_IMAGELENGTH, height);					// set the height of the image
	TIFFSetField(tiffHandle, TIFFTAG_SAMPLESPERPIXEL, samplePerPixel);		// set number of channels per pixel
	TIFFSetField(tiffHandle, TIFFTAG_BITSPERSAMPLE, 8);					// set the size of the channels
	TIFFSetField(tiffHandle, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);    // set the origin of the image.
																	//   Some other essential fields to set that you do not have to understand for now.
	TIFFSetField(tiffHandle, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tiffHandle, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);


	tsize_t bytesPerLine = samplePerPixel * width;						// length in memory of one row of pixel in the image.
	unsigned char *buffer = nullptr;										// buffer used to store the row of pixel information for writing to file

																	// Allocating memory to store the pixels of current row
	if (TIFFScanlineSize(tiffHandle))
		buffer = (unsigned char *)_TIFFmalloc(bytesPerLine);
	else
		buffer = (unsigned char *)_TIFFmalloc(TIFFScanlineSize(tiffHandle));

	// set the strip size of the file to be size of one row of pixels
	TIFFSetField(tiffHandle, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tiffHandle, width*samplePerPixel));

	//Now writing image to the file one strip at a time
	for (uint32 row = 0; row < height; row++)
	{
		memcpy(buffer, &image[(height - row - 1)*bytesPerLine], bytesPerLine);    // check the index here, and figure tiffHandle why not using h*bytesPerLine
		if (TIFFWriteScanline(tiffHandle, buffer, row, 0) < 0)
			break;
	}

	//close the output file
	(void)TIFFClose(tiffHandle);

	//destroy the buffer
	if (buffer)
		_TIFFfree(buffer);

	return 0;
}


*/