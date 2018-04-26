#pragma once
#include <tiffio.h>		//for Tiff files
#include <iostream>
#include "Const.h"
using namespace Const;

int writeFrametoTiff(unsigned char *imageIn, std::string fileName);
//int readTiff(void);
//int writeSyntheticTiff(void);

