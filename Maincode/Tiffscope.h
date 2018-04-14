#pragma once

#include <tiffio.h>		//for Tiff files
#include <iostream>
#include <iostream>
#include "Const.h"
using namespace Const;

int writeFrameToTiff(unsigned char *imageIn, std::string fileName);
int readTiff(void);
int writeSyntheticTiff(void);

