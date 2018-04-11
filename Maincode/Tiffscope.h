#pragma once

#include <tiffio.h>		//for Tiff files
#include <iostream>
#include <iostream>
#include "Const.h"
using namespace Const;

void WriteFrameTiff(unsigned char *imageIn, std::string fileName);
int ReadTiff(void);
int WriteSyntheticTiff(void);

