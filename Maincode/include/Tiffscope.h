#pragma once
#include <tiffio.h>		//for Tiff files
#include <iostream>
#include "Const.h"
#include <experimental/filesystem> //standard method in C++14 but not C++11

using namespace Const;
using namespace Parameters;

void saveAsTiff(unsigned char *imageIn, std::string fileName);
std::string file_exists(std::string filename);
//int readTiff(void);
//int writeSyntheticTiff(void);

