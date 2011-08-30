//pixelFixer.h

#pragma once

#include "stdafx.h"

class PixelFixer {

public:
	
	PixelFixer(int, int, int);

	WORD* generateBadPixelTable(char*);

private:

	std::vector<double> detectWeirdPixels(std::vector<double>, char*, int);

	int xSize;

	int ySize;

	int pixelSize;
};