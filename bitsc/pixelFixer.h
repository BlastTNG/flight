//pixelFixer.h

#pragma once

#include "stdafx.h"

class PixelFixer {

public:
	
	PixelFixer(int, int, int);

	unsigned short* generateBadPixelTable(const char*);

	unsigned short* loadSavedTable(char*, unsigned short*);

	int savePixelTable(char*, unsigned short*);

	void getImageBackground(void*);

private:

	std::vector<double> detectWeirdPixels(std::vector<double>, char*, int);

	int xSize;

	int ySize;

	int pixelSize;
};
