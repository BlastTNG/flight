//locator.h
//a class for locating a blob in the image file

#pragma once

#include <vector>
#include "camera.h"

struct coords{//the coordinates of the object
	int x;
	int y;
};

struct blob{//a structure representing a blob
	coords centroid;
	int size;
	coords source;
	coords max;
	coords min;
};

class Locator{

	public:

		Locator(int, int, int, int, Camera*);

		blob locate(void*);//finds coordinates given an image

		blob locate();

		void writeData();//writes out a text file of the coordinates
		
	private:

		int totalPixels;

		int xPixels;

		int yPixels;
		
		int pixelDataSize;

		int sigmaNumber;

		double averageValue;

		double standardDeviation;

		blob lastBlob;
		
		std::vector<blob> blobVector;

		blob detectBlob(void *, void *);

		bool blobWasFound(void*, void*, std::vector<blob>);

		Camera* camera;
};