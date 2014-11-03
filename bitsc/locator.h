//locator.h
//a class for locating a blob in the image file

#pragma once

#include <vector>
#include "camera.h"

#define SPIDER_MODE 	1
#define LIGHT_TEST 	2
#define	STAR_TRACKER	3

#define LIGHT_THRESHOLD 200

struct coords{//the coordinates of the object
	double x;
	double y;
};

struct blob{//a structure representing a blob
	coords centroid;
	int size;
	int intensity;
	coords source;
	coords max;
	coords min;
};

class Locator{

	public:

		Locator(int, int, int, int, int, Camera*);

		~Locator();

		blob locate(void*, int, int, int, int);

		void writeData();//writes out a text file of the coordinates
		
	private:

		int totalPixels;

		int xPixels;

		int yPixels;
		
		int pixelDataSize;

		int sigmaNumber;

		int locationMode;

		double averageValue;

		double standardDeviation;

		blob lastBlob;
		
		std::vector<blob> blobVector;

		blob detectBlob(void *, void *);

		bool blobWasFound(void*, void*, std::vector<blob>);

		blob spiderLocate(void*);

		blob lightFinder(void*, int, int);
	
		blob starTracker(void*, int, int);

		Camera* camera;

		void* oldPicture;
};
