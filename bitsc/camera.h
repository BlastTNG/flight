//camera.h
//a class to interface with the camera

#pragma once


class Camera{

	public:

		Camera();

		bool closeCamera(); //shut down the camera, frees the memory

		void* capture(); //takes a picture when triggered, returns the address of the picture 

		void* subFrameCapture(int, int, int, int);

		int getXSize();

		int getYSize();

		int getPixelSize();

		double getExposureTime();

		void saveActiveData(wchar_t *);

		void characterizeCameraNoise();

	private:
		
		bool initCamera(); //starts up the camera, initializes the memory

		HIDS cameraHandle;

		bool isOpen;

		void* activeMemoryLocation;

		char* memoryPointer;

		SENSORINFO sensorInfo;

		BOARDINFO boardInfo;

		int colourDepth;
		
		int memoryID;

		double exposureTime;
};
