//camera.cpp
//a wrapper class for the camera fuctionality, used to return the pictures

#include "stdafx.h"
#include "camera.h"
#include "pixelFixer.h"

#define EXPOSURE_TIME 		50
#define GAIN_PERCENTAGE 	100
#define PIXEL_CLOCK_RATE 	100
#define FRAMERATE		10.00

Camera::Camera(){
	if(!initCamera()){
		printf("Camera failed to initialize. Giving up.\n");
		isOpen = false;
	}else{
		isOpen = true;
	}
}

bool Camera::closeCamera(){
	is_FreeImageMem(cameraHandle, memoryPointer, memoryID);//frees the memory that was used
	is_ExitCamera(cameraHandle);//closes the camera
	isOpen = false;//tell the wrapper class that it is closed
	return true;
}

void* Camera::capture(){
	if(!isOpen){//makes sure that the camera is still open
		printf("Camera is not open for pictures. Returning.\n");
		return NULL;
	}

	timeval systemTime1;
	//timeval systemTime2;

	int status;
	IS_RECT aoiRectangle;
	status = is_AOI(cameraHandle, IS_AOI_IMAGE_GET_AOI, (void*) &aoiRectangle, sizeof(aoiRectangle));

	if(status == IS_NO_SUCCESS){
		printf("IS_AOI fails in camera::capture\n");
	}
	
	//ensures that the AOI is set to refer to the full frame
	if(aoiRectangle.s32Width != getXSize() || aoiRectangle.s32Height != getYSize()){
		aoiRectangle.s32X = 0;
		aoiRectangle.s32Y = 0;
		aoiRectangle.s32Width = getXSize();
		aoiRectangle.s32Height = getYSize();
		status = is_AOI(cameraHandle, IS_AOI_IMAGE_SET_AOI, (void*)&aoiRectangle, sizeof(aoiRectangle));

		if(status != IS_SUCCESS){
			printf("Setting full frame subsample fails\n");
		}
		double framerate;
		status = is_SetFrameRate(cameraHandle, FRAMERATE, &framerate);

		if(status != IS_SUCCESS){
			printf("setting framerate fails, error %d\n", status);
		}
	}

	gettimeofday(&systemTime1, NULL);
	status = is_FreezeVideo(cameraHandle, IS_WAIT);//takes a picture
	//printf("%ld\t%ld\t", systemTime1.tv_sec, systemTime1.tv_usec);
	//gettimeofday(&systemTime2, NULL);
	//printf("elapsed time: %ld\n", 1000000*(systemTime2.tv_sec - systemTime1.tv_sec) + systemTime2.tv_usec - systemTime1.tv_usec);
	if(status == IS_NO_SUCCESS){
		printf("capture image fails. Checking for issues\n"); 
		UEYE_CAPTURE_STATUS_INFO info;
		status = is_CaptureStatus(cameraHandle, IS_CAPTURE_STATUS_INFO_CMD_GET, (void*)&info, sizeof(UEYE_CAPTURE_STATUS_INFO));
		
		for(int i= 0; i<256; i++){
			if(info.adwCapStatusCnt_Detail[i]){
				printf("Freezevideo fails. Error: %d", i);
			}
		}
		closeCamera();
		return NULL;
	}
	return activeMemoryLocation;	
}

void* Camera::subFrameCapture(int xSubSize, int ySubSize, int xSubCenter, int ySubCenter){	
	if(!isOpen){//makes sure that the camera is still open
		printf("Camera is not open for pictures. Returning.\n");
		return NULL;
	}

	//timeval systemTime1;
	//timeval systemTime2;

	int status;
	IS_RECT aoiRectangle;
	//Ensures that the proper subframe AOI is set
	status = is_AOI(cameraHandle, IS_AOI_IMAGE_GET_AOI, (void*) &aoiRectangle, sizeof(aoiRectangle));

	if(status == IS_NO_SUCCESS){
		printf("IS_AOI fails in camera::subframecapture\n");
	}
	if((aoiRectangle.s32Width != xSubSize) || (aoiRectangle.s32Height != ySubSize) || (aoiRectangle.s32X != xSubCenter) || (aoiRectangle.s32Y != ySubCenter)){
		aoiRectangle.s32X = xSubCenter;
		aoiRectangle.s32Y = ySubCenter;
		aoiRectangle.s32Width = xSubSize;
		aoiRectangle.s32Height = ySubSize;
		status = is_AOI(cameraHandle, IS_AOI_IMAGE_SET_AOI, &aoiRectangle, sizeof(IS_RECT));

		if(status != IS_SUCCESS){
			printf("Setting subframe fails, error %d\n", status);
		}
		double framerate;
		status = is_SetFrameRate(cameraHandle, 1000, &framerate);

		if(status != IS_SUCCESS){
			printf("setting framerate fails, error %d\n", status);
		}else{
			printf("framerate = %lf\n", framerate);
		}
	}

	//gettimeofday(&systemTime1, NULL);
	status = is_FreezeVideo(cameraHandle, IS_WAIT);//takes a picture
	//gettimeofday(&systemTime2, NULL);
	//printf("elapsed time: %ld\n", 1000000*(systemTime2.tv_sec - systemTime1.tv_sec) + systemTime2.tv_usec - systemTime1.tv_usec);
	if(status == IS_NO_SUCCESS){
		printf("capture image fails. Checking for issues\n"); 
		UEYE_CAPTURE_STATUS_INFO info;
		status = is_CaptureStatus(cameraHandle, IS_CAPTURE_STATUS_INFO_CMD_GET, (void*)&info, sizeof(UEYE_CAPTURE_STATUS_INFO));
		
		for(int i= 0; i<256; i++){
			if(info.adwCapStatusCnt_Detail[i]){
				printf("Freezevideo fails. Error: %d", i);
			}
		}
		closeCamera();
		return NULL;
	}
	return activeMemoryLocation;	
}

int Camera::getXSize(){
	return sensorInfo.nMaxWidth;
}

int Camera::getYSize(){
	return sensorInfo.nMaxHeight;
}

int Camera::getPixelSize(){
	return colourDepth;
}

double Camera::getExposureTime(){
	return exposureTime;
}

void Camera::saveActiveData(wchar_t* name){
	if(!isOpen){//makes sure camera is open
		printf("Camera is not open to save data. Returning\n");
		return;
	}
	
	int status;

	IMAGE_FILE_PARAMS imageFileData;
	imageFileData.pwchFileName = name;
	imageFileData.nFileType = IS_IMG_BMP;
	imageFileData.ppcImageMem = NULL;
	imageFileData.pnImageID = NULL;

	status = is_ImageFile(cameraHandle, IS_IMAGE_FILE_CMD_SAVE, (void*)&imageFileData, sizeof(IMAGE_FILE_PARAMS));//saves the image
	if(status == IS_NO_SUCCESS){
		printf("save image fails\n");
	}
}


bool Camera::initCamera(){

	cameraHandle = 1;

	int status = is_InitCamera(&cameraHandle, NULL);//initiates the camera
	if(status != IS_SUCCESS){
		printf("is_InitCamera() failed. Checking to see whether you need to upgrade firmware...\n");
		if (status == IS_STARTER_FW_UPLOAD_NEEDED)
		{
		printf("Upgrading starter firmware. This may take a minute! Do NOT disconnect the camera from the PC or from power during this step.\n");
			status = is_InitCamera((HIDS*)((unsigned long) &cameraHandle | IS_ALLOW_STARTER_FW_UPLOAD), NULL); //helpful overloaded function upgrades our fw for us
			if (status != IS_SUCCESS)
			{
				printf("OpenCamera() still failed. Ensure firmware is up-to-date.\n"); 
				return false;
			}
			else
			{ //it must have returned IS_SUCCESS. Thus the new starter FW is uploaded during initialisation.
				printf("Starter firmware upgraded successfully.\n");
			}
		}
		else 
		{ // if the initial nRet did not return IS_SUCCESS or IS_STARTER_FW_UPLOAD_NEEDED then it returned a general error message.
			printf("OpenCamera() returned IS_NO_SUCCESS. Check code please?\n");
			return false;
		}
	}

	// get sensor info:
	status = is_GetSensorInfo(cameraHandle, &sensorInfo);
	if (status != IS_SUCCESS) 
	{ 
		printf("Failed to get sensor info!\n"); 
		return false;
	}

	int colourDepth;
	int colourMode;
	//gets the colour depth, and sets the camera to 8 bit greyscale
	is_GetColorDepth(cameraHandle, &colourDepth, &colourMode);
	is_SetColorMode(cameraHandle, IS_CM_MONO8);

	this->colourDepth = 8;

	char* memoryStartingPointer;
	int memoryId;
	//allocates memory for a single picture
	status = is_AllocImageMem(cameraHandle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth, &memoryStartingPointer, &memoryId);

	if(status != IS_SUCCESS){
		printf("Allocating image memory failed\n");
		return false;
	}
	//sets the memory to be the active memory
	status = is_SetImageMem(cameraHandle, memoryStartingPointer, memoryId);

	if(status != IS_SUCCESS){
		printf("Setting image memory failed\n");
		return false;
	}

	memoryPointer = memoryStartingPointer;

	memoryID = memoryId; 

	void* memoryPointer;

	status = is_GetImageMem(cameraHandle, &memoryPointer);
	if(status != IS_SUCCESS){
		printf("get image memory fails\n");
		return false;
	}

	activeMemoryLocation = memoryPointer;
	/*
	NOTE: the default mode appears to be the desired mode, and attempting to set it fails, so will just ignore this for now. Hopefully it is not an issue.
	//sets the display mode to be the correct one
	status = is_SetDisplayMode(cameraHandle, IS_SET_DM_DIB);
	
	if(status != IS_SUCCESS){
		printf("set display mode fails, error %d\n", status);
		return false;
	}*/

	//sets the trigger to be a software trigger
	status = is_SetExternalTrigger(cameraHandle, IS_SET_TRIGGER_SOFTWARE);
	
	if(status != IS_SUCCESS){
		printf("Set trigger fails\n");
		return false;
	}

	unsigned int range[3]; 
	//sets the pixel clock to the maximum rate

	status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_GET_RANGE, (void*) range, sizeof(range));
	int pixelSpeed = PIXEL_CLOCK_RATE;/*range[1]*/
	status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &pixelSpeed, sizeof(int));

	if(status != IS_SUCCESS){
		printf("Setting the pixel clock fails\n");
	}

	/*IS_RECT aoiRectangle;

	aoiRectangle.s32X = 0;
	aoiRectangle.s32Y = 0;
	aoiRectangle.s32Width = 48;//this is the 48 one
	aoiRectangle.s32Height = 50;

	status = is_AOI(cameraHandle, IS_AOI_IMAGE_SET_AOI, &aoiRectangle, sizeof(IS_RECT));
	
	if(status != IS_SUCCESS){
		printf("setting AOI fails. Error %d\n", status);
		return false;
	}*/

	double framerate;
	status = is_SetFrameRate(cameraHandle, FRAMERATE, &framerate);

	if(status != IS_SUCCESS){
		printf("getting framerate fails, error %d\n", status);
		return false;
	} else {
		printf("framerate = %lf\n", framerate);
	}


	double exposure = EXPOSURE_TIME;
	//sets one millisecond exposure time
	status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
	if(status != IS_SUCCESS){
		printf("Set exposure time fails\n");
	}
	printf("Integreation time is %lf\n", exposure);

	exposureTime = exposure;

	/*int delay =*/ is_SetTriggerDelay(IS_GET_TRIGGER_DELAY, 0);
	//printf("trigger delay: %d\n", delay);

	status = is_SetHardwareGain(cameraHandle, GAIN_PERCENTAGE, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

	if(status != IS_SUCCESS){
		printf("setting gain failed\n");
	}

	char saveFile[100];
	strcpy(saveFile, "/home/snaffle/source/badPixels.txt");
	//corrects for bad pixels
	PixelFixer pixelFixer = PixelFixer(sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth);
	unsigned short* badPixelTable = NULL;
	//loads the pixel table from a file
	badPixelTable = pixelFixer.loadSavedTable(saveFile, badPixelTable);

	if(badPixelTable == NULL){
		printf("Bad pixel correction table does not exist, error %d. Generating it from files. This could take some time.\n", status);

		PixelFixer pixelFixer = PixelFixer(sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth);

		char* pictureLocation = (char*)"./calibImages\0";
		badPixelTable = pixelFixer.generateBadPixelTable(pictureLocation);
		if(badPixelTable == NULL){
			printf("bad pixel callibration files not found. Please ensure that the lens cap in ON and then press any key to regenerate them.\n");
			getchar();
			system("rm calibImages/*");

			wchar_t filename[100];
						
			status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &range[0], sizeof(range[0]));

			if(status != IS_SUCCESS){
				printf("Setting the pixel clock fails\n");
			}
			
			exposure = 3000;
			status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
			if(status != IS_SUCCESS){
				printf("Set exposure time fails\n");
			}
			printf("Integreation time is %lf\n", exposure);

			isOpen = true;
			for(int i = 0; i<100; i++){
				swprintf(filename, 100, L"calibImages/LensOn%d-%d.bmp", (int) exposure, i);
				this->capture();
				this->saveActiveData(filename);
			}
			status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &range[1], sizeof(range[1]));

			if(status != IS_SUCCESS){
				printf("Setting the pixel clock fails\n");
			}
			exposure = EXPOSURE_TIME;
			status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
			if(status != IS_SUCCESS){
				printf("Set exposure time fails\n");
			}
			printf("Integreation time is %lf\n", exposure);
			
			badPixelTable = pixelFixer.generateBadPixelTable(pictureLocation);
			isOpen = false;
		}
	}
	unsigned short size = *badPixelTable;
	status = is_HotPixel(cameraHandle, IS_HOTPIXEL_SET_SOFTWARE_USER_LIST, (void*)badPixelTable, sizeof(short)*(2*((int) size) + 1));

	if(status != IS_SUCCESS){
		printf("Bad pixel correction table generation has failed. Sorry. Error: %d\n", status);
	}else{
		printf("Bad pixel correction table created. Huzzah!\n");
	}
	
		
	status = is_HotPixel(cameraHandle, IS_HOTPIXEL_ENABLE_SOFTWARE_USER_CORRECTION, NULL, 0);

	if(status != IS_SUCCESS){
		printf("Bad pixel correction has failed. Sorry.\n");
	}
	

	status = pixelFixer.savePixelTable(saveFile, badPixelTable);
	if(status != IS_SUCCESS){
		printf("Saving bad pixel table has failed. Sorry.\n");
	}
	free(badPixelTable);
	//is_SaveParameters(cameraHandle, NULL);

	return true;
}

void Camera::characterizeCameraNoise(){

	PixelFixer fixy = PixelFixer(sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth);

	UINT range[3];

	int status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_GET_RANGE, (void*) range, sizeof(range));

	if(status != IS_SUCCESS){
		printf("getting pixel clock values has failed\n");
		return;
	}
	status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &range[1], sizeof(range[1]));
	if(status != IS_SUCCESS){
		printf("setting pixle clock to max fails\n");
		return;
	}

	double exposure = 0.03;
	void* image;
	for(int i = 0; i<5; i++){
		status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
		if(status != IS_SUCCESS){
			printf("setting exposure to %lf fails\n", exposure);
			return;
		}

		printf("%lf\t", exposure);
		image = this->capture();
		fixy.getImageBackground(image);
		exposure += 0.2;
	}
	status = is_PixelClock(cameraHandle, IS_PIXELCLOCK_CMD_SET, (void*) &range[0], sizeof(range[0]));
	if(status != IS_SUCCESS){
		printf("setting pixle clock to min fails\n");
		return;
	}
	for(int i = 1; i<2000; i++){
		exposure = (double)i;
		status = is_Exposure(cameraHandle, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(double));
		if(status != IS_SUCCESS){
			printf("setting exposure to %lf fails\n", exposure);
			return;
		}

		printf("%lf\t", exposure);
		image = this->capture();
		fixy.getImageBackground(image);
	}
	return;
}
		
