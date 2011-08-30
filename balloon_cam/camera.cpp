//camera.cpp
//a wrapper class for the camera fuctionality, used to return the pictures

#include "StdAfx.h"
#include "camera.h"
#include "pixelfixer.h"

Camera::Camera(){
	if(!initCamera()){
		printf("Camera failed to initialize. Giving up.\n");
		isOpen = false;
	}else{
		isOpen = true;
	}
}

bool Camera::closeCamera(){
	is_StopLiveVideo(cameraHandle, IS_WAIT);
	is_ClearSequence(cameraHandle);
	for(int i = 0; i<100; i++){
		is_FreeImageMem(cameraHandle, ringBuffer[i+1], i+1);//frees the memory that was used
	}

	is_ExitCamera(cameraHandle);//closes the camera
	isOpen = false;//tell the wrapper class that it is closed
	return true;
}

void* Camera::capture(){
	if(!isOpen){//makes sure that the camera is still open
		printf("Camera is not open for pictures. Returning.\n");
		return NULL;
	}
	int status;
	if(memoryID != 0){
		//status = is_UnlockSeqBuf(cameraHandle, IS_IGNORE_PARAMETER, (char*)activeMemoryLocation);

		if(status != IS_SUCCESS){
			printf("unlocking buffer fails\n");
		}
	}

	int memoryId;
	char* currMemory;
	char* lastMemory;

	status = is_GetActSeqBuf(cameraHandle, &memoryId, &currMemory, &lastMemory); 

	if(status != IS_SUCCESS){
		printf("getting buffer fails\n");
		return NULL;
	}

	activeMemoryLocation = lastMemory;
	memoryID = memoryId;
	int i =0;
	for(; i<=100; i++){
		if(ringBuffer[i] == lastMemory){
			break;
		}
	}


	//status = is_LockSeqBuf(cameraHandle, IS_IGNORE_PARAMETER, ringBuffer[i]);

	if(status != IS_SUCCESS){
		printf("Locking buffer fails\n");
	}

	return (void*)lastMemory;	
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

void Camera::saveActiveData(){
	if(!isOpen){//makes sure camera is open
		printf("Camera is not open to save data. Returning\n");
		return;
	}
	
	int status;
	//status = is_SetImageMem(cameraHandle, (char*)activeMemoryLocation, memoryID);

	status = is_SaveImage(cameraHandle, NULL);//saves the image
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
	// get board info:
	status = is_GetBoardInfo(cameraHandle, &boardInfo);
	if (status != IS_SUCCESS)
	{
		printf("Failed to get board info!\n"); 
		return false;
	}

	int colourDepth;
	int colourMode;
	//gets the colour depth, and sets the camera to 8 bit greyscale
	is_GetColorDepth(cameraHandle, &colourDepth, &colourMode);
	is_SetColorMode(cameraHandle, IS_CM_MONO8);

	this->colourDepth = 8;

	//sets the display mode to be the correct one
	status = is_SetDisplayMode(cameraHandle, IS_SET_DM_DIB);

	if(status != IS_SUCCESS){
		printf("set display mode fails\n");
		return false;
	}

	char* memoryStartingPointer;
	int memoryId;

	for(int i = 0; i< 100; i++){
		memoryStartingPointer = NULL;
		memoryId = 0;
		//allocates memory for a single picture
		status = is_AllocImageMem(cameraHandle, sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth, &memoryStartingPointer, &memoryId);
		if(status != IS_SUCCESS){
			printf("Allocating image memory failed\n");
			return false;
		}

		status = is_AddToSequence(cameraHandle, memoryStartingPointer, memoryId);
		if(status != IS_SUCCESS){
			printf("Adding memory to buffer fails\n");
			return false;
		}
		ringBuffer[memoryId] = memoryStartingPointer;
	
	}

	//sets the memory to be the active memory
	//status = is_SetImageMem(cameraHandle, memoryStartingPointer, memoryId);

	if(status != IS_SUCCESS){
		printf("Setting image memory failed\n");
		return false;
	}

	memoryID = 0;

	//sets the trigger to be off
	status = is_SetExternalTrigger(cameraHandle, IS_SET_TRIGGER_OFF);
	
	if(status != IS_SUCCESS){
		printf("Set trigger fails\n");
		return false;
	}

	int pMax;
	int pMin;

	status = is_GetPixelClockRange(cameraHandle, &pMin, &pMax);
	status = is_SetPixelClock(cameraHandle, pMax);

	if(status != IS_SUCCESS){
		printf("Setting the pixel clock fails\n");
	}

	double fps;

	status = is_SetFrameRate(cameraHandle, 10.0, &fps);

	if(status != IS_SUCCESS){
		printf("setting frame rate fails\n");
		return false;
	}

	status = is_HotPixel(cameraHandle, IS_HOTPIXEL_GET_CAMERA_USER_LIST_EXISTS, NULL, NULL);

	if(status != IS_SUCCESS){
		printf("Bad pixel correction table does not exist. Generating it from files. This could take some time.\n");

		PixelFixer pixelFixer = PixelFixer(sensorInfo.nMaxWidth, sensorInfo.nMaxHeight, this->colourDepth);

		WORD* badPixelTable = pixelFixer.generateBadPixelTable("../../Pictures");

		int quantity = (int) (*badPixelTable);

		status = is_HotPixel(cameraHandle, IS_HOTPIXEL_SET_CAMERA_USER_LIST, (void*) badPixelTable, (2*quantity +1) *sizeof(WORD));

		if(status != IS_SUCCESS){
			printf("Bad pixel correction table generation has failed. Sorry.\n");
		}
	}
		
	status = is_HotPixel(cameraHandle, IS_HOTPIXEL_ENABLE_CAMERA_CORRECTION, NULL, NULL);

	if(status != IS_SUCCESS){
		printf("Bad pixel correction has failed. Sorry.\n");
	}

	status = is_CaptureVideo(cameraHandle, IS_WAIT);

	if(status != IS_SUCCESS){
		printf("Video Capture fails. Sorry\n");
		return false;
	}

	return true;
}