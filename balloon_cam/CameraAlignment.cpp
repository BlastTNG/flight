// CameraAlignment.cpp : main project file.

#include "stdafx.h"
#include "cameraCommunicator.h"
using namespace System;

int main(array<System::String ^> ^args)
{
	Camera camera = Camera();//opens a camera
	if(!camera.open()){//checks to ensure it's open
		return -1;
	}
	Locator locator = Locator(camera.getXSize(), camera.getYSize(), camera.getPixelSize(), 10, &camera);//starts a locator
	CameraCommunicator communicator = CameraCommunicator(&locator);//and a communicator
	std::string hostname = "hog\n";
	communicator.addConnection(hostname);//adds a connection to the server computer
	printf("connection added\n");
	Sleep(30000);//waits a while
	
	communicator.removeConnection(hostname);//removes the connection
	void* pointer;
	blob data;
	for(int i = 0; i<10; i++){
		Sleep(1000);
		pointer = camera.capture();
		data = locator.locate(pointer);
		printf("blob found at: %d, %d, size: %d\n", data.centroid.x, data.centroid.y, data.size);
	}
	camera.closeCamera();//closes the camera
	system("Pause");//waits so that you can look at the output
}
