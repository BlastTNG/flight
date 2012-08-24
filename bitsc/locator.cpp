//locator.cpp

#include "stdafx.h"
#include "locator.h"


Locator::Locator(int xSize, int ySize, int pixelSize, int nSigma, int mode, Camera* camera){
	totalPixels = xSize * ySize;
	xPixels = xSize;
	yPixels = ySize;
	this->camera = camera;
	pixelDataSize = pixelSize;
	if((pixelDataSize%8) != 0){
		printf("The pixel size is not a multiple of 8, which will cause undefined behaviour in locator. Don't trust the output.\n");
	}

	if(pixelDataSize > 32){
		printf("The pixel data size is too large for locator to handle. Please reduce it.\n");
	}
	srand((int)time(NULL));
	lastBlob.size = 0;
	blobVector = std::vector<blob>();
	sigmaNumber = nSigma;

	locationMode = mode;
	if(locationMode < 1){
		printf("invalid location mode in locator.\n");
	}
}

//calls the correct type of locator depending on the flag passed to the constructor
blob Locator::locate(void* picture, int xSubSize, int ySubSize, int xCenter, int yCenter){
	switch(locationMode){

		case SPIDER_MODE:
			if(picture == NULL){
				picture = camera->capture();
			}	
			return spiderLocate(picture);
			
			break;
		
		case LIGHT_TEST:
			/*if(lastBlob.size == 0){
				if(picture == NULL){
					return spiderLocate(camera->capture());
				} else {
					return spiderLocate(picture);
				}
			}else{*/
				if(picture != NULL){
					printf("non-null picture being ignored in locate\n");
				}
				return lightFinder(camera->subFrameCapture(xSubSize, ySubSize, xCenter, yCenter), xSubSize, ySubSize);
			//}
			break;
		
		case STAR_TRACKER:
			if(picture != NULL){
				printf("non-null picture being ignored in locate\n");
			}
			return starTracker(camera->subFrameCapture(xSubSize, ySubSize, xCenter, yCenter), xSubSize, ySubSize);
			break;
	
		default:
			printf("Invalid mode selection: %d in locator\n", locationMode);
			blob fakeBlob;
			fakeBlob.size = 0;
			return fakeBlob;
	}
}

//A short function that determines if a light is on or off in the specified region. Just uses a threshold
blob Locator::lightFinder(void* address, int xSubSize, int ySubSize){
	int sum = 0;
	for(int i = 0; i< xSubSize*ySubSize; i++){
		sum += (char)(*(char*) ((unsigned long long)address + xSubSize*ySubSize*pixelDataSize/8));
	}
	sum /= (xSubSize*ySubSize);

	blob fakeBlob;
	if(sum > LIGHT_THRESHOLD){
		fakeBlob.centroid.x = -1;
		fakeBlob.centroid.y = -1;
	} else {
		fakeBlob.centroid.x = -2;
		fakeBlob.centroid.y = -2;
	}
	return fakeBlob;
}

//This function is used to track stars accross the sky. It will select the first star it finds and then proceed to track it in all subsequent iterations.
blob Locator::starTracker(void* address, int xSubSize, int ySubSize){

	double average = 0;
	double stdDev = 0;
	int pictureSize = xSubSize*ySubSize;
	int pixelValue;
	void* pixelAddress;
	int temp;
	blob tempBlob;

	//computes average and standard deviation
	for(int i = 0; i<pictureSize; i++){
		pixelAddress = (void*)((unsigned long long)address + ((i)*pixelDataSize/8));
		pixelValue = 0;
		
		for(int j = 0; j<pixelDataSize/8; j++){
			pixelValue += (*(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
		}

		average += pixelValue;
		stdDev += (int)pow((float)pixelValue, 2);
	}

	average = average/(totalPixels/pictureSize);
	stdDev = sqrt(stdDev/(totalPixels/pictureSize) - pow(average, 2));

		int step = 1;
		unsigned long long startLocation = (unsigned long long) address + (xSubSize/2 + xSubSize*ySubSize/2 )*(pixelDataSize/8);
		bool stillLooking = true;
		//looks in a snake like pattern for any feature pixels.
		while(stillLooking){
			for(int x = 1; x<=step; x++){
				
				if(step%2 == 1){
					temp = 1;
				}else{
					temp = -1;
				}
				startLocation += temp*(pixelDataSize/8);
				if((startLocation > (unsigned long long) address + pictureSize*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){
					printf("Search has gone out of bounds in the x direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (void*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){
					pixelValue += (*(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}
				//if the pixel stands out from the background
				if(abs(pixelValue - average) > (sigmaNumber*stdDev)){
					tempBlob = detectBlob(pixelAddress, address);
					//if there is no previous blob, pick this one and return
					if(lastBlob.size == 0){
						blobVector.push_back(tempBlob);
						lastBlob = tempBlob;
						return tempBlob;
					}

					if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/5)&&(tempBlob.size != 0)&&(abs(tempBlob.intensity - lastBlob.intensity) < stdDev)){//if we are pretty sure this is the right blob
						blobVector.push_back(tempBlob);
						lastBlob = tempBlob;
						return tempBlob;
					}
				}
			}
			if(!stillLooking){
				break;
			}
			//the y direction of the snake
			for(int y = 1; y<=step; y++){
				if(step % 2 == 1){
					temp = 1;
				}else{
					temp = -1;
				}

				startLocation += xSubSize*temp*(pixelDataSize/8);
				if((startLocation > (unsigned long long) address + totalPixels*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){
					printf("Search has gone out of bounds in the y direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (void*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){
					pixelValue += ( *(unsigned char*)((unsigned long long)pixelAddress + j)) * (int)pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}

				//if it's not background
				if(abs(pixelValue - average) > (sigmaNumber*stdDev)){
					tempBlob = detectBlob(pixelAddress, address);
					//pick this one if it's the first time
					if(lastBlob.size == 0){
						blobVector.push_back(tempBlob);
						lastBlob = tempBlob;
						return tempBlob;
					}				

					if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/5)&&(tempBlob.size != 0) && (abs(tempBlob.intensity - lastBlob.intensity) < stdDev)){//if we are pretty sure this is the right blob
						blobVector.push_back(tempBlob);
						lastBlob = tempBlob;
						return tempBlob;
					}
				}
			}
			step++;
			//if we are going to go out of bounds
			if(step > (xSubSize-10)/2 || step > (ySubSize-10)/2 ){
				break;
			}
		}

	lastBlob.size = 0;
	printf("can't find any blobs\n");
	return starTracker(address, xSubSize, ySubSize);
}


//this is the code written to track the spider baloon ball
blob Locator::spiderLocate(void* address){
	double average = 0;
	double stdDev = 0;
	void* pixelAddress;
	int temp;
	int pixelValue;
	blob tempBlob;
	tempBlob.size = 0;
	std::vector<blob> foundBlobs;

	for(int i = 0; i<(totalPixels/1000); i++){
		pixelAddress = (void*)((unsigned long long)address + ((rand()%totalPixels)*pixelDataSize/8));//picks a random pixel
		pixelValue = 0;
		
		for(int j = 0; j<pixelDataSize/8; j++){
			pixelValue += (*(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
		}

		average += pixelValue;
		stdDev += (int)pow((float)pixelValue, 2);
	}

	average = average/(totalPixels/1000);
	stdDev = sqrt(stdDev/(totalPixels/1000) - pow(average, 2));

	averageValue = average;
	standardDeviation = stdDev;
	if(standardDeviation < 2){
		standardDeviation = 2;
		stdDev = 2;
	}
	
	if(lastBlob.size == 0){//if we have never found the blob before, look at the whole picture
		
		blob maxBlob;
		maxBlob.size = 0;
		for(int i = 0; i<totalPixels; i++){
			pixelAddress = (void*)((unsigned long long) address + i*(pixelDataSize/8));
			pixelValue = 0;
			for(int j = 0; j<pixelDataSize/8; j++){
				pixelValue += ( *(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
			}

			if(abs(pixelValue - average) > sigmaNumber*stdDev){
				if(blobWasFound(pixelAddress, address, foundBlobs)){
					continue;
				}
				tempBlob = detectBlob(pixelAddress, address);
				foundBlobs.push_back(tempBlob);
				if(tempBlob.size > maxBlob.size){
					maxBlob = tempBlob;
				}
			}
		}

		lastBlob = maxBlob;

		blobVector.push_back(lastBlob);
		return lastBlob;
	
	}else{//otherwise start looking at the previous location in a snake-like pattern

		int step = 1;
		unsigned long long startLocation = (unsigned long long) address + (xPixels*lastBlob.centroid.y + lastBlob.centroid.x)*(pixelDataSize/8);
		int index = (xPixels*lastBlob.centroid.y + lastBlob.centroid.x)*(pixelDataSize/8);
		printf("starting at: %d, %d\n", index %xPixels , index /xPixels);
		bool stillLooking = true;
	
		while(stillLooking){
			for(int x = 1; x<=step; x++){
				
				if(step%2 == 1){
					temp = 1;
				}else{
					temp = -1;
				}
				startLocation += temp*(pixelDataSize/8);
				index = (int) (startLocation - (unsigned long long)address);
				printf("looking at %d, %d\n", index%xPixels, index /xPixels);
				if((startLocation > (unsigned long long) address + totalPixels*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){
					printf("Search has gone out of bounds in the x direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (void*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){
					pixelValue += (*(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}

				if(abs(pixelValue - average) > (sigmaNumber*stdDev)){
					printf("pixel is a feature\n");
					tempBlob = detectBlob(pixelAddress, address);
				}

				if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/5)&&(tempBlob.size != 0)){//if we are pretty sure this is the right blob
					blobVector.push_back(tempBlob);
					return tempBlob;
				}
			}
			if(!stillLooking){
				break;
			}

			for(int y = 1; y<=step; y++){
				if(step % 2 == 1){
					temp = 1;
				}else{
					temp = -1;
				}

				startLocation += xPixels*temp*(pixelDataSize/8);
				index = (int)(startLocation - (unsigned long long)address);
				printf("looking at %d, %d\n", index%xPixels, index/xPixels);
				if((startLocation > (unsigned long long) address + totalPixels*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){
					printf("Search has gone out of bounds in the y direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (void*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){
					pixelValue += ( *(unsigned char*)((unsigned long long)pixelAddress + j)) * (int)pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}

				if(abs(pixelValue - average) > (sigmaNumber*stdDev)){
					printf("pixel is a feature\n");
					tempBlob = detectBlob(pixelAddress, address);
				}

				if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/5)&&(tempBlob.size != 0)){//if we are pretty sure this is the right blob
					blobVector.push_back(tempBlob);
					return tempBlob;
				}
			}
			step++;
			if(step > 100){
				break;
			}
		}

		lastBlob.size = 0;
		lastBlob.centroid.x = -1;
		lastBlob.centroid.y = -1;
		printf("No blob found near previous blob. Conducting thorough search\n");
		return locate(address, 0, 0, 0, 0);
	}
}


void Locator::writeData(){//writes out a formatted summary of the blobs that have been found
	FILE* file = fopen("BlobSummary.txt", "w");
	fprintf(file, "Image Number\tX Value\tY Value\tSize\tSource X\tSource Y\n");
	blob current;
	for(int i =0; i<(int)blobVector.size(); i++){
		current = blobVector.at(i);
		fprintf(file, "%d\t\t%d\t%d\t%d\t%d\t%d\n", i, current.centroid.x, current.centroid.y, current.size, current.source.x, current.source.y);
	}
	fclose(file);
}

//does a quick check to see if the blob was already found
bool Locator::blobWasFound(void* pixel, void* address, std::vector<blob> foundBlobs){

	int pixelNum = (int)((unsigned long long int)pixel - (unsigned long long int)address);
	double x = pixelNum % xPixels;
	double y = (double)(pixelNum / xPixels);
	double ycenter;
	double xcenter;
	double ymin;
	double xmin;
	double ymax;
	double xmax;

	for(unsigned int i = 0; i<foundBlobs.size(); i++){
		ycenter = foundBlobs[i].centroid.y;
		xcenter = foundBlobs[i].centroid.x;
		xmax = foundBlobs[i].max.x;
		xmin = foundBlobs[i].min.x;
		ymax = foundBlobs[i].max.y;
		ymin = foundBlobs[i].min.y;
		
		//if(	((y - ycenter) * (y - ycenter)/((xmax  - xmin)*(xmax - xmin)/4)) + (x - xcenter)*(x - xcenter)/((ymax - ymin)*(ymax - ymin)/4) < 4){
		if( (y <= ymax + 1)&&(y >= ymin - 1)&&(x <= xmax + 1)&&(x >= xmin - 1)){
			return true;
		}
	}
	return false;
}

//gets the extent of the blob found containing the given pixel
blob Locator::detectBlob (void* pixel, void* picture){
	blob returnBlob;
	bool edgeSeeker = true;
	unsigned char* rightPointer;
	unsigned char* leftPointer;
	unsigned char* storedRight;
	unsigned char* storedLeft;
	int offset = 0;
	bool leftEdge = false;
	bool rightEdge = false;

	while(edgeSeeker){
		rightPointer = (unsigned char*) pixel + offset;
		leftPointer = (unsigned char*) pixel - offset;

		if((abs(*rightPointer - averageValue) < sigmaNumber*standardDeviation)&&(!rightEdge)){
			storedRight = rightPointer - 1;
			rightEdge = true;
		}
		if((abs(*leftPointer - averageValue) < sigmaNumber*standardDeviation)&&(!leftEdge)){
			storedLeft = leftPointer + 1;
			leftEdge = true;
		}
		offset ++;

		if((leftEdge) && (rightEdge)){
			edgeSeeker = false;
		}
	}

	unsigned char* centerLine = (unsigned char*) (((unsigned long long int)storedRight + (unsigned long long int)storedLeft)/2);

	edgeSeeker = true;
	unsigned char* topPointer;
	unsigned char* bottomPointer;
	unsigned char* storedTop;
	unsigned char* storedBottom;
	offset = 0;
	bool foundTop = false;
	bool foundBottom = false;
	int intensity = 0;
	int pixelCount = 0;

	while (edgeSeeker){

		topPointer = centerLine - offset*xPixels;
		bottomPointer = centerLine + offset*xPixels;

		if((abs(*topPointer - averageValue) < sigmaNumber*standardDeviation)&&(!foundTop)){
			storedTop = topPointer + xPixels;
			foundTop = true;
		}else{
			pixelCount ++;
			intensity += *topPointer;
		}

		if((abs(*bottomPointer - averageValue) < sigmaNumber*standardDeviation)&&(!foundBottom)){
			storedBottom = bottomPointer - xPixels;
			foundBottom = true;
		}else{
			pixelCount ++;
			intensity += *bottomPointer;
		}

		if(foundTop && foundBottom){
			edgeSeeker = false;
		}
		offset ++;
	}

	returnBlob.max.y = (int)(( storedBottom - (unsigned char*) picture )/xPixels);
	returnBlob.min.y = (int)((storedTop - (unsigned char*) picture)/xPixels);

	centerLine = (unsigned char*)((unsigned long long int)storedTop + ((((int)(storedBottom - storedTop)/xPixels)/2)*xPixels));
	edgeSeeker = true;
	offset = 0;
	leftEdge = false;
	rightEdge = false;

	while(edgeSeeker){
	
		leftPointer = centerLine - offset;
		rightPointer = centerLine + offset;
	
		if((abs(*rightPointer - averageValue) < sigmaNumber*standardDeviation)&&(!rightEdge)){
			storedRight = rightPointer - 1;
			rightEdge = true;
		}else{
			pixelCount ++;
			intensity += *rightPointer;
		}
		if((abs(*leftPointer - averageValue) < sigmaNumber*standardDeviation)&&(!leftEdge)){
			storedLeft = leftPointer + 1;
			leftEdge = true;
		}else{
			pixelCount ++;
			intensity += *leftPointer;
		}
		offset ++;
		if((leftEdge) && (rightEdge)){
			edgeSeeker = false;
		}
	}

	returnBlob.max.x = (int) ((storedRight - (unsigned char*) picture) % xPixels);
	returnBlob.min.x = (int) ((storedLeft - (unsigned char*) picture) % xPixels);

	returnBlob.centroid.x = (returnBlob.max.x + returnBlob.min.x)/2;
	returnBlob.centroid.y = (returnBlob.max.y + returnBlob.min.y)/2;
	returnBlob.size = (int) (3.14/4.0 * ((double)((returnBlob.max.x - returnBlob.min.x + 1)*(returnBlob.max.y - returnBlob.min.y + 1))));
	returnBlob.intensity = intensity / pixelCount; 
	//printf("Blob found at %d, %d (center x = %d, y = %d), height: %d, width %d, size %d\n", (int)((unsigned long long int)pixel - (unsigned long long int)picture)%xPixels, (int)((unsigned long long int)pixel - (unsigned long long int)picture)/xPixels, returnBlob.centroid.x, returnBlob.centroid.y, 1+ returnBlob.max.y - returnBlob.min.y, 1+ returnBlob.max.x - returnBlob.min.x, returnBlob.size);
	return returnBlob;
}
