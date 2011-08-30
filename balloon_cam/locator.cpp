//locator.cpp

#include "stdafx.h"
#include "locator.h"


Locator::Locator(int xSize, int ySize, int pixelSize, int nSigma, Camera* camera){//constructor
	totalPixels = xSize * ySize;
	xPixels = xSize;
	yPixels = ySize;
	this->camera = camera;
	pixelDataSize = pixelSize;
	if((pixelDataSize%8) != 0){
		printf("The pixel size is not a multiple of 8, which will cause undefined behaviour in locator. Don't trust the output.\n");
	}

	if(pixelDataSize > 32){//otherwise ints will be too small to contain it
		printf("The pixel data size is too large for locator to handle. Please reduce it.\n");
	}
	srand((int)time(NULL));//starts the random generator
	lastBlob.size = 0;
	blobVector = std::vector<blob>();
	sigmaNumber = nSigma;
}


blob Locator::locate(){
	return locate(camera->capture());
}

blob Locator::locate(void* address){//find the largest blob in a picture given an address
	double average = 0;
	double stdDev = 0;
	unsigned char* pixelAddress;
	int temp;
	unsigned int pixelValue;
	blob tempBlob;
	tempBlob.size = 0;
	std::vector<blob> foundBlobs;//a list of all the blobs that have been found

	for(int i = 0; i<(totalPixels/1000); i++){//calculates the average and stdDev of a sample of the image, should be representitive
		pixelAddress = (unsigned char*)((unsigned long long)address + ((rand()%totalPixels)*pixelDataSize/8));//picks a random pixel
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
	if(standardDeviation < 2){//if the image is too uniform, any non white pixel can show up - this fixes this issue (kind of a hack)
		standardDeviation = 2;
		stdDev = 2;
	}
	
	if(lastBlob.size == 0){//if we have never found the blob before, look at the whole picture
		blob maxBlob;
		maxBlob.size = 0;
		pixelAddress = (unsigned char*)address;//the pixel we are looking at
		for(int i = 0; i<totalPixels; i++){
			pixelValue =(*pixelAddress);//the intensity of that pixel
			if(abs(pixelValue - average) >= sigmaNumber*stdDev){//if the pixel is significant
				if(!blobWasFound(pixelAddress, address, foundBlobs)){//if a blob has not already been found containing this pixel
					tempBlob = detectBlob(pixelAddress, address);//returns the blob object containing the pixel in question
					foundBlobs.push_back(tempBlob);//adds the blob to the list
					if(tempBlob.size > maxBlob.size){//calculates the biggest blob. For a real application, should probably find the second biggest
						maxBlob = tempBlob;//so as to correct of the blimp
					}
				}
			}
			pixelAddress += 1;
		}
		lastBlob = maxBlob;

		blobVector.push_back(lastBlob);//archives the blob in case we are interested
		if((lastBlob.centroid.x == 0)&&(lastBlob.centroid.y == 0)){//if for some reason the blob cannot be found
			printf("blob not found anywhere. Returning 0, 0\n");
			lastBlob.size = 0;
		}
		return lastBlob;//returns the largest blob
	
	}else{//otherwise start looking at the previous location in a snake-like pattern

		int step = 1;
		unsigned long long startLocation = (unsigned long long) address + (xPixels*lastBlob.centroid.y + lastBlob.centroid.x)*(pixelDataSize/8);//starts at the center of the last blob
		int index = (xPixels*lastBlob.centroid.y + lastBlob.centroid.x)*(pixelDataSize/8);//makes an int of the above value
		//printf("starting at: %d, %d\n", index %xPixels , index /xPixels);
		bool stillLooking = true;
	
		while(stillLooking){
			for(int x = 1; x<=step; x++){
				
				if(step%2 == 1){//calculates the correct direction
					temp = 1;
				}else{
					temp = -1;
				}
				startLocation += temp*(pixelDataSize/8);//caculates the current pixel
				index = (int) (startLocation - (unsigned long long)address);
				//printf("looking at %d, %d\n", index%xPixels, index /xPixels);
				if((startLocation > (unsigned long long) address + totalPixels*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){//ensures the value is still valid
					printf("Search has gone out of bounds in the x direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (unsigned char*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){//gets the value of the pixel 
					pixelValue += (*(unsigned char*)((unsigned long long)pixelAddress + j)) *(int) pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}

				if((abs(pixelValue - average) > (sigmaNumber*stdDev))&&(!blobWasFound(pixelAddress, address, foundBlobs))){//if the pixel is significant and has not been found
					tempBlob = detectBlob(pixelAddress, address);//detects the blob
					foundBlobs.push_back(tempBlob);//adds it to the list
				}

				if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/4)&&(tempBlob.size != 0)){//if we are pretty sure this is the right blob
					blobVector.push_back(tempBlob);//archives the blob
					return tempBlob;//returns it
				}
			}
			if(!stillLooking){
				break;
			}
			//this bit is the same as the previous bit except in the y direction instead of the x

			for(int y = 1; y<=step; y++){//figures out the direction
				if(step % 2 == 1){
					temp = 1;
				}else{
					temp = -1;
				}

				startLocation += xPixels*temp*(pixelDataSize/8);//calculates the correct pixel
				index = (int)(startLocation - (unsigned long long)address);
				//printf("looking at %d, %d\n", index%xPixels, index/xPixels);
				if((startLocation > (unsigned long long) address + totalPixels*(pixelDataSize/8))||(startLocation < (unsigned long long)address)){//checks that the address is valid
					printf("Search has gone out of bounds in the y direction. Terminating\n");
					stillLooking = false;
					break;
				}

				pixelAddress = (unsigned char*)(startLocation);
				pixelValue = 0;
				for(int j = 0; j<pixelDataSize/8; j++){//gets the pixel value
					pixelValue += ( *(unsigned char*)((unsigned long long)pixelAddress + j)) * (int)pow((float)2, j*pixelDataSize);//adds the value of the next byte to the total
				}

				if((abs(pixelValue - average) > (sigmaNumber*stdDev))&&(!blobWasFound(pixelAddress, address, foundBlobs))){//if the pixel is significant and hasn't been found
					tempBlob = detectBlob(pixelAddress, address);//detct the blob
					foundBlobs.push_back(tempBlob);//add to the list 
				}

				if(abs(tempBlob.size - lastBlob.size) < (lastBlob.size/4)&&(tempBlob.size != 0)){//if we are pretty sure this is the right blob
					blobVector.push_back(tempBlob);
					return tempBlob;
				}
			}
			step++;
			if(step > 250){//if we are outside of the region close to the previous bolb, it is a good guess that we are lost and should start from scratch
				break;
			}
		}

		lastBlob.size = 0;//if we did not find a blob for whatever reason, start over with no prior knowledge
		lastBlob.centroid.x = NULL;
		lastBlob.centroid.y = NULL;
		printf("No blob found near previous blob. Conducting thorough search\n");
		return locate(address);
	}
}


void Locator::writeData(){//writes out a formatted summary of the blobs that have been found
	FILE** file = NULL;
	fopen_s(file, "BlobSummary.txt", "w");
	fprintf(*file, "Image Number\tX Value\tY Value\tSize\tSource X\tSource Y\n");
	blob current;
	for(int i =0; i<(int)blobVector.size(); i++){
		current = blobVector.at(i);
		fprintf(*file, "%d\t\t%d\t%d\t%d\t%d\t%d\n", i, current.centroid.x, current.centroid.y, current.size, current.source.x, current.source.y);
	}
	fclose(*file);
}


bool Locator::blobWasFound(void* pixel, void* address, std::vector<blob> foundBlobs){//checks to determine if a blob was already found using the pixel in question
	int pixelNum = (int)((unsigned long long int)pixel - (unsigned long long int)address);//the int location of the pixel
	double x = pixelNum % xPixels;
	double y = (double)(pixelNum / xPixels);
	double ycenter;
	double xcenter;
	double ymin;
	double xmin;
	double ymax;
	double xmax;

	for(unsigned int i = 0; i<foundBlobs.size(); i++){//goes through the list of blobs
		ycenter = foundBlobs[i].centroid.y;
		xcenter = foundBlobs[i].centroid.x;
		xmax = foundBlobs[i].max.x;
		xmin = foundBlobs[i].min.x;
		ymax = foundBlobs[i].max.y;
		ymin = foundBlobs[i].min.y;
		
		/* Checks if the pixel falls within the area of the blob. The condition being used assumes a rectangular blob, which seems to work well enough
		for small blobs. The condition commented out assumes a oval blob, which is more accurate, but more fiddly to try and get it to work right*/

		//if(	((y - ycenter) * (y - ycenter)/((xmax  - xmin)*(xmax - xmin)/4)) + (x - xcenter)*(x - xcenter)/((ymax - ymin)*(ymax - ymin)/4) < 4){
		if( (y <= ymax + 1)&&(y >= ymin - 1)&&(x <= xmax + 1)&&(x >= xmin - 1)){
			return true;
		}
	}
	return false;
}


blob Locator::detectBlob (void* pixel, void* picture){//determines the dimentions and centroid of the blob containing the pixel given
	blob returnBlob;
	bool edgeSeeker = true;
	unsigned char* rightPointer;
	unsigned char* leftPointer;
	unsigned char* storedRight;
	unsigned char* storedLeft;
	int offset = 0;
	bool leftEdge = false;
	bool rightEdge = false;

	while(edgeSeeker){//while we are still looking for the edge
		rightPointer = (unsigned char*) pixel + offset;//moves the right pointer right
		leftPointer = (unsigned char*) pixel - offset;//moves the left pointer left

		if((!rightEdge)&&((abs(*rightPointer - averageValue) < sigmaNumber*standardDeviation)||(rightPointer > ((unsigned char*)picture + xPixels*yPixels)))){
			storedRight = rightPointer - 1;//if the pixel is no longer significant, we have found the edge of the blob
			rightEdge = true;
		}
		if((!leftEdge)&&((abs(*leftPointer - averageValue) < sigmaNumber*standardDeviation)||(leftPointer < (unsigned char*)picture))){
			storedLeft = leftPointer + 1;//same as above but for the left side
			leftEdge = true;
		}
		offset ++;

		if((leftEdge) && (rightEdge)){//if we have found both the right and left edges
			edgeSeeker = false;//move on
		}
	}
	//calculates the middle of the two edges found above
	unsigned char* centerLine = (unsigned char*) (((unsigned long long int)storedRight + (unsigned long long int)storedLeft)/2);

	edgeSeeker = true;
	unsigned char* topPointer;
	unsigned char* bottomPointer;
	unsigned char* storedTop;
	unsigned char* storedBottom;
	offset = 0;
	bool foundTop = false;
	bool foundBottom = false;

	while (edgeSeeker){

		topPointer = centerLine - offset*xPixels;//starts at the middle line calculated above, moves up
		bottomPointer = centerLine + offset*xPixels;//and down
		if((!foundTop)&&((abs(*topPointer - averageValue) < sigmaNumber*standardDeviation)||(topPointer < (unsigned char*)picture))){
			storedTop = topPointer + xPixels;//looks for the top edge as before
			foundTop = true;
		}
		if((!foundBottom)&&((abs(*bottomPointer - averageValue) < sigmaNumber*standardDeviation)||(bottomPointer > (unsigned char*)picture + xPixels*yPixels))){
			storedBottom = bottomPointer - xPixels;//looks for the bottom edge
			foundBottom = true;
		}
		if(foundTop && foundBottom){//if we have found the top and bottom
			edgeSeeker = false;//move on
		}
		offset ++;
	}
	returnBlob.max.y = (int)(( storedBottom - (unsigned char*) picture )/xPixels);//stores the top and bottom
	returnBlob.min.y = (int)((storedTop - (unsigned char*) picture)/xPixels);

	//calculates the vertical center of the blob using the top and bottom edges
	centerLine = (unsigned char*)((unsigned long long int)storedTop + ((((int)(storedBottom - storedTop)/xPixels)/2)*xPixels));
	edgeSeeker = true;
	offset = 0;
	leftEdge = false;
	rightEdge = false;
	while(edgeSeeker){
	
		leftPointer = centerLine - offset;//looks left
		rightPointer = centerLine + offset;//and right
	
		if((!rightEdge)&&((abs(*rightPointer - averageValue) < sigmaNumber*standardDeviation)|| (rightPointer > ((unsigned char*)picture + xPixels*yPixels)))){
			storedRight = rightPointer - 1;//looks for the right edge 
			rightEdge = true;
		}
		if((!leftEdge)&&((abs(*leftPointer - averageValue) < sigmaNumber*standardDeviation)|| (leftPointer < (unsigned char*)picture))){
			storedLeft = leftPointer + 1;//looks for the left edge
			leftEdge = true;
		}
		offset ++;

		if((leftEdge) && (rightEdge)){//if we found both edges
			edgeSeeker = false;//move on
		}
	}

	returnBlob.max.x = (int) ((storedRight - (unsigned char*) picture) % xPixels);//stores the right edge
	returnBlob.min.x = (int) ((storedLeft - (unsigned char*) picture) % xPixels);//and the left

	returnBlob.centroid.x = (returnBlob.max.x + returnBlob.min.x)/2;//calculates the horisontal center
	returnBlob.centroid.y = (returnBlob.max.y + returnBlob.min.y)/2;//and the vertical center
	//calculates the approximate area
	returnBlob.size = (int) (3.14/4.0 * ((double)((returnBlob.max.x - returnBlob.min.x + 1)*(returnBlob.max.y - returnBlob.min.y + 1))));

	//printf("Blob found at %d, %d (center x = %d, y = %d), height: %d, width %d, size %d\n", (int)((unsigned long long int)pixel - (unsigned long long int)picture)%xPixels, (int)((unsigned long long int)pixel - (unsigned long long int)picture)/xPixels, returnBlob.centroid.x, returnBlob.centroid.y, 1+ returnBlob.max.y - returnBlob.min.y, 1+ returnBlob.max.x - returnBlob.min.x, returnBlob.size);
	return returnBlob;
}