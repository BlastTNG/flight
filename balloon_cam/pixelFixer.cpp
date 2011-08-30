//pixelFivxer.cpp
//a class that decides on the bad pixels in the camera and saves them into the correction table

#include "stdafx.h"
#include "pixelFixer.h"

PixelFixer::PixelFixer(int width, int height, int colourDepth){
	xSize = width;
	ySize = height;
	
	if(colourDepth%8 != 0){
		printf("Pixel size is not a multiple of 8. This causes pixel fixer to be incorrect. Ignore it.\n");
	}

	pixelSize = colourDepth;
}

WORD* PixelFixer::generateBadPixelTable(char* directory){

	WIN32_FIND_DATA data;
	HANDLE handle;

	std::vector<double> pixelVector;
	pixelVector.resize(xSize*ySize, 0);
	char newDirectory[100];
	sprintf_s(newDirectory, 100, "%s/LensOn*.bmp" ,directory);//makes a search string for the files we want

	wchar_t* filePath = (wchar_t *) malloc (200);

	MultiByteToWideChar(CP_ACP, MB_COMPOSITE, newDirectory, -1, filePath, 200);//converts the filename to the correct format

	handle = FindFirstFile(filePath, &data);//finds the first file in the directory

	free(filePath);
	char* fileName = (char *) malloc (200);
	char* newFileName =(char*) malloc(200);
	int exposureTime;

	if(handle != INVALID_HANDLE_VALUE){//if the first file exists
		
		WideCharToMultiByte(CP_ACP, 0, data.cFileName, -1, fileName, 200, NULL, NULL);

		sscanf_s(fileName, "LensOn%d.bmp", &exposureTime);

		sprintf_s(newFileName, 200, "%s/%s", directory, fileName);

		pixelVector = detectWeirdPixels(pixelVector, newFileName, exposureTime);//calls the helper method to put the data from the file into a vector
	}else{
		printf("Data files cannot be opened in generateBadPixelTable. Error Code %d\n", GetLastError());
		return NULL;
	}

	int count = 1;

	while(FindNextFile(handle, &data)){//while there are more files
		WideCharToMultiByte(CP_ACP, 0, data.cFileName, -1, fileName, 200, NULL, NULL);
		sprintf_s(newFileName, 200, "%s/%s", directory, fileName);
		sscanf_s(fileName, "LensOn%d.bmp", &exposureTime);
		pixelVector = detectWeirdPixels(pixelVector, newFileName, exposureTime);//repeat the above process
		count ++;
	}
	free(fileName);
	free(newFileName);

	char* charValues =(char *) malloc(pixelVector.size());

	FILE* textFile; 
	fopen_s(&textFile, "signaltonoise.txt", "w");//makes a file with the S/N data for plotting
	fprintf(textFile, "X\t\tY\t\tIntensity\n");

	for(unsigned int i = 0; i<pixelVector.size(); i++){
		pixelVector[i] /= count;
		fprintf(textFile, "%5d\t%5d\t%5d\n", i%xSize, i/xSize, (int) pixelVector[i]);//prints the data into the file
		*(char *)((unsigned long long int)charValues + i) = (char)(int)pixelVector[i];//puts the data into a block of memory to generate the image
	}

	fclose(textFile);
	double average =0;
	double stdDev = 0;
	char compositeFilename[100];
	sprintf_s(compositeFilename, 100, "%s/Composite.bmp" ,directory);

	std::fstream file = std::fstream(compositeFilename , std::ios_base::in | std::ios_base::binary);//opens the old composite image for reading
	if(!file.is_open()){
		printf("File was not opened in detectWeirdPixels\n");
	}

	char* header = (char* ) malloc(14);//gets from it the relevant values
	file.read(header, 14);
	unsigned int offset = (unsigned int) *((int*)((long long int)header + 10));
	unsigned int size = (unsigned int) *((int*)((long long int)header + 2));
	free(header);

	file.seekg(0);

	char* fullHeader = (char*) malloc(offset);
	file.read(fullHeader, offset);//reads out the header from the file

	file.seekg(size - offset - xSize*ySize*pixelSize/8);

	char* footer = (char*) malloc(size - offset -xSize*ySize*pixelSize/8);//reads out the footer from the file
	file.read(footer, size - offset - xSize*ySize*pixelSize/8);

	file.close();//closes the file

	file.open(compositeFilename, std::ios_base::out | std::ios_base::binary);//opens the fil for writing

	file.write(fullHeader, offset);//writes the header to the file
	if(file.bad()){
		printf("Something went wrong with writing composite file header\n");
	}

	file.write(charValues, pixelVector.size());//writes the data to the file
	if(file.bad()){
		printf("Something went wrong with writing composite file data\n");
	}

	file.write(footer, size - offset -xSize*ySize*pixelSize/8);//writes the footer to the file
	if(file.bad()){
		printf("Something went wrong with writing composite file footer\n");
	}

	file.close();

	free(charValues);
	free(fullHeader);
	free(footer);

	for(unsigned int i = 0; i<pixelVector.size(); i++){//calculates the average and stdDev of the pixels
		average += pixelVector[i];
		stdDev += pixelVector[i] * pixelVector[i];
	}
	average = average/pixelVector.size();
	stdDev = sqrt(stdDev/pixelVector.size() - average*average);

	double noiseAverage = 0;
	double noiseStdDev = 0;
	int noiseCount = 0;

	for(unsigned int i = 0; i < pixelVector.size(); i++){//calculates the average and stdDev of only the non zero pixels
		if(abs(pixelVector[i] - average) > stdDev){
			noiseAverage += pixelVector[i];
			noiseStdDev += pixelVector[i] * pixelVector[i];
			noiseCount ++;
		}
	}

	noiseAverage /= noiseCount;
	noiseStdDev = sqrt(noiseStdDev/noiseCount - noiseAverage*noiseAverage);
	
	std::vector<WORD> badPixels;
	badPixels.push_back(0);
	WORD badCount = 0;

	for(unsigned int i = 0; i<pixelVector.size(); i++){
		if((abs(pixelVector[i] - /*noiseA*/ average) > 50* /*noiseS*/ stdDev) /*&&(abs(pixelVector[i] - average) > stdDev)*/){
			badCount ++;//increments the count of bad pixels

			badPixels.push_back(i%xSize);//puts the x coordinate in the array
			badPixels.push_back(i/xSize);//puts the y coordinate in the array
			
			printf("Bad pixel found at x = %d, y = %d\n", i%xSize, i/xSize);
		}
	}
	printf("Found %d bad pixels.\n", badCount);
	badPixels[0] = badCount;
	WORD* returnValue =(WORD *) malloc(badPixels.size()*sizeof(WORD));

	for(unsigned int i =0; i<badPixels.size(); i++){
		*(WORD*)((unsigned long long int)returnValue + (i)*sizeof(WORD)) = badPixels[i];
	}
	return returnValue;//returns the array of bad pixels so that they can be corrected for
}

std::vector<double> PixelFixer::detectWeirdPixels(std::vector<double> data, char* filename, int weight){

	std::fstream file = std::fstream(filename, std::ios_base::in | std::ios_base::binary);
	if(!file.is_open()){
		printf("File was not opened in detectWeirdPixels\n");
	}
	
	char* header = (char* ) malloc(14);//ignores the header file
	file.read(header, 14);
	int offset = (int) *((int*)((long long int)header + 10));
	free(header);

	file.ignore(offset - 14);

	char* pixels = (char*)malloc(xSize*ySize*pixelSize/8);
	file.read(pixels, xSize*ySize*pixelSize/8);
	file.close();

	double average = 0;
	double stdDev = 0;

	std::vector<int> tempVector;
	tempVector.reserve(xSize*ySize);
	unsigned int temp;

	for(int i =0; i<(xSize*ySize*pixelSize/8);){//goes through each pixel 
		temp = 0;
		for(int j = 0; j<pixelSize/8; j++){
			temp += (*((unsigned char*)(unsigned long long int)pixels + i)) * (int)pow((double)2, pixelSize/8 - j - 1);
			i++;
		}
		tempVector.push_back(temp);
		average += temp;
	}

	average = average/(xSize*ySize);
	free(pixels);

	std::vector<double> returnValue;
	returnValue.resize(xSize*ySize);

	for(int i = 0; i < (xSize * ySize); i++){
		returnValue[i] = data[i] + abs(tempVector[i] - average)/weight * 350;//adds each "badness" value in turn to the old vector of badness
	}
	return returnValue;//then returns it
}