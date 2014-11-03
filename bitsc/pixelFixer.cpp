//pixelFixer.cpp
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

WORD* PixelFixer::generateBadPixelTable(const char* directory){

	DIR* data = opendir(directory);

	if(data == NULL){
		printf("calibration directory not found, creating.\n");

		system("mkdir calibImages");
		return NULL;
	}

	std::vector<double> pixelVector;
	pixelVector.resize(xSize*ySize, 0);
	char newDirectory[100];
	sprintf(newDirectory, "%s/LensOn*.bmp" ,directory);//makes a search string for the files we want

	dirent* fileStructure = readdir(data);//finds the first file in the directory

	char* fileName;
	char* newFileName =(char*) malloc(200);
	int exposureTime;
	int pictureNumber;
	bool filesWereFound = false;

	if(fileStructure != NULL){//if the first file exists

		fileName = fileStructure->d_name;
		if(*fileName == 'L'){
			sscanf(fileName, "LensOn%d-%d.bmp", &exposureTime, &pictureNumber);
			filesWereFound == true;
			sprintf(newFileName, "%s/%s", directory, fileName);
			pixelVector = detectWeirdPixels(pixelVector, newFileName, exposureTime);//calls the helper method to put the data from the file into a vector
		}
	}else{
		printf("Data files cannot be opened in generateBadPixelTable. Error Code %d\n", errno);
		return NULL;
	}

	int count = 1;
	fileStructure = readdir(data);
	while(fileStructure != NULL){//while there are more files
		fileName = fileStructure->d_name;
		if(*fileName == 'L'){
			filesWereFound = true;
			sprintf(newFileName, "%s/%s", directory, fileName);
			sscanf(fileName, "LensOn%d-%d.bmp", &exposureTime, &pictureNumber);
			pixelVector = detectWeirdPixels(pixelVector, newFileName, exposureTime);//repeat the above process
			count ++;
		}
		fileStructure = readdir(data);
	}
	free(newFileName);

	if(filesWereFound == false){
		printf("no calibration files found in the directory\n");
		return NULL;
	}

	char* charValues =(char *) malloc(pixelVector.size());

	FILE* textFile = fopen("signaltonoise.txt", "w");//makes a file with the S/N data for plotting
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
	sprintf(compositeFilename, "%s/Composite.bmp" ,directory);

	std::fstream file (compositeFilename , std::ios_base::in | std::ios_base::binary);//opens the old composite image for reading
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

	file.open(compositeFilename, std::ios_base::out | std::ios_base::binary);//opens the file for writing

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

	printf("average was %lf, stddev was %lf\n");

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
	
	std::vector<unsigned short> badPixels;
	badPixels.push_back(0);
	unsigned short badCount = 0;

	for(unsigned int i = 0; i<pixelVector.size(); i++){
		if((abs(pixelVector[i] - /*noiseA*/ average) > 4 * /*noiseS*/ stdDev) /*&&(abs(pixelVector[i] - average) > stdDev)*/){
			badCount ++;//increments the count of bad pixels

			badPixels.push_back(i%xSize);//puts the x coordinate in the array
			badPixels.push_back(i/xSize);//puts the y coordinate in the array
			
			printf("Bad pixel found at x = %d, y = %d\n", i%xSize, i/xSize);
		}
	}
	printf("Found %d bad pixels.\n", badCount);
	badPixels[0] = badCount;
	unsigned short* returnValue =(unsigned short *) malloc(badPixels.size()*sizeof(unsigned short));

	for(unsigned int i =0; i<badPixels.size(); i++){
		*(unsigned short*)((unsigned long long int)returnValue + (i)*sizeof(short)) = badPixels[i];
	}
	return returnValue;//returns the array of bad pixels so that they can be corrected for
}

//gets all pixels that are non-uniform
std::vector<double> PixelFixer::detectWeirdPixels(std::vector<double> data, char* filename, int weight){


	std::fstream file (filename, std::ios_base::in | std::ios_base::binary);
	if(!file.is_open()){
		printf("File was not opened in detectWeirdPixels\n");
	}
	
	char* header = (char* ) malloc(14);
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

	for(int i =0; i<(xSize*ySize*pixelSize/8);){
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
		returnValue[i] = data[i] + abs(tempVector[i] - average)*weight /* 10000*/;
	}
	return returnValue;
}

//loads the table of bad pixels from a file
unsigned short* PixelFixer::loadSavedTable(char* filename, unsigned short* tablePointer){

	FILE* file = fopen(filename, "r");
	if(file == NULL){
		printf("fails to open table in loadsavedtable\n");
		return NULL;
	}

	int count = 0;
	char line[200];
	if(fgets(line, 199, file) == NULL){
		printf("file is empty in loadsavedtable\n");
		return NULL;
	}
	int size;
	sscanf(line, "%d\n", &size);
	tablePointer = (unsigned short*) malloc((2*size +1)*sizeof(short));
	*tablePointer = (unsigned short)size;
	while(fgets(line, 199, file) != NULL){

		int xcoords;
		int ycoords;
		sscanf(line, "%d, %d\n", &xcoords, &ycoords);

		*(tablePointer + (1 + 2*count)) = (unsigned short)xcoords;
		*(tablePointer + (2 + 2*count)) = (unsigned short)ycoords;
		count++;
	}

	fclose(file);
	return tablePointer;
}

//saves the bad pixels to a file
int PixelFixer::savePixelTable(char* filename, unsigned short* tablePointer){

	FILE* file = fopen(filename, "w");

	if(file == NULL){
		return IS_NO_SUCCESS;
	}
	unsigned short number = *tablePointer;
	fprintf(file, "%d\n", (int)number);
	unsigned short xcoords;
	unsigned short ycoords;

	for(int i = 0; i<number; i++){
		xcoords = *(tablePointer + (1+2*i));
		ycoords = *(tablePointer + (2+2*i));

		fprintf(file, "%d, %d\n", (int)xcoords, (int)ycoords);
	}

	fclose(file);
	return IS_SUCCESS;
}

void PixelFixer::getImageBackground(void* picture){

	double average = 0;
	double stdDev = 0;
	int temp;

	for(int i = 0; i<xSize*ySize*pixelSize/8; i++){

		temp = *((char*)((unsigned long long int)picture + i));
		average += temp;
		stdDev += pow(temp, 2);
	}

	average/= xSize*ySize*pixelSize/8;
	stdDev/= xSize*ySize*pixelSize/8;

	stdDev = sqrt(stdDev - pow(average, 2));

	int noisyCount = 0;
	for(int i = 0; i<xSize*ySize*pixelSize/8; i++){

		if(abs(*((char*)((unsigned long long int)picture +i)) - average) > 3*stdDev){
			noisyCount ++;
		}
	}

	printf("%lf\t\t%lf\t\t%d\n", average, stdDev, noisyCount);
}
