/* camstruct.h
 * struct of return values from star camera
 */


#ifndef CAMSTRUCT_H
#define CAMSTRUCT_H

//camera related defines
#define CAM_WIDTH 1530.0  //should always be the larger dimension
#define CAM_HEIGHT 1020.0

#ifdef __cplusplus
#include <string>
#include <sys/time.h>

using namespace std;

struct StarcamReturn {
	//image info
	unsigned long int frameNum;     //frame number
	double mapmean;                 //mean value of image map
	double sigma;                   //error level (stnadard deviation) of map
	double exposuretime;            //exposure duration in seconds
	timeval imagestarttime;         //time set immediately after exposure started
	string camID;                   //some number to uniquely identify camera
	double ccdtemperature;          //CCD temperature measured by camera's sensor
	uint16_t  cputemperature;  //cpu temperature measured by sensors
	double focusposition;		//focus position
	
	//blob info (on 15 brightest blobs)
	int numblobs;                   //number of blobs found (or 15 if larger than 15)
	int flux[15];                   //flux of blob
	double mean[15];                //men value around blob
	double snr[15];                 //SNR of blob
	double x[15];                   //position of centroid
	double y[15];                   //...
	
	//pointing info won't be included since only coarse pointing needed in flight
};
#endif        //__cplusplus

#endif        //CAMSTRUCT_H
