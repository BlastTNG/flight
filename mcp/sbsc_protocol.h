/* sbsc_protocol.h
 * struct of return values from spider bore sight camera
 */


#ifndef SBSC_PROTOCOL_H
#define SBSC_PROTOCOL_H

//camera related defines
#define CAM_WIDTH 765.0  //should always be the larger dimension
#define CAM_HEIGHT 510.0

#ifdef __cplusplus
#include <string>
#include <sys/time.h>

using namespace std;

struct SBSCReturn {
	//image info
	unsigned long int frameNum;     //frame number
	double mapmean;                 //mean value of image map
	double sigma;                   //error level (stnadard deviation) of map
	double exposuretime;            //exposure duration in seconds
	timeval imagestarttime;         //time set immediately after exposure started
	string camID;                   //some number to uniquely identify camera
	double ccdtemperature;          //CCD temperature measured by camera's sensor
	
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

#endif        //SBSC_PROTOCOL_H
