/* focusstruct.h
 * struct of blob positions and fluxes during autoFocus (in mycam.cpp)
 */


#ifndef FOCUSSTRUCT_H
#define FOCUSSTRUCT_H

#ifdef __cplusplus
#include <string>
#include <sys/time.h>

using namespace std;

struct focusStruct {
	int focpos;			//distance to infinity when the image was taken
	int numblobs;                   //number of blobs found (or 15 if larger than 15)
	int flux[15];                   //flux of blob
	double x[15];                   //position of centroid
	double y[15];                   //...
};
#endif        //__cplusplus

#endif        //FOCUSSTRUCT_H
