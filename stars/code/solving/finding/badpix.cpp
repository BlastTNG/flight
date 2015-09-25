/*
 * Added by Krystle Sy to remove bad pixels.
 */

#include "badpix.h"
#include "../blob.h"
#include "../logger.h"
#include "../../shared/image/raw.h"
#include "../../shared/solving/settings.h"
#include "../../tools/math.h"
#include "../../parameters/manager.h"
#include "../../parameters/housekeeping.h"
#include <boost/format.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>

using namespace Solving::Finding;
//using namespace boost::filesystem;
using namespace std;
using Solving::Blob;
using std::vector;
using std::string;

#define shared_settings (*(Shared::Solving::settings.r))
//#define shared_blobs (*(Shared::Image::blobs_solver_for_main.w))
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_camera.w))


Badpix::Badpix(Parameters::Manager& params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);
	    
    
	satval = 16383;				         // numerical value of a saturated pixel
    //satcol = new int[image_width];     // array contains # sat pixels each column
  
 }

Badpix::~Badpix(void)
{
	// delete[] satcol;
	
}

int Badpix::load_badpix(std::string fname) {
  string thisline;
  int count;     // Number of elements read from the current line
  int x, y;   // coordinates of the bad pixel in this line
  std::vector<badpixels> badpixlist;
  badpixels badpair;
  
  // Read through file line-by-line reading in pixel coordinates.
  // Use the blob linked-list to dynamically add coordinates as we
  // read them in. At the end, put the array indices calculated from the
  // pixel coordinates into an array for quick access when we process
  // frames.


  ifstream badfile(fname);
  if (badfile.is_open())
  {
   numbadpix = 0;   
  
	while( getline(badfile,thisline) ) {
   	 count = sscanf(thisline.c_str(),"%i %i",&x, &y);
    if( count == 2 ) { // If we read in a coordinate pair
        if( x <= image_width){
		badpair.xpix = x;
		if( y <= image_height) {
		badpair.ypix = (image_height - y - 1);
		badpairs.push_back(badpair);
		numbadpix ++;
		}
		}
     }
   }
  }
  
  badfile.close();
  
  logger.log(format("Opened bad pixel file with %i bad pixels.")
          %int(numbadpix));

  return 1;
}



void Badpix::fix_badpix(Shared::Image::Raw& image, int val) {
  int i, k;
 
	if( &badpixlist != NULL ) 
       
	for( i=0, k=0; i<(int)numbadpix; i++ ) { 
		k = badpairs[i].ypix*image_width + badpairs[i].xpix;
		image.pixels[k] =  val;
	}
}

void Badpix::calc_mapstat(Shared::Image::Raw& image) {
  int i, j;
  unsigned int count, index;
  // initialize variables
  
  mapmean = 0;
  count = 0;
  //memset(satcol,0,image_width*sizeof(int));
  
  for( j=0; j<image_height; j++ ) { // j=row
    index = j*image_width;         // inner loop columns to speed up addressing
    for( i=0; i<image_width; i++ ) {
            
		if( satval < image.pixels[index]){
			satcol++;}
			else {                // otherwise contributes to mean
        if( image.pixels[index] != satval ) { // pixel isn't bad
          mapmean += image.pixels[index];
          count ++;
        }
      }
      index++;
	 
	}
  }
  if( count > 0 ) mapmean = mapmean / (double) count; // prevent  / zero
  if( mapmean < 1 ) mapmean = 1;                      // prevent 0 means
 }


int Badpix::get_mapmean(Shared::Image::Raw& image) {
  calc_mapstat(image);
  mapmeans = int(round(mapmean));
  return mapmeans;
}

