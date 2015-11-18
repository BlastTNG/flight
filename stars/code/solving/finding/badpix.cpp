/*
 * Added by Krystle Sy to remove bad pixels.
 */

#include "badpix.h"
#include "../blob.h"
#include "../logger.h"
#include "../../shared/image/stats.h"
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
using namespace std;
using Solving::Blob;
using std::vector;
using std::string;

#define shared_stats (*(Shared::Image::stats_solver_for_main.r))
#define shared_settings (*(Shared::Solving::settings.r))
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_camera.w))


Badpix::Badpix(Parameters::Manager& params)
{
	image_width =  params.general.image_width;
	image_height = params.general.image_height;
	satval = params.general.image_depth;
	  
 }

Badpix::~Badpix(void)
{	
}

int Badpix::load_badpix(std::string fname) {
	string thisline;
	int count;     // Number of elements read from the current line
	int x, y;   // coordinates of the bad pixel in this line
	badpixels badpair;

	// Read through file line-by-line reading in pixel coordinates.
	// Use the badpairs vector to dynamically add coordinates as we
	// read them in. 


	ifstream badfile(fname);
	if (badfile.is_open())
	{
		numbadpix = 0;

		while (getline(badfile, thisline)) {
			count = sscanf_s(thisline.c_str(), "%i %i", &x, &y);
			if (count == 2) { // If we read in a coordinate pair
				if (x <= image_width){
					badpair.xpix = x;
					if (y <= image_height) {
						badpair.ypix = (image_height - y - 1);
						badpairs.push_back(badpair);
						numbadpix++;
					}
				}
			}
		}
	}

	badfile.close();

	logger.log(format("Opened bad pixel file with %i bad pixels.")
		% int(numbadpix));

	return 1;
}



void Badpix::fix_badpix(Shared::Image::Raw& image) {
	int i, k;

	if (&badpixlist != NULL)  {
		short val = (short)shared_stats.mean;
		for (i = 0, k = 0; i < (int)numbadpix; i++) {
			k = badpairs[i].ypix * image.width + badpairs[i].xpix;
			image.pixels[k] = val;
		}
	}
}



