#pragma once

#ifndef SOLVING__FINDING__BADPIX_H
#define SOLVING__FINDING__BADPIX_H

#include <string>
#include <vector>
#include <map>
#include "code/solving/finding/estimator.h"


namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

namespace Parameters
{
    class Manager;
}

namespace Solving
{
	class Blob;

    namespace Finding
    {
        class Badpix;
    }
}

class Solving::Finding::Badpix
{
  public:
	Badpix(Parameters::Manager& params);
    ~Badpix();
    int load_badpix( std::string fname);// Load bad pixel map from file
	void fix_badpix(Shared::Image::Raw& image, int val);	 // set bad pixels to val
   	std::string badpixlist;
 	void calc_mapstat(Shared::Image::Raw& image);
	int get_mapmean(Shared::Image::Raw& image);
	//double get_mapmean(Shared::Image::Raw& image);
	int *satcol;
	int satval;
	//unsigned short satval;
	double mapmean; 
	int mapmeans;

	struct badpixels 
	{
		int xpix;
		int ypix;
	} ;

  private:
	//unsigned int *badpixels;  // array of indices of bad pixels in CCD frames
	//std::vector<badpixels> badpair;
	std::vector<badpixels> badpairs;
	unsigned int numbadpix;   // # of bad pixels 
	int image_width;
    int image_height;
  };

#endif 

