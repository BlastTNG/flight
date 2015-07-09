#pragma once

#ifndef SOLVING__FINDING__BADPIX_H
#define SOLVING__FINDING__BADPIX_H

#include <string>
#include <vector>
//#include "estimator.h"
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
    int load_badpix(std::string fname); // Load bad pixel map from file
	void fix_badpix(Shared::Image::Raw& image); // set bad pixels to val
    //std::vector<Blob> fit_blobs(Shared::Image::Raw& image, std::vector<Blob> &blobs);	used as model
	void setnextblob( std::vector<Blob> &nextblob_in );
	void setprevblob( std::vector<Blob> &prevblob_in );
	std::vector<Blob> getnextblob();
	std::vector<Blob> getprevblob();
	std::string badpixlist;
 
  private:
  	unsigned int *badpixels;  // array of indices of bad pixels in CCD frames
	unsigned int numbadpix;   // # of bad pixels 
	std::vector<Blob> &nextblob;    // head/tail pointers for
	std::vector<Blob> &prevblob;    // the list.  
	int image_width;
    int image_height;
    
};

#endif 

