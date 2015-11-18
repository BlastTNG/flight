#pragma once

#ifndef SOLVING__FINDING__BADPIX_H
#define SOLVING__FINDING__BADPIX_H

#include <string>
#include <vector>
#include <map>
#include "estimator.h"


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
	void fix_badpix(Shared::Image::Raw& image);	 // set bad pixels to map mean
   	std::string badpixlist;
	int *satcol;
	int satval;
	double mapmean; 
	int mapmeans;

	struct badpixels 
	{
		int xpix;
		int ypix;
	} ;

  private:
	std::vector<badpixels> badpairs;
	unsigned int numbadpix;   
	int image_width;
    int image_height;
  };

#endif 

