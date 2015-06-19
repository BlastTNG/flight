/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__FINDING__FINDER_H
#define SOLVING__FINDING__FINDER_H

#include <vector>
#include "estimator.h"
#include "leveler.h"
#include "smoother.h"
#include "fitter.h"
#include "pedestal_fitter.h"
#include "badpix.h"
#include "../../shared/image/raw.h"

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
        class Finder;
    }
}

class Solving::Finding::Finder
{
  public:
    Finder(Parameters::Manager& params);
    ~Finder();
    std::vector<Blob> fit_blobs(Shared::Image::Raw& image, std::vector<Blob> &blobs);
    bool is_local_max_in_smooth(int x, int y);
    std::vector<Blob> find_blobs(Shared::Image::Raw& image, double noise);

  private:
    std::vector<Blob> search_for_peaks_in_cell(unsigned int cell_size, unsigned int ucell, unsigned int vcell, double threshold, double noise, unsigned int max_num_blobs);
    std::vector<Blob> search_for_peaks(int halfwidth, double noise);
    bool are_duplicates(Blob& blob0, Blob& blob1);
    std::vector<Blob> unique(std::vector<Blob>& original_blobs);
    std::vector<Blob> estimate_blobs(Shared::Image::Raw& image, std::vector<Blob> possible_blobs);
    void print_blobs (std::vector<Blob>& blobs);
    void crop_vector(std::vector<Blob>& blobs, unsigned int num);

    int image_width;
    int image_height;
    double* pixels_smoothed;
    unsigned short* pixels_leveled;
	//const char* badpixlist;
	int satval;
	//unsigned short satval;
	int mapmeans;
	//double mapmean;
	std::string badpixlist;
    Smoother smoother;
    Estimator estimator;
	Badpix badpix;
    Leveler leveler;
    Fitter fitter;
    PedestalFitter pedestal_fitter;

    bool bypass_blobs_enabled;
    std::vector<Blob> bypass_blobs;
};

#endif

