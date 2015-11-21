/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "finder.h"
#include <cstdio>
#include <limits>
#include <algorithm>
#include <boost/math/special_functions/erf.hpp>
#include "../../tools/math.h"
#include "../../shared/image/blobs.h"
#include "../../shared/image/raw.h"
#include "../../shared/solving/settings.h"
#include "../../shared/solving/mask.h"
#include "../../shared/solving/motion_psf.h"
#include "../update.h"
#include "../../parameters/manager.h"
#include "../../parameters/housekeeping.h"
#include "../logger.h"

#include "../../tools/quick_cout.h"

using namespace Solving::Finding;
using Solving::Blob;
using std::vector;
using std::max;
using std::min;

#define shared_settings (*(Shared::Solving::settings.r))
#define shared_mask (*(Shared::Solving::mask_network_for_solver.r))
#define shared_motion_psf (*(Shared::Solving::motion_psf_network_for_solver.r))
#define shared_blobs (*(Shared::Image::blobs_solver_for_main.w))

Finder::Finder(Parameters::Manager& params):
    smoother(params), leveler(params), badpix(params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);
    pixels_smoothed = new double[image_width*image_height];
    pixels_leveled = new unsigned short[image_width*image_height];
    bypass_blobs_enabled = params.general.try_get("solver.blob_finder.bypass_with_blobs.enabled", false);
	bypass_blobs.clear();
	badpixlist = params.general.try_get("solver.blob_finder.badpixfilename", string("badpixels_ISC.cam"));
	badpix.load_badpix(badpixlist);
	satval = 16383; 
	//mapmean = 0;

    if (bypass_blobs_enabled) {
        unsigned int num_blobs = params.general.try_get("solver.blob_finder.bypass_with_blobs.num_blobs", (unsigned int) 0);
        for (unsigned int i=0; i<num_blobs; i++) {
            Blob blob;
            blob.x = params.general.try_get((boost::format("solver.blob_finder.bypass_with_blobs.blob%d.x") %i).str(), 0.0);
            blob.y = params.general.try_get((boost::format("solver.blob_finder.bypass_with_blobs.blob%d.y") %i).str(), 0.0);
            blob.u = int(round(blob.x + (double(image_width)-1.0)/2.0));
            blob.v = int(round(blob.y + (double(image_height)-1.0)/2.0));
			//blob.saturated = 
            bypass_blobs.push_back(blob);
        }
    }
}

Finder::~Finder()
{
    delete [] pixels_smoothed;
    delete [] pixels_leveled;
}

vector<Blob> Finder::fit_blobs(Shared::Image::Raw& image, vector<Blob> &blobs) {
    vector<Blob> fitted_blobs;
    for (unsigned int i=0; i<blobs.size(); i++) {
        Blob fitted_blob = fitter.fit(image, blobs[i]);
        if (fitted_blob.fit_was_good) {
            fitted_blobs.push_back(fitted_blob);
        }
    }
    return fitted_blobs;
}

bool Finder::is_local_max_in_smooth(int x, int y)
{
    if ((x>0 && x<image_width-1) && (y>0 && y<image_height-1)) {
        for (int dx=-1; dx<=1; dx++) {
            for (int dy=-1; dy<=1; dy++) {
                if (dx != 0 || dy != 0) {
                    if (pixels_smoothed[(y+dy)*image_width+(x+dx)] > pixels_smoothed[y*image_width+x]) {
                        return false;
                    }
                }
            }
        }
    }
    else{
        for (int dx=-1; dx<=1; dx++) {
            for (int dy=-1; dy<=1; dy++) {
                if (dx != 0 || dy != 0) {
                    if ((x+dx) >= 0 && (x+dx) < image_width &&
                        (y+dy) >= 0 && (y+dy) < image_height) {
                        if (pixels_smoothed[(y+dy)*image_width+(x+dx)] > pixels_smoothed[y*image_width+x]) {
                            return false;
                        }
                    }
                }
            }
        }
    }
    return true;
}

vector<Blob> Finder::search_for_peaks_in_cell(unsigned int cell_size, unsigned int ucell, unsigned int vcell, double threshold, double noise, unsigned int max_num_blobs) {
    unsigned int validity_padding = 10;
    unsigned int umin, umax, vmin, vmax;
    vector<Blob> cellblobs;

    umin = max(ucell, validity_padding);
    umax = min(ucell+cell_size, image_width-validity_padding);
    vmin = max(vcell, validity_padding);
    vmax = min(vcell+cell_size, image_height-validity_padding);
    for (unsigned int v=vmin; v<vmax; v++) {
        for (unsigned int u=umin; u<umax; u++) {
            if (pixels_smoothed[v*image_width+u] > pixels_leveled[v*image_width+u]+threshold*noise) {
                if (is_local_max_in_smooth(u, v)) {
                    Blob new_blob;
                    new_blob.u = u;
                    new_blob.v = v;
                    new_blob.correlated_peak = pixels_smoothed[v*image_width+u]
                        - pixels_leveled[v*image_width+u];
                    cellblobs.push_back(new_blob);
                }
            }
        }
    }
    sort(cellblobs.begin(), cellblobs.end(), Blob::sort_by_correlated_peak);
    if (cellblobs.size() > max_num_blobs) {
        cellblobs.resize(max_num_blobs);
    }
    return cellblobs;
}

vector<Blob> Finder::search_for_peaks(int halfwidth, double noise) {
    unsigned int ucell, vcell;
    double threshold;
    vector<Blob> blobs, cell_blobs;
    int cell_skip = shared_settings.cell_size;

    //threshold = 3.0 / (2*halfwidth+1);
    threshold = shared_settings.snr_threshold;

    for (vcell=0; vcell<(unsigned int)image_height; vcell+=cell_skip) {
        for (ucell=0; ucell<(unsigned int)image_width; ucell+=cell_skip) {
            if (!shared_mask.cell_masked(ucell, vcell)) {
                cell_blobs = search_for_peaks_in_cell(cell_skip, ucell, vcell, threshold, noise, shared_settings.max_num_blobs_per_cell);
                blobs.insert(blobs.end(), cell_blobs.begin(), cell_blobs.end());
                sort(blobs.begin(), blobs.end(), Blob::sort_by_correlated_peak);
                crop_vector(blobs, 100);
            }
        }
    }
    return blobs;
}

bool Finder::are_duplicates(Blob& blob0, Blob& blob1)
{
    int max_hw = 20;
    if (abs(blob0.u - blob1.u) > max_hw) return false;
    if (abs(blob0.v - blob1.v) > max_hw) return false;

    double distance;
    distance = sqrt(pow(double(blob0.u-blob1.u), 2) + pow(double(blob0.v-blob1.v), 2));
    if (distance < abs(blob0.approximate_size)*1.5) {
        return true;
    }
    if (distance < abs(blob1.approximate_size)*1.5) {
        return true;
    }
    return false;
}

vector<Blob> Finder::unique(vector<Blob>& original_blobs)
{
    vector<Blob> blobs;
    for (unsigned int i=0; i<original_blobs.size(); i++) {
        bool duplicate_found = false;
        for (unsigned int j=0; j<blobs.size() && !duplicate_found; j++) {
            if (are_duplicates(original_blobs[i], blobs[j])) {
                duplicate_found = true;
                if (Blob::sort_by_flux_confidence(original_blobs[i], blobs[j])) {
                    blobs[j] = original_blobs[i];
                }
            }
        }
        if (!duplicate_found) {
            blobs.push_back(original_blobs[i]);
        }
    }
    return blobs;
}

vector<Blob> Finder::estimate_blobs(Shared::Image::Raw& image, vector<Blob> possible_blobs)
{

    vector<Blob> blobs;
    bool valid;
    Blob new_blob;
    for (unsigned int i=0; i<possible_blobs.size(); i++) {
        new_blob = estimator.estimate_blob(image, possible_blobs[i], valid);
        if (valid) {
            blobs.push_back(new_blob);
        }
    }
    return blobs;
}


void Finder::print_blobs (vector<Blob>& blobs)
{
    logger.log(format("finder: found %d blobs")%blobs.size());
    for (unsigned int i=0; i<blobs.size() && i<100; i++) {
        blobs[i].print(logger, "finder: found blob with ");
    }
}

void Finder::crop_vector(vector<Blob>& blobs, unsigned int num) {
    if (blobs.size() > num) {
        blobs.resize(num);
    }
}

vector<Blob> Finder::find_blobs(Shared::Image::Raw& image, double noise)
{

    Tools::Timer timer;
    double sigma = 0.0;
    int halfwidth = 1;
    vector<Blob> blobs, possible_blobs;
    blobs.clear();
    possible_blobs.clear();

    if (bypass_blobs_enabled) {
        return bypass_blobs;
    }

    shared_blobs.motion_psf_used = false;

	//bad pixel removal goes here
	timer.start();
	if(!done()) {
		badpix.fix_badpix(image);			
	}
	                          	
	logger.log(format("finder: fixing bad pixels took %s s")%timer.time());

    timer.start();
    if (!done()) {
        leveler.level(image, pixels_leveled);
    }
    logger.log(format("finder: leveling took %s s")%timer.time());

    if (!done() && !shared_motion_psf.valid(image)) {
        timer.start();
        halfwidth = 1;
        sigma = 1.0;
        smoother.make_smooth(image, pixels_smoothed, 1, sigma);
        possible_blobs = search_for_peaks(halfwidth, noise);
        blobs.insert(blobs.end(), possible_blobs.begin(), possible_blobs.end());
        logger.log(format("finder: finding possible small blobs took %s s")%timer.time());
    }

    if (!done() && shared_settings.robust_mode_enabled && !shared_motion_psf.valid(image)) {

        timer.start();
        halfwidth = 5;
        sigma = 3.5;
        smoother.make_smooth(image, pixels_smoothed, halfwidth, sigma);
        possible_blobs = search_for_peaks(halfwidth, noise);
        blobs.insert(blobs.end(), possible_blobs.begin(), possible_blobs.end());
        halfwidth = 30;
        sigma = 15.0;
        smoother.make_smooth(image, pixels_smoothed, halfwidth, sigma);
        possible_blobs = search_for_peaks(halfwidth, noise);
        blobs.insert(blobs.end(), possible_blobs.begin(), possible_blobs.end());
        logger.log(format("finder: finding possible large blobs took %s s")%timer.time());
    }

    if (!done() && !shared_motion_psf.valid(image)) {

        timer.start();
        if (!done()) {
            blobs = estimate_blobs(image, blobs);
            sort(blobs.begin(), blobs.end(), Blob::sort_by_flux_confidence);
            blobs = unique(blobs);
        }
        logger.log(format("finder: estimating blobs took %s s")%timer.time());

        timer.start();
        if (!done()) {
            pedestal_fitter.fit_pedestals(image, blobs);
        }
        logger.log(format("finder: fitting pedestals took %s s")%timer.time());

    }

    if (shared_motion_psf.enabled && !shared_motion_psf.valid(image)) {
        logger.log("finder: motion_psf is enabled but not valid, waiting for data or timeout");
        while (!done() && !shared_motion_psf.valid(image) && image.age.time() < 10.0) {
            usleep(100000);
        }
    }

    if (shared_motion_psf.valid(image)) {
        blobs.clear();
        shared_blobs.motion_psf_used = true;
        timer.start();
        smoother.smooth_with_motion_psf(image, pixels_smoothed);
        possible_blobs = search_for_peaks(1, noise);
        blobs.insert(blobs.end(), possible_blobs.begin(), possible_blobs.end());
        logger.log(format("finder: finding motion psf blobs took %s s")%timer.time());
    }

    timer.start();
    if (!done()) {
        blobs = fit_blobs(image, blobs);
    }
    logger.log(format("finder: fitting blobs took %s s")%timer.time());

/*
    for (unsigned int i=0; i<blobs.size(); i++) {
        cout << "blob[" << i << "].sigma_x is " << blobs[i].sigma_x << endl;
    }
*/

    timer.start();
    if (!done()) {
		sort(blobs.begin(), blobs.end(), Blob::sort_by_flux_confidence);
		crop_vector(blobs, shared_settings.max_num_blobs);
        for (unsigned int i=0; i<blobs.size(); i++) {
            blobs[i].id = i;
            blobs[i].image_noise = noise;
        }
        logger.log(format("finder: sorting and assigning took %s s")%timer.time());
        print_blobs(blobs);
        return blobs;
    }

    blobs.clear();
    return blobs;
}


