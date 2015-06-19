/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "statistician.h"
#include <boost/format.hpp>
#include "../shared/image/stats.h"
#include "../shared/image/status.h"
#include "../shared/image/leveled.h"
#include "../shared/general/main_settings.h"
#include "../tools/stats.h"
#include "../parameters/manager.h"
#include "logger.h"

using namespace Solving;

#define shared_stats (*(Shared::Image::stats_solver_for_main.w))
#define shared_status (*(Shared::Image::status_solver_for_main.w))
#define shared_leveled (*(Shared::Image::leveled.w))
#define shared_main_settings (*(Shared::General::main_settings_main_for_solver.r))

Statistician::Statistician(Parameters::Manager& params):
    saturation_limit(params.general.try_get("solver.general.saturation_limit", 1.0))
{
    cell_size = 16;
}

void Statistician::get_cell_stats(Shared::Image::Raw& raw, int icell, int jcell,
    double& mean, double& stdev, int& num_saturated)
{
    bool pixel_saturated = false;
    int num_pixels_in_cell = cell_size*cell_size;
    if (num_pixels_in_cell < 1) {
        return;
    }
    mean = 0;
    num_saturated = 0;
    for (int j = jcell*cell_size; j < (jcell+1)*cell_size; j++) {
        for (int i = icell*cell_size; i < (icell+1)*cell_size; i++) {
            mean += raw.pixels[j*raw.width+i];
            pixel_saturated = false;
            for (unsigned int k=0; k<raw.num_exposures && k<raw.max_num_exposures && !pixel_saturated; k++) {
                if (raw.separate_buffers[k][j*raw.width+i] >= raw.single_depth-1) {
                    pixel_saturated = true;
                }
            }
            if (pixel_saturated) {
                num_saturated++;
            }
        }
    }
    mean /= num_pixels_in_cell;
    stdev = 0;
    for (int j = jcell*cell_size; j < (jcell+1)*cell_size; j++) {
        for (int i = icell*cell_size; i < (icell+1)*cell_size; i++) {
            stdev += pow(mean - raw.pixels[j*raw.width+i], 2);
        }
    }
    if (num_pixels_in_cell >= 2) {
        stdev /= double(cell_size*cell_size-1.0);
    }
    stdev = sqrt(stdev);
}

void Statistician::get_stats(Shared::Image::Raw& raw)
{
    using namespace Shared::Image;
    using std::vector;

    double mean=0, stdev=0;
    int num_saturated = 0;
    int num_saturated_in_cell = 0;
    vector <double> means;
    vector <double> noises;
    vector <double> gains;
    int num_icells = int( floor( double(raw.width) /double(cell_size) ) );
    int num_jcells = int( floor( double(raw.height)/double(cell_size) ) );
    for (int jcell=0; jcell < num_jcells; jcell++) {
        for (int icell=0; icell < num_icells; icell++) {
            get_cell_stats(raw, icell, jcell, mean, stdev, num_saturated_in_cell);
            num_saturated += num_saturated_in_cell;
            means.push_back(mean);
            noises.push_back(stdev);
            if (mean > 100 && num_saturated_in_cell == 0) {
                gains.push_back(pow(stdev, 2) / mean);
            }
        }
    }
    sort(noises.begin(), noises.end());
    sort(gains.begin(), gains.end());
    if (means.size() > 0) {
        shared_stats.mean_known = true;
        shared_stats.mean = get_mean(means);
    }
    else {
        shared_stats.mean_known = false;
        shared_stats.mean = 0.0;
    }
    if (noises.size() > 0){
        shared_stats.noise_known = true;
        shared_stats.noise = noises[int(floor(noises.size()/2.0))];
    }
    else{
        shared_stats.noise_known = false;
        shared_stats.noise = 1.0;
    }
    shared_stats.num_pixels_saturated = num_saturated;
    shared_stats.fraction_pixels_saturated = double(num_saturated) / double(raw.width*raw.height);

    double gain_ideal = 4096./100000.;
    unsigned int bottom_index = int(floor(gains.size()*0.0/6.0));
    unsigned int top_index = int(floor(gains.size()*1.0/6.0));
    double total = 0;
    if (top_index - bottom_index > 100) {
        for (unsigned int i=bottom_index; i<top_index; i++) {
            total += gains[i];
        }
        shared_stats.gain_known = true;
        shared_stats.gain = total/double(top_index-bottom_index);
    } else {
        shared_stats.gain_known = false;
        shared_stats.gain = gain_ideal;
    }
    shared_stats.gaindb = 20*log10( shared_stats.gain / gain_ideal );
    stats_solver_for_main.share();

    if (shared_stats.fraction_pixels_saturated > saturation_limit) {
        shared_status.stage = Status::done;
        shared_status.reason_for_being_done = Status::saturated;
    }
}

void Statistician::get_default_display_bounds(Shared::Image::Raw& raw, double& default_lower_bound, double& default_width)
{
    int lower_bound = 0;
    int upper_bound = raw.depth;
    unsigned int* histogram = new unsigned int[raw.depth];
    int histogram_skip = 4;
    int num_pixels = raw.width * raw.height / (histogram_skip * histogram_skip);
    {
        for (int i=0; i<raw.depth; i++){
            histogram[i] = 0;
        }
        unsigned short pixel;
        for (int j = 0; j < raw.height; j+=histogram_skip) {
            for (int i = 0; i < raw.width; i+=histogram_skip) {
                pixel = raw.pixels[j*raw.width+i];
                if (pixel >= 0 && pixel < raw.depth) {
                    histogram[pixel]++;
                }
            }
        }

        lower_bound = 0;
        int sum = 0;
        while ((sum < 0.01*num_pixels) && (lower_bound < raw.depth)) {
            sum += histogram[lower_bound];
            lower_bound++;
        }
        lower_bound -= 2;

        upper_bound = raw.depth-1;
        sum = 0;
        while ((sum < 0.01*num_pixels) && (upper_bound >= 0)) {
            sum += histogram[upper_bound];
            upper_bound--;
        }
        upper_bound += 2;

        if (upper_bound <= lower_bound) {
            lower_bound = 0;
            upper_bound = raw.depth;
        }
    }
    delete [] histogram;
    default_lower_bound = double(lower_bound);
    default_width = double(upper_bound - lower_bound)*1.3;
}

void Statistician::make_display_data(Shared::Image::Raw& raw)
{
    volatile double value = 0;
    double lower_bound = 0;
    double width = raw.depth;
    get_default_display_bounds(raw, lower_bound, width);

    double brightness = shared_main_settings.display_image_brightness;
    logger.log(format("using image brightness %f") % brightness);
    if (brightness > 1.0) {
        double upper_bound = lower_bound + width;
        width *= brightness;
        lower_bound = upper_bound - width;
    }
    if (brightness > 0.0 && brightness < 1.0) {
        width /= brightness;
    }

    int k=0;
    for (int j=0; j<raw.height; j++) {
        for (int i=0; i<raw.width; i++) {
            value = raw.pixels[j*raw.width+i];
            value = floor(255.0*(value - lower_bound)/width);
            if (value < 0.0) {
                value = 0.0;
            }
            if (value > 255.0) {
                value = 255.0;
            }
            for (k=0; k<3; k++) {
                shared_leveled.pixels[(j*raw.width+i)*4+k] = (char) value;
            }
            shared_leveled.pixels[(j*raw.width+i)*4+3] = (char) 255;
        }
    }

    shared_leveled.counter_stars = raw.counter_stars;
    shared_leveled.valid = true;
    Shared::Image::leveled.share();
}

void Statistician::print_stats()
{
    if (shared_stats.mean_known) {
        logger.log(format("stats: mean: %0.2f") % shared_stats.mean);
    }
    if (shared_stats.noise_known) {
        logger.log(format("stats: noise: %0.2f") % shared_stats.noise);
    }
    if (shared_stats.gain_known) {
        logger.log(format("stats: gain: %0.2f gaindb: %0.2f") % shared_stats.gain % shared_stats.gaindb);
    }
}

