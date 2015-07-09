/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "fake_sky.h"
#include <time.h>
#include "../shared/simulations/brightness.h"
#include "../tools/stats.h"
#include "../tools/quick_cout.h"

using namespace Imaging;

#define shared_brightness (*(Shared::Simulations::brightness.r))

FakeSky::FakeSky(): sto((int) time(0))
{
}

void FakeSky::match_brightness(unsigned short pixels[], int width, int height, int depth)
{
    double final_level_e = shared_brightness.level_kepsa*1000.0*shared_brightness.simulated_exposure;
    double scaling = shared_brightness.simulated_exposure / shared_brightness.actual_exposure;
    double gain = 0.04096 * pow(10.0, shared_brightness.gain_db/20.0);
    double existing_level_e = get_mean(pixels, width*height) * scaling / gain;
    double missing_level_e = final_level_e - existing_level_e;
    if (missing_level_e < 0) {
        missing_level_e = 0;
    }

    for (int j=0; j<height; j++) {
        for (int i=0; i<width; i++) {
            int k = j*width + i;
            double pixel = pixels[k];
            pixel *= scaling;
            pixel += sto.Poisson(missing_level_e)*gain;
            pixels[k] = int(pixel);
            if (pixels[k] < 0) {
                pixels[k] = 0;
            } else if (pixels[k] >= depth) {
                pixels[k] = depth-1;
            }
        }
    }
}

