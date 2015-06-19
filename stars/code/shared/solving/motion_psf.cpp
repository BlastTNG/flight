/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "motion_psf.h"
#include "../../tools/angles.h"
#include "../../parameters/manager.h"
#include "../image/raw.h"

using namespace Shared::Solving;

MotionPSF::MotionPSF()
{
    sample_period = 1.0 / 100.16;
    enabled = false;
    counter_fcp = -1;
    counter_stars = -1;
    summation_mode = false;
    hroll = 0.0;
    el = 45.0;
    platescale = 1.0/from_arcsec(9.5);
}

void MotionPSF::init(Parameters::Manager& params)
{
    enabled = params.general.try_get("solver.blob_finder.motion_psf.enabled", false);
    if (enabled) {
        summation_mode = params.general.try_get("solver.blob_finder.motion_psf.summation_mode", false);
        unsigned int num_timesteps = params.general.try_get("solver.blob_finder.motion_psf.num_timesteps", (unsigned int) 0);
        hroll = from_degrees(params.general.try_get("solver.blob_finder.motion_psf.hroll_deg", 0.0));
        el = from_degrees(params.general.try_get("solver.blob_finder.motion_psf.el_deg", 45.0));
        platescale = 1.0 / from_arcsec(params.general.try_get("solver.blob_finder.motion_psf.iplatescale_arcsec", 9.5));
        timesteps.clear();
        for (unsigned int i=0; i<num_timesteps && i<params.general.max_exposure_time_cs; i++) {
            MotionPSFTimestep motion_psf_timestep;
            motion_psf_timestep.exposure_num =
                params.general.try_get((boost::format("solver.blob_finder.motion_psf.exposure_num_t%d")%i).str(), -1);
            motion_psf_timestep.gy_az = from_degrees(params.general.try_get(
                (boost::format("solver.blob_finder.motion_psf.gy_az_deg_t%d")%i).str(), 0.0));
            motion_psf_timestep.gy_el = from_degrees(params.general.try_get(
                (boost::format("solver.blob_finder.motion_psf.gy_el_deg_t%d")%i).str(), 0.0));
            timesteps.push_back(motion_psf_timestep);
        }
    }
}

bool MotionPSF::valid(Shared::Image::Raw& image)
{
    if (enabled && (counter_fcp == image.counter_fcp) && (counter_stars == image.counter_stars)) {
        return true;
    }
    return false;
}


