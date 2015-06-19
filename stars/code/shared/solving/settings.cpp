/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "settings.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/operations.hpp>
#include "../../parameters/manager.h"
#include "../../tools/angles.h"

using namespace Shared::Solving;

Settings::Settings()
{
    enabled = true;
    abort_counter = 0;
    timeout = 30.0;

    snr_threshold = 1.0;
    max_num_blobs = 10;
    robust_mode_enabled = false;
    fitting_method = xC_solver_fitting_method_none;
    cell_size = 128;
    max_num_blobs_per_cell = 2;

    pattern_matcher_enabled = true;
    display_names = false;
    match_tolerance_px = 6.0;
    iplatescale_min = from_arcsec(9.45);
    iplatescale_max = from_arcsec(9.56);
    platescale_always_fixed = false;
    iplatescale_fixed = from_arcsec(9.5);

    debug_timing = false;
    catalog = "catalogs/combo_9p0_8p0";
    precession_epoch_source = precession_none;
    precession_manual_epoch = 2000.0;
}

void Settings::init(Parameters::Manager& params)
{
    using std::string;

    enabled = params.general.try_get("solver.general.enabled", true);
    timeout = params.general.try_get("solver.general.timeout", 30.0);

    snr_threshold = params.general.try_get("solver.blob_finder.snr_threshold", 1.0);
    max_num_blobs = params.general.try_get("solver.blob_finder.max_num_blobs", 10);
    robust_mode_enabled = params.general.try_get("solver.blob_finder.robust_mode_enabled", false);
    string fitting_str;
    fitting_str = params.general.try_get("solver.blob_finder.fitting_method", string("none"));
    boost::to_lower(fitting_str);
    fitting_method = xC_solver_fitting_method_none;
    if (fitting_str == "gaussian") {
        fitting_method = xC_solver_fitting_method_gaussian;
    }
    if (fitting_str == "double_gaussian") {
        fitting_method = xC_solver_fitting_method_double_gaussian;
    }
    cell_size = params.general.try_get("solver.blob_finder.cell_size", (unsigned int) 128);
    max_num_blobs_per_cell = params.general.try_get("solver.blob_finder.max_num_blobs_per_cell", (unsigned int) 2);

    pattern_matcher_enabled = params.general.try_get("solver.pattern_matcher.enabled", true);
    display_names = params.general.try_get("solver.pattern_matcher.display_names", false);
    match_tolerance_px = params.general.try_get("solver.pattern_matcher.match_tolerance_px", 6.0);
    iplatescale_min = params.general.try_get("solver.pattern_matcher.iplatescale_min", 9.45);
    iplatescale_min = from_arcsec(iplatescale_min);
    iplatescale_max = params.general.try_get("solver.pattern_matcher.iplatescale_max", 9.45);
    iplatescale_max = from_arcsec(iplatescale_max);
    platescale_always_fixed = params.general.try_get("solver.pattern_matcher.platescale_always_fixed", false);
    iplatescale_fixed = params.general.try_get("solver.pattern_matcher.iplatescale_fixed", 9.45);
    iplatescale_fixed = from_arcsec(iplatescale_fixed);

    debug_timing = params.general.try_get("solver.general.debug_timing", true);
    catalog = params.general.try_get("solver.pattern_matcher.catalog", string("catalogs/combo_9p0_8p0"));
    catalog = boost::filesystem::system_complete(params.stars_dir + catalog).string();

    string precession_from;
    precession_from = params.general.try_get("solver.pattern_matcher.precession_from", string("none"));
    boost::to_lower(precession_from);
    if (precession_from == "system_time") {
        precession_epoch_source = precession_system_time;
    } else if (precession_from == "manual") {
        precession_epoch_source = precession_manual;
    }
    precession_manual_epoch = params.general.try_get("solver.pattern_matcher.precession_manual_epoch", 2000.0);

    refraction.enabled = params.general.try_get("solver.refraction.enabled", false);
    refraction.pressure_mbar = params.general.try_get("solver.refraction.pressure_mbar", 1013.25);
    refraction.temperature = params.general.try_get("solver.refraction.temperature", 296.15);


}

Settings& Settings::operator=(const Settings &rhs)
{
    if (this != &rhs) {
        enabled = rhs.enabled;
        abort_counter = rhs.abort_counter;
        timeout = rhs.timeout;

        snr_threshold = rhs.snr_threshold;
        max_num_blobs = rhs.max_num_blobs;
        robust_mode_enabled = rhs.robust_mode_enabled;
        fitting_method = rhs.fitting_method;
        cell_size = rhs.cell_size;
        max_num_blobs_per_cell = rhs.max_num_blobs_per_cell;

        pattern_matcher_enabled = rhs.pattern_matcher_enabled;
        display_names = rhs.display_names;
        match_tolerance_px = rhs.match_tolerance_px;
        iplatescale_min = rhs.iplatescale_min;
        iplatescale_max = rhs.iplatescale_max;
        platescale_always_fixed = rhs.platescale_always_fixed;
        iplatescale_fixed = rhs.iplatescale_fixed;

        debug_timing = rhs.debug_timing;
        catalog = rhs.catalog;
        precession_epoch_source = rhs.precession_epoch_source;
        precession_manual_epoch = rhs.precession_manual_epoch;
        refraction = rhs.refraction;
    }
    return *this;
}

