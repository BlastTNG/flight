/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "filters.h"
#include <boost/algorithm/string.hpp>
#include "settings.h"
#include "../../parameters/manager.h"
#include "../../tools/angles.h"
#include "../../solving/solution.h"
#include "../../logging/logger.h"

using namespace Shared::Solving;
#define shared_settings (*(Shared::Solving::settings.r))

Filters::Filters()
{
    horizontal_from_fcp_age_limit = 5*60.0;
    image_width = 0;
    image_height = 0;
}

void Filters::init(Parameters::Manager& params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);

    double angle = 0;
    horizontal_from_settings.enabled = params.general.try_get("solver.filters.horizontal_source.settings.enabled", false);
    angle = params.general.try_get("solver.filters.horizontal_source.settings.lat_degrees", 0.0);
    horizontal_from_settings.lat = from_degrees(angle);
    angle = params.general.try_get("solver.filters.horizontal_source.settings.lst_hours", 0.0);
    horizontal_from_settings.lst = from_hours(angle);

    horizontal_from_fits.enabled = params.general.try_get("solver.filters.horizontal_source.fits.enabled", false);
    horizontal_from_fcp.enabled = params.general.try_get("solver.filters.horizontal_source.fcp.enabled", true);

    horizontal_location.enabled = params.general.try_get("solver.filters.horizontal.location_limit.enabled", false);
    angle = params.general.try_get("solver.filters.horizontal.location_limit.radius_degrees", 30.0);
    horizontal_location.radius = from_degrees(angle);
    angle = params.general.try_get("solver.filters.horizontal.location_limit.az_degrees", 0.0);
    horizontal_location.az = from_degrees(angle);
    angle = params.general.try_get("solver.filters.horizontal.location_limit.el_degrees", 0.0);
    horizontal_location.el = from_degrees(angle);

    horizontal_roll_limit.enabled = params.general.try_get("solver.filters.horizontal.roll_limit.enabled", false);
    angle = params.general.try_get("solver.filters.horizontal.roll_limit.min_degrees", -180.0);
    horizontal_roll_limit.min_roll = from_degrees(angle);
    angle = params.general.try_get("solver.filters.horizontal.roll_limit.max_degrees", 180.0);
    horizontal_roll_limit.max_roll = from_degrees(angle);

    horizontal_elevation_limit.enabled = params.general.try_get("solver.filters.horizontal.elevation_limit.enabled", false);
    angle = params.general.try_get("solver.filters.horizontal.elevation_limit.min_degrees", 10.0);
    horizontal_elevation_limit.min_el = from_degrees(angle);
    angle = params.general.try_get("solver.filters.horizontal.elevation_limit.max_degrees", 80.0);
    horizontal_elevation_limit.max_el = from_degrees(angle);

    equatorial_location.enabled = params.general.try_get("solver.filters.equatorial.location_limit.enabled", false);
    angle = params.general.try_get("solver.filters.equatorial.location_limit.radius_degrees", 30.0);
    equatorial_location.radius = from_degrees(angle);
    angle = params.general.try_get("solver.filters.equatorial.location_limit.ra_degrees", 0.0);
    equatorial_location.ra = from_degrees(angle);
    angle = params.general.try_get("solver.filters.equatorial.location_limit.dec_degrees", 0.0);
    equatorial_location.dec = from_degrees(angle);

    equatorial_roll_limit.enabled = params.general.try_get("solver.filters.equatorial.roll_limit.enabled", false);
    angle = params.general.try_get("solver.filters.equatorial.roll_limit.min_degrees", -180.0);
    equatorial_roll_limit.min_roll = from_degrees(angle);
    angle = params.general.try_get("solver.filters.equatorial.roll_limit.max_degrees", 180.0);
    equatorial_roll_limit.max_roll = from_degrees(angle);

    matching.pointing_error_threshold = from_arcsec(params.general.try_get("solver.filters.matching.pointing_error_threshold_arcsec", 10.0));
    matching.fit_error_threshold_px = params.general.try_get("solver.filters.matching.fit_error_threshold_px", 2000.0);
    matching.num_matched = params.general.try_get("solver.filters.matching.num_matched", (unsigned int) 8);
}

Filters& Filters::operator=(const Filters &rhs)
{
    if (this != &rhs) {
        image_width = rhs.image_width;
        image_height = rhs.image_height;
        horizontal_from_fcp_age_limit = rhs.horizontal_from_fcp_age_limit;
        horizontal_from_settings = rhs.horizontal_from_settings;
        horizontal_from_fits = rhs.horizontal_from_fits;
        horizontal_from_fcp = rhs.horizontal_from_fcp;
        horizontal_elevation_limit = rhs.horizontal_elevation_limit;
        horizontal_roll_limit = rhs.horizontal_roll_limit;
        horizontal_location = rhs.horizontal_location;
        equatorial_location = rhs.equatorial_location;
        matching = rhs.matching;
    }
    return *this;
}

bool Filters::try_get_horizontal(double& lat, double& lst)
{
    if (horizontal_from_fcp.enabled && horizontal_from_fcp.age.time() < horizontal_from_fcp_age_limit) {
        lat = horizontal_from_fcp.lat;
        lst = horizontal_from_fcp.lst;
        return true;
    }
    if (horizontal_from_settings.enabled) {
        lat = horizontal_from_settings.lat;
        lst = horizontal_from_settings.lst;
        return true;
    }
    if (horizontal_from_fits.enabled && horizontal_from_fits.valid) {
        lat = horizontal_from_fits.lat;
        lst = horizontal_from_fits.lst;
        return true;
    }
    return false;
}

double Filters::lat() const
{
    if (horizontal_from_fcp.enabled && horizontal_from_fcp.age.time() < horizontal_from_fcp_age_limit) {
        return horizontal_from_fcp.lat;
    }
    if (horizontal_from_settings.enabled) {
        return horizontal_from_settings.lat;
    }
    if (horizontal_from_fits.enabled && horizontal_from_fits.valid) {
        return horizontal_from_fits.lat;
    }
    return 0.0;
}

double Filters::lst() const
{
    if (horizontal_from_fcp.enabled && horizontal_from_fcp.age.time() < horizontal_from_fcp_age_limit) {
        return horizontal_from_fcp.lst;
    }
    if (horizontal_from_settings.enabled) {
        return horizontal_from_settings.lst;
    }
    if (horizontal_from_fits.enabled && horizontal_from_fits.valid) {
        return horizontal_from_fits.lst;
    }
    return 0.0;
}

bool Filters::horizontal_known() const
{
    if (horizontal_from_fcp.enabled && horizontal_from_fcp.age.time() < horizontal_from_fcp_age_limit) {
        return true;
    }
    if (horizontal_from_settings.enabled) {
        return true;
    }
    if (horizontal_from_fits.enabled && horizontal_from_fits.valid) {
        return true;
    }
    return false;
}

bool Filters::horizontal_known_and_filters_enabled() const
{
    if (horizontal_known()) {
        if (horizontal_elevation_limit.enabled || horizontal_roll_limit.enabled ||
            horizontal_location.enabled)
        {
            return true;
        }
    }
    return false;
}

bool Filters::check_object(double ra, double dec, double max_distance)
{
    if (equatorial_location.enabled) {
        if (!angles_within(ra, dec, equatorial_location.ra, equatorial_location.dec,
                equatorial_location.radius + max_distance)) {
            return false;
        }
    }
    if (horizontal_location.enabled || horizontal_elevation_limit.enabled) {
        bool horizontal_known = false;
        double lat = 0;
        double lst = 0;
        horizontal_known = try_get_horizontal(lat, lst);
        if (horizontal_known) {
            double az = 0;
            double el = 0;
            equatorial_to_horizontal(ra, dec, lat, lst, az, el);
            if (horizontal_elevation_limit.enabled) {
                if (el < (horizontal_elevation_limit.min_el - max_distance) ||
                    el > (horizontal_elevation_limit.max_el + max_distance))
                {
                    return false;
                }
            }
            if (horizontal_location.enabled) {
                if (!angles_within(az, el, horizontal_location.az, horizontal_location.el,
                        horizontal_location.radius + max_distance)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Filters::check_field_star(double ra, double dec, double iplatescale_max)
{
    double corner_distance = sqrt(double(pow(image_width, 2.0)) + double(pow(image_height, 2.0)))/2.0;
    corner_distance *= iplatescale_max;
    return check_object(ra, dec, corner_distance);
}

bool Filters::check_roll_limit(RollLimit& roll_limit, double roll)
{
    double adjusted_roll = 
        (roll_limit.min_roll + M_PI) +
        #if _MSC_VER
                 fmod(roll-(roll_limit.min_roll+M_PI), 2.0*M_PI);
        #else
            remainder(roll-(roll_limit.min_roll+M_PI), 2.0*M_PI);
        #endif
    if (adjusted_roll < roll_limit.min_roll || adjusted_roll > roll_limit.max_roll) {
        return false;
    }
    return true;
}

bool Filters::check_solution(::Solving::Solution& solution, Logging::Logger& logger, bool final_check)
{
    bool debug = false;

    // both checks:
    //   1) equatorial attitude
    //   2) horizontal attitude

    // final check adds:
    //   3) platescale
    //   4) error
    //   5) matched_blobs.size()
    //   5) matched_blobs.size() / num_blobs_total


    if (solution.equatorial.valid) {
        if (!check_object(solution.equatorial.ra, solution.equatorial.dec, 0)) {
            // Checks equatorial location (guess) and horizontal location (guess and elevation limit)
            if (debug) {
                logger.log("rejected solution based on location");
            }
            return false;
        }
        if (equatorial_roll_limit.enabled) {
            if (!check_roll_limit(equatorial_roll_limit, solution.equatorial.roll)) {
                if (debug) {
                    logger.log("rejected solution based on equatorial roll limit");
                }
                return false;
            }
        }
    }

    if (solution.horizontal.valid) {
        if (horizontal_roll_limit.enabled) {
            if (!check_roll_limit(horizontal_roll_limit, solution.horizontal.roll)) {
                if (debug) {
                    logger.log("rejected solution based on horizontal roll limit");
                }
                return false;
            }
        }
    }

    if (final_check) {
        if (solution.equatorial.valid) {
            if (solution.equatorial.iplatescale < shared_settings.iplatescale_min || solution.equatorial.iplatescale > shared_settings.iplatescale_max) {
                if (debug) {
                    logger.log(format("rejected solution based on iplatescale (%f arcsec/px)") % to_arcsec(solution.equatorial.iplatescale));
                }
                return false;
            }
            if (solution.equatorial.sigma_pointing > matching.pointing_error_threshold) {
                if (debug) {
                    logger.log(format("rejected solution based on pointing_error (%f arcsec)") % to_arcsec(solution.equatorial.sigma_pointing));
                }
                return false;
            }
            if (solution.equatorial.fit_error > matching.fit_error_threshold_px) {
                if (debug) {
                    logger.log(format("rejected solution based on fit_error (%f px)") % solution.equatorial.fit_error);
                }
                return false;
            }
        }
        if (solution.matched_blobs.size() < matching.num_matched) {
            if (debug) {
                logger.log(format("rejected solution on num_match condition, matched %i of %i (requires %i)")
                    % solution.matched_blobs.size() % solution.num_blobs_total % matching.num_matched);
            }
            return false;
        }
    }
    
    return true;
}

