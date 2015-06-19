/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "solution.h"
#include "../tools/angles.h"
#include <boost/format.hpp>
#include "../shared/image/raw.h"
#include "../logging/logger.h"

using namespace Solving;

Solution::Solution()
{
    equatorial.valid = false;
    equatorial.ra = 0.0;
    equatorial.dec = 0.0;
    equatorial.roll = 0.0;
    equatorial.sigma_ra = 2.0*M_PI;
    equatorial.sigma_dec = 2.0*M_PI;
    equatorial.sigma_roll = 2.0*M_PI;
    equatorial.sigma_pointing = 2.0*M_PI;
    equatorial.iplatescale = from_arcsec(9.5);

    horizontal.valid = false;
    horizontal.az = 0.0;
    horizontal.el = 0.0;
    horizontal.roll = 0.0;
    horizontal.sigma_az = 2.0*M_PI;
    horizontal.sigma_el = 2.0*M_PI;
    horizontal.sigma_roll = 2.0*M_PI;
    horizontal.sigma_pointing = 2.0*M_PI;
    horizontal.iplatescale = from_arcsec(9.5);

    num_blobs_total = 0;
    measured_exposure = 0.0;
}

void Solution::print(Logging::Logger& logger, bool print_matches, bool print_base_matches)
{
    logger.log(format("solution: found %d catalog stars in fov") % stars_in_fov.size());
    for (unsigned int i=0; i<stars_in_fov.size(); i++) {
        stars_in_fov[i].print(logger, "solution: star ");
    }
    for (unsigned int i=0; i<matched_blobs.size() && i<matched_stars.size(); i++) {
        logger.log(format("solution: blob with id %d matches star with id %d and ccd pos %.2f %.2f")
            % matched_blobs[i].id % matched_stars[i].id
            % matched_stars[i].fitted_x % matched_stars[i].fitted_y);
    }
    if (print_base_matches) {
        for (unsigned int i=0; i<base_blobs.size() && i<base_stars.size(); i++) {
            logger.log(format("solution: base blob with id %d matches base star with id %d")
                % base_blobs[i].id % base_stars[i].id);
        }
    }
    logger.log(format("solution: matched %d of %d") % matched_blobs.size() % num_blobs_total);

    if (equatorial.valid) {
        logger.log(format("solution: equatorial attitude (deg) %0.4f %0.4f %0.4f")
            % to_degrees(equatorial.ra) % to_degrees(equatorial.dec) % to_degrees(equatorial.roll));
        logger.log(format("solution: equatorial errors (arcsec) %0.3f %0.3f %0.3f")
            % to_arcsec(equatorial.sigma_ra) % to_arcsec(equatorial.sigma_dec) % to_arcsec(equatorial.sigma_roll));
        logger.log(format("solution: equatorial pointing error (arcsec) %0.3f")
            % to_arcsec(equatorial.sigma_pointing));
        logger.log(format("solution: equatorial iplatescale (arcsec/px) %0.3f")
            % to_arcsec(equatorial.iplatescale));
    }

    if (horizontal.valid) {
        logger.log(format("solution: horizontal attitude (deg) %0.4f %0.4f %0.4f")
            % to_degrees(horizontal.az) % to_degrees(horizontal.el) % to_degrees(horizontal.roll));
        logger.log(format("solution: horizontal errors (arcsec) %0.3f %0.3f %0.3f")
            % to_arcsec(horizontal.sigma_az) % to_arcsec(horizontal.sigma_el) % to_arcsec(horizontal.sigma_roll));
        logger.log(format("solution: horizontal pointing error (arcsec) %0.3f")
            % to_arcsec(horizontal.sigma_pointing));
        logger.log(format("solution: horizontal iplatescale (arcsec/px) %0.3f")
            % to_arcsec(horizontal.iplatescale));
    }
}

bool Solution::sort_by_num_matches(Solution first, Solution second)
{
    return first.matched_blobs.size() > second.matched_blobs.size();
}

