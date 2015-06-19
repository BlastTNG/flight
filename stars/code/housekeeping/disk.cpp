/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "housekeeper.h"
#include <boost/filesystem.hpp>
#include <limits>
#include "logger.h"

// This file exists because boost/filesystem.hpp and cbw.h don't play well together

using namespace Housekeeping;

void Housekeeper::get_disk()
{
    using namespace boost::filesystem;
    double giga = pow(2.0, 30.0);
    try {
        space_info info = space(path(output_dir));
        disk_space = info.available/giga;
        last_successful_disk_space_timer.start();

        if (timer_since_first_disk_space_measurement.started()) {
            if (disk_space == disk_space_first_measurement) {
                disk_time = std::numeric_limits<double>::infinity();
            } else {
                disk_time = timer_since_first_disk_space_measurement.time() *
                    disk_space / (disk_space_first_measurement - disk_space);
                if (disk_time < 0) {
                    disk_time = std::numeric_limits<double>::infinity();
                }
            }
            disk_time_valid = true;
        } else {
            timer_since_first_disk_space_measurement.start();
            disk_space_first_measurement = disk_space;
        }

        string logdata = "disk";
        logdata += (format(" %.3d GB")%disk_space).str();
        if (disk_time_valid) {
            logdata += (format(" %.3d days")%(disk_time/(24.0*3600.0))).str();
        }
        if (write_disk_counter == 0) {
            logger.log(logdata);
        }
        write_disk_counter = (write_disk_counter+1)%1;

    } catch (...) {
    }
}

