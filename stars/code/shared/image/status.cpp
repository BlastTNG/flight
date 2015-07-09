/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "status.h"
#include "raw.h"
#include "../../logging/logger.h"

using namespace Shared::Image;

Status::Status()
{
    counter_stars = -1;
    from_camera = false;
    abort_counter = 0;
    num_exposures = 0;
    counter_fcp = -1;
    horizontal_known = false;
    lat = 0.0;
    lst = 0.0;
    filename = "";
    stage = empty;
    reason_for_being_done = no_reason;
    width = 64;
    height = 64;
    depth = 64;
}

Status& Status::operator=(const Status &rhs)
{
    if (this != &rhs) {
        counter_stars = rhs.counter_stars;
        from_camera = rhs.from_camera;
        abort_counter = rhs.abort_counter;
        num_exposures = rhs.num_exposures;
        counter_fcp = rhs.counter_fcp;
        horizontal_known = rhs.horizontal_known;
        lat = rhs.lat;
        lst = rhs.lst;
        filename = rhs.filename;
        stage = rhs.stage;
        reason_for_being_done = rhs.reason_for_being_done;
        age = rhs.age;
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;
    }
    return *this;
}

Status& Status::operator=(const Raw &rhs)
{
    counter_stars = rhs.counter_stars;
    from_camera = rhs.from_camera;
    num_exposures = rhs.num_exposures;
    counter_fcp = rhs.counter_fcp;
    horizontal_known = rhs.filters.horizontal_known();
    if (horizontal_known) {
        lat = rhs.filters.lat();
        lst = rhs.filters.lst();
    }
    filename = rhs.filename;
    age = rhs.age;
    width = rhs.width;
    height = rhs.height;
    depth = rhs.depth;
    return *this;
}

