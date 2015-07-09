/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__SOLVING__FILTERS_H
#define SHARED__SOLVING__FILTERS_H

#include "../circular_buffer.h"
#include "../../tools/timing.h"
#include "../../tools/angles.h"

namespace Logging
{
    class Logger;
}

namespace Solving
{
    class Solution;
}

namespace Shared
{
    namespace Solving
    {

struct HorizontalFromSettings
{
    HorizontalFromSettings(): enabled(false), lat(0), lst(0) {}
    bool enabled;
    double lat;
    double lst;
};

struct HorizontalFromFits
{
    HorizontalFromFits(): enabled(false), valid(false), lat(0), lst(0) {}
    bool enabled;
    bool valid;
    double lat;
    double lst;
};

struct HorizontalFromFCP
{
    HorizontalFromFCP(): enabled(false), lat(0), lst(0) {}
    bool enabled;
    Tools::Timer age;
    double lat;
    double lst;
};

struct HorizontalLocation
{
    HorizontalLocation(): enabled(false), radius(30), az(0), el(0) {}
    bool enabled;
    double radius;
    double az;
    double el;
};

struct RollLimit
{
    RollLimit(): enabled(false), min_roll(0), max_roll(2.0*M_PI) {}
    bool enabled;
    double min_roll;
    double max_roll;
};

struct HorizontalElevationLimit
{
    HorizontalElevationLimit(): enabled(false), min_el(-M_PI/2.0), max_el(M_PI/2.0) {}
    bool enabled;
    double min_el;
    double max_el;
};

struct EquatorialLocation
{
    EquatorialLocation(): enabled(false), radius(30), ra(0), dec(0) {}
    bool enabled;
    double radius;
    double ra;
    double dec;
};

struct Matching
{
    Matching(): pointing_error_threshold(from_arcsec(10.0)), fit_error_threshold_px(2000.0), num_matched(8) {}
    double pointing_error_threshold;
    double fit_error_threshold_px;
    unsigned int num_matched;
};

class Filters
{
  public:
    Filters();
    void init(Parameters::Manager& params);
    Filters& operator=(const Filters& rhs);
    bool try_get_horizontal(double& lat, double& lst);
    double lat() const;
    double lst() const;
    bool horizontal_known() const;
    bool horizontal_known_and_filters_enabled() const;
    bool check_object(double ra, double dec, double max_distance);
    bool check_field_star(double ra, double dec, double platescale_max);
    bool check_roll_limit(RollLimit& roll_limit, double roll);
    bool check_solution(::Solving::Solution& solution, Logging::Logger& logger, bool final_check=false);

    int image_width;
    int image_height;
    double horizontal_from_fcp_age_limit;
    HorizontalFromSettings horizontal_from_settings;
    HorizontalFromFits horizontal_from_fits;
    HorizontalFromFCP horizontal_from_fcp;

    HorizontalLocation horizontal_location;
    RollLimit horizontal_roll_limit;
    HorizontalElevationLimit horizontal_elevation_limit;
    EquatorialLocation equatorial_location;
    RollLimit equatorial_roll_limit;
    Matching matching;
};

// writes: network
//  reads: main
extern Shared::CircularBufferPass <Filters> filters_net_to_main;

// writes: main
//  reads: camera
extern Shared::CircularBuffer <Filters> filters_main_to_camera;

    }
}

#endif
