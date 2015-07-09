/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__SOLVING__SETTINGS_H
#define SHARED__SOLVING__SETTINGS_H

#include <string>
#include "../circular_buffer.h"
extern "C" {
#include "../../networking/xsc_protocol/xsc_protocol.h"
}

namespace Shared
{
    namespace Solving
    {

struct Refraction
{
    Refraction(): enabled(false), pressure_mbar(1013.25), temperature(296.15) {}
    bool enabled;
    double pressure_mbar;
    double temperature;
};

class Settings
{
  public:
    Settings();
    void init(Parameters::Manager& params);
    Settings& operator=(const Settings& rhs);

    enum PrecessionEpochSourceType {
        precession_none,
        precession_system_time,
        precession_manual
    };

    bool enabled;
    double timeout;

    int abort_counter;

    double snr_threshold;
    int max_num_blobs;
    bool robust_mode_enabled;
    xsc_solver_fitting_method_t fitting_method;
    unsigned int cell_size;
    unsigned int max_num_blobs_per_cell;

    bool pattern_matcher_enabled;
    bool display_names;
    double match_tolerance_px;
    double iplatescale_min;
    double iplatescale_max;
    bool platescale_always_fixed;
    double iplatescale_fixed;

    bool debug_timing;
    std::string catalog;
    Refraction refraction;
    PrecessionEpochSourceType precession_epoch_source;
    double precession_manual_epoch;
};

// writes: network
//  reads: solver
extern Shared::CircularBuffer <Settings> settings;

    }
}

#endif
