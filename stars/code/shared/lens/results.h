/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__LENS__RESULTS_H
#define SHARED__LENS__RESULTS_H

#include "../circular_buffer.h"
#include <string>
#include "../../imaging/commands.h"
#include "../../tools/timing.h"
#include "requests.h"

namespace Shared
{
    namespace Lens
    {

class Results
{
  public:
    Results();
    Results& operator=(const Results& rhs);
    bool is_focus_valid(Requests& requests);
    bool is_aperture_valid(Requests& requests);

    static const unsigned int num_commands = Imaging::LensCommands::num_commands;
    int focus_value;
    bool focus_found;
    int aperture_value;
    bool aperture_found;
    int command_counters[num_commands];
    double max_exposure_and_readout_time;
    std::string device_name;
    bool device_found;
};


//   path: 0
// writes: lens
//  reads: camera
extern Shared::CircularBufferPass <Results> fcp_results_lens_to_camera;

//   path: 0
// writes: camera
//  reads: main
extern Shared::CircularBufferPass <Results> fcp_results_camera_to_main;

//   path: 0
// writes: main
//  reads: network
extern Shared::CircularBuffer <Results> fcp_results_main_to_network;


//   path: 1
// writes: lens
//  reads: camera
extern Shared::CircularBufferPass <Results> stars_results_lens_to_camera;

//   path: 1
// writes: camera
//  reads: main
extern Shared::CircularBuffer <Results> stars_results_camera_to_main;

    }
}

#endif
