/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__IMAGE__STATUS_H
#define SHARED__IMAGE__STATUS_H

#include <string>
#include "../circular_buffer.h"
#include "../../tools/timing.h"

namespace Logging
{
    class Logger;
}

namespace Shared
{
    namespace Image
    {

class Raw;

class Status
{
  public:

    enum Stages {
        empty=0,
        doing_statistics,
        blob_finding,
        pattern_matching,
        refitting,
        getting_names,
        done
    };

    enum ReasonsForBeingDone {
        no_reason,
        solved,
        not_solving,
        saturated,
        tried_all_patterns,
        timed_out,
        aborted
    };

    Status();
    void init(Parameters::Manager& params);
    Status& operator=(const Status& rhs);
    Status& operator=(const Raw& rhs);

    int counter_stars;
    bool from_camera;
    int abort_counter;
    int num_exposures;
    int counter_fcp;
    bool horizontal_known;
    double lat;
    double lst;
    std::string filename;
    Stages stage;
    ReasonsForBeingDone reason_for_being_done;
    Tools::Timer age;
    int width;
    int height;
    int depth;
};

// writes: solver
//  reads: main
extern Shared::CircularBufferPass <Status> status_solver_for_main;

// writes: main
//  reads: camera
extern Shared::CircularBufferPass <Status> status_main_for_camera;

// writes: camera
//  reads: network
extern Shared::CircularBuffer <Status> status_camera_for_network;


    }
}

#endif
