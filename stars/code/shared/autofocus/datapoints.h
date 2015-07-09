/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__AUTOFOCUS__DATAPOINTS_H
#define SHARED__AUTOFOCUS__DATAPOINTS_H

#include <vector>
#include "../circular_buffer.h"
#include "../../tools/timing.h"

namespace Logging
{
    class Logger;
}

namespace Shared
{
    namespace Autofocus
    {

enum datapoints_t
{
    metric_brightest_blob_flux,
    metric_star_flux,
    metric_sobel
};

struct CurvePoint
{
    double focus;
    double value;
    CurvePoint(double focus_=0.0, double value_=0.0): focus(focus_), value(value_){}
};

class Curve
{
  public:
    Curve();
    bool get_peak(double& focus, double& value);
    void log(Logging::Logger& logger);
    Curve& operator=(const Curve& rhs);
    static bool sort_by_type(Curve first, Curve second)
    {
        return first.type > second.type;
    }

    datapoints_t type;
    int star_id;
    std::vector<CurvePoint> points;
    Tools::Timer age_since_last_datapoint_added;
};

class Datapoints
{
  public:
    Datapoints();
    Datapoints& operator=(const Datapoints& rhs);

    std::vector<Curve> curves;
    int last_fully_solved_counter_stars;
};

// writes: solver
//  reads: lens
extern Shared::CircularBufferPass <Datapoints> datapoints_solver_to_lens;

// writes: lens
//  reads: main
extern Shared::CircularBuffer <Datapoints> datapoints_lens_to_main;

    }
}

#endif

