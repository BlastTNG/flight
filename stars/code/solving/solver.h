/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__SOLVER_H
#define SOLVING__SOLVER_H

#include <boost/thread/thread.hpp>
#include "finding/finder.h"
#include "matching/matcher.h"
#include "refitting/refitter.h"
#include "statistician.h"
#include "autofocus_helper.h"

namespace Parameters
{
    class Manager;
}

namespace Solving
{
    class Solver;
}

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
}

class Solving::Solver
{
  public:
    Solver(Parameters::Manager& params, Shared::Image::Raw& solvers_working_image);
    void solve();
    void solve_thread_function();
    void wait_for_quit();

  private:
    Shared::Image::Raw* working_image;
    Finding::Finder blob_finder;
    Matching::Matcher pattern_matcher;
    Statistician statistician;
    Refitting::Refitter refitter;
    AutofocusHelper autofocus_helper;
    bool quit_after_one;
    double quit_after_one_delay;

    boost::thread solve_thread;
};

#endif

