/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "solver.h"
#include "logger.h"
#include <cstdio>
#include "../tools/math.h"
#include <vector>
#include <algorithm>
#include <cstring>
#include <boost/bind.hpp>
#include "update.h"
#include "../shared/image/raw.h"
#include "../shared/image/blobs.h"
#include "../shared/image/solution_summary.h"
#include "../shared/image/stats.h"
#include "../shared/image/status.h"
#include "../shared/general/quit.h"
#include "../shared/solving/settings.h"
#include "../parameters/manager.h"

using namespace Solving;
#define shared_status (*(Shared::Image::status_solver_for_main.w))
#define shared_settings (*(Shared::Solving::settings.r))
#define shared_blobs (*(Shared::Image::blobs_solver_for_main.w))
#define shared_stats (*(Shared::Image::stats_solver_for_main.w))
#define shared_solution_summary (*(Shared::Image::solution_summary_for_main.w))

Solver::Solver(Parameters::Manager& params, Shared::Image::Raw& solvers_working_image):
    working_image(&solvers_working_image),
    blob_finder(params),
    pattern_matcher(params),
    statistician(params),
    autofocus_helper(params),
    quit_after_one(params.general.try_get("imaging.camera_filesystem.quit_after_one", false)),
    quit_after_one_delay(params.general.try_get("imaging.camera_filesystem.quit_after_one_delay", 1.0)),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    solve_thread(boost::bind(&Solver::solve_thread_function, this))
    #pragma warning(pop)
{
}

void Solver::solve()
{
    using namespace Shared::Image;
    Tools::Timer timer;
    logger.log(format("starting to solve image %s") % working_image->filename);

    timer.start();
    // setting up
    {
        shared_solution_summary.blob_names.clear();
        Shared::Image::solution_summary_for_main.share();
        shared_stats.clear(working_image->counter_stars);
        Shared::Image::stats_solver_for_main.share();
        Shared::Image::blobs_solver_for_main.share();
        update_shared();
    }

    // stats
    {
        statistician.make_display_data(*working_image);
        statistician.get_stats(*working_image);
        statistician.print_stats();
        update_shared();
    }
    autofocus_helper.extract_sobel_datapoint(*working_image);

    // blob finding
    if (!done()) {
        shared_status.stage = Status::blob_finding;
        status_solver_for_main.share();
        update_shared();

        double noise = 1.0;
        if (shared_stats.noise_known) {
            noise = shared_stats.noise;
        }
        shared_blobs.blobs.clear();
        shared_blobs.blobs = blob_finder.find_blobs(*working_image, noise);
        shared_blobs.counter_stars =  working_image->counter_stars;
        Shared::Image::blobs_solver_for_main.share();
        update_shared();
    }

    autofocus_helper.extract_brightest_blob_datapoint(*working_image, shared_blobs.blobs);

    bool solution_found = false;
    Solution solution;

    // pattern matching
    if (!done() && shared_settings.pattern_matcher_enabled) {
        shared_status.stage = Status::pattern_matching;
        status_solver_for_main.share();

        solution_found = pattern_matcher.match(shared_blobs.blobs, *working_image, solution);
        update_shared();
    }

    // refitting
    if (!done() && solution_found) {
        shared_status.stage = Status::refitting;
        status_solver_for_main.share();

        refitter.fit(solution, *working_image);

        shared_solution_summary = solution;
        shared_solution_summary.counter_stars = shared_status.counter_stars;
        shared_solution_summary.solving_time = shared_status.age.time();
        Shared::Image::solution_summary_for_main.share();

        pattern_matcher.inform_blobs_of_matches(solution);
        Shared::Image::blobs_solver_for_main.share();
    }

    // getting blob names
    if (!done() && solution_found && shared_settings.display_names) {
        shared_status.stage = Status::getting_names;
        status_solver_for_main.share();

        pattern_matcher.get_blob_names(solution);
        Shared::Image::solution_summary_for_main.share();
    }

    if (solution_found) {
        shared_status.reason_for_being_done = Status::solved;
        solution.print(logger, true);
        autofocus_helper.extract_star_datapoints(*working_image, solution);
    }
    shared_status.stage = Status::done;
    status_solver_for_main.share();
    autofocus_helper.set_fully_solved(working_image->counter_stars);

    update_shared();

    logger.log(format("total time spent solving %.3f s") % timer.time());
    logger.log(format("finished solving image %s") % working_image->filename);
    logger.flush();
}

void Solver::solve_thread_function()
{
    using namespace Shared::Image;
    pattern_matcher.init();

    while (!Shared::General::quit) {
        update_shared();

        if ((raw_from_camera.r->counter_stars != working_image->counter_stars) &&
            raw_from_camera.r->to_be_solved)
        {
            working_image = (raw_from_camera.r);
            shared_status = *working_image;
            shared_status.abort_counter = shared_settings.abort_counter;
            shared_status.stage = Status::doing_statistics;
            shared_status.reason_for_being_done = Status::no_reason;
            status_solver_for_main.share();
            update_shared();
            solve();
            if (quit_after_one) {
                usleep(int(round(1e6*quit_after_one_delay)));
                Shared::General::quit = true;
            }
        }

        usleep(100000);
    }
}

void Solver::wait_for_quit()
{
    solve_thread.join();
}

