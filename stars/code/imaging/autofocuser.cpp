/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "autofocuser.h"
#include <limits>
#include "logger_lens.h"
#include "commands.h"
#include "../tools/math.h"
#include "../shared/autofocus/datapoints.h"
#include "../shared/autofocus/requests.h"
#include "../shared/autofocus/latest_image.h"
#include "../shared/autofocus/results.h"
#include "../shared/lens/requests.h"
#include "../shared/lens/results.h"

using namespace Imaging;
using namespace Shared::Autofocus;
using Lensing::logger;
#define shared_datapoints (*(datapoints_solver_to_lens.r))
#define shared_requests (*(requests_main_to_lens.r))
#define shared_latest_image (*(latest_image.r))
#define shared_results (*(results_lens_to_solver.w))
#define shared_lens_results (*(Shared::Lens::stars_results_camera_to_main.r))
#define shared_lens_requests (*(Shared::Lens::stars_requests_lens_to_main.w))

Autofocuser::Autofocuser()
{
    run_counter = 0;
    //run_counter = -1; // HACK TO MAKE IT RUN
    abort_counter = 0;
    last_counter_stars = -1;
    waiting_for_image_since_focus_change = false;
}

void Autofocuser::make_focus_request()
{
    logger.log(format("autofocuser requesting focus %i")%shared_results.current_focus_requested);
    shared_lens_requests.commands[LensCommands::set_focus].counter++;
    shared_lens_requests.commands[LensCommands::set_focus].value =
        shared_results.current_focus_requested;
    Shared::Lens::stars_requests_lens_to_main.share();
    waiting_for_image_since_focus_change = true;
}

void Autofocuser::end_run(bool use_solution)
{
    logger.log(format("autofocuser at end of run found %i curves")%shared_datapoints.curves.size());
    time_since_last_finish.start();
    shared_results.mode = Results::mode_finished_and_gracing;
    results_lens_to_solver.share();

    if (!use_solution) {
        return;
    }

    int best_star_flux_curve_index = -1;
    double best_star_flux_peak = -std::numeric_limits<double>::infinity();
    int brightest_blob_curve_index = -1;

    for (unsigned int i=0; i<shared_datapoints.curves.size(); i++) {
        double focus = 0.0;
        double value = 0.0;
        if (shared_datapoints.curves[i].get_peak(focus, value)) {
            if (shared_datapoints.curves[i].type == metric_brightest_blob_flux) {
                brightest_blob_curve_index = i;
                shared_datapoints.curves[i].log(logger);
            }
            if (shared_datapoints.curves[i].type == metric_star_flux) {
                if (value > best_star_flux_peak) {
                    best_star_flux_peak = value;
                    best_star_flux_curve_index = i;
                }
            }
        }
    }

    double best_focus = 0.0;
    double best_value = 0.0;
    bool best_focus_found = false;
    if (best_star_flux_curve_index > -1 && best_star_flux_curve_index < (signed int) shared_datapoints.curves.size())
    {
        shared_datapoints.curves[best_star_flux_curve_index].get_peak(best_focus, best_value);
        shared_datapoints.curves[best_star_flux_curve_index].log(logger);
        best_focus_found = true;
        logger.log(format("autofocuser found best focus %i from the best star_flux metric")%best_focus);
    }
    else if (brightest_blob_curve_index > -1 && brightest_blob_curve_index < (signed int) shared_datapoints.curves.size())
    {
        shared_datapoints.curves[brightest_blob_curve_index].get_peak(best_focus, best_value);
        best_focus_found = true;
        logger.log(format("autofocuser found best focus %i from brightest_blob metric")%best_focus);
    }
    if (best_focus_found) {
        shared_results.best_focus_known = true;
        shared_results.best_focus = int(round(best_focus));
        shared_results.current_focus_requested = shared_results.best_focus;
        results_lens_to_solver.share();
        make_focus_request();
        waiting_for_image_since_focus_change = false;
    }

}

void Autofocuser::update()
{

    // begin the run
    if (shared_requests.run_counter != run_counter) {
        logger.log("autofocuser beginning run");
        run_counter = shared_requests.run_counter;
        last_counter_stars = shared_latest_image.counter_stars;
        shared_results.mode = Results::mode_running;
        autofocus_run_time.start();
        shared_results.current_focus_requested = shared_requests.focus_search_min;
        shared_results.age_of_last_run.start();
        shared_results.best_focus_known = false;
        results_lens_to_solver.share();
        make_focus_request();

    }

    if (shared_results.mode == Results::mode_running) {

        // if the camera just received a new image
        if ((shared_latest_image.counter_stars != last_counter_stars) || (!waiting_for_image_since_focus_change)) {
            last_counter_stars = shared_latest_image.counter_stars;
            waiting_for_image_since_focus_change = false;

			// make sure we moved to the previous position
			if (shared_lens_results.focus_value == shared_results.current_focus_requested) {
				// take a focus step if we're not at the max focus
				if (shared_results.current_focus_requested < shared_requests.focus_search_max) {
					shared_results.current_focus_requested += shared_requests.focus_search_step;
					results_lens_to_solver.share();
					make_focus_request();
				}
			}
        }

        // end the run if we're at the max focus and we're not waiting for an image
        if (shared_results.current_focus_requested >= shared_requests.focus_search_max &&
            !waiting_for_image_since_focus_change)
        {
            if (!time_since_image_captured_at_last_focus.started()) {
                time_since_image_captured_at_last_focus.start();
            }

            // and either the solver finished with the latest image
            if (shared_datapoints.last_fully_solved_counter_stars >= last_counter_stars)
            {
                logger.log("autofocus: ending run because solver is finished with the latest image");
                end_run(true);
            }
            // or it's been 30 seconds since capturing an image at the last focus
            else if (time_since_image_captured_at_last_focus.time() > 30)
            {
                logger.log("autofocus: ending run because it's been 30 seconds since capturing an image at the last focus");
                end_run(true);
            }
        }
        else {
            time_since_image_captured_at_last_focus.reset();
        }

        if (autofocus_run_time.time() > 60.0*20.0) {
            end_run(false);
        }

    }

    if (shared_requests.abort_counter != abort_counter) {
        abort_counter = shared_requests.abort_counter;
        if (shared_results.mode == Results::mode_running) {
            end_run(shared_requests.abort_still_use_solution);
        }
    }

    // end the grace period
    if (shared_results.mode == Results::mode_finished_and_gracing) {
        if (time_since_last_finish.time() > 30) {
            shared_results.mode = Results::mode_not_running;
            results_lens_to_solver.share();
        }
    }

}

