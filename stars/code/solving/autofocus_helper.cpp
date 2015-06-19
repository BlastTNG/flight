/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "autofocus_helper.h"
#include "blob.h"
#include "solution.h"
#include "../shared/image/raw.h"
#include "../shared/image/stats.h"
#include "../shared/autofocus/datapoints.h"
#include "../shared/autofocus/results.h"
#include "../tools/timing.h"
#include "../tools/correlate.h"
#include "../parameters/manager.h"

using namespace Solving;
using std::vector;
#define shared_datapoints (*(Shared::Autofocus::datapoints_solver_to_lens.w))
#define shared_stats (*(Shared::Image::stats_solver_for_main.w))
#define shared_results (*(Shared::Autofocus::results_lens_to_solver.r))

AutofocusDatapoint::AutofocusDatapoint()
{
    type = Shared::Autofocus::metric_brightest_blob_flux;
    star_id = 0;
    focus = 0.0;
    value = 0.0;
}

AutofocusDatapoint& AutofocusDatapoint::operator=(const AutofocusDatapoint &rhs)
{
    if (this != &rhs) {
        type = rhs.type;
        star_id = rhs.star_id;
        focus = rhs.focus;
        value = rhs.value;
        age = rhs.age;
    }
    return *this;
}

AutofocusHelper::AutofocusHelper(Parameters::Manager& params)
{
    datapoints.clear();
    image_width = params.general.image_width;
    image_height = params.general.image_height;
    scratch = new double[image_width*image_height];
}

AutofocusHelper::~AutofocusHelper()
{
    delete [] scratch;
}

void AutofocusHelper::update_and_share()
{
    using namespace Shared::Autofocus;
    double max_age = Time::from_minutes(30.0);


    // only keep data points that are younger than the last autofocus run and younger than max_age
    std::vector<AutofocusDatapoint> temp_datapoints = datapoints;
    datapoints.clear();
    for (unsigned int i=0; i<temp_datapoints.size(); i++) {
        if (temp_datapoints[i].age.time() < max_age &&
            temp_datapoints[i].age.time() < shared_results.age_of_last_run.time())
        {
            datapoints.push_back(temp_datapoints[i]);
        }
    }
    sort(datapoints.begin(), datapoints.end(),
        AutofocusDatapoint::sort_by_focus);

    shared_datapoints.curves.clear();
    for (unsigned int i=0; i<datapoints.size(); i++) {
        datapoints_t type = datapoints[i].type;
        int star_id = datapoints[i].star_id;
        double focus = datapoints[i].focus;
        double value = datapoints[i].value;
        Tools::Timer age = datapoints[i].age;

        bool curve_found = false;
        for (unsigned int j=0; j<shared_datapoints.curves.size(); j++) {
            if (shared_datapoints.curves[j].type == type &&
                (shared_datapoints.curves[j].type == metric_brightest_blob_flux ||
                 shared_datapoints.curves[j].star_id == star_id))
            {
                curve_found = true;
                shared_datapoints.curves[j].points.push_back(CurvePoint(focus, value));
                if (age.time() < shared_datapoints.curves[j].age_since_last_datapoint_added.time()) {
                    shared_datapoints.curves[j].age_since_last_datapoint_added = age;
                }
            }
        }
        if (!curve_found) {
            Curve curve;
            curve.type = type;
            curve.star_id = star_id;
            curve.points.push_back(CurvePoint(focus, value));
            curve.age_since_last_datapoint_added = age;
            shared_datapoints.curves.push_back(curve);
        }
    }
    sort(shared_datapoints.curves.begin(), shared_datapoints.curves.end(),
        Curve::sort_by_type);

    datapoints_solver_to_lens.share();
}

void AutofocusHelper::extract_sobel_datapoint(Shared::Image::Raw& image)
{
    double sum_x = 0.0;
    double kernel_x[9] = {-1.0, 0.0, 1.0, -2.0, 0.0, 2.0, -1.0, 0.0, 1.0};
    Tools::correlate2d(image.pixels, kernel_x, scratch, 1, 1, image_width, image_height);
    for (int j=0; j<image_height; j++) {
        for (int i=0; i<image_width; i++) {
            sum_x += scratch[j*image.width+i];
        }
    }
    sum_x = fabs(sum_x);
    /*
    // only do the x kernel because there can be significant variations across rows due to readout
    double sum_y = 0.0; 
    double kernel_y[9] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0};
    Tools::correlate2d(image.pixels, kernel_y, scratch, 1, 1, image_width, image_height);
    for (int j=0; j<image_height; j++) {
        for (int i=0; i<image_width; i++) {
            sum_y += scratch[j*image.width+i];
        }
    }
    sum_y = fabs(sum_y);
    */
    AutofocusDatapoint datapoint;
    datapoint.type = Shared::Autofocus::metric_sobel;
    datapoint.focus = image.focus;
    datapoint.value = sum_x;
    datapoint.age = image.age;
    if (shared_results.mode == Shared::Autofocus::Results::mode_running) {
        datapoints.push_back(datapoint);
        update_and_share();
    }
}

void AutofocusHelper::extract_brightest_blob_datapoint(Shared::Image::Raw& image, vector<Blob>& blobs)
{
    if (image.focus_known) {
        double peak = 0.0;
        bool at_least_one_blob = false;
        for (unsigned int i=0; i<blobs.size(); i++) {
            if (!blobs[i].saturated) {
                at_least_one_blob = true;
                if (blobs[i].peak > peak) {
                    peak = blobs[i].peak;
                }
            }
        }
        if (at_least_one_blob) {
            AutofocusDatapoint datapoint;
            datapoint.type = Shared::Autofocus::metric_brightest_blob_flux;
            datapoint.focus = image.focus;
            datapoint.value = peak;
            datapoint.age = image.age;
            if (shared_results.mode == Shared::Autofocus::Results::mode_running) {
                datapoints.push_back(datapoint);
                update_and_share();
                shared_stats.autofocus_metric_valid = true;
                shared_stats.autofocus_metric = datapoint.value;
                Shared::Image::stats_solver_for_main.share();
            }
        }
    }
}

void AutofocusHelper::extract_star_datapoints(Shared::Image::Raw& image, Solution& solution)
{
    if (image.focus_known) {
        bool at_least_one_star = false;
        for (unsigned int i=0; i<solution.matched_stars.size() && i<solution.matched_blobs.size(); i++) {
            at_least_one_star = true;
            AutofocusDatapoint datapoint;
            datapoint.type = Shared::Autofocus::metric_star_flux;
            datapoint.star_id = solution.matched_stars[i].id;
            datapoint.focus = image.focus;
            datapoint.value = solution.matched_blobs[i].peak;
            datapoint.age = image.age;
            if (shared_results.mode == Shared::Autofocus::Results::mode_running) {
                datapoints.push_back(datapoint);
            }
        }
        if (at_least_one_star) {
            if (shared_results.mode == Shared::Autofocus::Results::mode_running) {
                update_and_share();
            }
        }
    }
}

void AutofocusHelper::set_fully_solved(int counter_stars)
{
    shared_datapoints.last_fully_solved_counter_stars = counter_stars;
    Shared::Autofocus::datapoints_solver_to_lens.share();
}

