/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "connection.h"
#include "logger.h"
#include "../shared/update.h"
#include "../shared/camera/results.h"
#include "../shared/housekeeping/housekeeper.h"
#include "../shared/lens/results.h"
#include "../shared/image/blobs.h"
#include "../shared/image/stats.h"
#include "../shared/image/status.h"
#include "../shared/image/solution_summary.h"

#define shared_camera_results (*(Shared::Camera::results_for_network.r))
#define shared_camera_requests (*(Shared::Camera::requests_for_main.w))
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_network.r))
#define shared_lens_results (*(Shared::Lens::fcp_results_main_to_network.r))
#define shared_image_blobs (*(Shared::Image::blobs_main_for_net.r))
#define shared_image_stats (*(Shared::Image::stats_main_for_net.r))
#define shared_image_status (*(Shared::Image::status_camera_for_network.r))
#define shared_image_solution (*(Shared::Image::solution_summary_main_for_net.r))

using namespace Networking;

void Connection::load_server_data_housekeeping()
{
    for (unsigned int i=0; i<shared_housekeeper.measurements.size(); i++) {
        if (shared_housekeeper.measurements[i].valid) {
            if (shared_housekeeper.measurements[i].name.compare("lens") == 0) {
                server_data.channels[xN_hk_temp_lens].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("comp") == 0) {
                server_data.channels[xN_hk_temp_comp].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("vessel") == 0) {
                server_data.channels[xN_hk_temp_vessel].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("plate") == 0) {
                server_data.channels[xN_hk_temp_plate].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("dcdc") == 0) {
                server_data.channels[xN_hk_temp_dcdc].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("flange") == 0) {
                server_data.channels[xN_hk_temp_flange].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("press") == 0) {
                server_data.channels[xN_hk_pressure].value_double = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("disk") == 0) {
                server_data.channels[xN_hk_disk].value_double = shared_housekeeper.measurements[i].value;
            }
        }
    }
}

void Connection::load_server_data_camera_and_lens()
{
    server_data.channels[xN_cam_gain_valid].value_bool = false;
    if (shared_camera_results.is_gain_valid(shared_camera_requests)) {
        server_data.channels[xN_cam_gain_valid].value_bool = true;
    }
    server_data.channels[xN_cam_gain_db].value_double = shared_camera_results.get_gain.value;
    server_data.channels[xN_lens_focus].value_int = shared_lens_results.focus_value;
    server_data.channels[xN_lens_aperture].value_int = shared_lens_results.aperture_value;
}

void Connection::load_server_data_image()
{
    server_data.channels[xN_image_ctr_stars].value_int = shared_image_status.counter_stars;
    server_data.channels[xN_image_ctr_fcp].value_int = shared_image_status.counter_fcp;
    server_data.channels[xN_image_num_exposures].value_int = shared_image_status.num_exposures;

    server_data.channels[xN_image_stats_mean].value_double = -1.0;
    server_data.channels[xN_image_stats_noise].value_double = -1.0;
    server_data.channels[xN_image_stats_gaindb].value_double = -1.0;
    server_data.channels[xN_image_stats_num_px_sat].value_int = -1;
    server_data.channels[xN_image_stats_frac_px_sat].value_double = -1.0;
    server_data.channels[xN_image_afocus_metric_valid].value_bool = false;
    server_data.channels[xN_image_afocus_metric].value_bool = 0.0;
    if (shared_image_stats.counter_stars == shared_image_status.counter_stars) {
        if (shared_image_stats.mean_known) {
            server_data.channels[xN_image_stats_mean].value_double = shared_image_stats.mean;
            server_data.channels[xN_image_stats_num_px_sat].value_int = shared_image_stats.num_pixels_saturated;
            server_data.channels[xN_image_stats_frac_px_sat].value_double = shared_image_stats.fraction_pixels_saturated;
        }
        if (shared_image_stats.noise_known) {
            server_data.channels[xN_image_stats_noise].value_double = shared_image_stats.noise;
        }
        if (shared_image_stats.gain_known) {
            server_data.channels[xN_image_stats_gaindb].value_double = shared_image_stats.gaindb;
        }
        server_data.channels[xN_image_afocus_metric_valid].value_bool = shared_image_stats.autofocus_metric_valid;
        server_data.channels[xN_image_afocus_metric].value_double = shared_image_stats.autofocus_metric;
    }

    server_data.channels[xN_image_eq_valid].value_bool = false;
    server_data.channels[xN_image_hor_valid].value_bool = false;
    if (shared_image_solution.counter_stars == shared_image_status.counter_stars) {
        server_data.channels[xN_image_eq_valid].value_bool = shared_image_solution.equatorial.valid;
        server_data.channels[xN_image_eq_ra].value_double = shared_image_solution.equatorial.ra;
        server_data.channels[xN_image_eq_dec].value_double = shared_image_solution.equatorial.dec;
        server_data.channels[xN_image_eq_roll].value_double = shared_image_solution.equatorial.roll;
        server_data.channels[xN_image_eq_sigma_ra].value_double = shared_image_solution.equatorial.sigma_ra;
        server_data.channels[xN_image_eq_sigma_dec].value_double = shared_image_solution.equatorial.sigma_dec;
        server_data.channels[xN_image_eq_sigma_roll].value_double = shared_image_solution.equatorial.sigma_roll;
        server_data.channels[xN_image_eq_sigma_pointing].value_double = shared_image_solution.equatorial.sigma_pointing;
        server_data.channels[xN_image_eq_iplate].value_double = shared_image_solution.equatorial.iplatescale;

        server_data.channels[xN_image_hor_valid].value_bool = shared_image_solution.horizontal.valid;
        server_data.channels[xN_image_hor_az].value_double = shared_image_solution.horizontal.az;
        server_data.channels[xN_image_hor_el].value_double = shared_image_solution.horizontal.el;
        server_data.channels[xN_image_hor_roll].value_double = shared_image_solution.horizontal.roll;
        server_data.channels[xN_image_hor_sigma_az].value_double = shared_image_solution.horizontal.sigma_az;
        server_data.channels[xN_image_hor_sigma_el].value_double = shared_image_solution.horizontal.sigma_el;
        server_data.channels[xN_image_hor_sigma_roll].value_double = shared_image_solution.horizontal.sigma_roll;
        server_data.channels[xN_image_hor_sigma_pointing].value_double = shared_image_solution.horizontal.sigma_pointing;
        server_data.channels[xN_image_hor_iplate].value_double = shared_image_solution.horizontal.iplatescale;

        server_data.channels[xN_image_num_blobs_found].value_int = shared_image_solution.num_blobs_total;
        server_data.channels[xN_image_num_blobs_matched].value_int = shared_image_solution.num_blobs_matched;
    }

    server_data.blobs.counter_stars = shared_image_blobs.counter_stars;
    server_data.blobs.num_blobs = 0;
    for (unsigned int i=0; i < XSC_BLOBS_ARRAY_SIZE && i < shared_image_blobs.blobs.size(); i++) {
        server_data.blobs.blobs[i].x = shared_image_blobs.blobs[i].x;
        server_data.blobs.blobs[i].y = shared_image_blobs.blobs[i].y;
        server_data.blobs.blobs[i].flux = shared_image_blobs.blobs[i].flux;
        server_data.blobs.blobs[i].peak_to_flux = shared_image_blobs.blobs[i].peak_to_flux;
        server_data.blobs.num_blobs++;
    }
}

void Connection::load_server_data()
{
    Shared::update(Shared::ThreadNames::net_server);
    xsc_clear_server_data(&server_data);

    server_data.channels[xN_ctr_stars].value_int = shared_camera_results.counter_stars;
    server_data.channels[xN_stars_run_time].value_int = (unsigned int) logger.age->time();
    load_server_data_housekeeping();
    load_server_data_camera_and_lens();
    load_server_data_image();
}

