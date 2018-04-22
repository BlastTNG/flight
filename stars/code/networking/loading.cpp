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
                server_data.channels.hk_temp_lens = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("comp") == 0) {
                server_data.channels.hk_temp_comp = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("plate") == 0) {
                server_data.channels.hk_temp_plate = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("flange") == 0) {
                server_data.channels.hk_temp_flange = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("press") == 0) {
                server_data.channels.hk_pressure = shared_housekeeper.measurements[i].value;
            } else if (shared_housekeeper.measurements[i].name.compare("disk") == 0) {
                server_data.channels.hk_disk = shared_housekeeper.measurements[i].value;
            }
        }
    }
}

void Connection::load_server_data_camera_and_lens()
{
    server_data.channels.cam_gain_valid = 0;
    if (shared_camera_results.is_gain_valid(shared_camera_requests)) {
        server_data.channels.cam_gain_valid = 1;
    }
    server_data.channels.cam_gain_db = shared_camera_results.get_gain.value;
    server_data.channels.lens_focus = shared_lens_results.focus_value;
    server_data.channels.lens_aperture = shared_lens_results.aperture_value;
}

int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
	// Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
	// This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
	// until 00:00:00 January 1, 1970 
	static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

	SYSTEMTIME  system_time;
	FILETIME    file_time;
	uint64_t    time;

	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);
	time = ((uint64_t)file_time.dwLowDateTime);
	time += ((uint64_t)file_time.dwHighDateTime) << 32;

	tp->tv_sec = (long)((time - EPOCH) / 10000000L);
	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
	return 0;
}

void Connection::load_server_data_image()
{
    server_data.channels.image_ctr_stars = shared_image_status.counter_stars;
    server_data.channels.image_ctr_fcp = shared_image_status.counter_fcp;
    server_data.channels.image_num_exposures = shared_image_status.num_exposures;

    server_data.channels.image_stats_mean = UINT16_MAX;
    server_data.channels.image_stats_noise = UINT16_MAX;
    server_data.channels.image_stats_gaindb = -1.0;
    server_data.channels.image_stats_num_px_sat = -1;
    server_data.channels.image_stats_frac_px_sat = -1.0;
    server_data.channels.image_afocus_metric_valid = 0;
    server_data.channels.image_afocus_metric = 0.0;
    if (shared_image_stats.counter_stars == shared_image_status.counter_stars) {
        if (shared_image_stats.mean_known) {
            server_data.channels.image_stats_mean = shared_image_stats.mean;
            server_data.channels.image_stats_num_px_sat = shared_image_stats.num_pixels_saturated;
            server_data.channels.image_stats_frac_px_sat = shared_image_stats.fraction_pixels_saturated;
        }
        if (shared_image_stats.noise_known) {
            server_data.channels.image_stats_noise = shared_image_stats.noise;
        }
        if (shared_image_stats.gain_known) {
            server_data.channels.image_stats_gaindb = shared_image_stats.gaindb;
        }
        server_data.channels.image_afocus_metric_valid = shared_image_stats.autofocus_metric_valid;
        server_data.channels.image_afocus_metric = shared_image_stats.autofocus_metric;
    }

    server_data.channels.image_eq_valid = false;
    server_data.channels.image_hor_valid = false;
    if (shared_image_solution.counter_stars == shared_image_status.counter_stars) {
        server_data.channels.image_eq_valid = shared_image_solution.equatorial.valid;
        server_data.channels.image_eq_ra = shared_image_solution.equatorial.ra;
        server_data.channels.image_eq_dec = shared_image_solution.equatorial.dec;
        server_data.channels.image_eq_roll = shared_image_solution.equatorial.roll;
        server_data.channels.image_eq_sigma_ra = shared_image_solution.equatorial.sigma_ra;
        server_data.channels.image_eq_sigma_dec = shared_image_solution.equatorial.sigma_dec;
        server_data.channels.image_eq_sigma_roll = shared_image_solution.equatorial.sigma_roll;
        server_data.channels.image_eq_sigma_pointing = shared_image_solution.equatorial.sigma_pointing;
        server_data.channels.image_eq_iplate = shared_image_solution.equatorial.iplatescale;

        server_data.channels.image_hor_valid = shared_image_solution.horizontal.valid;
        server_data.channels.image_hor_az = shared_image_solution.horizontal.az;
        server_data.channels.image_hor_el = shared_image_solution.horizontal.el;
        server_data.channels.image_hor_roll = shared_image_solution.horizontal.roll;
        server_data.channels.image_hor_sigma_az = shared_image_solution.horizontal.sigma_az;
        server_data.channels.image_hor_sigma_el = shared_image_solution.horizontal.sigma_el;
        server_data.channels.image_hor_sigma_roll = shared_image_solution.horizontal.sigma_roll;
        server_data.channels.image_hor_sigma_pointing = shared_image_solution.horizontal.sigma_pointing;
        server_data.channels.image_hor_iplate = shared_image_solution.horizontal.iplatescale;

        server_data.channels.image_num_blobs_found = shared_image_solution.num_blobs_total;
        server_data.channels.image_num_blobs_matched = shared_image_solution.num_blobs_matched;

    }
    struct timeval tv = { 0 };
    gettimeofday(&tv, NULL);
    server_data.channels.timestamp_s = tv.tv_sec;
    server_data.channels.timestamp_us = tv.tv_usec;

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

    server_data.channels.ctr_stars = shared_camera_results.counter_stars;
    server_data.channels.stars_run_time = (unsigned int) logger.age->time();
    load_server_data_housekeeping();
    load_server_data_camera_and_lens();
    load_server_data_image();
}

