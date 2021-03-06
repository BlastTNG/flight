/**
 * @file xsc_protocol.h
 *
 * @date Nov 23, 2012
 * @author chappy
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2013 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#pragma once
#ifndef INCLUDE_XSC_PROTOCOL_H
#define INCLUDE_XSC_PROTOCOL_H

#include <stdint.h>
#ifdef _MSC_VER
    #ifndef __cplusplus
        typedef int bool;
        #define true 1
        #define false 0
    #endif
#else
    #include <stdbool.h>
#endif

#define XSC_BLOBS_ARRAY_SIZE 20

#pragma pack(push, 4)

// Server Structs //

typedef struct XSCImageBlob
{
    double x;
    double y;
    double flux;
    double peak_to_flux;
} XSCImageBlob;

typedef struct XSCImageBlobs
{
    int counter_stars;
    int num_blobs;
    XSCImageBlob blobs[XSC_BLOBS_ARRAY_SIZE];
} XSCImageBlobs;


typedef struct
{
    int32_t image_ctr_stars;
    int32_t image_ctr_mcp;
    int32_t ctr_stars;

    uint32_t stars_run_time;

    float hk_temp_lens;
    float hk_temp_comp;
    float hk_temp_plate;
    float hk_temp_flange;
    float hk_pressure;
    uint32_t hk_disk;

    uint8_t cam_gain_valid;
    float   cam_gain_db;

    uint16_t lens_focus;
    uint16_t lens_aperture;

    uint16_t image_num_exposures;
    uint16_t image_stats_mean;
    uint16_t image_stats_noise;
    double image_stats_gaindb;
    int32_t image_stats_num_px_sat;
    double image_stats_frac_px_sat;
    double image_afocus_metric;

    uint8_t image_afocus_metric_valid;
    uint8_t image_eq_valid;
    uint8_t image_hor_valid;

    double image_eq_ra;
    double image_eq_dec;
    double image_eq_roll;
    double image_eq_sigma_ra;
    double image_eq_sigma_dec;
    double image_eq_sigma_roll;
    double image_eq_sigma_pointing;
    double image_eq_iplate;

    double image_hor_az;
    double image_hor_el;
    double image_hor_roll;
    double image_hor_sigma_az;
    double image_hor_sigma_el;
    double image_hor_sigma_roll;
    double image_hor_sigma_pointing;
    double image_hor_iplate;
    uint16_t image_num_blobs_found;
    uint16_t image_num_blobs_matched;

    uint32_t timestamp_s;
    uint32_t timestamp_us;
} xsc_channels_t;

typedef struct XSCServerData
{
    xsc_channels_t channels;
    XSCImageBlobs blobs;
    unsigned int xsc_protocol_version;
}
XSCServerData;


// XSC enum types //

typedef enum
{
    xC_solver_fitting_method_none = 0,
    xC_solver_fitting_method_gaussian = 1,
    xC_solver_fitting_method_double_gaussian = 2,
}
xsc_solver_fitting_method_t;

typedef enum
{
    xC_autofocus_display_mode_auto,
    xC_autofocus_display_mode_on,
    xC_autofocus_display_mode_off,
}
xsc_autofocus_display_mode_t;


// Client Structs //

enum
{
    xC_quit,
    xC_shutdown,
    xC_network_reset,
    xC_main_settings,
    xC_display_zoom,
    xC_image_client,
    xC_multi_triggering,
    xC_flush_birger,
    xC_init_focus,
    xC_get_focus,
    xC_set_focus,
    xC_stop_focus,
    xC_set_focus_incremental,
    xC_define_focus,
    xC_define_aperture,
    xC_run_autofocus,
    xC_set_autofocus_range,
    xC_abort_autofocus,
    xC_autofocus_display_mode,
    xC_init_aperture,
    xC_get_aperture,
    xC_set_aperture,
    xC_stop_aperture,
    xC_get_gain,
    xC_set_gain,
    xC_brightness,
    xC_solver_general,
    xC_solver_abort,
    xC_solver_mask,
    xC_solver_blob_finder,
    xC_solver_blob_cells,
    xC_solver_pattern_matcher,
    xC_solver_filter_hor_location,
    xC_solver_filter_hor_roll,
    xC_solver_filter_hor_el,
    xC_solver_filter_eq_location,
    xC_solver_filter_matching,

    xC_num_command_admins
};

typedef struct
{
    unsigned int is_new_countdown;
    int counter;
}
xsc_command_admin_t;

typedef struct XSCFilters
{
    bool   hor_location_limit_enabled;
    double hor_location_limit_radius;
    double hor_location_limit_az;
    double hor_location_limit_el;

    bool   hor_roll_limit_enabled;
    double hor_roll_limit_min;
    double hor_roll_limit_max;

    bool   hor_el_limit_enabled;
    double hor_el_limit_min;
    double hor_el_limit_max;

    bool   eq_location_limit_enabled;
    double eq_location_limit_radius;
    double eq_location_limit_ra;
    double eq_location_limit_dec;

    double matching_pointing_error_threshold;
    double matching_fit_error_threshold_px;
    unsigned int matching_num_matched;
} XSCFilters;

typedef struct XSCSolverMask
{
    bool enabled;
    unsigned int field0;
    unsigned int field1;
    unsigned int field2;
} XSCSolverMask;

typedef struct XSCSolver
{
    bool enabled;
    double timeout;
    XSCSolverMask mask;

    double snr_threshold;
    int max_num_blobs;
    bool robust_mode_enabled;
    xsc_solver_fitting_method_t fitting_method;
    unsigned int cell_size;
    unsigned int max_num_blobs_per_cell;

    bool pattern_matcher_enabled;
    bool display_star_names;
    double match_tolerance_px;
    double iplatescale_min;
    double iplatescale_max;
    bool platescale_always_fixed;
    double iplatescale_fixed;

    XSCFilters filters;
}
XSCSolver;

typedef struct XSCBrightness
{
    int counter;
    bool enabled;
    double level_kepsa;
    double gain_db;
    double actual_exposure;
    double simulated_exposure;
} XSCBrightness;

typedef struct XSCShutdown
{
    int counter;
    bool shutdown_now;
    bool include_restart;
} XSCShutdown;

typedef struct XSCNetworkReset
{
    bool reset_now;
    bool reset_on_lull_enabled;
    double reset_on_lull_delay;
}
XSCNetworkReset;

typedef struct XSCMainSettings
{
    int counter;
    double display_frequency;
    bool display_fullscreen;
    bool display_image_only;
    bool display_solving_filters;
    double display_image_brightness;
    int display_zoom_x;
    int display_zoom_y;
    double display_zoom_magnitude;
}
XSCMainSettings;

typedef struct XSCHorizontal
{
    bool valid;
    double lat;
    double lst;
} XSCHorizontal;

typedef struct XSCClientData
{
    bool in_charge;
    int counter_mcp;

    int quit_counter;
    XSCShutdown       shutdown;
    XSCNetworkReset   network_reset;
    XSCMainSettings   main_settings;
    bool image_client_enabled;

    double multi_triggering_readout_delay;

    int set_focus_value;
    int set_focus_incremental_value;
    int define_focus_value;
    int autofocus_search_min;
    int autofocus_search_max;
    int autofocus_search_step;
    bool abort_autofocus_still_use_solution;
    xsc_autofocus_display_mode_t autofocus_display_mode;
    int set_aperture_value;
    double set_gain_value;
    int define_aperture_value;

    XSCBrightness     brightness;
    XSCSolver         solver;
    XSCHorizontal     horizontal;

    xsc_command_admin_t command_admins[xC_num_command_admins];

    unsigned int xsc_protocol_version;

    unsigned int heater_state;
}
XSCClientData;

void xsc_clear_server_data(XSCServerData* server_data);
void xsc_clear_client_data(XSCClientData* client_data);
void xsc_decrement_is_new_countdowns(XSCClientData* client_data);
void xsc_zero_command_admins(xsc_command_admin_t* admins);
void xsc_init_server_data(XSCServerData* server_data);

#pragma pack(pop)


#endif
