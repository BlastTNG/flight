/**
 * @file xsc_protocol.c
 *
 * @date Nov 23, 2012
 * @author chappy
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2012 Columbia University
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "xsc_protocol.h"
#include <string.h>
#include <stdbool.h>

#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(push)
#pragma warning(disable:4996)
#endif

#define PI 3.141592653589793

extern int16_t InCharge;

void xsc_clear_horizontal(XSCHorizontal* horizontal)
{
    horizontal->valid = false;
    horizontal->lat = 0.0;
    horizontal->lst = 0.0;
}

void xsc_clear_image_blob(XSCImageBlob* blob)
{
    blob->x = 0.0;
    blob->y = 0.0;
    blob->flux = 0.0;
    blob->peak_to_flux = 0.0;
}

void xsc_clear_image_blobs(XSCImageBlobs* blobs)
{
    unsigned int i = 0;
    blobs->counter_stars = -1;
    blobs->num_blobs = 0;
    for (i = 0; i < XSC_BLOBS_ARRAY_SIZE; i++) {
        xsc_clear_image_blob(&blobs->blobs[i]);
    }
}

void xsc_clear_server_data(XSCServerData* server_data)
{
    xsc_clear_image_blobs(&server_data->blobs);
    server_data->xsc_protocol_version = 0;
}



void xsc_clear_filters(XSCFilters* filters)
{
    filters->hor_location_limit_enabled = true;
    filters->hor_location_limit_radius = 3;
    filters->hor_location_limit_az = 0;
    filters->hor_location_limit_el = 0;

    filters->hor_roll_limit_enabled = true;
    filters->hor_roll_limit_min = -PI/2.0;
    filters->hor_roll_limit_max = PI/2.0;

    filters->hor_el_limit_enabled = true;
    filters->hor_el_limit_min = 0.0;
    filters->hor_el_limit_max = PI/4.0;

    filters->eq_location_limit_enabled = false;
    filters->eq_location_limit_radius = 30;
    filters->eq_location_limit_ra = 0;
    filters->eq_location_limit_dec = 0;

    filters->matching_pointing_error_threshold = (10.0 / 3600.0) * (PI / 180.0);
    filters->matching_fit_error_threshold_px = 2000.0;
    filters->matching_num_matched = 4;
}

void xsc_clear_solver_mask(XSCSolverMask* mask)
{
    mask->enabled = false;
    mask->field0 = 0x00000000;
    mask->field1 = 0x00000000;
    mask->field2 = 0x00000000;
}

void xsc_clear_brightness(XSCBrightness* brightness)
{
    brightness->counter = 0;
    brightness->enabled = true;
    brightness->level_kepsa = 0.0;
    brightness->gain_db = 0.0;
    brightness->actual_exposure = 0.120;
    brightness->simulated_exposure = 0.120;
}

void xsc_clear_shutdown(XSCShutdown* shutdown)
{
    shutdown->counter = 0;
    shutdown->shutdown_now = false;
    shutdown->include_restart = false;
}

void xsc_clear_network_reset(XSCNetworkReset* network_reset)
{
    network_reset->reset_now = false;
    network_reset->reset_on_lull_enabled = true;
    network_reset->reset_on_lull_delay = 2.0*3600.0;
}

void xsc_clear_main_settings(XSCMainSettings* main_settings)
{
    main_settings->counter = 0;
    main_settings->display_frequency = 10.0;
    main_settings->display_fullscreen = true;
    main_settings->display_image_only = false;
    main_settings->display_solving_filters = false;
    main_settings->display_image_brightness = 1.0;
    main_settings->display_zoom_x = 320;
    main_settings->display_zoom_y = 240;
    main_settings->display_zoom_magnitude = 1.0;
}

void xsc_clear_solver(XSCSolver* solver)
{
    solver->enabled = true;
    solver->timeout = 30.0;
    xsc_clear_solver_mask(&solver->mask);

    solver->snr_threshold = 1.0;
    solver->max_num_blobs = 10;
    solver->robust_mode_enabled = false;
    solver->fitting_method = xC_solver_fitting_method_none;
    solver->cell_size = 128;
    solver->max_num_blobs_per_cell = 2;

    solver->pattern_matcher_enabled = true;
    solver->display_star_names = false;
    solver->match_tolerance_px = 6.0;
    solver->iplatescale_min = 4.5572E-5;
    solver->iplatescale_max = 4.6542E-5;
    solver->platescale_always_fixed = false;
    solver->iplatescale_fixed = 4.60573E-5;

    xsc_clear_filters(&solver->filters);
}

void xsc_clear_client_data(XSCClientData* client_data)
{
    client_data->in_charge = false;
    client_data->counter_mcp = -1;

    client_data->quit_counter = 0;
    xsc_clear_shutdown(&client_data->shutdown);
    xsc_clear_network_reset(&client_data->network_reset);
    xsc_clear_main_settings(&client_data->main_settings);
    client_data->image_client_enabled = true;

    client_data->multi_triggering_readout_delay = 1.0;

    client_data->set_focus_value = 0;
    client_data->set_focus_incremental_value = 0;
    client_data->define_focus_value = 0;
    client_data->autofocus_search_min = 500;
    client_data->autofocus_search_max = 2000;
    client_data->autofocus_search_step = 10;
    client_data->abort_autofocus_still_use_solution = false;
    client_data->autofocus_display_mode = xC_autofocus_display_mode_auto;
    client_data->set_aperture_value = 0;
    client_data->set_gain_value = 0.0;
    client_data->define_aperture_value = 0;

    xsc_clear_brightness(&client_data->brightness);

    xsc_clear_solver(&client_data->solver);

    xsc_clear_horizontal(&client_data->horizontal);

    client_data->xsc_protocol_version = 0;
}

void xsc_decrement_is_new_countdowns(XSCClientData* client_data)
{
    for (int i = 0; i < xC_num_command_admins; i++) {
        if (client_data->command_admins[i].is_new_countdown) client_data->command_admins[i].is_new_countdown--;
    }
}

void xsc_zero_command_admins(xsc_command_admin_t* admins)
{
    unsigned int i = 0;
    for (i = 0; i < xC_num_command_admins; i++) {
        admins[i].is_new_countdown = 0;
        admins[i].counter = 0;
    }
}

void xsc_init_server_data(XSCServerData* server_data)
{
    memset(server_data, 0, sizeof(XSCServerData));
    server_data->channels.ctr_stars = -1;
    server_data->channels.image_ctr_mcp = -1;
    server_data->channels.image_ctr_stars = -1;
}


#if defined(_MSC_VER) && _MSC_VER >= 1400
#pragma warning(pop)
#endif
