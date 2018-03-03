/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "connection.h"
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "logger.h"
#include "../tools/angles.h"
#include "../shared/lens/requests.h"
#include "../shared/lens/results.h"
#include "../imaging/commands.h"
#include "../shared/solving/filters.h"
#include "../shared/solving/settings.h"
#include "../shared/solving/mask.h"
#include "../shared/network/packets.h"
#include "../shared/network/client.h"
#include "../shared/network/image_client_settings.h"
#include "../shared/camera/requests.h"
#include "../shared/camera/results.h"
#include "../shared/housekeeping/housekeeper.h"
#include "../shared/autofocus/requests.h"
#include "../shared/simulations/brightness.h"
#include "../shared/general/main_settings.h"
#include "../shared/general/shutdown.h"
#include "../shared/general/network_reset.h"
#include "../shared/general/quit.h"
#include "../tools/quick_cout.h"

#define shared_lens_requests (*(Shared::Lens::fcp_requests_network_to_main.w))
#define shared_lens_results (*(Shared::Lens::fcp_results_main_to_network.r))
#define shared_packets (*(Shared::Network::packets.w))
#define shared_image_client1_settings (*(Shared::Network::image_client_settings1.w))
#define shared_image_client2_settings (*(Shared::Network::image_client_settings2.w))
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper.r))
#define shared_camera_requests (*(Shared::Camera::requests_for_main.w))
#define shared_camera_results (*(Shared::Camera::results_for_network.r))
#define shared_filters (*(Shared::Solving::filters_net_to_main.w))
#define shared_solving_settings (*(Shared::Solving::settings.w))
#define shared_mask (*(Shared::Solving::mask_network_for_solver.w))
#define shared_autofocus_requests (*(Shared::Autofocus::requests_network_to_main.w))
#define shared_brightness (*(Shared::Simulations::brightness.w))
#define shared_shutdown (*(Shared::General::shutdown_for_main.w))
#define shared_network_reset (*(Shared::General::network_reset_for_net_reset.w))
#define shared_main_settings (*(Shared::General::main_settings_net_for_main.w))
#define shared_client (*(Shared::Network::client_for_main.w))

using namespace Networking;

bool Connection::check_command(unsigned int command_index)
{
    if (command_index < xC_num_command_admins) {
        if (client_data.command_admins[command_index].is_new_countdown > 0) {
            if (local_command_admins[command_index].counter != client_data.command_admins[command_index].counter)
            {
                local_command_admins[command_index].counter = client_data.command_admins[command_index].counter;
                logger.log(format("network unloading: received command %i") % command_index);
                return true;
            }
        }
    }
    else {
        logger.log(format("warning: tried to access command (%u) outside of array (size %u)")
            % command_index % (unsigned int) xC_num_command_admins);
    }
    return false;
}

void Connection::unload_client_data_lens_and_camera()
{
    using namespace Imaging::LensCommands;
    bool passed_something = false;

    passed_something = false;
    if (check_command(xC_flush_birger)) {
        shared_lens_requests.commands[flush_birger].counter = client_data.command_admins[xC_flush_birger].counter;
        passed_something = true;
    }
    if (check_command(xC_init_focus)) {
        shared_lens_requests.commands[init_focus].counter = client_data.command_admins[xC_init_focus].counter;
        passed_something = true;
    }
    if (check_command(xC_get_focus)) {
        shared_lens_requests.commands[get_focus].counter = client_data.command_admins[xC_get_focus].counter;
        passed_something = true;
    }
    if (check_command(xC_set_focus)) {
        shared_lens_requests.commands[set_focus].counter = client_data.command_admins[xC_set_focus].counter;
        shared_lens_requests.commands[set_focus].value = client_data.set_focus_value;
        passed_something = true;
    }
    if (check_command(xC_set_focus_incremental)) {
        shared_lens_requests.commands[set_focus_incremental].counter = client_data.command_admins[xC_set_focus_incremental].counter;
        shared_lens_requests.commands[set_focus_incremental].value = client_data.set_focus_incremental_value;
        passed_something = true;
    }
    if (check_command(xC_define_focus)) {
        shared_lens_requests.commands[define_focus].counter = client_data.command_admins[xC_define_focus].counter;
        shared_lens_requests.commands[define_focus].value = client_data.define_focus_value;
        passed_something = true;
    }
    if (check_command(xC_init_aperture)) {
        shared_lens_requests.commands[init_aperture].counter = client_data.command_admins[xC_init_aperture].counter;
        passed_something = true;
    }
    if (check_command(xC_get_aperture)) {
        shared_lens_requests.commands[get_aperture].counter = client_data.command_admins[xC_get_aperture].counter;
        passed_something = true;
    }
    if (check_command(xC_set_aperture)) {
        shared_lens_requests.commands[set_aperture].counter = client_data.command_admins[xC_set_aperture].counter;
        shared_lens_requests.commands[set_aperture].value = client_data.set_aperture_value;
        passed_something = true;
    }
    if (check_command(xC_define_aperture)) {
        shared_lens_requests.commands[define_aperture].counter = client_data.command_admins[xC_define_aperture].counter;
        shared_lens_requests.commands[define_aperture].value = client_data.define_aperture_value;
        passed_something = true;
    }
    if (passed_something) {
        Shared::Lens::fcp_requests_network_to_main.share();
    }

    passed_something = false;
    if (check_command(xC_get_gain)) {
        shared_camera_requests.get_gain.counter = client_data.command_admins[xC_get_gain].counter;
        passed_something = true;
    }
    if (check_command(xC_set_gain)) {
        shared_camera_requests.set_gain.counter = client_data.command_admins[xC_set_gain].counter;
        shared_camera_requests.set_gain.value = client_data.set_gain_value;
        passed_something = true;
    }
    if (check_command(xC_multi_triggering)) {
        shared_camera_requests.multi_triggering_delay = client_data.multi_triggering_readout_delay;
        passed_something = true;
    }
    if (passed_something) {
        Shared::Camera::requests_for_main.share();
    }
}

void Connection::unload_client_data_autofocus()
{
    bool passed_something = false;
    if (check_command(xC_run_autofocus)) {
        passed_something = true;
        shared_autofocus_requests.run_counter = client_data.command_admins[xC_run_autofocus].counter;
    }
    if (check_command(xC_set_autofocus_range)) {
        passed_something = true;
        shared_autofocus_requests.focus_search_min = client_data.autofocus_search_min;
        shared_autofocus_requests.focus_search_max = client_data.autofocus_search_max;
        shared_autofocus_requests.focus_search_step = client_data.autofocus_search_step;
    }
    if (check_command(xC_abort_autofocus)) {
        passed_something = true;
        shared_autofocus_requests.abort_counter = client_data.command_admins[xC_abort_autofocus].counter;
        shared_autofocus_requests.abort_still_use_solution = client_data.abort_autofocus_still_use_solution;
    }
    if (check_command(xC_autofocus_display_mode)) {
        passed_something = true;
        shared_autofocus_requests.display_mode = client_data.autofocus_display_mode;
    }
    if (passed_something) {
        Shared::Autofocus::requests_network_to_main.share();
    }
}

void Connection::unload_client_data_solver()
{
    bool share_solving_settings = false;
    if (check_command(xC_solver_general)) {
        shared_solving_settings.enabled = client_data.solver.enabled;
        shared_solving_settings.timeout = client_data.solver.timeout;
        share_solving_settings = true;
    }
    if (check_command(xC_solver_abort)) {
        shared_solving_settings.abort_counter = client_data.command_admins[xC_solver_abort].counter;
        share_solving_settings = true;
    }
    if (check_command(xC_solver_mask)) {
        shared_mask.enabled = client_data.solver.mask.enabled;
        shared_mask.fields[0] = client_data.solver.mask.field0;
        shared_mask.fields[1] = client_data.solver.mask.field1;
        shared_mask.fields[2] = client_data.solver.mask.field2;
        Shared::Solving::mask_network_for_solver.share();
    }
    if (check_command(xC_solver_blob_finder)) {
        shared_solving_settings.snr_threshold = client_data.solver.snr_threshold;
        shared_solving_settings.max_num_blobs = client_data.solver.max_num_blobs;
        shared_solving_settings.robust_mode_enabled = client_data.solver.robust_mode_enabled;
        shared_solving_settings.fitting_method = client_data.solver.fitting_method;
        share_solving_settings = true;
    }
    if (check_command(xC_solver_blob_cells)) {
        shared_solving_settings.cell_size = client_data.solver.cell_size;
        shared_solving_settings.max_num_blobs_per_cell = client_data.solver.max_num_blobs_per_cell;
        share_solving_settings = true;
    }
    if (check_command(xC_solver_pattern_matcher)) {
        shared_solving_settings.pattern_matcher_enabled = client_data.solver.pattern_matcher_enabled;
        shared_solving_settings.display_names = client_data.solver.display_star_names;
        shared_solving_settings.match_tolerance_px = client_data.solver.match_tolerance_px;
        shared_solving_settings.iplatescale_min = client_data.solver.iplatescale_min;
        shared_solving_settings.iplatescale_max = client_data.solver.iplatescale_max;
        shared_solving_settings.platescale_always_fixed = client_data.solver.platescale_always_fixed;
        shared_solving_settings.iplatescale_fixed = client_data.solver.iplatescale_fixed;
        share_solving_settings = true;
    }
    if (share_solving_settings) {
        Shared::Solving::settings.share();
    }
}

void Connection::unload_client_data_solver_filters()
{
    bool passed_something = false;

    if (client_data.horizontal.valid) {
        shared_filters.horizontal_from_fcp.age.start();
        shared_filters.horizontal_from_fcp.lat = client_data.horizontal.lat;
        shared_filters.horizontal_from_fcp.lst = client_data.horizontal.lst;
        passed_something = true;
    }

    if (check_command(xC_solver_filter_hor_location)) {
        shared_filters.horizontal_location.enabled = client_data.solver.filters.hor_location_limit_enabled;
        shared_filters.horizontal_location.radius = client_data.solver.filters.hor_location_limit_radius;
        passed_something = true;
    }
    if (shared_filters.horizontal_location.enabled) {
        shared_filters.horizontal_location.az = client_data.solver.filters.hor_location_limit_az;
        shared_filters.horizontal_location.el = client_data.solver.filters.hor_location_limit_el;
        passed_something = true;
    }

    if (check_command(xC_solver_filter_hor_roll)) {
        shared_filters.horizontal_roll_limit.enabled = client_data.solver.filters.hor_roll_limit_enabled;
        shared_filters.horizontal_roll_limit.min_roll = client_data.solver.filters.hor_roll_limit_min;
        shared_filters.horizontal_roll_limit.max_roll = client_data.solver.filters.hor_roll_limit_max;
        passed_something = true;
    }

    if (check_command(xC_solver_filter_hor_el)) {
        shared_filters.horizontal_elevation_limit.enabled = client_data.solver.filters.hor_el_limit_enabled;
        shared_filters.horizontal_elevation_limit.min_el = client_data.solver.filters.hor_el_limit_min;
        shared_filters.horizontal_elevation_limit.max_el = client_data.solver.filters.hor_el_limit_max;
        passed_something = true;
    }

    if (check_command(xC_solver_filter_eq_location)) {
        shared_filters.equatorial_location.enabled = client_data.solver.filters.eq_location_limit_enabled;
        shared_filters.equatorial_location.radius = client_data.solver.filters.eq_location_limit_radius;
        passed_something = true;
    }
    if (shared_filters.equatorial_location.enabled) {
        shared_filters.equatorial_location.ra = client_data.solver.filters.eq_location_limit_ra;
        shared_filters.equatorial_location.dec = client_data.solver.filters.eq_location_limit_dec;
        passed_something = true;
    }

    if (check_command(xC_solver_filter_matching)) {
        shared_filters.matching.pointing_error_threshold = client_data.solver.filters.matching_pointing_error_threshold;
        shared_filters.matching.fit_error_threshold_px = client_data.solver.filters.matching_fit_error_threshold_px;
        shared_filters.matching.num_matched = client_data.solver.filters.matching_num_matched;
        passed_something = true;
    }

    if (passed_something) {
        Shared::Solving::filters_net_to_main.share();
    }
}

void Connection::unload_main_and_display_settings()
{
    bool passed_something = false;
    if (check_command(xC_main_settings)) {
        shared_main_settings.display_period = 1.0 / client_data.main_settings.display_frequency;
        shared_main_settings.display_fullscreen = client_data.main_settings.display_fullscreen;
        shared_main_settings.display_fullscreen_counter++;
        shared_main_settings.display_image_only = client_data.main_settings.display_image_only;
        shared_main_settings.display_solving_filters = client_data.main_settings.display_solving_filters;
        shared_main_settings.display_image_brightness = client_data.main_settings.display_image_brightness;
        passed_something = true;
    }
    if (check_command(xC_display_zoom)) {
        shared_main_settings.display_zoom_x = client_data.main_settings.display_zoom_x;
        shared_main_settings.display_zoom_y = client_data.main_settings.display_zoom_y;
        shared_main_settings.display_zoom_magnitude = client_data.main_settings.display_zoom_magnitude;
        passed_something = true;
    }
    if (passed_something) {
        Shared::General::main_settings_net_for_main.share();
    }
}

void Connection::unload_client_data()
{
    if (check_command(xC_quit)) {
        Shared::General::quit = true;
    }

    if (shared_client.counter_fcp != client_data.counter_fcp) {
        shared_client.counter_fcp = client_data.counter_fcp;
        Shared::Network::client_for_main.share();
    }

    if (check_command(xC_shutdown)) {
        shared_shutdown.shutdown_now = client_data.shutdown.shutdown_now;
        shared_shutdown.include_restart = client_data.shutdown.include_restart;
        Shared::General::shutdown_for_main.share();
    }

    unload_main_and_display_settings();
    unload_client_data_lens_and_camera();
    unload_client_data_autofocus();

    if (check_command(xC_image_client)) {
        shared_image_client1_settings.enabled = client_data.image_client_enabled;
        shared_image_client2_settings.enabled = client_data.image_client_enabled;
        Shared::Network::image_client_settings1.share();
        Shared::Network::image_client_settings2.share();
    }

    if (check_command(xC_brightness)) {
        shared_brightness = client_data.brightness;
        Shared::Simulations::brightness.share();
    }

    unload_client_data_solver();
    unload_client_data_solver_filters();

    if (check_command(xC_network_reset)) {
        if (client_data.network_reset.reset_now) {
            shared_network_reset.reset_now_counter++;
        }
        shared_network_reset.reset_on_lull_enabled = client_data.network_reset.reset_on_lull_enabled;
        shared_network_reset.reset_on_lull_delay = client_data.network_reset.reset_on_lull_delay;
        Shared::General::network_reset_for_net_reset.share();
    }

}

