/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "update.h"
#include "shared_list.h"

using namespace Shared;

void Shared::update(ThreadNames::Name thread_name)
{

    Autofocus::datapoints_solver_to_lens.retry_share(thread_name);
    Autofocus::datapoints_solver_to_lens.update_and_pass_to(

    Autofocus::datapoints_lens_to_main, thread_name);
    Autofocus::datapoints_lens_to_main.update(thread_name);

    Autofocus::latest_image.retry_share(thread_name);
    Autofocus::latest_image.update(thread_name);

    Autofocus::requests_network_to_main.retry_share(thread_name);
    Autofocus::requests_network_to_main.update_and_pass_to(

    Autofocus::requests_main_to_lens, thread_name);
    Autofocus::requests_main_to_lens.update(thread_name);

    Autofocus::results_lens_to_solver.retry_share(thread_name);
    Autofocus::results_lens_to_solver.update_and_pass_to(

    Autofocus::results_solver_to_main, thread_name);
    Autofocus::results_solver_to_main.update_and_pass_to(

    Autofocus::results_main_to_network, thread_name);
    Autofocus::results_main_to_network.update(thread_name);

    Camera::requests_for_main.retry_share(thread_name);
    Camera::requests_for_main.update_and_pass_to(

    Camera::requests_for_camera, thread_name);
    Camera::requests_for_camera.update(thread_name);

    Camera::results_for_main.retry_share(thread_name);
    Camera::results_for_main.update_and_pass_to(

    Camera::results_for_network, thread_name);
    Camera::results_for_network.update(thread_name);

    General::main_settings_net_for_main.retry_share(thread_name);
    General::main_settings_net_for_main.update_and_pass_to(

    General::main_settings_main_for_solver, thread_name);
    General::main_settings_main_for_solver.update(thread_name);

    General::shutdown_for_main.retry_share(thread_name);
    General::shutdown_for_main.update(thread_name);

    General::network_reset_for_net_reset.retry_share(thread_name);
    General::network_reset_for_net_reset.update(thread_name);

    General::network_reset_status_for_main.retry_share(thread_name);
    General::network_reset_status_for_main.update(thread_name);

    Housekeeping::housekeeper_for_camera.retry_share(thread_name);
    Housekeeping::housekeeper_for_camera.update_and_pass_to(

    Housekeeping::housekeeper_for_network, thread_name);
    Housekeeping::housekeeper_for_network.update(thread_name);

    Image::blobs_solver_for_main.retry_share(thread_name);
    Image::blobs_solver_for_main.update_and_pass_to(

    Image::blobs_main_for_net, thread_name);
    Image::blobs_main_for_net.update(thread_name);

    Image::leveled.retry_share(thread_name);
    Image::leveled.update(thread_name);

    Image::matching.retry_share(thread_name);
    Image::matching.update(thread_name);

    Image::matching_progress.retry_share(thread_name);
    Image::matching_progress.update(thread_name);

    Image::raw_from_camera.retry_share(thread_name);
    Image::raw_from_camera.update(thread_name);

    Image::raw_for_image_client1.retry_share(thread_name);
    Image::raw_for_image_client1.update(thread_name);

    Image::raw_for_image_client2.retry_share(thread_name);
    Image::raw_for_image_client2.update(thread_name);

    Image::solution_summary_for_main.retry_share(thread_name);
    Image::solution_summary_for_main.update_and_pass_to(

    Image::solution_summary_main_for_net, thread_name);
    Image::solution_summary_main_for_net.update(thread_name);

    Image::stats_solver_for_main.retry_share(thread_name);
    Image::stats_solver_for_main.update_and_pass_to(

    Image::stats_main_for_net, thread_name);
    Image::stats_main_for_net.update(thread_name);

    Image::status_solver_for_main.retry_share(thread_name);
    Image::status_solver_for_main.update_and_pass_to(

    Image::status_main_for_camera, thread_name);
    Image::status_main_for_camera.update_and_pass_to(

    Image::status_camera_for_network, thread_name);
    Image::status_camera_for_network.update(thread_name);

    Lens::fcp_requests_network_to_main.retry_share(thread_name);
    Lens::fcp_requests_network_to_main.update_and_pass_to(

    Lens::fcp_requests_main_to_camera, thread_name);
    Lens::fcp_requests_main_to_camera.update_and_pass_to(

    Lens::fcp_requests_camera_to_lens, thread_name);
    Lens::fcp_requests_camera_to_lens.update(thread_name);

    Lens::stars_requests_lens_to_main.retry_share(thread_name);
    Lens::stars_requests_lens_to_main.update_and_pass_to(

    Lens::stars_requests_main_to_camera, thread_name);
    Lens::stars_requests_main_to_camera.update_and_pass_to(

    Lens::stars_requests_camera_to_lens, thread_name);
    Lens::stars_requests_camera_to_lens.update(thread_name);

    Lens::fcp_results_lens_to_camera.retry_share(thread_name);
    Lens::fcp_results_lens_to_camera.update_and_pass_to(

    Lens::fcp_results_camera_to_main, thread_name);
    Lens::fcp_results_camera_to_main.update_and_pass_to(

    Lens::fcp_results_main_to_network, thread_name);
    Lens::fcp_results_main_to_network.update(thread_name);

    Lens::stars_results_lens_to_camera.retry_share(thread_name);
    Lens::stars_results_lens_to_camera.update_and_pass_to(

    Lens::stars_results_camera_to_main, thread_name);
    Lens::stars_results_camera_to_main.update(thread_name);

    Network::client_for_main.retry_share(thread_name);
    Network::client_for_main.update_and_pass_to(

    Network::client_for_camera, thread_name);
    Network::client_for_camera.update(thread_name);

    Network::packets_from_fc1.retry_share(thread_name);
    Network::packets_from_fc1.update(thread_name);

    Network::packets_from_fc2.retry_share(thread_name);
    Network::packets_from_fc2.update(thread_name);

    Network::image_client_settings1.retry_share(thread_name);
    Network::image_client_settings1.update(thread_name);

    Network::image_client_settings2.retry_share(thread_name);
    Network::image_client_settings2.update(thread_name);

    Simulations::brightness.retry_share(thread_name);
    Simulations::brightness.update(thread_name);

    Solving::filters_net_to_main.retry_share(thread_name);
    Solving::filters_net_to_main.update_and_pass_to(

    Solving::filters_main_to_camera, thread_name);
    Solving::filters_main_to_camera.update(thread_name);

    Solving::mask_network_for_solver.retry_share(thread_name);
    Solving::mask_network_for_solver.update_and_pass_to(

    Solving::mask_solver_for_main, thread_name);
    Solving::mask_solver_for_main.update(thread_name);

    Solving::settings.retry_share(thread_name);
    Solving::settings.update(thread_name);

}

