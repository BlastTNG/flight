/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "general.h"
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include "../logger_main.h"

using namespace Parameters;
using namespace boost::program_options;
using Main::logger;

General::General(int width, int height, int depth, std::string stars_absolute_dir)
{
    using std::string;

    image_width = width;
    image_height = height;
    image_depth = depth;
    stars_dir = stars_absolute_dir;

    options.add_options()
        ("main.which_sensor", value<string>(), "")
        ("main.parse_custom_settings", value<bool>(), "")
        ("main.update_frequency", value<float>(), "")
        ("main.display_frequency", value<float>(), "")
        ("main.display_enabled", value<bool>(), "")
        ("main.display_fullscreen", value<bool>(), "")
        ("main.display_image_brightness", value<float>(), "")
        ("main.output_dir", value<string>(), "")
        ("main.network_reset_on_lull_enabled", value<bool>(), "")
        ("main.network_device_name", value<string>(), "")
        ("imaging.camera.device", value<string>(), "")
        ("imaging.camera.enabled", value<bool>(), "")
        ("imaging.camera_real.internal_triggering", value<bool>(), "")
        ("imaging.camera_real.internal_exposure_time", value<float>(), "")
        ("imaging.camera_real.internal_period", value<float>(), "")
        ("imaging.camera_filesystem.startup_delay", value<float>(), "")
        ("imaging.camera_filesystem.loading_period", value<float>(), "The time between files.")
        ("imaging.camera_filesystem.dirname", value<string>(), "")
        ("imaging.camera_filesystem.load_single_image", value<bool>(), "")
        ("imaging.camera_filesystem.single_image_filename", value<string>(), "")
        ("imaging.camera_filesystem.stack_parts", value<bool>(), "")
        ("imaging.camera_filesystem.flip_vertically", value<bool>(), "")
        ("imaging.camera_filesystem.repeat", value<bool>(), "")
        ("imaging.camera_filesystem.resave_images", value<bool>(), "")
        ("imaging.camera_filesystem.quit_after_one", value<bool>(), "")
        ("imaging.camera_filesystem.quit_after_one_delay", value<float>(), "")
        ("imaging.image_client.enabled", value<bool>(), "")
        ("imaging.lens.enabled", value<bool>(), "")
        ("imaging.lens.init_on_startup", value<bool>(), "")
        ("imaging.autofocus.focus_search_min", value<int>(), "")
        ("imaging.autofocus.focus_search_max", value<int>(), "")
        ("imaging.autofocus.focus_search_step", value<int>(), "")
        ("imaging.brightness_simulator.allow_enable", value<bool>(), "")
        ("imaging.brightness_simulator.enabled", value<bool>(), "")
        ("imaging.brightness_simulator.level_kepsa", value<float>(), "")
        ("imaging.brightness_simulator.gain_db", value<float>(), "")
        ("imaging.brightness_simulator.actual_exposure", value<float>(), "")
        ("imaging.brightness_simulator.simulated_exposure", value<float>(), "")
        ("imaging.selective_mask.enabled", value<bool>(), "")
        ("imaging.selective_mask.field0", value<unsigned int>(), "")
        ("imaging.selective_mask.field1", value<unsigned int>(), "")
        ("imaging.selective_mask.field2", value<unsigned int>(), "")
        ("solver.general.enabled", value<bool>(), "")
        ("solver.general.timeout", value<float>(), "")
        ("solver.general.debug_timing", value<bool>(), "")
        ("solver.general.saturation_limit", value<float>(), "")
        ("solver.blob_finder.snr_threshold", value<float>(), "")
        ("solver.blob_finder.max_num_blobs", value<int>(), "")
        ("solver.blob_finder.robust_mode_enabled", value<bool>(), "")
        ("solver.blob_finder.fitting_method", value<string>(), "")
        ("solver.blob_finder.cell_size", value<unsigned int>(), "")
        ("solver.blob_finder.max_num_blobs_per_cell", value<unsigned int>(), "")
		("solver.blob_finder.badpixfilename", value<string>(), "")
        ("solver.pattern_matcher.enabled", value<bool>(), "")
        ("solver.pattern_matcher.display_names", value<bool>(), "")
        ("solver.pattern_matcher.catalog", value<string>(), "")
        ("solver.pattern_matcher.match_tolerance_px", value<float>(), "")
        ("solver.pattern_matcher.iplatescale_min", value<float>(), "")
        ("solver.pattern_matcher.iplatescale_max", value<float>(), "")
        ("solver.pattern_matcher.platescale_always_fixed", value<bool>(), "")
        ("solver.pattern_matcher.iplatescale_fixed", value<float>(), "")
        ("solver.pattern_matcher.precession_from", value<string>(), "")
        ("solver.pattern_matcher.precession_manual_epoch", value<float>(), "")
        ("solver.refraction.enabled", value<bool>(), "")
        ("solver.refraction.pressure_mbar", value<float>(), "")
        ("solver.refraction.temperature", value<float>(), "")
        ("solver.filters.horizontal.location_limit.enabled", value<bool>(), "")
        ("solver.filters.horizontal.location_limit.radius_degrees", value<float>(), "")
        ("solver.filters.horizontal.location_limit.az_degrees", value<float>(), "")
        ("solver.filters.horizontal.location_limit.el_degrees", value<float>(), "")
        ("solver.filters.horizontal.roll_limit.enabled", value<bool>(), "")
        ("solver.filters.horizontal.roll_limit.min_degrees", value<float>(), "")
        ("solver.filters.horizontal.roll_limit.max_degrees", value<float>(), "")
        ("solver.filters.horizontal.elevation_limit.enabled", value<bool>(), "")
        ("solver.filters.horizontal.elevation_limit.min_degrees", value<float>(), "")
        ("solver.filters.horizontal.elevation_limit.max_degrees", value<float>(), "")
        ("solver.filters.equatorial.location_limit.enabled", value<bool>(), "")
        ("solver.filters.equatorial.location_limit.radius_degrees", value<float>(), "")
        ("solver.filters.equatorial.location_limit.ra_degrees", value<float>(), "")
        ("solver.filters.equatorial.location_limit.dec_degrees", value<float>(), "")
        ("solver.filters.equatorial.roll_limit.enabled", value<bool>(), "")
        ("solver.filters.equatorial.roll_limit.min_degrees", value<float>(), "")
        ("solver.filters.equatorial.roll_limit.max_degrees", value<float>(), "")
        ("solver.filters.horizontal_source.settings.enabled", value<bool>(), "")
        ("solver.filters.horizontal_source.settings.lat_degrees", value<float>(), "")
        ("solver.filters.horizontal_source.settings.lst_hours", value<float>(), "")
        ("solver.filters.horizontal_source.fits.enabled", value<bool>(), "")
        ("solver.filters.horizontal_source.fcp.enabled", value<bool>(), "")
        ("solver.filters.matching.pointing_error_threshold_arcsec", value<float>(), "")
        ("solver.filters.matching.fit_error_threshold_px", value<float>(), "")
        ("solver.filters.matching.num_matched", value<unsigned int>(), "")
    ;
    add_motion_psf();
    add_bypass_with_blobs();

}

void General::add_motion_psf()
{
    options.add_options()
        ("solver.blob_finder.motion_psf.enabled", value<bool>(), "")
        ("solver.blob_finder.motion_psf.summation_mode", value<bool>(), "")
        ("solver.blob_finder.motion_psf.num_timesteps", value<unsigned int>(), "")
        ("solver.blob_finder.motion_psf.hroll_deg", value<float>(), "")
        ("solver.blob_finder.motion_psf.iplatescale_arcsec", value<float>(), "")
    ;
    for (unsigned int i=0; i<max_exposure_time_cs; i++) {
        options.add_options()
            ((boost::format("solver.blob_finder.motion_psf.exposure_num_t%d") %i).str().c_str(), value<int>(), "")
            ((boost::format("solver.blob_finder.motion_psf.gy_az_deg_t%d")        %i).str().c_str(), value<float>(), "")
            ((boost::format("solver.blob_finder.motion_psf.gy_el_deg_t%d")        %i).str().c_str(), value<float>(), "")
        ;
    }
}

void General::add_bypass_with_blobs()
{
    options.add_options()
        ("solver.blob_finder.bypass_with_blobs.enabled", value<bool>(), "")
        ("solver.blob_finder.bypass_with_blobs.num_blobs", value<unsigned int>(), "")
    ;
    for (unsigned int i=0; i<max_num_bypass_blobs; i++) {
        options.add_options()
            ((boost::format("solver.blob_finder.bypass_with_blobs.blob%d.x") %i).str().c_str(), value<float>(), "")
            ((boost::format("solver.blob_finder.bypass_with_blobs.blob%d.y") %i).str().c_str(), value<float>(), "")
        ;
    }
}

void General::load(int argc, char* argv[])
{
    using namespace boost::filesystem;
    bool parse_custom_settings = true;

    try {
        store(parse_command_line(argc, argv, options,
            command_line_style::default_style & ~command_line_style::allow_guessing), map);
    } catch(std::exception& e) {
        logger.log(format("Caught exception parsing command line options: %s") % e.what());
    }

    notify(map);
    if (map.count("main.parse_custom_settings")) {
        if (!map["main.parse_custom_settings"].as<bool>()) {
            parse_custom_settings = false;
        }
    }

    if (parse_custom_settings) {
        std::ifstream infile0(system_complete(stars_dir + "settings/custom.txt").string().c_str());
        try {
            store(parse_config_file(infile0, options), map);
        } catch(std::exception& e) {
            logger.log(format("Caught exception parsing config file: %s") % e.what());
        }
        infile0.close();
    }

    std::ifstream infile1(system_complete(stars_dir + "settings/flight.txt").string().c_str());
    try {
        store(parse_config_file(infile1, options), map);
    } catch(std::exception& e) {
        logger.log(format("Caught exception parsing config file: %s") % e.what());
    }
    infile1.close();

    notify(map);

}

