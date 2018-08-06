/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "abstract_camera.h"
#ifdef _MSC_VER
    #ifndef __wtypes_h__
        #include <wtypes.h>
    #endif
#endif
#include <boost/format.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <fitsio.h>
#include <string.h>
#include <limits>
#include "../shared/update.h"
#include "../shared/image/raw.h"
#include "../shared/image/status.h"
#include "../shared/autofocus/latest_image.h"
#include "../shared/lens/requests.h"
#include "../shared/lens/results.h"
#include "../shared/solving/filters.h"
#include "../shared/network/client.h"
#include "../shared/camera/results.h"
#include "../shared/simulations/brightness.h"
#include "../shared/housekeeping/housekeeper.h"
#include "../parameters/manager.h"
#include "logger_camera.h"

#include "../tools/quick_cout.h"

using namespace Imaging;
using std::string;
using Cameraing::logger;

#define shared_filters (*(Shared::Solving::filters_main_to_camera.r))
#define shared_image (*(Shared::Image::raw_from_camera.w))
#define shared_autofocus (*(Shared::Autofocus::latest_image.w))
#define shared_client (*(Shared::Network::client_for_camera.r))
#define shared_results (*(Shared::Camera::results_for_main.w))
#define shared_fcp_lens_requests   (*(Shared::Lens::fcp_requests_camera_to_lens.w))
#define shared_fcp_lens_results    (*(Shared::Lens::fcp_results_lens_to_camera.r))
#define shared_stars_lens_requests (*(Shared::Lens::stars_requests_camera_to_lens.w))
#define shared_stars_lens_results  (*(Shared::Lens::stars_results_lens_to_camera.r))
#define shared_status (*(Shared::Image::status_main_for_camera.r))
#define shared_brightness (*(Shared::Simulations::brightness.r))
#define shared_housekeeper (*(Shared::Housekeeping::housekeeper_for_camera.r))

AbstractCamera::AbstractCamera(Parameters::Manager& params)
{
	image_width = params.general.try_get("imaging.camera_real.image_width", params.general.image_width);
	image_height = params.general.try_get("imaging.camera_real.image_height", params.general.image_height);

    enabled = params.general.try_get("imaging.camera.enabled", true);
    internal_triggering = params.general.try_get("imaging.camera_real.internal_triggering", false);
    internal_exposure_time = params.general.try_get("imaging.camera_real.internal_exposure_time", 0.120);
    internal_period = params.general.try_get("imaging.camera_real.internal_period", 20.0);
    output_dir = params.general.try_get("main.output_dir", string("C:\\stars_data\\output"));

    camera_ready = false;
    check_camera_ready_period = 120.0;
    max_exposure_time = 0.500;

    intermediate_buffer = new unsigned short[image_width*image_height];
    memset(intermediate_buffer, 0, sizeof(unsigned short)*image_width*image_height);

    for (unsigned int i=0; i<max_num_buffers; i++) {
        buffers[i] = new unsigned short[image_width*image_height];
        memset(buffers[i], 0, sizeof(unsigned short)*image_width*image_height);
    }

    last_lens_requests_focus_counter = -1;
}

AbstractCamera::~AbstractCamera()
{
    delete [] intermediate_buffer;
    for (unsigned int i=0; i<max_num_buffers; i++) {
        delete [] buffers[i];
    }
}

void AbstractCamera::update()
{
    Shared::update(Shared::ThreadNames::camera);
    logger.update();
}

void AbstractCamera::fill_real_camera_admin(boost::posix_time::ptime& timestamp, bool multiple_triggers)
{
    shared_image.from_camera = true;
    shared_image.dirname = (boost::format("%04d-%02d-%02d")
            % timestamp.date().year()
            % int(timestamp.date().month())
            % timestamp.date().day()
        ).str();
    shared_image.filename_base = (boost::format("%04d-%02d-%02d--%02d-%02d-%02d--%03d")
            % timestamp.date().year()
            % int(timestamp.date().month())
            % timestamp.date().day()
            % timestamp.time_of_day().hours()
            % timestamp.time_of_day().minutes()
            % timestamp.time_of_day().seconds()
            % int(timestamp.time_of_day().fractional_seconds()/1000.0)
        ).str();
    if (multiple_triggers) {
        shared_image.filename = shared_image.filename_base + "_pX.fits";
    }
    else {
        shared_image.filename = shared_image.filename_base + ".fits";
    }

    if (shared_fcp_lens_results.is_focus_valid(shared_fcp_lens_requests) &&
        shared_stars_lens_results.is_focus_valid(shared_stars_lens_requests))
    {
        shared_image.focus_known = true;
        shared_image.focus = shared_fcp_lens_results.focus_value;
    } else {
        shared_image.focus_known = false;
        shared_image.focus = -1;
    }
    if (shared_fcp_lens_results.is_aperture_valid(shared_fcp_lens_requests) &&
        shared_stars_lens_results.is_aperture_valid(shared_stars_lens_requests))
    {
        shared_image.aperture_known = true;
        shared_image.aperture = shared_fcp_lens_results.aperture_value;
    } else {
        shared_image.aperture_known = false;
        shared_image.aperture = -1;
    }
    if (shared_results.get_gain.found) {
        shared_image.gain_known = true;
        shared_image.gain = shared_results.get_gain.value;
    } else {
        shared_image.gain_known = false;
        shared_image.gain = -1.0;
    }
}

void AbstractCamera::fill_general_admin()
{
    using namespace Shared::Image;
    shared_image.filters = shared_filters;
	
    shared_image.counter_fcp = shared_client.counter_fcp;
    shared_image.counter_stars = shared_results.counter_stars;
    shared_results.counter_stars++;
	shared_image.to_be_solved = false;
	update();
	update();
    if (shared_status.stage == Status::empty || shared_status.stage == Status::done || 1) {
        shared_image.to_be_solved = true;
		logger.log("Image will be solved");
	}
	else {
		logger.log(format("Image will not be solved because stage is %d\n") % shared_status.stage );
	}
    
    shared_autofocus.counter_stars = shared_image.counter_stars;
    if (shared_brightness.allow_enable && shared_brightness.enabled) {
        logger.log("adding fake sky brightness to image");
        for (unsigned int i=0; i<shared_image.num_exposures; i++) {
            fake_sky.match_brightness(shared_image.separate_buffers[i],
                shared_image.width, shared_image.height, shared_image.single_depth);
        }
    }

    memset(shared_image.pixels, 0, sizeof(unsigned short)*shared_image.width*shared_image.height);
    for (unsigned int i=0; i<shared_image.num_exposures; i++) {
        for (int j=0; j<shared_image.width*shared_image.height; j++) {
            shared_image.pixels[j] += shared_image.separate_buffers[i][j];
        }
    }
    shared_image.depth = shared_image.single_depth*shared_image.num_exposures;
}

void AbstractCamera::share_image()
{
    Shared::Image::raw_from_camera.share();
    Shared::Camera::results_for_main.share();
    Shared::Autofocus::latest_image.share();
    *(Shared::Image::raw_for_image_client1.w) = *(Shared::Image::raw_from_camera.w);
    *(Shared::Image::raw_for_image_client2.w) = *(Shared::Image::raw_from_camera.w);
    Shared::Image::raw_for_image_client1.share();
    Shared::Image::raw_for_image_client2.share();
}

bool AbstractCamera::enough_space_to_save_images()
{
    double free_disk_space_gb = std::numeric_limits<double>::infinity();
    for (unsigned int i=0; i<shared_housekeeper.measurements.size(); i++) {
        if (shared_housekeeper.measurements[i].valid) {
            if (shared_housekeeper.measurements[i].name.compare("disk") == 0) {
                free_disk_space_gb = shared_housekeeper.measurements[i].value;
            }
        }
    }

    if (free_disk_space_gb < 0.300) {
        logger.log(format("warning: not enough disk space to save images (%f GB)") % free_disk_space_gb);
        return false;
    }
    return true;
}

void AbstractCamera::save_image(unsigned short* pixels, int buffer_num, bool multiple_triggers)
{
    using namespace Shared::Image;
    namespace fs = boost::filesystem;
    fitsfile *fptr;
    int status = 0;
    long naxis = 2;
    long naxes[2] = {shared_image.width, shared_image.height};
    int unknown = -1;
    if (!fs::exists(output_dir)) {
        output_dir = "D:/";
    }
    fs::path images_dir = fs::path(output_dir) / "images";
    if (!fs::exists(images_dir)) {
        fs::create_directory(images_dir);
    }
    fs::path dir = fs::path(output_dir) / "images" / shared_image.dirname;
    if (!fs::exists(dir)) {
        fs::create_directory(dir);
    }
    string filename = (dir / shared_image.filename).string();	//changed from file_string() to string() 
    if (multiple_triggers) {
/*
        string tail_filename = (boost::format("%s_p%i.fits.gz")
            %shared_image.filename_base
            %buffer_num).str();
*/
        string tail_filename = (boost::format("%s_p%i.fits")
            %shared_image.filename_base
            %buffer_num).str();

        filename = (dir / tail_filename).string();	//changed from file_string() to string()
    }
    logger.log(format("saving image %s") % filename);
    fits_create_file(&fptr, filename.c_str(), &status);
    fits_create_img(fptr, SHORT_IMG, naxis, naxes, &status);
    fits_update_key(fptr, TSTRING, (char *)"WHICH", (char *)shared_image.which_sensor.c_str(), (char *)"", &status);
    fits_update_key(fptr, TINT, (char *)"CT_STARS", &shared_image.counter_stars, (char *)"", &status);
    fits_update_key(fptr, TINT, (char *)"CT_MCP", &shared_image.counter_fcp, (char *)"", &status);
    if (shared_image.filters.horizontal_known()) {
        double lat = to_degrees(shared_image.filters.lat());
        double lst = to_hours(shared_image.filters.lst());
        fits_update_key(fptr, TDOUBLE, (char *)"APRX_LAT", &lat, (char *)"DEGREES", &status);
        fits_update_key(fptr, TDOUBLE, (char *)"APRX_LST", &lst, (char *)"HOURS", &status);
    } else {
        fits_update_key(fptr, TINT, (char *)"APRX_LAT", &unknown, (char *)"UNKNOWN", &status);
        fits_update_key(fptr, TINT, (char *)"APRX_LST", &unknown, (char *)"UNKNOWN", &status);
    }

    if (shared_image.focus_known) {
        fits_update_key(fptr, TINT, (char *)"FOCUS", &shared_image.focus, (char *)"ADU", &status);
    } else {
        fits_update_key(fptr, TINT, (char *)"FOCUS", &unknown, (char *)"UNKNOWN", &status);
    }
    if (shared_image.aperture_known) {
        fits_update_key(fptr, TINT, (char *)"APERTURE", &shared_image.aperture, (char *)"ADU", &status);
    }
    else {
        fits_update_key(fptr, TINT, (char *)"APERTURE", &unknown, (char *)"UNKNOWN", &status);
    }
    if (shared_image.gain_known) {
        float gain = float(shared_image.gain);
        fits_update_key(fptr, TFLOAT, (char *)"GAIN", &gain, (char *)"DB", &status);
    }
    else {
        fits_update_key(fptr, TDOUBLE, (char *)"GAIN", &unknown, (char *)"UNKNOWN", &status);
    }

    fits_write_img(fptr, TSHORT, 1, shared_image.width*shared_image.height, pixels, &status);
    fits_close_file(fptr, &status);

    logger.log("done saving image");
}

bool AbstractCamera::need_to_try_camera_ready()
{
    if (camera_ready) {
        return false;
    }
    else if (check_camera_ready_timer.time() > check_camera_ready_period) {
        check_camera_ready_timer.start();
        return true;
    }
    return false;
}

void AbstractCamera::wait_for_quit()
{
}

