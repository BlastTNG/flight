/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "camera_filesystem.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "../parameters/manager.h"
#include "../shared/image/raw.h"
#include "../shared/general/quit.h"
#include "logger_camera.h"
#include "../tools/quick_cout.h"

using namespace Imaging;
using Cameraing::logger;
using std::string;

#define shared_image (*(Shared::Image::raw_from_camera.w))


struct Imaging::Fileset
{
    fs::path parent;
    string stem_base;
    string extension;
    std::vector<string> parts;
};

static bool fileset_sort (Fileset fileset0, Fileset fileset1)
{
    return (fileset0.stem_base < fileset1.stem_base);
}

CameraFilesystem::CameraFilesystem(Parameters::Manager& params): AbstractCamera(params),
    dirname(params.general.try_get("imaging.camera_filesystem.dirname", string("../images"))),
    load_single_image(params.general.try_get("imaging.camera_filesystem.load_single_image", false)),
    single_image_filename(params.general.try_get("imaging.camera_filesystem.single_image_filename", string("test.fits"))),
    stack_parts(params.general.try_get("imaging.camera_filesystem.stack_parts", true)),
    filesets(new std::vector<Fileset>),
    fileset_index(0),
    resave_images(params.general.try_get("imaging.camera_filesystem.resave_images", false)),
    output_dir(params.general.try_get("main.output_dir", string("D:\\stars_data\\output"))),		
    flip_vertically(params.general.try_get("imaging.camera_filesystem.flip_vertically", false)),
    temp_pixels(new unsigned short[shared_image.width*shared_image.height]),
    first_run(true),
    repeat(params.general.try_get("imaging.camera_filesystem.repeat", false)),
    startup_delay(params.general.try_get("imaging.camera_filesystem.startup_delay", 1.5)),
    loading_period(params.general.try_get("imaging.camera_filesystem.loading_period", 10.0)),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    thread(boost::bind(&CameraFilesystem::thread_function, this))
    #pragma warning(pop)
{
}

CameraFilesystem::~CameraFilesystem()
{
    delete filesets;
    delete [] temp_pixels;
}

void CameraFilesystem::add_fileset(fs::path path)
{
    Fileset fileset;

    if (boost::algorithm::ends_with(path.string(), ".fits") ||
        boost::algorithm::ends_with(path.string(), ".fgz"))
    {
        fileset.parent = path.parent_path();
		fileset.stem_base = path.stem().string();
			fileset.extension = path.extension().string();
    }
    else if (boost::algorithm::ends_with(path.string(), ".fits.gz"))
    {
        fileset.parent = path.parent_path();
        fs::path temp_path = path.stem();
		fileset.stem_base = temp_path.stem().string();
        fileset.extension = ".fits.gz";
    }

    bool added_to_existing_fileset = false;
    if (stack_parts) {
        string::size_type part_location = fileset.stem_base.find("_p", 0);
        if (part_location != string::npos) {
            string::size_type extension_location = fileset.stem_base.find(fileset.extension, part_location);
            fileset.parts.push_back(fileset.stem_base.substr(part_location+2, extension_location));
            fileset.stem_base = fileset.stem_base.substr(0, part_location);
        }
        for (unsigned int i=0; i<filesets->size(); i++) {
            if (fileset.parent == (*filesets)[i].parent && fileset.stem_base == (*filesets)[i].stem_base) {
                if (fileset.parts.size() > 0) {
                    (*filesets)[i].parts.push_back(fileset.parts[0]);
                    sort((*filesets)[i].parts.begin(), (*filesets)[i].parts.end());
                }
                added_to_existing_fileset = true;
            }
        }
    }

    if (!added_to_existing_fileset) {
        (*filesets).push_back(fileset);
    }
}

void CameraFilesystem::try_add_fileset(fs::path path)
{
    try{
        if (fs::is_regular_file(path)) {
            if (boost::algorithm::ends_with(path.string(), ".fits") ||
                boost::algorithm::ends_with(path.string(), ".fits.gz") ||
                boost::algorithm::ends_with(path.string(), ".fgz"))
            {
                add_fileset(path);
            }
        }
    }
    catch (const std::exception & ex) {
        logger.log(format("file loader: exception on %s: %s") % path % ex.what());
    }
}

void CameraFilesystem::build_filename_list()
{
    filesets->clear();
    fs::path fullpath( fs::initial_path<fs::path>() );
    fullpath = fs::system_complete(dirname);
    if (!fs::exists(fullpath)) {
        logger.log(format("file loader: warning: %s does not exist") % fullpath);
    }
    else if (!fs::is_directory(fullpath)) {
        logger.log(format("file loader: warning: %s is not a directory") % fullpath);
    }
    else if (fs::is_directory(fullpath)) {

        if (load_single_image) {
            fs::path path = fullpath / fs::path(single_image_filename);
            try_add_fileset(path);
        } else {
            fs::directory_iterator end_iter;
            for(fs::directory_iterator dir_iter(fullpath); dir_iter != end_iter; ++dir_iter){
                try_add_fileset(dir_iter->path());
            }
        }
    }
    sort((*filesets).begin(), (*filesets).end(), fileset_sort);
    logger.log(format("camera_filesystem found %i filesets") % filesets->size());
}

bool CameraFilesystem::init_camera()
{
    return false;
}

void CameraFilesystem::clean_up_camera()
{
}

void CameraFilesystem::resave_image(string filename)
{
    Tools::Timer timer;
    timer.start();
    fitsfile *fptr;
    int status = 0;
    long naxis = 2;
    long naxes[2] = {shared_image.width, shared_image.height};
    if (!fs::exists(output_dir)) {
        output_dir = "";
    }
    fs::path dir = fs::path(output_dir) / "resaved_images";
    if (!fs::exists(dir)) {
        fs::create_directory(dir);
    }
    string outfilename = (dir / filename).string();	//changed from file_string by KNS
    //string outfilename = (dir / "test1.fits.gz").file_string();
    fits_create_file(&fptr, outfilename.c_str(), &status);
    fits_create_img(fptr, SHORT_IMG, naxis, naxes, &status);
    fits_write_img(fptr, TSHORT, 1, shared_image.width*shared_image.height, shared_image.pixels, &status);
    fits_close_file(fptr, &status);
    logger.log(format("file loader: done saving %s, took %f s") % outfilename % timer.time());
}

void CameraFilesystem::convert_according_to_comment(double& x, string comment)
{
    boost::to_lower(comment);
    if (comment.find("hours") != string::npos) {
        x = from_hours(x);
    }
    if (comment.find("degrees") != string::npos) {
        x = from_degrees(x);
    }
}

void CameraFilesystem::read_keys(string full_filename)
{
    int status = -1;
    int status2 = -1;
    int int_value = -1;
    double lat = -1;
    double lst = -1;
    char comment[256];
    char comment2[256];
    fitsfile *file;

    if (fits_open_file(&file, full_filename.c_str(), READONLY, &status)) {
        logger.log(format("file loader: warning: could not open %s") % full_filename);
        return;
    }

    status = -1;
    fits_read_key(file, TINT, "FOCUS", &int_value, NULL, &status);
    if (status == -1) {
        shared_image.focus_known = true;
        shared_image.focus = int_value;
    }

    status = -1;
    fits_read_key(file, TINT, "APERTURE", &int_value, NULL, &status);
    if (status == -1) {
        shared_image.aperture_known = true;
        shared_image.aperture = int_value;
    }

    status = -1;
    fits_read_key(file, TINT, "COUNTER_STARS", &int_value, NULL, &status);
    if (status == -1) {
        shared_image.key_counter_stars = int_value;
    }

    status = -1;
    fits_read_key(file, TINT, "COUNTER_FCP", &int_value, NULL, &status);
    if (status == -1) {
        shared_image.counter_fcp = int_value;
    }

    status = -1;
    fits_read_key(file, TINT, "FRAMENUM", &int_value, NULL, &status);
    if (status == -1) {
        shared_image.has_netisc_framenum = true;
        shared_image.netisc_framenum = int_value;
    }

    if (shared_image.filters.horizontal_from_fits.enabled) {
        shared_image.filters.horizontal_from_fits.valid = false;

        status = -1;
        fits_read_key(file, TDOUBLE, "LAT", &lat, comment, &status);
        status2 = -1;
        fits_read_key(file, TDOUBLE, "LST", &lst, comment2, &status2);
        if (status==-1 && status2==-1) {
            convert_according_to_comment(lat, string(comment));
            convert_according_to_comment(lst, "hours");
            shared_image.filters.horizontal_from_fits.lst = lst;
            shared_image.filters.horizontal_from_fits.lat = lat;
            shared_image.filters.horizontal_from_fits.valid = true;
        }
        status = -1;
        fits_read_key(file, TDOUBLE, "APPROX_LAT", &lat, comment, &status);
        status2 = -1;
        fits_read_key(file, TDOUBLE, "APPROX_LST", &lst, comment2, &status2);
        if (status==-1 && status2==-1) {
            convert_according_to_comment(lat, string(comment));
            convert_according_to_comment(lst, "hours");
            shared_image.filters.horizontal_from_fits.lst = lst;
            shared_image.filters.horizontal_from_fits.lat = lat;
            shared_image.filters.horizontal_from_fits.valid = true;
        }
    }

    fits_close_file(file, &status);
}

bool CameraFilesystem::add_pixels(string full_filename, unsigned int exposure_num)
{
    bool success = false;
    fitsfile *file;
    int status = 0;
    int hdu_type, num_axes;
    long dimensions[2] = {0, 0};
    bool failed_to_open = false;

    if (fits_open_file(&file, full_filename.c_str(), READONLY, &status)) {
        logger.log(format("file loader: warning: could not open %s") % full_filename);
        failed_to_open = true;
    }
    else if (fits_get_hdu_type(file, &hdu_type, &status)) {
        logger.log(format("file loader: warning: couldn't get the type of HDU for %s") % full_filename);
    }
    else if (hdu_type != IMAGE_HDU) {
        logger.log(format("file loader: warning: %s is not an image type") % full_filename);
    }
    else if (fits_get_img_dim(file, &num_axes, &status)) {
        logger.log(format("file loader: warning: couuld not get num_axes of %s") % full_filename);
    }
    else if (num_axes != 2) {
        logger.log(format("file loader: warning: %s does not have 2 axes") % full_filename);
    }
    else if (fits_get_img_size(file, 2, dimensions, &status)) {
        logger.log(format("file loader: warning: could not get dimensions of %s") % full_filename);
    }
    else if (num_axes != 2) {
        logger.log(format("file loader: warning: %s does not have 2 axes") % full_filename);
    }
    else if ((dimensions[0] != shared_image.width) ||  (dimensions[1] != shared_image.height)){
        logger.log("file loader: warning: dimensions do not match those of first image in set");
    }
    else{
        long start[2] = {1, 1};
        fits_read_pix(file, TUSHORT, start, dimensions[0]*dimensions[1],
            NULL, temp_pixels, NULL, &status);
        for (int j=0; j<dimensions[1]; j++) {
            for (int i=0; i<dimensions[0]; i++) {
                int k = (j) * dimensions[0] + i;
                if (flip_vertically) {
                    int kp = (dimensions[1]-j-1) * dimensions[0] + i;
                    //shared_image.pixels[kp] += temp_pixels[k];
                    shared_image.separate_buffers[exposure_num][kp] = temp_pixels[k];
                } else {
                    //shared_image.pixels[k] += temp_pixels[k];
                    shared_image.separate_buffers[exposure_num][k] = temp_pixels[k];
                }
            }
        }
        success = true;
    }
    if (!failed_to_open) {
        fits_close_file(file, &status);
    }
    return success;
}

void CameraFilesystem::read_image_if_available()
{
    string filename, full_filename, first_full_filename;

    if ((*filesets).size() > 0 && (first_run || repeat)) {
        Fileset* fileset = &((*filesets)[fileset_index]);

        unsigned int num_exposures = 0;
        //memset(shared_image.pixels, 0, sizeof(unsigned short)*shared_image.width*shared_image.height);
        if (fileset->parts.size() > 0) {
            filename = fileset->stem_base + "_pX" + fileset->extension;
            for (unsigned int i=0; i < fileset->parts.size() && num_exposures < shared_image.max_num_exposures; i++)
            {
                full_filename = (fileset->parent / (fileset->stem_base + "_p" + fileset->parts[i] + fileset->extension)).string();
                if (i==0) {
                    first_full_filename = full_filename;
                }
                if(add_pixels(full_filename, num_exposures)) {
                    num_exposures++;
                }
            }
        } else {
            filename = fileset->stem_base + fileset->extension;
            full_filename = (fileset->parent / (fileset->stem_base + fileset->extension)).string();
            first_full_filename = full_filename;
            if(add_pixels(full_filename, 0)) {
                num_exposures++;
            }
        }

        if (num_exposures > 0) {
            shared_image.age.start();
            logger.log(format("loaded image %s")%filename);
            shared_image.from_camera = false;
            shared_image.num_exposures = num_exposures;

            shared_image.filename = filename;
            std::vector<string> filename_parts;
            boost::split(filename_parts, filename, boost::is_any_of("_."));
            shared_image.filename_base = filename_parts[0];

            fill_general_admin();
            read_keys(first_full_filename);
            share_image();
            if (resave_images && enough_space_to_save_images()) {
                logger.log(format("resaving image %s")%filename);
                resave_image(filename);
            }
        }

        if (fileset_index == (int) ((*filesets).size()-1))
        {
            first_run = false;
        }
        fileset_index = (fileset_index+1) % (*filesets).size();
    }
}

void CameraFilesystem::thread_function()
{
    startup_timer.start();
    build_filename_list();
    Tools::Timer age_of_last_read_attempt;
    while (!Shared::General::quit) {
        update();
        if (enabled) {
            if ((age_of_last_read_attempt.time() > loading_period) &&
                (startup_timer.time() > startup_delay))
            {
                age_of_last_read_attempt.start();
                logger.log("camera_filesystem about to read_image_if_available");
                read_image_if_available();
            }
        }
        usleep(50000);
    }
    clean_up_camera();
}

void CameraFilesystem::wait_for_quit()
{
    thread.join();
}

