/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__CAMERA_FILESYSTEM_H
#define IMAGING__CAMERA_FILESYSTEM_H

#include "abstract_camera.h"
#include <string>
#include <vector>
#include <fitsio.h>
#include <boost/thread/thread.hpp>
#include <boost/filesystem/path.hpp>
#include "../tools/timing.h"

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    struct Fileset;
    class CameraFilesystem;
    namespace fs = boost::filesystem;
}

class Imaging::CameraFilesystem: public Imaging::AbstractCamera
{
  public:
    CameraFilesystem(Parameters::Manager& params);
    ~CameraFilesystem();
    void add_fileset(fs::path path);
    void try_add_fileset(fs::path path);
    void build_filename_list();
    bool init_camera();
    void clean_up_camera();
    void resave_image(std::string filename);
    void convert_according_to_comment(double& x, std::string comment);
    void read_keys(std::string full_filename);
    bool add_pixels(std::string full_filename, unsigned int exposure_num);
    void read_image_if_available();
    void thread_function();
    void wait_for_quit();

  private:

    std::string dirname;
    bool load_single_image;
    std::string single_image_filename;
    bool stack_parts;
    std::vector<Fileset>* filesets;
    int fileset_index;
    bool resave_images;
    std::string output_dir;

    bool flip_vertically;

    unsigned short* temp_pixels;

    bool first_run;
    bool repeat;
    double startup_delay;
    Tools::Timer startup_timer;
    double loading_period;

    boost::thread thread;
};

#endif
