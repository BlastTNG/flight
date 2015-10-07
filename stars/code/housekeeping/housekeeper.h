/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef HOUSEKEEPING__HOUSEKEEPER_H
#define HOUSEKEEPING__HOUSEKEEPER_H

#include <boost/program_options.hpp>
#include "measurement.h"
#include "../tools/timing.h"
#include "../star_camera.h"
#include <string>

#if HAVEDAQ
#include "../dmm.h"
#endif

namespace Parameters
{
    class Manager;
}

namespace Housekeeping
{
    using namespace boost::program_options;
    class Housekeeper;
}

class Housekeeping::Housekeeper
{
  public:
    Housekeeper(Parameters::Manager& params, dmm *card);
    void add_channel(variables_map map, int channel_num);
    void update_shared();
    void get_disk();
    void update();

  private:
    Tools::Timer last_temps_measurement_timer;
    Tools::Timer last_disk_measurement_timer;
    Tools::Timer last_successful_disk_space_timer;
    Tools::Timer timer_since_first_disk_space_measurement;
    double disk_space;
    double disk_space_first_measurement;
    double disk_time;
    bool disk_time_valid;
    unsigned int write_temps_counter;
    unsigned int write_disk_counter;
    std::vector<Measurement> measurements;
	std::string output_dir;
	
	dmm *ad_card;
};

#endif
