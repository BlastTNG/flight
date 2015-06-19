/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "results.h"

using namespace Shared::Lens;

Results::Results()
{
    device_name = "";
    device_found = false;
    focus_value = 0;
    focus_found = false;
    aperture_value = 0;
    aperture_found = false;
    for (unsigned int i=0; i<num_commands; i++) {
        command_counters[i] = 0;
    }
    max_exposure_and_readout_time = 0.500;
}

Results& Results::operator=(const Results &rhs)
{
    if (this != &rhs) {
        device_name = rhs.device_name;
        device_found = rhs.device_found;
        focus_value = rhs.focus_value;
        focus_found = rhs.focus_found;
        aperture_value = rhs.aperture_value;
        aperture_found = rhs.aperture_found;
        for (unsigned int i=0; i<num_commands; i++) {
            command_counters[i] = rhs.command_counters[i];
        }
        max_exposure_and_readout_time = rhs.max_exposure_and_readout_time;
    }
    return *this;
}

bool Results::is_focus_valid(Requests& requests)
{
    if (!focus_found) {
        return false;
    }
    using namespace Imaging::LensCommands;
    if (command_counters[init_focus] == requests.commands[init_focus].counter &&
        command_counters[get_focus] == requests.commands[get_focus].counter &&
        command_counters[set_focus] == requests.commands[set_focus].counter &&
        command_counters[set_focus_incremental] == requests.commands[set_focus_incremental].counter)
    {
        return true;
    }
    return false;
}

bool Results::is_aperture_valid(Requests& requests)
{
    if (!aperture_found) {
        return false;
    }
    using namespace Imaging::LensCommands;
    if (command_counters[init_aperture] == requests.commands[init_aperture].counter &&
        command_counters[get_aperture] == requests.commands[get_aperture].counter &&
        command_counters[set_aperture] == requests.commands[set_aperture].counter)
    {
        return true;
    }
    return false;
}

