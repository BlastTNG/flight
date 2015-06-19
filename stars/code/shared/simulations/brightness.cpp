/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "brightness.h"
#include "../../tools/math.h"
#include "../../parameters/manager.h"
extern "C" {
#include "../../networking/xsc_protocol/xsc_protocol.h"
}

using namespace Shared::Simulations;

Brightness::Brightness()
{
    counter = 0;
    allow_enable = false;
    enabled = false;
    level_kepsa = 0.0;
    gain_db = 0.0;
    actual_exposure = 0.1;
    simulated_exposure = 0.1;
}

void Brightness::init(Parameters::Manager& params)
{
    allow_enable = params.general.try_get("imaging.brightness_simulator.allow_enable", false);
    enabled = params.general.try_get("imaging.brightness_simulator.enabled", false);
    level_kepsa = params.general.try_get("imaging.brightness_simulator.level_kepsa", 0.0);
    actual_exposure = params.general.try_get("imaging.brightness_simulator.actual_exposure", 0.1);
    simulated_exposure = params.general.try_get("imaging.brightness_simulator.simulated_exposure", 0.1);
    gain_db = params.general.try_get("imaging.brightness_simulator.gain_db", 0.0);
}

Brightness& Brightness::operator=(const Brightness &rhs)
{
    if (this != &rhs) {
        counter = rhs.counter;
        allow_enable = rhs.allow_enable;
        enabled = rhs.enabled;
        level_kepsa = rhs.level_kepsa;
        gain_db = rhs.gain_db;
        actual_exposure = rhs.actual_exposure;
        simulated_exposure = rhs.simulated_exposure;
    }
    return *this;
}

Brightness& Brightness::operator=(const XSCBrightness &rhs)
{
    counter = rhs.counter;
    enabled = rhs.enabled;
    level_kepsa = rhs.level_kepsa;
    gain_db = rhs.gain_db;
    actual_exposure = rhs.actual_exposure;
    simulated_exposure = rhs.simulated_exposure;
    return *this;
}

