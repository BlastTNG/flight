/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "network_reset.h"
#include "../../parameters/manager.h"

using namespace Shared::General;

NetworkReset::NetworkReset()
{
    reset_now_counter = 0;
    reset_on_lull_enabled = true;
    reset_on_lull_delay = 12.0*60.0;
    device_name = "";
}

void NetworkReset::init(Parameters::Manager& params)
{
    reset_on_lull_enabled = params.general.try_get("main.network_reset_on_lull_enabled", true);
    device_name = params.general.try_get("main.network_device_name", string(""));
}

NetworkResetStatus::NetworkResetStatus()
{
    resetting = false;
}

