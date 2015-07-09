/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "image_client_settings.h"
#include "../../parameters/manager.h"

using namespace Shared::Network;

ImageClientSettings::ImageClientSettings()
{
    enabled = true;
}

void ImageClientSettings::init(Parameters::Manager& params)
{
    enabled = params.general.try_get("imaging.image_client.enabled", true);
}

