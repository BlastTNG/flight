/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__NETWORK__IMAGE_CLIENT_SETTINGS_H
#define SHARED__NETWORK__IMAGE_CLIENT_SETTINGS_H

#include "../circular_buffer.h"

namespace Shared
{
    namespace Network
    {

class ImageClientSettings
{
  public:
    ImageClientSettings();
    void init(Parameters::Manager& params);

    bool enabled;
};

// writes: network
//  reads: net_image_client1

extern Shared::CircularBuffer <ImageClientSettings> image_client_settings1;

// writes: network
//  reads: net_image_client2

extern Shared::CircularBuffer <ImageClientSettings> image_client_settings2;

    }
}

#endif
