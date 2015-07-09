/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__LOGGER_CAMERA_H
#define IMAGING__LOGGER_CAMERA_H

#include "../logging/logger.h"

namespace Imaging
{
    namespace Cameraing
    {
        // To be used only by the camera thread
        extern Logging::Logger logger;
    }
}

#endif
