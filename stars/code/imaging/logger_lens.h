/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__LOGGER_LENS_H
#define IMAGING__LOGGER_LENS_H

#include "../logging/logger.h"

namespace Imaging
{
    namespace Lensing
    {
        // To be used only by the lens thread
        extern Logging::Logger logger;
    }
}

#endif
