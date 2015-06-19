/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef NETWORKING__LOGGER_H
#define NETWORKING__LOGGER_H

#include "../logging/logger.h"

namespace Networking
{
    // To be used only by the network thread
    extern Logging::Logger logger;
}

namespace NetworkingImageClient
{
    // To be used only by the NetworkingImageClient thread
    extern Logging::Logger logger1;
    extern Logging::Logger logger2;
}

#endif
