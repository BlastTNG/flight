/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef LOGGER_MAIN_H
#define LOGGER_MAIN_H

#include "logging/logger.h"

namespace Main
{
    // To be used only by the main thread
    extern Logging::Logger logger;
}

#endif
