/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SOLVING__LOGGER_H
#define SOLVING__LOGGER_H

#include "../logging/logger.h"

namespace Solving
{
    // To be used only by the solver thread
    extern Logging::Logger logger;
}

#endif
