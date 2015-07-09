/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef HOUSEKEEPING__LOGGER_H
#define HOUSEKEEPING__LOGGER_H

#include "../logging/logger.h"

namespace Housekeeping
{
    // To be used only by the housekeeper
    extern Logging::Logger logger;
}

#endif
