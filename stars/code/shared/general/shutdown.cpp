/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "shutdown.h"

using namespace Shared::General;

Shutdown::Shutdown()
{
    shutdown_now = false;
    include_restart = false;
}

