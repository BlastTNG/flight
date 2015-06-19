/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__THREAD_NAMES_H
#define SHARED__THREAD_NAMES_H

namespace Shared
{
    namespace ThreadNames
    {
        enum Name
        {
            nobody,
            main,
            solver,
            camera,
            lens,
            net_server,
            net_client,
            net_reset,
            net_image_client1,
            net_image_client2
        };
    }
}

#endif

