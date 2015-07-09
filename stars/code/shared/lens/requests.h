/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__LENS__REQUESTS_H
#define SHARED__LENS__REQUESTS_H

#include "../circular_buffer.h"
#include "../request.h"
#include "../../imaging/commands.h"

namespace Shared
{
    namespace Lens
    {

class Requests
{
  public:
    Requests();
    Requests& operator=(const Requests& rhs);

    static const unsigned int num_requests = Imaging::LensCommands::num_commands;
    ::Shared::Request<int> commands[num_requests];
};


//   path: 0
// writes: network
//  reads: main
extern Shared::CircularBufferPass <Requests> fcp_requests_network_to_main;

//   path: 0
// writes: main
//  reads: camera
extern Shared::CircularBufferPass <Requests> fcp_requests_main_to_camera;

//   path: 0
// writes: camera
//  reads: lens
extern Shared::CircularBuffer <Requests> fcp_requests_camera_to_lens;


//   path: 1
// writes: lens
//  reads: main
extern Shared::CircularBufferPass <Requests> stars_requests_lens_to_main;

//   path: 1
// writes: main
//  reads: camera
extern Shared::CircularBufferPass <Requests> stars_requests_main_to_camera;

//   path: 1
// writes: camera
//  reads: lens
extern Shared::CircularBuffer <Requests> stars_requests_camera_to_lens;


    }
}

#endif

