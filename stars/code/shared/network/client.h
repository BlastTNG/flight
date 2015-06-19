/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__NETWORK__CLIENT_H
#define SHARED__NETWORK__CLIENT_H

#include "../circular_buffer.h"

namespace Shared
{
namespace Network
{

class Client
{
  public:
    Client();
    int counter_fcp;
};

// writes: network
//  reads: main
extern Shared::CircularBufferPass <Client> client_for_main;

// writes: main
//  reads: camera
extern Shared::CircularBuffer <Client> client_for_camera;

}
}

#endif
