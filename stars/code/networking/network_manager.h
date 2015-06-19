/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef NETWORKING__NETWORK_MANAGER_H
#define NETWORKING__NETWORK_MANAGER_H

#include <boost/thread/thread.hpp>
#include "../tools/timing.h"

namespace Networking
{
    class NetworkManager;
}

class Networking::NetworkManager
{
  public:
    NetworkManager();
    void run_server();
    void run_image_client_fc1();
    void run_image_client_fc2();
    void reset_network_adapter();
    void watch_for_reset();
    void wait_for_quit();

  private:

    Tools::Timer time_since_last_reset;

    boost::thread server_thread;
    boost::thread image_client_fc1_thread;
    boost::thread image_client_fc2_thread;
    boost::thread reset_thread;
};

#endif
