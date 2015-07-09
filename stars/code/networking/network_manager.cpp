/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "network_manager.h"
#include <cstdlib>
#include <boost/bind.hpp>
#include <limits>
#include "server.h"
#include "image_client.h"
#include "logger.h"
#include "../shared/general/network_reset.h"
#include "../shared/network/packets.h"
#include "../shared/network/image_client_settings.h"
#include "../shared/general/quit.h"
#include "../shared/update.h"
#include "../tools/timing.h"
#include "../tools/quick_cout.h"

#define shared_reset_request (*(Shared::General::network_reset_for_net_reset.r))
#define shared_reset_status (*(Shared::General::network_reset_status_for_main.w))
#define shared_packets_from_fc1 (*(Shared::Network::packets_from_fc1.r))
#define shared_packets_from_fc2 (*(Shared::Network::packets_from_fc2.r))

using namespace Networking;

NetworkManager::NetworkManager():
    #pragma warning(push)
    #pragma warning(disable: 4355)
    server_thread(boost::bind(&NetworkManager::run_server, this)),
    image_client_fc1_thread(boost::bind(&NetworkManager::run_image_client_fc1, this)),
    image_client_fc2_thread(boost::bind(&NetworkManager::run_image_client_fc2, this)),
    reset_thread(boost::bind(&NetworkManager::watch_for_reset, this))
    #pragma warning(pop)
{
}

void NetworkManager::run_server()
{
    try {
        boost::asio::io_service io_service;
        Server server(io_service, 2017);
        io_service.run();
    } catch (std::exception&) { }
}

void NetworkManager::run_image_client_fc1()
{
    if (Shared::Network::image_client_settings1.r->enabled) {
        try {
            boost::asio::io_service io_service;
            NetworkingImageClient::ImageClient image_client(io_service, 1);
        } catch (std::exception&) { }
    }
}

void NetworkManager::run_image_client_fc2()
{
    if (Shared::Network::image_client_settings2.r->enabled) {
        try {
            boost::asio::io_service io_service;
            NetworkingImageClient::ImageClient image_client(io_service, 2);
        } catch (std::exception&) { }
    }
}

void NetworkManager::reset_network_adapter()
{
    shared_reset_status.resetting = true;
    Shared::General::network_reset_status_for_main.share();

    logger.log("(when the code is implemented) resetting the network adapter");
    string command = (format("devcon disable %s")%shared_reset_request.device_name).str();
    logger.log(format("calling system command: %s")%command);
    if(system(command.c_str()) != 0) {
        logger.log("warning: system call may have failed");
    }

    for (int i=0; i<5; i++) {
        Shared::update(Shared::ThreadNames::net_reset);
        usleep(1000000);
    }

    command = (format("devcon enable %s")%shared_reset_request.device_name).str();
    logger.log(format("calling system command: %s")%command);
    if(system(command.c_str()) != 0) {
        logger.log("warning: system call may have failed");
    }

    shared_reset_status.resetting = false;
    Shared::General::network_reset_status_for_main.share();

    time_since_last_reset.start();
}

void NetworkManager::watch_for_reset()
{
    int last_reset_now_counter = shared_reset_request.reset_now_counter;
    bool received_any_packets_so_far = false;
    time_since_last_reset.start();

    while (!Shared::General::quit) {
        bool activate_reset = false;

        if (last_reset_now_counter != shared_reset_request.reset_now_counter) {
            last_reset_now_counter = shared_reset_request.reset_now_counter;
            activate_reset = true;
        }
        if (shared_reset_request.reset_on_lull_enabled) {
            if (time_since_last_reset.time() > shared_reset_request.reset_on_lull_delay
                && shared_packets_from_fc1.time_since_received.time() > shared_reset_request.reset_on_lull_delay
                && shared_packets_from_fc2.time_since_received.time() > shared_reset_request.reset_on_lull_delay)
            {
                activate_reset = true;
            }
        }
        if (activate_reset) {
            reset_network_adapter();
        }


        if (   shared_packets_from_fc1.time_since_received.time() < std::numeric_limits<double>::infinity()
            || shared_packets_from_fc2.time_since_received.time() < std::numeric_limits<double>::infinity())
        {
            received_any_packets_so_far = true;
        }

        if (shared_reset_request.reset_on_lull_enabled) {
            if (received_any_packets_so_far
                && shared_packets_from_fc1.time_since_received.time() > (shared_reset_request.reset_on_lull_delay + 10.0*60.0)
                && shared_packets_from_fc2.time_since_received.time() > (shared_reset_request.reset_on_lull_delay + 10.0*60.0))
            {
                Shared::General::quit = true;
            }
        }


        Shared::update(Shared::ThreadNames::net_reset);
        logger.update();
        usleep(100000);
    }
}

void NetworkManager::wait_for_quit()
{
    //server_thread.join();
    image_client_fc1_thread.join();
    image_client_fc2_thread.join();
    reset_thread.join();
}

