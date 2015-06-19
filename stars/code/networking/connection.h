/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef NETWORKING__CONNECTION_H
#define NETWORKING__CONNECTION_H

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../tools/timing.h"
extern "C" {
#include "xsc_protocol/xsc_protocol.h"
}

namespace Networking
{
    typedef enum {
        client_name_fc1,
        client_name_fc2,
        client_name_unknown
    } client_name_t;

    class Connection;
}

class Networking::Connection: public boost::enable_shared_from_this<Connection>
{
  public:
    typedef boost::shared_ptr<Connection> pointer;
    static pointer create(boost::asio::io_service& io_service)
    {
        return pointer(new Connection(io_service));
    }
    boost::asio::ip::tcp::socket& socket();
    void start();

  private:
    Connection(boost::asio::io_service& io_service);

    void load_server_data_housekeeping();
    void load_server_data_camera_and_lens();
    void load_server_data_image();
    void load_server_data();
    void write_server_data(/*boost::system::error_code& error*/);
    void handle_write(const boost::system::error_code& error, size_t bytes_transferred);

    bool check_command(unsigned int command_index);
    void unload_client_data_lens_and_camera();
    void unload_client_data_autofocus();
    void unload_client_data_solver();
    void unload_client_data_solver_filters();
    void unload_client_data_motion_psf();
    void unload_main_and_display_settings();
    void unload_client_data();
    void read_client_data();
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);

    boost::asio::ip::tcp::socket socket_;
    client_name_t client_name;
    XSCClientData client_data;
    XSCServerData server_data;
    xsc_command_admin_t local_command_admins[xC_num_command_admins];
    bool ready_to_read, ready_to_write;
    boost::asio::deadline_timer write_timer;
    bool first_packet;
};

#endif
