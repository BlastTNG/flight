/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "connection.h"
#include <boost/bind.hpp>
#include <string>
#include "../tools/angles.h"
#include "../imaging/commands.h"
#include "../shared/update.h"
#include "../shared/network/packets.h"

#include "../tools/quick_cout.h"

#define XSC_PROTOCOL_VERSION 3

#define shared_packets_from_fc1 (*(Shared::Network::packets_from_fc1.w))
#define shared_packets_from_fc2 (*(Shared::Network::packets_from_fc2.w))

using namespace Networking;
using boost::asio::ip::tcp;

Connection::Connection(boost::asio::io_service& io_service):
    socket_(io_service),
    write_timer(io_service)
{
}

void Connection::start()
{
    XSCChannelInfos xsc_channel_infos;
    xsc_clear_server_data(&server_data);
    xsc_init_channels(&xsc_channel_infos);
    xsc_init_server_data(&server_data, &xsc_channel_infos);
    xsc_clear_client_data(&client_data);
    xsc_zero_command_admins(local_command_admins);

    std::string client_address = socket_.remote_endpoint().address().to_string();
    if (strcmp(client_address.c_str(), "192.168.1.3") == 0) {
        client_name = client_name_fc1;
    } else if (strcmp(client_address.c_str(), "192.168.1.4") == 0) {
        client_name = client_name_fc2;
    } else {
        client_name = client_name_unknown;
    }

    write_timer.expires_from_now(boost::posix_time::milliseconds(250));
    write_timer.async_wait(boost::bind(&Connection::write_server_data, shared_from_this()));

    first_packet = true;
    read_client_data();
}

tcp::socket& Connection::socket()
{
    return socket_;
}


// ----------------
//  write server
// ----------------

void Connection::write_server_data(/*boost::system::error_code& error*/)
{
    load_server_data();
    server_data.xsc_protocol_version = XSC_PROTOCOL_VERSION;
    boost::asio::async_write(socket_, boost::asio::buffer(&server_data, sizeof(server_data)),
        boost::bind(&Connection::handle_write, shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void Connection::handle_write(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (!error) {
        if (client_name == client_name_fc1) {
            shared_packets_from_fc1.time_since_sent.start();
            Shared::Network::packets_from_fc1.share();
        } else if (client_name == client_name_fc2) {
            shared_packets_from_fc2.time_since_sent.start();
            Shared::Network::packets_from_fc2.share();
        }
        write_timer.expires_from_now(boost::posix_time::milliseconds(500));
        write_timer.async_wait(boost::bind(&Connection::write_server_data, shared_from_this()));
    }
}


// ----------------
//  read client
// ----------------

void Connection::read_client_data()
{
    boost::asio::async_read(socket_, boost::asio::buffer(&client_data, sizeof(client_data)),
        boost::bind(&Connection::handle_read, shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
}

void Connection::handle_read(const boost::system::error_code& error, size_t bytes_transferred)
{
    if (!error && client_data.xsc_protocol_version == XSC_PROTOCOL_VERSION) {

        if (client_data.in_charge) {
            if (first_packet) {
                first_packet = false;
                for (unsigned int i=0; i<xC_num_command_admins; i++) {
                    local_command_admins[i].counter = client_data.command_admins[i].counter;
                }
            }
            unload_client_data();
        }

        if (client_name == client_name_fc1) {
            shared_packets_from_fc1.time_since_received.start();
            shared_packets_from_fc1.in_charge = client_data.in_charge;
            Shared::Network::packets_from_fc1.share();
        } else if (client_name == client_name_fc2) {
            shared_packets_from_fc2.time_since_received.start();
            shared_packets_from_fc2.in_charge = client_data.in_charge;
            Shared::Network::packets_from_fc2.share();
        }

        Shared::update(Shared::ThreadNames::net_client);
        read_client_data();
    }
}

