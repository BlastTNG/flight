/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef NETWORKING__SERVER_H
#define NETWORKING__SERVER_H

#include <boost/asio.hpp>
#include "connection.h"

namespace Networking
{
    class Server;
}

class Networking::Server
{
  public:
    Server(boost::asio::io_service& io_service, int port);
    void start_accept();
    void handle_accept(Connection::pointer new_connection, const boost::system::error_code& error);

    boost::asio::ip::tcp::acceptor acceptor;
};

#endif
