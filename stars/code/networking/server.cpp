/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "server.h"
#include <boost/bind.hpp>

using namespace Networking;
using boost::asio::ip::tcp;

Server::Server(boost::asio::io_service& io_service, int port):
    acceptor(io_service, tcp::endpoint(tcp::v4(), port))
{
    start_accept();
}

void Server::start_accept()
{
    Connection::pointer new_connection = Connection::create(acceptor.get_io_service());
    acceptor.async_accept(new_connection->socket(),
        boost::bind(&Server::handle_accept, this, new_connection, boost::asio::placeholders::error));
}

void Server::handle_accept(Connection::pointer new_connection, const boost::system::error_code& error)
{
    if (!error) {
        new_connection->start();
        start_accept();
    }
}


