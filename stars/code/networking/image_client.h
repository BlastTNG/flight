/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef NETWORKING__IMAGE_CLIENT_H
#define NETWORKING__IMAGE_CLIENT_H

#include <string>
#include <boost/asio.hpp>
#include "ebex_images.h"

namespace Shared
{
    namespace Image
    {
        class Raw;
    }
    namespace Network
    {
        class ImageClientSettings;
    }
}

namespace Logging
{
    class Logger;
}

namespace NetworkingImageClient
{
    class ImageClient;
}

class NetworkingImageClient::ImageClient
{
  public:
    ImageClient(boost::asio::io_service& io_service, int which_fc);
    ~ImageClient();

  private:
    void write_image(boost::asio::io_service& io_service);
    void write_exposure();
    void handle_write(const boost::system::error_code& error, size_t bytes_transferred);
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);

    boost::asio::ip::tcp::socket socket;
    boost::asio::deadline_timer read_timer;
    ebex_images_header_t* header;
    ebex_images_result_t read_buffer;

    std::string server_ip;
    Shared::Image::Raw* shared_image;
    Shared::Network::ImageClientSettings* shared_settings;
    Logging::Logger* logger;
    int counter_stars;
    size_t data_size;
    unsigned int current_exposure;
};

#endif
