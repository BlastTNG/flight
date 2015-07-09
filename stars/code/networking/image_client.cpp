/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "image_client.h"
#include <boost/bind.hpp>
#include "ebex_images.h"
#include "logger.h"
#include "../shared/general/quit.h"
#include "../shared/image/raw.h"
#include "../shared/network/image_client_settings.h"
#include "../shared/update.h"
#include "../tools/quick_cout.h"

using namespace NetworkingImageClient;
using namespace boost::asio::ip;

ImageClient::ImageClient(boost::asio::io_service& io_service, int which_fc):
    socket(io_service),
    read_timer(io_service)
{
    Shared::ThreadNames::Name thread_name = Shared::ThreadNames::nobody;
    if (which_fc == 1)
    {
        shared_image = (Shared::Image::raw_for_image_client1.r);
        shared_settings = Shared::Network::image_client_settings1.r;
        logger = &logger1;
        server_ip = "192.168.1.3";
        thread_name = Shared::ThreadNames::net_image_client1;
    }
    else if (which_fc == 2)
    {
        shared_image = Shared::Image::raw_for_image_client2.r;
        shared_settings = Shared::Network::image_client_settings2.r;
        logger = &logger2;
        server_ip = "192.168.1.4";
        thread_name = Shared::ThreadNames::net_image_client2;
    }
    else
    {
        logger1.log("which_fc invalid");
        logger2.log("which_fc invalid");
        return;
    }
    logger->log(format("image_client initialized for fc%i") % which_fc);

    counter_stars = -1;
    data_size = shared_image->width * shared_image->height * sizeof(unsigned short);
    current_exposure = 0;

    header = (ebex_images_header_t*) malloc(sizeof(ebex_images_header_t) + data_size);
    if (header == NULL) {
        logger->log("unable to allocate memory for image_client buffer");
        return;
    }

    while (!Shared::General::quit) {
        Shared::update(thread_name);
        if (which_fc == 1)
        {
            shared_image = (Shared::Image::raw_for_image_client1.r);
            shared_settings = Shared::Network::image_client_settings1.r;
        }
        else if (which_fc == 2)
        {
            shared_image = Shared::Image::raw_for_image_client2.r;
            shared_settings = Shared::Network::image_client_settings2.r;
        }

        if (counter_stars != shared_image->counter_stars) {
            counter_stars = shared_image->counter_stars;
            if (shared_settings->enabled) {
                usleep(1000000);
                write_image(io_service);
            }
        }
        usleep(1000000);
    }
}

ImageClient::~ImageClient()
{
    free(header);
}

void ImageClient::write_image(boost::asio::io_service& io_service)
{
    tcp::endpoint endpoint(address(address_v4::from_string(server_ip)), EBEX_IMAGES_PORT);
    boost::system::error_code error;
    socket.connect(endpoint, error);
    if (!error) {
        logger->log(format("connected to %s") % server_ip);
        current_exposure = 0;
        write_exposure();
        io_service.run();
        socket.close();
        io_service.reset();
        logger->log("connection closed");
    } else {
        logger->log(format("error connecting to %s") % boost::system::system_error(error).what());
    }
}

void ImageClient::write_exposure()
{
    if (current_exposure < shared_image->num_exposures) {
        header->magic = ebex_images_magic;
        header->length = data_size;
        std::string filename = (format("%s_p%i.raw") % shared_image->filename_base % current_exposure).str();
        if (current_exposure == 0 && shared_image->num_exposures == 1) {
            filename = (format("%s.raw") % shared_image->filename_base).str();
        }
        strncpy(header->filename, filename.c_str(), 63);
        header->filename[63] = '\0';
        memcpy(header+1, shared_image->separate_buffers[current_exposure], data_size);
        logger->log(format("sending image %s") % filename);
        boost::asio::async_write(socket, boost::asio::buffer(header, sizeof(ebex_images_header_t)+data_size),
            boost::bind(&ImageClient::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void ImageClient::handle_write(const boost::system::error_code& error, size_t bytes_transferred)
{
    current_exposure++;
    boost::asio::async_read(socket, boost::asio::buffer(&read_buffer, sizeof(read_buffer)),
        boost::bind(&ImageClient::handle_read, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    read_timer.expires_from_now(boost::posix_time::milliseconds(1000));
    read_timer.async_wait(boost::bind(&ImageClient::write_exposure, this));
}

void ImageClient::handle_read(const boost::system::error_code& error, size_t bytes_transferred)
{
    string result_string = "got async_read error";
    if (!error) {
        if (read_buffer.result == ebex_images_result_success) {
            result_string = "image stored successfully";
        } else {
            result_string = (format("image storage error: %i") % int(read_buffer.result)).str();
        }
    }
    logger->log(format("%s") % result_string);
    read_timer.cancel();
}

