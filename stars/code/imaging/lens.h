/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__LENS_H
#define IMAGING__LENS_H

#pragma warning(push)
#pragma warning(disable: 4996) // disable warning from boost::is_any_of
#include <boost/asio.hpp>
#pragma warning(pop)
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>
#include "autofocuser.h"
#include "commands.h"
#include "../tools/timing.h"

// Sending commands over the serial port
//   * process_requests calls send_message with a message string
//   * send_message calls clear_read_buffer
//   * clear_read_buffer calls asynchronous read with 0.100 s timeout
//   * clear_read_buffer sleeps 0.500 s
//   * send_message calls write
//   * send_message calls asynchronous read with a (unspecified here to avoid header compilation) timeout
//   * send_message sleeps 2000 ms by default or wait_ms milliseconds if specified

// Example of interthread communication
// A get_focus request comes from fcp into the networking thread.
//   * networking/connection.cpp passes request to imaging/camera_shell.cpp
//     * via Shared::Lens::requests_for_camera
//   * imaging/camera_shell.cpp passes request to imaging/lens.cpp
//     * via Shared::Lens::requests_for_lens
//   * lens.cpp decides if there's a new get_focus request
//     * by comparing class member request_counter_get_focus to Shared::Lens::requests_for_lens
//     * acts on the new request
//   * lens.cpp gets the result of the new get_focus request
//     * sets results_for_camera.get_focus.counter = requests_for_lens.get_focus.counter
//     * puts the new value into Shared::Lens::results_for_camera
//   * imaging/lens.cpp passes result to imaging/camera_shell.cpp
//     * via Shared::Lens::results_for_camera
//   * imaging/camera_shell.cpp passes result to networking/connection.cpp
//     * via Shared::Lens::results_for_network

namespace Parameters
{
    class Manager;
}

namespace Imaging
{
    using namespace LensCommands;
    class Lens;
}

class Imaging::Lens
{
  public:
    Lens(Parameters::Manager& params);
    ~Lens();
    void init();
    void parse_birger_result(std::string line, commands_t command);
    void process_request(commands_t command, std::string message,
        bool initiate_get_focus, bool initiate_get_aperture);
    void process_request(commands_t command, std::string message,
        bool initiate_get_focus, bool initiate_get_aperture, bool use_value);
    void process_requests();
    void send_message(std::string message, commands_t command, int wait_ms=2000);
    void handle_read_timeout(const boost::system::error_code& error);
    void handle_read(commands_t command, const boost::system::error_code& error, size_t size);
    void clear_read_buffer();
    void check_device(std::string device_name);
    void find_device();
    void connect();
    void update();
    void run();
    void wait_for_quit();
    int get_wait_ms(commands_t command, int command_value);

  private:
    Autofocuser autofocuser;

    boost::asio::io_service io;
    boost::asio::serial_port port;
    boost::asio::deadline_timer read_timeout;
    bool enabled;
    bool init_on_startup;
    std::string which_sensor;
    Tools::Timer find_device_timer;
    double find_device_period;
    std::vector<std::string> device_names;
    int baud_rate;
    boost::asio::streambuf instream;
    int command_fcp_counters[num_commands];
    int command_stars_counters[num_commands];

    boost::thread thread;
};

#endif
