/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "lens.h"
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <vector>
#include "../parameters/manager.h"
#include "../shared/update.h"
#include "../shared/lens/requests.h"
#include "../shared/lens/results.h"
#include "../shared/general/quit.h"
#include "logger_lens.h"

using namespace Imaging;
using Lensing::logger;

#define shared_fcp_requests (*(Shared::Lens::fcp_requests_camera_to_lens.r))
#define shared_fcp_results (*(Shared::Lens::fcp_results_lens_to_camera.w))
#define shared_stars_write_requests (*(Shared::Lens::stars_requests_lens_to_main.w))
#define shared_stars_requests (*(Shared::Lens::stars_requests_camera_to_lens.r))
#define shared_stars_results (*(Shared::Lens::stars_results_lens_to_camera.w))

string string_to_hex(string instring)
{
    string outstring = "";
    for (unsigned int i=0; i<instring.size(); i++) {
        if (i > 0) {
            outstring += " ";
        }
        outstring += (format("%|02X|")%((int) (unsigned char) instring[i])).str();
    }
    return outstring;
}

Lens::Lens(Parameters::Manager& params): port(io), read_timeout(io),
    enabled(params.general.try_get("imaging.lens.enabled", true)),
    init_on_startup(params.general.try_get("imaging.lens.init_on_startup", false)),
    #pragma warning(push)
    #pragma warning(disable: 4355)
    thread(boost::bind(&Lens::run, this))
    #pragma warning(pop)
{
}

Lens::~Lens()
{
}

void Lens::init()
{
    find_device_period = 60.0;
    baud_rate = 9600;
    for (unsigned int i=0; i<num_commands; i++) {
        command_fcp_counters[i] = 0;
        command_stars_counters[i] = 0;
    }
    for (unsigned int i=2; i<=12; i++) {
        device_names.push_back((format("COM%d")%i).str());
    }
    device_names.push_back("/dev/ttyS0");
    device_names.push_back("/dev/ttyS1");
    device_names.push_back("/dev/ttyS2");
    device_names.push_back("/dev/ttyACM");
    device_names.push_back("/dev/ttyACM0");
    device_names.push_back("/dev/ttyACM1");
    device_names.push_back("/dev/ttyACM2");
    device_names.push_back("/dev/ttyUSB");
    device_names.push_back("/dev/ttyUSB0");
    device_names.push_back("/dev/ttyUSB1");
    device_names.push_back("/dev/ttyUSB2");
}

void Lens::parse_birger_result(string full_line, commands_t command)
{
    using std::vector;
    using std::stringstream;

    int int0 = 0;
    string line;
    string found_value;

    if (full_line.size() > 4) {
        line = full_line.substr(4, string::npos);
    } else {
        line = full_line;
    }

    string message;

    logger.log(format("recieved birger message for command callback %i")%command);
    logger.log(       "                message (str) " + line);
    logger.log(       "                message (hex) " + string_to_hex(line));
    logger.log(       "I am done printing the hex ");
    switch (command) {
        case flush_birger:
            break;
        case get_focus:
            shared_fcp_results.command_counters[init_focus] = shared_fcp_requests.commands[init_focus].counter;
            shared_fcp_results.command_counters[get_focus] = shared_fcp_requests.commands[get_focus].counter;
            shared_fcp_results.command_counters[define_focus] = shared_fcp_requests.commands[define_focus].counter;
            shared_fcp_results.command_counters[set_focus] = shared_fcp_requests.commands[set_focus].counter;
            shared_fcp_results.command_counters[set_focus_incremental] = shared_fcp_requests.commands[set_focus_incremental].counter;
            shared_stars_results.command_counters[init_focus] = shared_stars_requests.commands[init_focus].counter;
            shared_stars_results.command_counters[get_focus] = shared_stars_requests.commands[get_focus].counter;
            shared_stars_results.command_counters[define_focus] = shared_stars_requests.commands[define_focus].counter;
            shared_stars_results.command_counters[set_focus] = shared_stars_requests.commands[set_focus].counter;
            shared_stars_results.command_counters[set_focus_incremental] = shared_stars_requests.commands[set_focus_incremental].counter;
            {

                found_value = line.substr(0, line.length()-2);
                if (found_value.length() > 0) {
                    int0 = atoi(found_value.c_str());
                    shared_fcp_results.focus_value = int0;
                    shared_fcp_results.focus_found = true;
                    shared_stars_results.focus_value = int0;
                    shared_stars_results.focus_found = true;
                    logger.log(format("focus found: %i")%(int0));
                    shared_stars_requests.commands[save_focus].value = shared_stars_results.focus_value;
                }
            }
            Shared::Lens::fcp_results_lens_to_camera.share();
            Shared::Lens::stars_results_lens_to_camera.share();
            break;
        case init_focus:
            break;
        case set_focus:
            shared_fcp_results.command_counters[init_focus] = shared_fcp_requests.commands[init_focus].counter;
            shared_fcp_results.command_counters[get_focus] = shared_fcp_requests.commands[get_focus].counter;
            shared_fcp_results.command_counters[define_focus] = shared_fcp_requests.commands[define_focus].counter;
            shared_fcp_results.command_counters[set_focus] = shared_fcp_requests.commands[set_focus].counter;
            shared_fcp_results.command_counters[set_focus_incremental] = shared_fcp_requests.commands[set_focus_incremental].counter;
            shared_stars_results.command_counters[init_focus] = shared_stars_requests.commands[init_focus].counter;
            shared_stars_results.command_counters[get_focus] = shared_stars_requests.commands[get_focus].counter;
            shared_stars_results.command_counters[define_focus] = shared_stars_requests.commands[define_focus].counter;
            shared_stars_results.command_counters[set_focus] = shared_stars_requests.commands[set_focus].counter;
            shared_stars_results.command_counters[set_focus_incremental] = shared_stars_requests.commands[set_focus_incremental].counter;
            Shared::Lens::fcp_results_lens_to_camera.share();
            Shared::Lens::stars_results_lens_to_camera.share();
            break;
        case save_focus:
            break;
        case set_focus_incremental:
            break;
        case get_aperture:
            shared_fcp_results.command_counters[init_aperture] = shared_fcp_requests.commands[init_aperture].counter;
            shared_fcp_results.command_counters[get_aperture] = shared_fcp_requests.commands[get_aperture].counter;
            shared_fcp_results.command_counters[set_aperture] = shared_fcp_requests.commands[set_aperture].counter;
            shared_fcp_results.command_counters[define_aperture] = shared_fcp_requests.commands[define_aperture].counter;
            shared_stars_results.command_counters[init_aperture] = shared_stars_requests.commands[init_aperture].counter;
            shared_stars_results.command_counters[get_aperture] = shared_stars_requests.commands[get_aperture].counter;
            shared_stars_results.command_counters[set_aperture] = shared_stars_requests.commands[set_aperture].counter;
            shared_stars_results.command_counters[define_aperture] = shared_stars_requests.commands[define_aperture].counter;
            {
                string found_value = line.substr(0, line.length() - 2);
                if (found_value.length() > 0) {
                    int0 = atoi(found_value.c_str());
                    logger.log(format("aperture found: %i") % (int0));
                    shared_fcp_results.aperture_value = int0;
                    shared_fcp_results.aperture_found = true;
                    shared_stars_results.aperture_value = int0;
                    shared_stars_results.aperture_found = true;
                    shared_stars_requests.commands[save_aperture].value = shared_stars_results.aperture_value;
                }
            }
            Shared::Lens::fcp_results_lens_to_camera.share();
            Shared::Lens::stars_results_lens_to_camera.share();
            break;
        case init_aperture:
            break;
        case set_aperture:
            shared_fcp_results.command_counters[init_aperture] = shared_fcp_requests.commands[init_aperture].counter;
            shared_fcp_results.command_counters[get_aperture] = shared_fcp_requests.commands[get_aperture].counter;
            shared_fcp_results.command_counters[set_aperture] = shared_fcp_requests.commands[set_aperture].counter;
            shared_fcp_results.command_counters[define_aperture] = shared_fcp_requests.commands[define_aperture].counter;
            shared_stars_results.command_counters[init_aperture] = shared_stars_requests.commands[init_aperture].counter;
            shared_stars_results.command_counters[get_aperture] = shared_stars_requests.commands[get_aperture].counter;
            shared_stars_results.command_counters[set_aperture] = shared_stars_requests.commands[set_aperture].counter;
            shared_stars_results.command_counters[define_aperture] = shared_stars_requests.commands[define_aperture].counter;
            Shared::Lens::fcp_results_lens_to_camera.share();
            Shared::Lens::stars_results_lens_to_camera.share();
            break;
        case version_string:
            {
                vector<string> words;
                boost::split(words, line, boost::is_any_of(" "));
                if (words.size() == 6) {
                    if (words[0].compare("EZStepper") == 0) {
                        logger.log("device found");
                        shared_fcp_results.device_found = true;
                        shared_stars_results.device_found = true;
                    }
                }
            }
            break;
        case load_aperture:
            break;
        case load_focus:
            break;
        case clearing_read_buffer:
            if (line.size() > 0) {
                logger.log(format("clearing read buffer returned %d characters")%line.size());
            }
            break;
        case define_focus:
            break;
        case define_aperture:
            break;
        default:
            break;
    }
}

void Lens::process_request(commands_t command, string message,
    bool initiate_get_focus, bool initiate_get_aperture)
{
    if (command_fcp_counters[command] != shared_fcp_requests.commands[command].counter ||
        command_stars_counters[command] != shared_stars_requests.commands[command].counter)
    {
        logger.log(format("initiating lens request (%i) because counters don't match:") % command);
        logger.log(format("      (fcp counters) %i %i") % command_fcp_counters[command] % shared_fcp_requests.commands[command].counter);
        logger.log(format("    (stars counters) %i %i") % command_stars_counters[command] % shared_stars_requests.commands[command].counter);

        command_fcp_counters[command] = shared_fcp_requests.commands[command].counter;
        command_stars_counters[command] = shared_stars_requests.commands[command].counter;
        if (initiate_get_focus) {
            shared_stars_write_requests.commands[get_focus].counter++;
            Shared::Lens::stars_requests_lens_to_main.share();
        }
        if (initiate_get_aperture) {
            shared_stars_write_requests.commands[get_aperture].counter++;
            Shared::Lens::stars_requests_lens_to_main.share();
        }
        if (command == get_focus) {
            shared_stars_write_requests.commands[save_focus].counter++;
            Shared::Lens::stars_requests_lens_to_main.share();
        }
        if (command == get_aperture) {
            shared_stars_write_requests.commands[save_aperture].counter++;
            Shared::Lens::stars_requests_lens_to_main.share();
        }
		if  ((command == init_aperture)) {
			send_message(message, command, 16000);
		} else if ((command == init_focus)) {
			send_message(message, command, 110000);
		} else if ((command == set_aperture)) {
			send_message(message, command, 8000);
		} else if ((command == set_focus)) {
			send_message(message, command, 55000);
		}
		else if ((command == set_focus_incremental)) {
			double wait_ms = shared_stars_requests.commands[command].value * 55000 / 3300;
			if (wait_ms < 0) wait_ms = -wait_ms;
			send_message(message, command, wait_ms);
		} else {
            send_message(message, command);
        }
        logger.log("send_message returned");
    }
}

void Lens::process_request(commands_t command, string message,
    bool initiate_get_focus, bool initiate_get_aperture, bool use_value)
{
    if (use_value) {
        if (command_fcp_counters[command] != shared_fcp_requests.commands[command].counter) {
            int value = shared_fcp_requests.commands[command].value;
            if (command == set_focus_incremental) { 
                if (value < 0) {
                    value = -value;
                    message[2] = 'D';
                }
            }
            process_request(command, (format(message)%value).str(), initiate_get_focus, initiate_get_aperture);
        }
        else if (command_stars_counters[command] != shared_stars_requests.commands[command].counter) {
            int value = shared_stars_requests.commands[command].value;
            if (command == set_focus_incremental) { 
                if (value < 0) {
                    value = -value;
                    message[2] = 'D';
                }
            }
            process_request(command, (format(message)%value).str(), initiate_get_focus, initiate_get_aperture);
        }
    }
}

void Lens::process_requests()
{
    process_request(flush_birger, "\r", false, false);
    process_request(init_focus, "/2z3500D3500M55000z0R\r", true, false);
    process_request(get_focus, "/2?8\r", false, false);
    process_request(set_focus, "/2A%dR\r", true, false, true);
    process_request(set_focus_incremental, "/2P%dR\r", true, false, true);
    process_request(init_aperture, "/1z650D650M8000z0R\r", false, true);
    process_request(get_aperture, "/1?8\r", false, false);
    process_request(set_aperture, "/1A%dR\r", false, true, true);
    process_request(save_aperture, "/1s1z%dR\r", false, false, true);
    process_request(save_focus, "/2s1z%dR\r", false, false, true);
    process_request(define_focus, "/2z%dR\r", true, false, true);
    process_request(define_aperture, "/1z%dR\r", false, true, true);
}

void Lens::send_message(string message, commands_t command, int wait_ms)
{
    try {
        if (port.is_open()) {
            clear_read_buffer();
            logger.log("sending motor command (str) " + message.substr(0, message.size()-1));
            io.reset();
            boost::asio::write(port, boost::asio::buffer(message.c_str(), message.size()));
            read_timeout.expires_from_now(boost::posix_time::milliseconds(8000));
            boost::asio::async_read_until(port, instream, '\r',
                boost::bind(&Lens::handle_read, this, command, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
            read_timeout.async_wait(boost::bind(&Lens::handle_read_timeout, this, boost::asio::placeholders::error));
            io.run();
            usleep(wait_ms*1000);
        } else {
            logger.log("port closed, wanted to send " + message.substr(0, message.size()-1));
        }
    } catch (boost::system::system_error&) {}
}

void Lens::handle_read_timeout(const boost::system::error_code& error)
{
    if (!error) {
        port.cancel();
    }
}

void Lens::handle_read(commands_t command, const boost::system::error_code& error, size_t size)
{
    if (!error && size) {
        std::istream is(&instream);
        string line = "";
        bool end_of_file = false;
        while (!end_of_file) {
            end_of_file = getline(is, line).eof();
            parse_birger_result(line, command);
        }
        read_timeout.cancel();
    }
}

void Lens::clear_read_buffer()
{
    // This calls asynchronous send with a 0.1 s timeout
    // calling function sleeps for 0.5 s
    io.reset();
    read_timeout.expires_from_now(boost::posix_time::milliseconds(100));
    boost::asio::async_read(port, instream,
        boost::bind(&Lens::handle_read, this, clearing_read_buffer, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    read_timeout.async_wait(boost::bind(&Lens::handle_read_timeout, this, boost::asio::placeholders::error));
    io.run();
    usleep(500);
}

void Lens::check_device(string device_name)
{
    try {
        port.open(device_name);
        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        logger.log(format("trying device %s")%device_name.c_str());
        if (port.is_open()) {
            string message = "/1&\r";
            logger.log(format("attempting message /1& to %s") %(device_name.c_str()));
            send_message(message, version_string);
        }
        port.close();
        if (shared_fcp_results.device_found) {
            shared_fcp_results.device_found = true;
            shared_stars_results.device_found = true;
            shared_fcp_results.device_name = device_name;
            shared_stars_results.device_name = device_name;
            Shared::Lens::fcp_results_lens_to_camera.share();
            Shared::Lens::stars_results_lens_to_camera.share();
        }
    } catch (boost::system::system_error&) {}
    find_device_timer.start();
}

void Lens::find_device()
{
    shared_fcp_results.device_found = false;
    Shared::Lens::fcp_results_lens_to_camera.share();
    for (unsigned int i=0; !shared_fcp_results.device_found && i<device_names.size(); i++) {
        check_device(device_names[i]);
    }
}

void Lens::connect()
{
    port.open(shared_fcp_results.device_name);
    port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    if (init_on_startup) {
        shared_stars_write_requests.commands[init_focus].counter++;
        shared_stars_write_requests.commands[init_aperture].counter++;
    } else {
        string message = "/1e1R\r";
        send_message(message, load_aperture);
        message = "/2e1R\r";
        send_message(message, load_focus);
        shared_stars_write_requests.commands[get_focus].counter++;
        shared_stars_write_requests.commands[get_aperture].counter++;
    }
    Shared::Lens::stars_requests_lens_to_main.share();
}

void Lens::update()
{
    Shared::update(Shared::ThreadNames::lens);
    logger.update();
}

void Lens::run()
{
    init();
    while (!Shared::General::quit) {
        update();
        autofocuser.update();
        if (enabled) {
        //if (true) {
            if (!shared_fcp_results.device_found && (find_device_timer.time() > find_device_period)) {
                find_device();
                find_device_timer.start();
                if (shared_fcp_results.device_found) {
                    connect();
                }
            }
            process_requests();
        }
        usleep(200000);
    }
    try {
        port.close();
    } catch (boost::system::system_error&) {}
}

void Lens::wait_for_quit()
{
    thread.join();
}

