/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "logger.h"
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include "../parameters/manager.h"
#include "../tools/timing.h"

using namespace Logging;
using std::string;
namespace fs = boost::filesystem;

Logger::Logger(string name_)
{
    construct(name_, false);
}

Logger::Logger(string name_, bool writing_to_stdout_)
{
    construct(name_, writing_to_stdout_);
}

void Logger::construct(string name_, bool writing_to_stdout_)
{
    age = new (std::nothrow) Tools::Timer;
    timer_since_last_flush = new (std::nothrow) Tools::Timer;
    my_stream = new (std::nothrow) std::ofstream;
    name = name_;
    writing_to_stdout = writing_to_stdout_;
    flush_period = 5.0;
}

Logger::~Logger()
{
    my_stream->flush();
    delete age;
    delete timer_since_last_flush;
    try {
        my_stream->close();
        delete my_stream;
    } catch (const std::exception&) {}
}

void Logger::init(Parameters::Manager& params, boost::posix_time::ptime& birthtime, Tools::Timer& age_)
{
    *age = age_;

    string output_dir;
    output_dir = params.general.try_get("main.output_dir", string("E:\\data"));		//changed to E:
    string dirname = (boost::format("%04d-%02d-%02d")
            % birthtime.date().year()
            % int(birthtime.date().month())
            % birthtime.date().day()
        ).str();
    if (!fs::exists(output_dir)) {
        output_dir = "";
    }
    fs::path logs_dir = fs::path(output_dir) / "logs";
    if (!fs::exists(logs_dir)) {
        fs::create_directory(logs_dir);
    }
    fs::path dir = fs::path(output_dir) / "logs" / dirname;
    if (!fs::exists(dir)) {
        fs::create_directory(dir);
    }
    string leaf_filename = (boost::format("%04d-%02d-%02d--%02d-%02d-%02d_%s.log")
            % birthtime.date().year()
            % int(birthtime.date().month())
            % birthtime.date().day()
            % birthtime.time_of_day().hours()
            % birthtime.time_of_day().minutes()
            % birthtime.time_of_day().seconds()
            % name
        ).str();
    string filename = (dir / leaf_filename).string();	//changed from file_string() to string()

    my_stream->open(filename.c_str(), std::fstream::out | std::fstream::app);
}

void Logger::log(string info)
{
    double time = age->time();
    (*my_stream) << format("%10.3f %4.4s - ")%time%name << info << "\n";
    if (writing_to_stdout) {
        std::cout << format("%10.3f %4.4s - ")%time%name << info << "\n";
    }
    update();
}

void Logger::log(boost::format& info)
{
    double time = age->time();
    (*my_stream) << format("%10.3f %4.4s - ")%time%name << info << "\n";
    if (writing_to_stdout) {
        std::cout << format("%10.3f %4.4s - ")%time%name << info << "\n";
    }
    update();
}

void Logger::flush()
{
    timer_since_last_flush->start();
    my_stream->flush();
}

void Logger::update()
{
    if (timer_since_last_flush->time() > flush_period) {
        flush();
    }
}

