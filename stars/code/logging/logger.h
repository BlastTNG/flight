/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef LOGGING__LOGGER_H
#define LOGGING__LOGGER_H

#include <string>
#include <boost/format.hpp>

using std::string;
using boost::format;

namespace Parameters
{
    class Manager;
}

namespace Tools
{
    class Timer;
}

namespace boost
{
    namespace posix_time
    {
        class ptime;
    }
}

namespace Logging
{
    class Logger;
}

class Logging::Logger
{
  public:
    Logger(string log_name_);
    Logger(string log_name_, bool writing_to_stdout_);
    void construct(string log_name_, bool writing_to_stdout_);
    ~Logger();
    void log(string info);
    void log(boost::format& info);
    void init(Parameters::Manager& params, boost::posix_time::ptime& birthtime, Tools::Timer& age_);
    void flush();
    void update();
    Tools::Timer* age;

  private:
    std::ofstream* my_stream;
    string name;
    bool writing_to_stdout;
    Tools::Timer* timer_since_last_flush;
    double flush_period;
};

#endif
