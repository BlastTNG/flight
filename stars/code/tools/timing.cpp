/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "timing.h"
#include <boost/format.hpp>

using namespace Tools;
using std::string;
using boost::format;

/*
boost::system_time Tools::relative_time(double seconds)
{
    return boost::get_system_time() + boost::posix_time::milliseconds(seconds*1000.0);
}
*/

#ifdef _MSC_VER
void usleep(int useconds)
{
    Sleep(useconds/1000);
}
#endif


Timer::Timer()
{
    started_ = false;
}

bool Timer::started()
{
    return started_;
}

void Timer::reset()
{
    started_ = false;
}

Timer& Timer::operator=(const Timer &rhs)
{
    if (this != &rhs) {
        started_ = rhs.started_;
        m_depart = rhs.m_depart;
    }
    return *this;
}

double Time::to_years(double time)
{
    return time / (3600.0*24.0*365.0);
}

double Time::from_years(double time)
{
    return time * (3600.0*24.0*365.0);
}

double Time::to_days(double time)
{
    return time / (3600.0*24.0);
}

double Time::from_days(double time)
{
    return time * (3600.0*24.0);
}

double Time::to_hours(double time)
{
    return time / (3600.0);
}

double Time::from_hours(double time)
{
    return time * (3600.0);
}

double Time::to_minutes(double time)
{
    return time / (60.0);
}

double Time::from_minutes(double time)
{
    return time * (60.0);
}

double Time::to_seconds(double time)
{
    return time;
}

double Time::from_seconds(double time)
{
    return time;
}

double Time::to_milliseconds(double time)
{
    return time * (1000.0);
}

double Time::from_milliseconds(double time)
{
    return time / (1000.0);
}

string Time::time_string(double time)
{
    string outstring = "";
    if (time == std::numeric_limits<double>::infinity()) {
        outstring = "inf time";
    } else if (time > from_years(999.0)) {
        outstring = (format("%.0e y") % to_years(time)).str();
    } else if (time > from_years(1.0)) {
        outstring = (format("%.1f yr") % to_years(time)).str();
    } else if (time > from_days(1.0)) {
        outstring = (format("%.1f dy") % to_days(time)).str();
    } else if (time > from_hours(1.0)) {
        outstring = (format("%.1f hr") % to_hours(time)).str();
    } else if (time > from_minutes(1.0)) {
        outstring = (format("%.1f mi") % to_minutes(time)).str();
    } else if (time > from_seconds(1.0)) {
        outstring = (format("%.1f s") % to_seconds(time)).str();
    } else {
        outstring = (format("%.1f ms") % to_milliseconds(time)).str();
    }
    return outstring;
}



