/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef TOOLS__TIMING_H
#define TOOLS__TIMING_H

#include "windows_header.h"
#ifndef _MSC_VER
    #include <sys/time.h>
#endif
#include <boost/thread/thread_time.hpp>
#include <limits>
#include <string>

namespace Tools
{
    class Timer;
    boost::system_time relative_time(double seconds);
}

namespace Time
{
    double to_years(double time);
    double from_years(double time);
    double to_days(double time);
    double from_days(double time);
    double to_hours(double time);
    double from_hours(double time);
    double to_minutes(double time);
    double from_minutes(double time);
    double to_seconds(double time);
    double from_seconds(double time);
    double to_milliseconds(double time);
    double from_milliseconds(double time);
    std::string time_string(double time);
}

#ifdef _MSC_VER
void usleep(int useconds);
#endif


class Tools::Timer
{
  public:
    Timer();
    void start();
    double time() const;
    bool started();
    void reset();
    Timer& operator=(const Timer &rhs);

  private:
    #if defined(_MSC_VER)
        LARGE_INTEGER m_depart;
    #else
        timeval m_depart;
    #endif
    bool started_;
};

#if defined(_MSC_VER)
inline void Tools::Timer::start()
{
    started_ = true;
    QueryPerformanceCounter(&m_depart);
}
inline double Tools::Timer::time() const
{
    if (started_) {
        LARGE_INTEGER now;
        LARGE_INTEGER freq;
        QueryPerformanceCounter(&now);
        QueryPerformanceFrequency(&freq);
        return (now.QuadPart - m_depart.QuadPart) / static_cast<double>(freq.QuadPart);
    }
    else {
        return std::numeric_limits<double>::infinity();
    }
}
#else
inline void Tools::Timer::start()
{
    started_ = true;
    gettimeofday(&m_depart, 0);
}
inline double Tools::Timer::time() const
{
    if (started_) {
        timeval now;
        gettimeofday(&now, 0);
        return now.tv_sec - m_depart.tv_sec + (now.tv_usec - m_depart.tv_usec) / 1000000.0f;
    }
    else {
        return std::numeric_limits<double>::infinity();
    }
}
#endif


#endif
