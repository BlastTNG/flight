/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef PARAMETERS__GROUP_H
#define PARAMETERS__GROUP_H

#include <string>
#include <boost/program_options.hpp>
#include <typeinfo>
#include "../logger_main.h"

#ifdef _MSC_VER
    #define typeof decltype
#endif

namespace Parameters
{
    class Group;
}

class Parameters::Group
{
  public:
    void load(std::string filename);
    double try_get(std::string name, double default_value);
    template <class T> T try_get(std::string name, T default_value);
    boost::program_options::variables_map map;
  protected:
    boost::program_options::options_description options;
};

// if (typeid(variable) == typeid(double)) use overloaded function in cpp file
template <typename T>
T Parameters::Group::try_get(std::string name, T default_value)
{
    T value = default_value;
    try {
        if (map.count(name)) {
            value = map[name].as<typeof(default_value)>();
        } else {
            Main::logger.log(format("failed to set parameter %s: not found in map") % name);
        }
    } catch(std::exception& e) {
        Main::logger.log(format("failed to set parameter %s: %s") % name % e.what());
    }
    return value;
}

#endif
