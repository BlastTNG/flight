/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "housekeeping.h"
#include <fstream>
#include <string>
#include <boost/format.hpp>
#include "../logger_main.h"

using namespace Parameters;
using Main::logger;

Housekeeping::Housekeeping()
{
    using namespace boost::program_options;
    using std::string;
    for (unsigned int num=0; num<16; num++) {
        options.add_options()
            ((boost::format("channel%01d.name")          %num).str().c_str(), value<string>(), "")
            ((boost::format("channel%01d.type")          %num).str().c_str(), value<string>(), "")
            ((boost::format("channel%01d.resistor_value")%num).str().c_str(), value<float>(), "")
            ((boost::format("channel%01d.voltage")       %num).str().c_str(), value<float>(), "")
        ;
    }
}

void Housekeeping::load(std::string filename)
{
    std::ifstream file(filename.c_str());
    try {
        store(parse_config_file(file, options), map);
    } catch(std::exception& e) {
        logger.log(format("Caught exception in Parameters::Housekeeping: %s") % e.what());
    }
    notify(map);
    file.close();
}

