/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "manager.h"
#include <boost/filesystem/operations.hpp>

using namespace Parameters;

Manager::Manager(std::string stars_absolute_dir):
    stars_dir(stars_absolute_dir),
    general(stars_absolute_dir)
{
}

void Manager::load(int argc, char* argv[])
{
    using namespace boost::filesystem;
    general.load(argc, argv);
    housekeeping.load(system_complete(stars_dir + "settings/housekeeping.txt").string());
}

