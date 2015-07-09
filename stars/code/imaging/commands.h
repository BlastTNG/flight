/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef IMAGING__COMMANDS_H
#define IMAGING__COMMANDS_H

namespace Imaging
{
    namespace LensCommands
    {

enum commands_t {
    flush_birger = 0,
    init_focus,
    get_focus,
    set_focus,
    set_focus_incremental,
    init_aperture,
    get_aperture,
    set_aperture,
    version_string,
    clearing_read_buffer
};
static const unsigned int num_commands = 9;

    }
}

#endif
