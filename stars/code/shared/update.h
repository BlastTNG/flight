/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#pragma once
#ifndef SHARED__UPDATE_H
#define SHARED__UPDATE_H

#include "thread_names.h"

namespace Shared
{
    void update(ThreadNames::Name thread_name);
}

#endif
