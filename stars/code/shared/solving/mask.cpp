/*
 *  © 2013 Columbia University.  All Rights Reserved.
 *  This file is part of STARS, the Star Tracking Attitude Reconstruction
 *  Software package, originally created for EBEX by Daniel Chapman.
 */

#include "mask.h"
#include <math.h>
#include <boost/format.hpp>
#include "../../parameters/manager.h"

using namespace Shared::Solving;

Mask::Mask()
{
    enabled = false;
    counter = 0;
    for (int i=0; i<num_fields; i++) {
        fields[i] = 0;
    }

    block_size = 128; // needs to be an integer multiple of cell_size
    bits_per_word = 32;
    num_blocks_x = 0;
    num_blocks_y = 0;
}

void Mask::init(Parameters::Manager& params)
{
    enabled = params.general.try_get("imaging.selective_mask.enabled", false);
    for (int i=0; i<num_fields; i++) {
        fields[i] = params.general.try_get((boost::format("imaging.selective_mask.field%d")%i).str(),
            (unsigned int) 0);
    }
    num_blocks_x = params.general.image_width / block_size;
    num_blocks_y = params.general.image_height / block_size;
}

Mask& Mask::operator=(const Mask &rhs)
{
    if (this != &rhs) {
        enabled = rhs.enabled;
        counter = rhs.counter;
        memcpy(fields, rhs.fields, sizeof(uint32_t)*num_fields);
    }
    return *this;
}

bool Mask::cell_masked(unsigned int corner_x, unsigned int corner_y)
{
    if (!enabled) {
        return false;
    }
    return block_masked(corner_x/block_size, corner_y/block_size);
}

bool Mask::block_masked(unsigned int i, unsigned int j)
{
    if (!enabled) {
        return false;
    }
    int n = j*num_blocks_x + i;
    int field_num = int(floor((double) n / (double) bits_per_word));
    int field_bit = n - field_num * bits_per_word;
    int value = (fields[field_num] >> field_bit) & 1;
	if (value == 1) {
		return true;
	}
	return false;
}

