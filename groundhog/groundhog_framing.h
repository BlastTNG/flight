/* 
 * groundhog_framing.h: 
 *
 * This software is copyright (C) 2013-2015 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 31, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_FRAMING_H_
#define INCLUDE_FRAMING_H_

#define NUM_FRAMES 20 
#include "FIFO.h"

struct DownLinkStruct {
    char name[32];
    char frame_name[RATE_END][32];
    struct Fifo fifo;
    void *data[RATE_END];
};

enum DownLinkTypes {PILOT, BI0, HIGHRATE, 
                      NUM_DOWNLINKS};

// int framing_init(channel_t *channel_list, derived_tng_t *m_derived);
int framing_init(void);
void framing_shutdown(void);

void framing_extract_and_publish(uint8_t *data_buffer, int index, E_RATE rate);
void groundhog_publish(void *);

extern struct DownLinkStruct downlink[NUM_DOWNLINKS];

#endif /* INCLUDE_FRAMING_H_ */
