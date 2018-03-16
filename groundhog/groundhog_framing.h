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

struct telemetries {
    uint8_t number;
    const char *types[3];
};

typedef struct
{
    int i_in;
    int i_out;
    uint8_t *framelist[NUM_FRAMES];
    size_t framesize[NUM_FRAMES];
} superframes_list_t;


void initialize_circular_superframes(superframes_list_t *superframes);
void push_superframe(const void *m_frame, superframes_list_t *superframes);

// int framing_init(channel_t *channel_list, derived_tng_t *m_derived);
int framing_init(void);
void framing_shutdown(void);

void framing_publish(void* m_frame, char *telemetry, E_RATE rate);
void framing_publish_244hz(void* m_frame, char *telemetry);
void framing_publish_200hz(void* m_frame, char *telemetry);
void framing_publish_100hz(void* m_frame, char *telemetry);
void framing_publish_5hz(void* m_frame, char *telemetry);
void framing_publish_1hz(void* m_frame, char *telemetry);
void framing_init_mutex();

#endif /* INCLUDE_FRAMING_H_ */
