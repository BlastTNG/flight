/* 
 * linklist_compress.h: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of the SuperBIT project, modified and adapted for BLAST-TNG.
 *
 * linklist is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * linklist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Jan 25, 2018 by Javier Romualdez
 */


#ifndef LINKLIST_COMPRESS_H_
#define LINKLIST_COMPRESSH_

#include "linklist.h"

#ifdef __cplusplus

extern "C"{

#endif

struct superframe
{
  uint32_t * offset; // locates the start of frames at a given rate
  uint32_t * skip; // number of bytes to skip in superframe between frames at a particular rate
  uint8_t * data; // 1 Hz data block for the superframe
};

typedef struct superframe superframe_t;

extern superframe_t superframe;
extern int (*compressFunc[]) (uint8_t *, struct link_entry *, uint8_t *);
extern int (*decompressFunc[]) (uint8_t *, struct link_entry *, uint8_t *);

int compress_linklist(uint8_t *, linklist_t * , uint8_t *);
double decompress_linklist(uint8_t *, linklist_t * , uint8_t *);

uint32_t allocate_superframe(const channel_t * const );
uint32_t get_channel_start_in_superframe(const channel_t * );
uint32_t get_channel_skip_in_superframe(const channel_t * );
unsigned int add_frame_to_superframe(void * , E_RATE );
unsigned int extract_frame_from_superframe(void * , E_RATE );

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */