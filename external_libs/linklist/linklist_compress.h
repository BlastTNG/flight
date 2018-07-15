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
#define LINKLIST_COMPRESS_H_

#include <inttypes.h>
#include "linklist.h"

#define ALLFRAME_SERIAL 0x42424242
#define BLOCK_FILE_MASK 0x8000
#define LINKLIST_FILESAVE_DIR "/data/etc/downloaded_files"

#ifdef __cplusplus

extern "C"{

#endif


enum dataCompressTypes {
  FIXED_PT_16BIT, // fixed point 16 bit compression
  FIXED_PT_32BIT, // fixed point 32 bit compression
  MEAN_FLOAT_DELTA_8BIT, // 8 bit derivative compression
  MEAN_FLOAT_8BIT, // 8 bit moving average compression

  NUM_COMPRESS_TYPES
};

struct dataCompressor {
  int id;
  char name[80];
  int (*compressFunc) (uint8_t *, linkentry_t *, uint8_t *);
  int (*decompressFunc) (uint8_t *, linkentry_t *, uint8_t *);
};

extern struct dataCompressor compRoutine[NUM_COMPRESS_TYPES+1];

#define COMPRESS(x) (int)x, #x
#define LL_CHANNEL_DATA(_chan) _chan->var

#define LL_IGNORE_CHECKSUM 0x02
int compress_linklist(uint8_t *, linklist_t *, uint8_t *);
int compress_linklist_opt(uint8_t *, linklist_t *, uint8_t *, uint32_t, int);
double decompress_linklist(uint8_t *, linklist_t * , uint8_t *);
double decompress_linklist_opt(uint8_t *, linklist_t *, uint8_t *, uint32_t, int);

uint8_t * allocate_superframe(superframe_t *);
int write_allframe(uint8_t *, superframe_t *, uint8_t *);
int read_allframe(uint8_t *, superframe_t *, uint8_t *);
void packetize_block_raw(struct block_container * , uint8_t *);
void depacketize_block_raw(struct block_container * , uint8_t *);
void send_file_to_linklist(linklist_t *, char *, char *);
FILE * fpreopenb(char *);
uint8_t randomized_buffer(uint8_t *, unsigned int, unsigned int);

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */
