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
#define LINKLIST_FILESAVE_DIR "/data/etc/downloaded_files"

#define BLOCK_FILE_MASK 0x80000000
#define BLOCK_OVERRIDE_CURRENT 0x01

#define STREAM_MUST_SEND 0x01
#define STREAM_DONT_CLEAR 0x02

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
int compress_linklist_internal(uint64_t, linklist_t *, uint8_t *);
int compress_linklist_internal_opt(uint64_t, linklist_t *, uint8_t *, uint32_t, int);
double decompress_linklist(uint8_t *, linklist_t * , uint8_t *);
double decompress_linklist_opt(uint8_t *, linklist_t *, uint8_t *, uint32_t, int);
double decompress_linklist_internal(uint64_t, linklist_t * , uint8_t *);
double decompress_linklist_internal_opt(uint64_t, linklist_t *, uint8_t *, uint32_t, int);

uint8_t * allocate_superframe(superframe_t *);
int write_allframe(uint8_t *, superframe_t *, uint8_t *);
int read_allframe(uint8_t *, superframe_t *, uint8_t *);
void packetize_block(struct block_container * , uint8_t *);
void depacketize_block(struct block_container * , uint8_t *);
unsigned int linklist_blocks_queued(linklist_t *);
void packetize_stream(struct stream_container * , uint8_t *);
void depacketize_stream(struct stream_container * , uint8_t *);
int assign_file_to_stream(stream_t *, char *, int, int);
int assign_file_to_streamlist(stream_t **, char *, int, int);
int remove_file_from_stream(stream_t *);
void write_next_stream(stream_t *, uint8_t *, unsigned int, unsigned int);
void write_next_streamlist(stream_t **, uint8_t *, unsigned int, unsigned int);

block_t * block_find_by_name(linklist_t *, char *);
int linklist_send_file_by_block(linklist_t *, char *, char *, int32_t, int);
int linklist_send_file_by_block_ind(linklist_t *, char *, char *, int32_t, int, unsigned int, unsigned int);

stream_t * stream_find_by_name(linklist_t *, char *);
stream_t ** linklist_get_streamlist(linklist_t **, char *);
int linklist_assign_file_to_stream(linklist_t *, char *, char *, int, int);
int linklist_remove_file_from_stream(linklist_t *, char *);
void linklist_write_next_stream(linklist_t *, char *, uint8_t *, unsigned int, unsigned int);

FILE * fpreopenb(char *);
uint8_t randomized_buffer(uint8_t *, unsigned int, unsigned int);

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */
