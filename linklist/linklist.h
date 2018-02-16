/* 
 * linklist.h: 
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

#ifndef LINKLIST_H_
#define LINKLIST_H_

#define NO_COMP 0xff
#define MAX_NUM_LINKFILE 32 
#define LINK_HASH_MULT 233
#define MIN_CHKSM_SPACING 200 // number of bytes after which a checksum is automatically appended
#define PACKET_HEADER_SIZE 12

#define MAX_DATA_BLOCKS 8 // maximum data blocks per linklist
#define DEF_BLOCK_ALLOC 10000 // default block buffer size [bytes] 

#define ALL_FRAME_SERIAL 0x42424242
#define LL_PARSE_CHECKSUM "_TLM_CHECKSUM_" 

#define SUPERFRAME_READY 0x1
#define COMPFRAME_READY 0x2

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#include <openssl/md5.h>

#include "channels_tng.h"

#ifdef __cplusplus

extern "C"{

#endif

struct link_entry
{  
  uint32_t start; // start byte for entry in compressed frame
  uint8_t comp_type; // compression type
  uint32_t blk_size; // size/allocation of entry in compressed frame
  uint32_t num; // number of samples per compressed frame
  double compvars[20]; // compression variable scratchpad
  channel_t * tlm; // pointer to corresponding telemetry entry from telemlist
};

struct block_container
{
  char name[80];
  uint16_t intname;
  unsigned int i, n, num;
  struct link_entry * le;
  unsigned int alloc_size;
  unsigned int curr_size;
  uint8_t * buffer;
};

struct link_list
{
  uint32_t n_entries; // number of entries in the list
  uint32_t blk_size; // size of entire compressed frame
  uint8_t serial[MD5_DIGEST_LENGTH]; // serial/id number for list
  struct link_entry * items; // pointer to entries in the list
  struct block_container * blocks; // pointer to blocks
  unsigned int num_blocks; // number of data block fields
  uint8_t data_ready; // indicates whether or not data can be read from the superframe/compframe
  uint8_t * superframe; // a pointer to the superframe to/from which data is decompressed/compressed
  uint8_t * compframe; // a pointer to the compressed from to/from which data is compressed/decompressed
};

typedef struct link_list linklist_t;
typedef struct link_entry linkentry_t;

linklist_t * parse_linklist(char *);
unsigned int get_channel_size(const channel_t *);
unsigned int get_channel_spf(const channel_t *);
unsigned int get_spf(unsigned int);

int linklist_generate_lookup(linklist_t **);
linklist_t * linklist_lookup_by_serial(uint32_t);
void linklist_set_superframe_ready(linklist_t *);
void linklist_set_compframe_ready(linklist_t *);
void set_all_linklist_superframe_ready(linklist_t **);
void set_all_linklist_compframe_ready(linklist_t **);

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */




