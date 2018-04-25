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
#define MAX_NUM_LINKLIST_FILES 128 
#define LINK_HASH_MULT 233
#define MIN_CHKSM_SPACING 200 // number of bytes after which a checksum is automatically appended
#define PACKET_HEADER_SIZE 12

#define MAX_DATA_BLOCKS 8 // maximum data blocks per linklist
#define DEF_BLOCK_ALLOC 10000 // default block buffer size [bytes] 

#define ALL_FRAME_SERIAL 0x42424242
#define LL_PARSE_CHECKSUM "_TLM_CHECKSUM_" 
#define DEFAULT_LINKLIST_DIR "/data/etc/linklists/"
#define ALL_TELEMETRY_NAME "all_telemetry.ll"

#define SUPERFRAME_EXT ".sf.bin"
#define SUPERFRAME_FORMAT_EXT ".sf.format"
#define LINKLIST_EXT ".ll.bin"
#define LINKLIST_FORMAT_EXT ".ll.format"
#define LINKLIST_FILE_SERIAL_IND "# Serial="
#define LINKLIST_FILE_SIZE_IND "# Blk Size="

#define LL_NO_AUTO_CHECKSUM 0x01
#define LL_INCLUDE_ALLFRAME 0x02

#define SUPERFRAME_READY 0x1
#define COMPFRAME_READY 0x2

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define SF_FIELD_LEN 80
#define SF_UNITS_LEN 80

#define linklist_info printf
#define linklist_err printf
#define linklist_warn printf
#define linklist_fatal printf


#include <stdio.h>
#include <openssl/md5.h>
#include <inttypes.h>

#ifdef __cplusplus

extern "C"{

#endif

enum SFType
{
  SF_UINT8, SF_UINT16, SF_UINT32, SF_UINT64,
  SF_INT8, SF_INT16, SF_INT32, SF_INT64,
  SF_FLOAT32, SF_FLOAT64, SF_NUM 
};

struct linklist_struct;
struct link_entry;
struct block_container;
struct sf_entry;
struct superframe_struct;

struct sf_entry 
{
	char field[SF_FIELD_LEN];      // name of entry for FileFormats and CalSpecs
	uint8_t type;               // Type of data stored
	uint32_t spf;               // Samples per frame
  uint32_t start;             // Start location of first sample in the superframe
  uint32_t skip;              // Bytes to skipe between samples
	char quantity[SF_UNITS_LEN];   // eg, "Temperature" or "Angular Velocity"
	char units[SF_UNITS_LEN];      // eg, "K" or "^o/s"
  void *var;                  // Pointer to data
  struct superframe_struct * superframe; // Pointer to corresponding superframe
};

struct superframe_struct
{
  unsigned int n_entries;
  unsigned int size;
  uint64_t serial;

  struct sf_entry * entries;
  struct sf_entry ** hash_table;
  unsigned int hash_table_size;
  unsigned int allframe_size;

	double (*datatodouble)(uint8_t *, uint8_t);
	int (*doubletodata)(uint8_t *, double, uint8_t);
};

struct link_entry
{  
  uint32_t start; // start byte for entry in compressed frame
  uint8_t comp_type; // compression type
  uint32_t blk_size; // size/allocation of entry in compressed frame
  uint32_t num; // number of samples per compressed frame
  double compvars[20]; // compression variable scratchpad
  struct sf_entry * tlm; // pointer to corresponding entry from superframe
  struct linklist_struct * linklist; // pointer to corresponding linklist
};

struct linklist_struct
{
  char name[64]; // name of the linklist file
  uint32_t n_entries; // number of entries in the list
  uint32_t blk_size; // size of entire compressed frame
  uint8_t serial[MD5_DIGEST_LENGTH]; // serial/id number for list
  struct link_entry * items; // pointer to entries in the list
  struct block_container * blocks; // pointer to blocks
  unsigned int num_blocks; // number of data block fields
  int flags; // flags for checksums, auto increment, etc
  struct superframe_struct * superframe; // pointer to corresponding superframe
};

struct block_container
{
  char name[80];
  uint16_t id;
  unsigned int i, n, num;
  struct link_entry * le;
  unsigned int alloc_size;
  unsigned int curr_size;
  uint8_t * buffer;

  char filename[80];
  FILE *fp;
};

typedef struct linklist_struct linklist_t;
typedef struct link_entry linkentry_t;
typedef struct block_container block_t;
typedef struct sf_entry superframe_entry_t;
typedef struct superframe_struct superframe_t;

extern const char * SF_TYPES_STR[]; 

#define STR(s) #s

linklist_t * parse_linklist_format(superframe_t *, char *);
linklist_t * parse_linklist_format_opt(superframe_t *, char *, int);
void write_linklist_format(linklist_t *, char *);
void write_linklist_format_opt(linklist_t *, char *, int);
linklist_t * generate_superframe_linklist(superframe_t *);
linklist_t * generate_superframe_linklist_opt(superframe_t *, int);
superframe_t * parse_superframe_format(char *);
void write_superframe_format(superframe_t *, char *);
void linklist_assign_datatodouble(superframe_t *, double (*func)(uint8_t *, uint8_t));
void linklist_assign_doubletodata(superframe_t *, int (*func)(uint8_t *, double, uint8_t));
uint64_t generate_superframe_serial(superframe_t *); 
superframe_t * linklist_build_superframe(superframe_entry_t *,
                                         double (*datatodouble)(uint8_t *, uint8_t), 
                                         int (*doubletodata)(uint8_t *, double, uint8_t));

superframe_entry_t * superframe_find_by_name(superframe_t *, char *);
uint32_t get_superframe_entry_size(superframe_entry_t *);
const char * get_sf_type_string(uint8_t);
uint8_t get_sf_type_int(char *);
int read_linklist_formatfile_size(char *);

int linklist_generate_lookup(linklist_t **);
linklist_t * linklist_lookup_by_serial(uint32_t);
void delete_linklist(linklist_t *);
int load_all_linklists(superframe_t *, char *, linklist_t **, unsigned int);
linklist_t * linklist_find_by_name(char *, linklist_t **);
block_t * linklist_find_block_by_pointer(linklist_t * ll, linkentry_t * le);

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */