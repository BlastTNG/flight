#ifndef LINKLIST_H_
#define LINKLIST_H_

#define NO_COMP 0xff
#define MAX_NUM_LINKFILE 32 
#define LINK_HASH_MULT 233
#define MIN_CHKSM_SPACING 200 // number of bytes after which a checksum is automatically appended

#define MAX_DATA_BLOCKS 8 // maximum data blocks per linklist
#define DEF_BLOCK_ALLOC 10000 // default block buffer size [bytes] 

#define ALL_FRAME_SERIAL 0x42424242
#define LL_PARSE_CHECKSUM "_TLM_CHECKSUM_" 

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

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
};

typedef struct link_list linklist_t;
typedef struct link_entry linkentry_t;

unsigned int get_channel_size(const channel_t * );
unsigned int get_channel_spf(const channel_t * );
unsigned int get_spf(unsigned int );

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */




