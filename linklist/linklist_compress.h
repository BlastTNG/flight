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
