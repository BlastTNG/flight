#ifndef LINKLIST_COMPRESS_H_
#define LINKLIST_COMPRESSH_

#include "linklist.h"

#ifdef __cplusplus

extern "C"{

#endif

struct superframe
{
  uint32_t * byte_map;
  uint8_t * data;
};

typedef struct superframe superframe_t;

extern superframe_t mainframe;
extern int (*compressFunc[]) (uint8_t *, struct link_entry *, uint8_t *);
extern int (*decompressFunc[]) (uint8_t *, struct link_entry *, uint8_t *);

uint32_t allocate_superframe(const channel_t * const );
uint32_t get_channel_start_in_superframe(const channel_t * );

#ifdef __cplusplus
}
#endif

#endif /* LINKLIST_H_ */
