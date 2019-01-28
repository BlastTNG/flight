#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#include <stdarg.h>

// groundhog linklist
#include "linklist.h"
#include "linklist_writer.h"
#include "linklist_compress.h"

#define GROUNDHOG_OPEN_NEW_RAWFILE 0x01
extern superframe_t * superframe;

// groundhog helper functions
linklist_rawfile_t * groundhog_open_new_rawfile(linklist_rawfile_t *, linklist_t *, char *);
int groundhog_check_for_fileblocks(linklist_t * ll);
int groundhog_unpack_fileblocks(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer);
int groundhog_process_and_write(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags);

// groundhog required functions (to be defined in the groundhog main c file)
superframe_t groundhog_init_superframe();
int groundhog_init_linklists(char * filedir, int flags);

struct UDPSetup {
  char name[80];
  char addr[80];
  unsigned int port;
  unsigned int maxsize;
  unsigned int packetsize;
  int downlink_index;
};

struct TlmReport {
  linklist_t * ll;
  uint64_t framenum;
  int allframe;
};




#endif
