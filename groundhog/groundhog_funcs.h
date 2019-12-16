#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#include <stdarg.h>

#define NOR "\x1B[0m"
#define RED "\x1B[31;1m"
#define GRN "\x1B[32;1m"
#define YLW "\x1B[33;1m"
#define BLU "\x1B[34;1m"
#define MAG "\x1B[35;1m"
#define CYN "\x1B[36;1m"
#define NOC "\x1B[?25l"
#define CUR "\x1B[?25h"

#define GROUNDHOG_LOG "/data/etc/groundhog.log"

// groundhog linklist
#include "linklist.h"
#include "linklist_writer.h"
#include "linklist_compress.h"

#define GROUNDHOG_OPEN_NEW_RAWFILE      0x01
#define GROUNDHOG_REUSE_VALID_RAWFILE   0x02
#define GROUNDHOG_EXTRACT_TO_DISK       0x04

extern superframe_t * superframe;
extern int verbose;

// groundhog helper functions
void daemonize();
linklist_rawfile_t * groundhog_open_rawfile(linklist_rawfile_t *, linklist_t *, char *, int);
int groundhog_check_for_fileblocks(linklist_t * ll, char *);
int groundhog_unpack_fileblocks(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str, 
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags);
int64_t groundhog_process_and_write(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags);

// groundhog required functions (to be defined in the groundhog main c file)
void groundhog_write_calspecs(char * fname);

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
  uint64_t prev_framenum;
  uint64_t stale;
  int allframe;
  int type;
};

struct LinklistState {
  uint32_t serial;
  char symname[LINKLIST_MAX_FILENAME_SIZE];
  linklist_rawfile_t * ll_rawfile;
};

// BLAST print functions (required)
// must define groundhog_info, _warn, and _fatal
#define groundhog_info(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define groundhog_warn(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define groundhog_fatal(fmt, ...) printf(fmt, ##__VA_ARGS__); exit(2)


#endif
