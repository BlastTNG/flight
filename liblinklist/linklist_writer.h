#ifndef INCLUDE_LINKLIST_WRITER_H
#define INCLUDE_LINKLIST_WRITER_H

#include <inttypes.h>
#include "linklist.h"

#define LL_RAWFILE_DUMMY 0x8
#define LL_DIRFILE_NUM_EXTRA 3 // the number of extra ll_dirfile-specific fields are included
#define LL_FRAMEBIN_NAME "ll_frame_tally"
#define LL_FRAMEBIN_READ_BLOCK_SIZE 4096

struct linklist_dirfile {
  char filename[LINKLIST_MAX_FILENAME_SIZE];
  unsigned int framenum;
  linklist_t * ll;
  uint8_t * map;
  FILE * format;
  FILE ** bin;
  FILE ** blockbin;
  FILE ** streambin;
  FILE ** extrabin;
  FILE *  framebin;

  float data_integrity;
  uint32_t local_time;

  uint8_t tally_word;
  unsigned int * missing_blks_start;
  unsigned int * missing_blks_end;
  unsigned int n_missing_blks;
  unsigned int n_missing_blks_alloc;
};

struct linklist_rawfile {
  char basename[LINKLIST_MAX_FILENAME_SIZE];
  unsigned int framenum; // current frame number
  unsigned int fileindex; // current file index for rawfile chunks
  unsigned int framesize; // size [Bytes] of a frame
  unsigned int fpf; // number of frames per rawfile chunk
  int isseekend; // file index for the end of the rawfile (i.e. last chunk)
  linklist_t * ll;
  FILE * fp;
};

typedef struct linklist_dirfile linklist_dirfile_t;
typedef struct linklist_rawfile linklist_rawfile_t;
typedef struct linklist_dirfile_meta linklist_dirfile_meta_t;

int seek_linklist_dirfile(linklist_dirfile_t *, unsigned int);
int flush_linklist_dirfile(linklist_dirfile_t *);

linklist_dirfile_t * open_linklist_dirfile(char *, linklist_t *);
linklist_dirfile_t * open_linklist_dirfile_opt(char *, linklist_t *, unsigned int);
void close_and_free_linklist_dirfile(linklist_dirfile_t *);
double write_linklist_dirfile(linklist_dirfile_t *, uint8_t *);
double write_linklist_dirfile_opt(linklist_dirfile_t *, uint8_t *, unsigned int);
void map_frames_linklist_dirfile(linklist_dirfile_t *);

int seek_linklist_rawfile(linklist_rawfile_t *, unsigned int);
int seekend_linklist_rawfile(linklist_rawfile_t *);
int flush_linklist_rawfile(linklist_rawfile_t *);
int tell_linklist_rawfile(linklist_rawfile_t *);

linklist_rawfile_t * open_linklist_rawfile(char *, linklist_t *);
linklist_rawfile_t * open_linklist_rawfile_opt(char *, linklist_t *, unsigned int);
void close_and_free_linklist_rawfile(linklist_rawfile_t *);
int write_linklist_rawfile(linklist_rawfile_t *, uint8_t *);
int write_linklist_rawfile_opt(linklist_rawfile_t *, uint8_t *, unsigned int);
int write_linklist_rawfile_with_allframe(linklist_rawfile_t *, uint8_t *, uint8_t *);
int read_linklist_rawfile(linklist_rawfile_t *, uint8_t *);

void make_linklist_rawfile_name(linklist_t *, char *);
void create_rawfile_symlinks(linklist_rawfile_t *, char *);

#endif /* INCLUDE_LINKLIST_WRITER_H */
