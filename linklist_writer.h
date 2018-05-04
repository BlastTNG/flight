#ifndef INCLUDE_LINKLIST_WRITER_H
#define INCLUDE_LINKLIST_WRITER_H

#include <inttypes.h>
#include "linklist.h"

struct linklist_dirfile_meta {
  char * text;
}

struct linklist_dirfile {
  char filename[80];
  unsigned int framenum;
  linklist_t * ll;
  uint8_t * map;
  FILE ** bin;
  struct linklist_dirfile_meta * meta;
};

struct linklist_rawfile {
  char basename[128];
  unsigned int framenum;
  unsigned int fileindex;
  unsigned int framesize;
  unsigned int fpf;
  linklist_t * ll;
  FILE * fp;
};

typedef struct linklist_dirfile linklist_dirfile_t;
typedef struct linklist_rawfile linklist_rawfile_t;
typedef struct linklist_dirfile_meta linklist_dirfile_meta_t;

int seek_linklist_dirfile(linklist_dirfile_t *, unsigned int);
linklist_dirfile_t * open_linklist_dirfile(char *, linklist_t *);
void close_and_free_linklist_dirfile(linklist_dirfile_t *);
double write_linklist_dirfile(linklist_dirfile_t *, uint8_t *);

int seek_linklist_rawfile(linklist_rawfile_t *, unsigned int);
int seekend_linklist_rawfile(linklist_rawfile_t *);
int tell_linklist_rawfile(linklist_rawfile_t *);

linklist_rawfile_t * open_linklist_rawfile(char *, linklist_t *);
void close_and_free_linklist_rawfile(linklist_rawfile_t *);
int write_linklist_rawfile(linklist_rawfile_t *, uint8_t *);
int read_linklist_rawfile(linklist_rawfile_t *, uint8_t *);

void make_linklist_rawfile_name(linklist_t *, char *);
void create_rawfile_symlinks(linklist_rawfile_t *, char *);

#endif /* INCLUDE_LINKLIST_WRITER_H */
