#ifndef INCLUDE_LINKLIST_WRITER_H
#define INCLUDE_LINKLIST_WRITER_H

#include <inttypes.h>
#include "linklist.h"

struct linklist_dirfile {
  char filename[80];
  unsigned int framenum;
  linklist_t * ll;
  FILE ** bin;
};

struct linklist_rawfile {
  char basename[128];
  unsigned int framenum;
  unsigned int filecount;
  linklist_t * ll;
  FILE * fp;
};

typedef struct linklist_dirfile linklist_dirfile_t;
typedef struct linklist_rawfile linklist_rawfile_t;

linklist_dirfile_t * open_linklist_dirfile(linklist_t *, char *);
void close_and_free_linklist_dirfile(linklist_dirfile_t *);
double write_linklist_dirfile(linklist_dirfile_t *, uint8_t *);

void increment_linklist_rawfile(linklist_rawfile_t *);
linklist_rawfile_t * open_linklist_rawfile(linklist_t *, char *);
void close_and_free_linklist_rawfile(linklist_rawfile_t *);
unsigned int write_linklist_rawfile(linklist_rawfile_t *, uint8_t *);

#endif /* INCLUDE_LINKLIST_WRITER_H */
