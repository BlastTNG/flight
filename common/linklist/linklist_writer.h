#ifndef INCLUDE_LINKLIST_WRITER_H
#define INCLUDE_LINKLIST_WRITER_H

#include "linklist.h"

struct linklist_dirfile {
  char filename[80];
  unsigned int framenum;
  linklist_t * ll;
  FILE ** bin;
};

typedef struct linklist_dirfile linklist_dirfile_t;

linklist_dirfile_t * open_linklist_dirfile(linklist_t *, char *);
void close_and_free_linklist_dirfile(linklist_dirfile_t *);
double write_linklist_dirfile(linklist_dirfile_t *, uint8_t *, unsigned int);

#endif /* INCLUDE_LINKLIST_WRITER_H */