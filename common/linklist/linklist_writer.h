#ifndef INCLUDE_LINKLIST_DIRFILE_H
#define INCLUDE_LINKLIST_DIRFILE_H

#include "linklist.h"

struct linklist_dirfile {
  char filename[80];
  linklist_t * ll;
  FILE ** bin;
};

typedef struct linklist_dirfile linklist_dirfile_t;

#endif /* INCLUDE_LINKLIST_DIRFILE_H */
