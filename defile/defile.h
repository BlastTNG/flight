/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004 D. V. Wiebe
 * 
 * This file is part of defile.
 * 
 * defile is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defile; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef DEFILE_H
#define DEFILE_H

#include <sys/time.h>
#include <linux/limits.h>

#define GPB_LEN (PATH_MAX * 4)
#define FILENAME_LEN (PATH_MAX + NAME_MAX + 1)

#if FIELD_LEN < 6
#  define FIELD_MAX 6
#else
#  define FIELD_MAX FIELD_LEN
#endif

typedef unsigned int chunkindex_t;

struct rc_struct {
  int framefile, persist, remount, write_curfile;
  int write_mode; /* 0 = normal ; 1 = overwrite ; 2 = resume */
  int sufflen;
  long int resume_at;
  int source_is_curfile;
  char* curfile_val;
  char* remount_dir;
  char* output_curfile;
  char* output_dirfile;
  char* source;
  char* dest_dir;

  struct timeval start;
  struct timezone tz;
  char* chunk;
  char* dirfile;
};

struct ri_struct {
  int read;
  int chunk_total;
  int old_total;
  int reader_done;

  int wrote;
  int dirfile_init;
  int writer_done;
  int tty;
};

/* interthread communication */
extern struct rc_struct rc;
extern struct ri_struct ri;

/* funxion prototypes */
void  DirFileWriter(void);
void  FrameFileReader(void);
void  GetDirFile(char*, const char*, char*);
void  PreInitialiseDirFile(void);
void  InitialiseDirFile(int);
void  PushFrame(unsigned short*);
void  Remount(const char*, char*);
int   StaticSourcePart(char*, const char*, chunkindex_t*);

#endif
