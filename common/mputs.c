/* mputs.c: BUOS backend for mcp/pcm/mpc
 *
 * This software is copyright (C) 2002-2013 University of Toronto
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "mputs.h"

/* multilog! */
#define MAX_LOGFILES 10
static int n_logfiles = 0;
static FILE* logfiles[10];

/* tid to name lookup list */
#define stringify_(x) #x
#define stringify(x) stringify_(x)
#define TID_NAME_LEN  6
#define TID_NAME_FMT  "%" stringify(TID_NAME_LEN) "s"
static struct tid_name {
  int tid;
  char name[TID_NAME_LEN+1];
  struct tid_name* next;
} *threadNames = NULL;

/* This function can be used to name a thread */
void nameThread(const char* name)
{
  struct tid_name* new_node = (struct tid_name*)malloc(sizeof(struct tid_name));
  new_node->tid = syscall(SYS_gettid);
  strncpy(new_node->name, name, TID_NAME_LEN);
  new_node->name[TID_NAME_LEN] = '\0';
  new_node->next = threadNames;
  threadNames = new_node;
  //bprintf(startup, "New thread (tid %d)", new_node->tid);
}

off_t openMCElog(const char *file)
{
  off_t offset = -1;
  struct stat statbuf;

  if (n_logfiles < MAX_LOGFILES) {
    if ((logfiles[n_logfiles] = fopen(file, "a")) == NULL) {
      berror(err, "Can't open log file %s", file);
    } else {
      if (fstat(fileno(logfiles[n_logfiles]), &statbuf) == 0)
        offset = (off_t)statbuf.st_size;

      fputs("!!!!!! LOG RESTART !!!!!!\n", logfiles[n_logfiles++]);
    }
  }

  return offset;
}

static char failed_lookup_buffer[TID_NAME_LEN+1];
static char* threadNameLookup(int tid)
{
  struct tid_name* p;
  for(p = threadNames; p != NULL; p = p->next)
    if (p->tid == tid)
      return p->name;

  //not found, just print tid
  snprintf(failed_lookup_buffer, TID_NAME_LEN, "%u", (unsigned)tid);
  failed_lookup_buffer[TID_NAME_LEN] = '\0';
  return failed_lookup_buffer;
}
  

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 6                /* 2*(marker+space) + EOL + NUL */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS.mmm " */ \
  - TID_NAME_LEN - 2 /* thread name "ThName: " */ \
)

/* I/O function to be used by bputs, bprintf, etc.  To use it, add
 *
 *     buos_use_func(mputs);
 *
 * to your initialisation code.
 */
void mputs(buos_t flag, const char* message) {
  char buffer[MPRINT_BUFFER_SIZE];
  struct timeval t;
  struct timezone tz; /* We never use this, but gettimeofday won't let us
                         give it a NULL -- also it's obsolete under linux */
  struct tm now;
  char local[1024];
  char *bufstart, *bufptr, *lastchr, *firstchr;
  int len;
  char marker[4];
  int tid = syscall(SYS_gettid);
  int i;

  /* time */
  gettimeofday(&t, &tz);
  t.tv_sec += TEMPORAL_OFFSET;

  switch(flag) {
    case err:
      strcpy(marker, "* ");
      break;
    case fatal:
      strcpy(marker, "! ");
      break;
    case info:
      strcpy(marker, "- ");
      break;
    case sched:
      strcpy(marker, "# ");
      break;
    case startup:
      strcpy(marker, "> ");
      break;
    case tfatal:
      strcpy(marker, "$ ");
      break;
    case warning:
      strcpy(marker, "= ");
      break;
    case mem:
      return;  /* don't record mem messages at all */
      strcpy(marker, "m ");
      break;
    case none:
        return;
    default:
      strcpy(marker, "? ");
      break;
  }
  strcpy(buffer, marker);

  /* we need a writable copy of the string */
  strncpy(local, message, 1023);
  local[1023] = '\0';

  for(bufstart = buffer; *bufstart != '\0' && bufstart < buffer
      + 1024; ++bufstart);


  strftime(bufstart, 1023, "%F %T", gmtime_r(&t.tv_sec, &now));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, ".%03li ", t.tv_usec/1000);
  strcat(buffer, marker);

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, TID_NAME_FMT ": ", threadNameLookup(tid));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

    /* output the formatted string line by line */
    for (firstchr = bufptr = local; *bufptr != '\0' &&
        bufptr < local + 1023; ++bufptr) {

      /* writeout the string when we find a newline or the EOS */
      if (*bufptr == '\n' || *(bufptr + 1) == '\0') {
        lastchr = (*bufptr == '\n') ? bufptr : bufptr + 1;
        *lastchr = '\0';

        /* compute length of string to writeout */
        len = lastchr - firstchr + 1;
        if (len > MAX_MPRINT_STRING - 1)
          len = MAX_MPRINT_STRING - 1;

        /* append string part and a newline to preamble */
        strncpy(bufstart, firstchr, len);
        *(bufstart + len + 1) = '\0';
        strcat(bufstart, "\n");
        for (i = 0; i < n_logfiles; ++i) {
          fputs(buffer, logfiles[i]);
          fflush(logfiles[i]);
        }
        if (n_logfiles == 0 || flag != mem) {
          fputs(buffer, stdout);
          fflush(stdout);
        }

        firstchr = bufptr + 1;
      }
    }

  if (flag == fatal) {
    for (i = 0; i < n_logfiles; ++i) {
      fputs("!! Last error is FATAL.  Cannot continue.\n", logfiles[i]);
      fflush(logfiles[i]);
    }
    fputs("!! Last error is FATAL.  Cannot continue.\n", stdout);
    fflush(stdout);

    exit(1);
  }

  if (flag == tfatal) {
    for (i = 0; i < n_logfiles; ++i) {
      fprintf(logfiles[i], "$$ Last error is THREAD FATAL. Thread [" 
          TID_NAME_FMT " (%5i)] exits.\n", threadNameLookup(tid), tid);
      fflush(logfiles[i]);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [" 
          TID_NAME_FMT " (%5i)] exits.\n", threadNameLookup(tid), tid);
    fflush(stdout);

    pthread_exit(NULL);
  }
}
