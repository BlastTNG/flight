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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include "mputs.h"
#include "crc.h"

/* multilog! */
#define MAX_LOGFILES 11
static int n_logfiles = 1;
static FILE* logfiles[11] = { 0 };

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
    for (p = threadNames; p != NULL; p = p->next)
        if (p->tid == tid) return p->name;

    // not found, just print tid
    snprintf(failed_lookup_buffer, TID_NAME_LEN, "%u", (unsigned) tid);
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
 * to your initialization code.
 */
void mputs(buos_t flag, const char* message) {
  char *buffer;
  static struct timeval last_time = {0};
  static uint32_t last_crc = 0;
  static int repeat_count = 0;
  uint32_t crc;

  struct timeval t;
  struct tm now;
  static const char month[12][4] = {
                                  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };
  static const char marker[mem + 1] = {
                                          [err] = '*',
                                          [fatal] = '!',
                                          [info] = '-',
                                          [sched] = '#',
                                          [startup] = '>',
                                          [tfatal] = '$',
                                          [warning] = '=',
                                          [mem] = '@',
                                          [none] = '?'
  };
  int tid = syscall(SYS_gettid);
  int i;

  /// Set this here to ensure it is correct when we write.
  logfiles[0] = stdout;

  if (flag == none || flag == mem) return;

  /* time */
  gettimeofday(&t, NULL);
  t.tv_sec += TEMPORAL_OFFSET;
  gmtime_r(&t.tv_sec, &now);

  blast_tmp_sprintf(buffer,
           "%c%3s-%02d-%02d "           // marker, Month-Day-Year
           "%02d:%02d:%02d.%03d%c "     // Hours:Minutes:Seconds, Marker
           TID_NAME_FMT ": "            // Thread name
           "%s\n",                      // Message
           marker[flag], month[now.tm_mon], now.tm_mday, now.tm_year - 100,
           now.tm_hour, now.tm_min, now.tm_sec, (int)(t.tv_usec/1000), marker[flag],
           threadNameLookup(tid),
           message);

    crc = crc32(BLAST_MAGIC32, message, strlen(message));

    if (crc != last_crc || t.tv_sec - last_time.tv_sec > 10) {
        for (i = 0; i < n_logfiles; ++i) {
            if (repeat_count) {
                fprintf(logfiles[i],
                        "%40s Last message repeats %d times\n", " ", repeat_count);
            }
            fputs(buffer, logfiles[i]);
            fflush(logfiles[i]);
        }
        repeat_count = 0;

        if (flag == fatal) {
            for (i = 0; i < n_logfiles; ++i) {
                fputs("!! Last error is FATAL.  Cannot continue.\n", logfiles[i]);
                fflush(logfiles[i]);
            }
            exit(1);
        }

        if (flag == tfatal) {
            for (i = 0; i < n_logfiles; ++i) {
                fprintf(logfiles[i], "$$ Last error is THREAD FATAL. Thread [" TID_NAME_FMT " (%5i)] exits.\n",
                        threadNameLookup(tid), tid);
                fflush(logfiles[i]);
            }
            pthread_exit(NULL);
        }
    } else {
        repeat_count++;
    }
    last_crc = crc;
    last_time.tv_sec = t.tv_sec;
}
