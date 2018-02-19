/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004-2005 D. V. Wiebe
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

#include <stdbool.h>
#include <sys/time.h>   /* SYSV time (struct timeval, struct timezone) */
#include <netinet/in.h> /* ARPA Internet specification (struct sockaddr_in) */

/* Don't use PATH_MAX.  It isn't. */
#define FR_PATH_MAX 8192

#include <dirent.h>     /* for MAXNAMELEN on BSD */
#ifndef NAME_MAX
#ifdef MAXNAMELEN
#define NAME_MAX MAXNAMELEN
#else
#define NAME_MAX 255 /* iunno */
#endif
#endif


#if FIELD_LEN < 6
#  define FIELD_MAX 6
#else
#  define FIELD_MAX FIELD_LEN
#endif

struct rc_struct {
  int auto_reconnect, daemonise, force_quenya, force_stdio, flakey_source,
      framefile, gzip_output, persist, quenya, remount, silent, write_curfile,
      extra_format;
  int write_mode; /* 0 = normal ; 1 = overwrite ; 2 = resume */

  struct sockaddr_in addr;
  int csock, dsock;

  int sufflen;
  long int resume_at;
  int source_is_curfile;
  char* symlink_name;
  char* remount_dir;
  char* output_curfile;
  char* output_dirfile;
  char* source;
  char* dest_dir;

  struct timezone tz;
  char* chunk;
  char* dirfile;
  char* dirname;
  char* hostname;
  char* telemetry; // lab, tdrss, biphase, pilot
};

struct ri_struct {
  int read;
  int chunk_total;
  int old_total;
  int reader_done;

  struct timeval last;  // Last time written to dirfile
  int lw;               // Total number of samples written previously
  int lr;               // Total number of samples written previously
  int wrote;                // Current number of samples written
  bool new_channels;        // Have we received a new channels structure?
  bool channels_ready;      // Is the channels structure initialized?
  bool symlink_updated;     // Is the symbolic link current with the new dirfile?
  bool writer_done;
  bool tty;              // Boolean is there a message for output?
  bool frame_rate_reset; // Boolean to reset the framerate to the default
};

/* interthread communication */
extern struct rc_struct rc;
extern struct ri_struct ri;

extern sigset_t signals;

#ifdef DEBUG
# define dtracevoid() printf("%s()\n", __FUNCTION__)
# define dtrace(fmt, ...) printf("%s(" fmt ")\n", __FUNCTION__, __VA_ARGS__)
# define dreturn(fmt, val) printf("%s = " fmt "\n", __FUNCTION__, val)
#if DEBUG & 0x1
# define DEBUG_FASTSAME
#endif
#if DEBUG & 0x2
# define DEBUG_SEQUENCING
#endif
#else /* not debug */
# define dtracevoid()
# define dtrace(...)
# define dreturn(...)
#endif /* defined DEBUG */

#endif
