/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2003-2010 University of Toronto
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
 * along with defile; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* MAINTAINERS PLEASE READ:
 * =======================
 *
 * IF YOU ARE READING THIS because you been given the unfortunate task of
 * maintaining this code in my absense, you have my deepest sympathy.
 *
 * Seriously.
 *
 * This code is an abomination, the result of too many field campaigns, kludgy
 * patches, and a general lack of big-picture foresight.  And make no mistake:
 * a good deal of the blame is mine.  At some point someone should really scrap
 * this thing in its entirety and re-write it from scratch. */
 
/* This code started life in the BLAST master control program (mcp) when we
 * realised that the old-style flat file format wasn't a good way for us to
 * store our data for analysis.  Barth wrote the kernel of this module, the
 * dirfile writing part that makes up the bulk of DirFileWritier, before the
 * BLAST 2003 test flight.  In the field for the test flight campaign, Enzo
 * discovered that overflows in the passacross ring buffer would lead to
 * data corruption.  This he fixed by writing the space-checking routine that
 * makes up the first part of PushFrame.  At the time, the fix was to discard
 * frames that didn't fit, since this code needed to run real-time in mcp.
 *
 * Early in 2004, after the test flight we decided it was best to have mcp go
 * back to writing the flat BiPhase stream, since it was certain to result in a
 * better data set, since any flakiness in the dirfile writer could be shoved
 * to post-analysis, where bugs could be found and fixed as needed and the
 * dirfiles recreated from the raw datastream.  Subsequent development has shown
 * this to have been a very good idea.  At any rate, as a result of this
 * decision, I removed the dirfile writer from mcp, replaced it with a framefile
 * writer, and then wrote defile to create the dirfiles from the framefiles.
 *
 * The original disk writing thread became this writer thread in defile, at
 * first little change from its manifestation in mcp.  This is also the reason
 * why defile is multithreaded.  A lot of harriness could likely be removed by
 * simply making defile unthreaded.  The initialisation routines for the
 * dirfile I extracted from mcp and dumped them in the InitialiseDirFile
 * function.  In defile, instead of dropping frames, we could just wait until
 * there was time to write them, so a wait was put in pushFrame.  With the
 * start of the defile project, complexity and craziness increased:  the
 * introduction of resume mode made a mess of InitialiseDirFile.  Later, Matt
 * added gzip write support, which added another level of complexity to the
 * writer kernel.  Some of this has since been simplified by the introduciton
 * of OpenField and WriteField.
 *
 * Giving defile the ability to switch on the fly to a new source file,
 * possibly with a different data format, meant that a proper CleanUp of the
 * writer was needed before cycling and re-initialisation after.  Things that
 * needed to be only done at program startup were removed from
 * InitialiseDirFile and placed in PreInitialiseDirFile and InitialiseDirFile
 * became the re-initialisation function.  This was certainly a kludge: a
 * complete shutdown and restart was the only easy way to adapt the code to
 * such a task.
 *
 * During this time Barth and I were fighting to get defile and kst to talk
 * to one-another happily while the one wrote dirfiles and the other read from
 * them concurrently.  This lead us to realise that defile needed to write
 * entire slow frames at a time, which resulted in the "staging" buffers which
 * are now present in PushFrame.  This ensured that the no partial frames would
 * be writen.  Later it was realised that an uninterrupted sequence of multiplex
 * indicies was required to ensure that the proper ratio of slow to fast data
 * was kept, so PreBuffer was introduced.
 *
 * If you've been paying attention, you might notice that before being writen
 * to disk, the data is TRIPLE buffered IN THE WRITER ALONE:
 *
 * 1) The pre_buffer which handles frame sequencing via the multiplex index to
 *     keep slow and fast data synchronised.
 *
 * 2) The staging buffers which hold the fast frames until a full slow frame
 *     has been composed so it can all be shipped out at once.
 *
 * 3) The passacross ring buffers, where the data are finally split up into
 *     channels and the data is passed from the input thread to the dirfile
 *     writer thread.
 *
 * It is supposed by some that this might be simplified... */

/* Now, if you enjoyed that, you should take a look at channels.c -- the channel
 * manipulation library.  It's much more entertaining.
 *
 * Cheers!
 * -dvw
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#define _GNU_SOURCE     /* Make string.h give us the "nice" basename() */
#include <stdlib.h>     /* ANSI C std library (atoi, exit) */
#include <errno.h>      /* ANSI C library errors (errno) */
#include <string.h>     /* ANSI C strings (strcpy, strcat, strlen, strcmp)  */
#include <dirent.h>     /* POSIX directory IO (DIR, opendir, closedir, &c.) */
#include <fcntl.h>      /* POSIX file descriptor manipulation (open, creat) */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <libgen.h>     /* basename */
#include <unistd.h>     /* unlink, &c. */
#ifdef HAVE_ZLIB_H
#  include <zlib.h>     /* libz compression library (gzwrite, gzopen, &c.) */
#endif

#include "blast.h"
#include "channels.h"
#include "defile.h"
#include "frameread.h"

extern struct ChannelStruct* WideSlowChannels;
extern struct ChannelStruct* SlowChannels;
extern struct ChannelStruct* WideFastChannels;
extern struct ChannelStruct* FastChannels;
extern struct ChannelStruct* DecomChannels;

#define MAXBUF 3000 /* This is a 30 second buffer */

unsigned long long int fc = 0;
unsigned short defile_flags;
unsigned short defile_flag_buf[FAST_PER_SLOW];

#define DEFILE_FLAG_ZEROED_FRAME   0x1
#define DEFILE_FLAG_MANGLED_INDEX  0x2
#define DEFILE_FLAG_INSERTED_FRAME 0x4
#define DEFILE_FLAG_SINGLE_FRAME   0x8

struct FieldType
{
  short i0;   /* start of field in rxframe, words */
  short size; /* size in words */
  int i_in;   /* index in elements */
  int i_out;  /* index in elements */
  long fp;
  void* b;    /* buffer */
  long int nw;
};

struct FieldType* normal_fast;
struct FieldType* slow_fields[FAST_PER_SLOW];
struct FieldType defile_field;

unsigned short* fast_frame[FAST_PER_SLOW];
unsigned short* slow_data[FAST_PER_SLOW];
int buf_overflow;
int n_fast;

int (*defileclose) ();

#if DAS_CARDS > 0
int bolo_i0;
struct FieldType bolo_fields[DAS_CARDS][DAS_CHS];
extern unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];
#endif

int FieldSize (char type, const char* field)
{
  switch(type) {
    case 's':
    case 'u':
      return 1;
    case 'S':
    case 'U':
    case 'I':
    case 'f':
      return 2;
    default:
      bprintf(fatal, "bad type in channel spec: %s %c\n", field, type);
      exit(1); /* can't get here -- added to suppress compiler warnings */
  }
}

/* ask user a yes/no question */
int YesNo(const char* message, int dflt)
{
  char gpb[GPB_LEN];

  if (rc.silent)
    return dflt;

  ri.tty = 1;

  strcpy(gpb, message);
  if (dflt)
    strcat(gpb, " (Y/n) ");
  else
    strcat(gpb, " (y/N) ");

  printf("\n");
  for (;;) {
    fputs(gpb, stdout);

    switch (fgetc(stdin)) {
      case 'y':
      case 'Y':
        ri.tty = 0;
        return 1;
      case 'n':
      case 'N':
        ri.tty = 0;
        return 0;
      case '\r':
      case '\n':
      case '\0':
        ri.tty = 0;
        return dflt;
      default:
        while (fgetc(stdin) != '\n'); /* flush input */
        printf("Invald response.\n");
    }
  }

  fprintf(stderr, "defile: Unexpected trap in YesNo().  Bailing.\n");
  exit(1);
}

static inline long int GetNumFrames(size_t size, char type, const char* field)
{
  return (long int)size / 2 / FieldSize(type, field);
}

/* Check to see if overwriting the directory is appropriate */
int CheckWriteAllow(int mkdir_err)
{
  dtrace("%i", mkdir_err);
  DIR* dir;
  char* dirfile_end;
  char fullname[FILENAME_LEN];
  char gpb[GPB_LEN];
  struct stat stat_buf;
  struct dirent* lamb;
  unsigned int i, found, n_fast = 0, n_slow = 0, n_bolo = 0;
#if DAS_CARDS > 0
  unsigned int j;
  int bolo_node = 0;
#endif
  long int n, min_wrote = 0x7fffffff;

  /* if user hasn't told us to try, don't */
  if (rc.write_mode == 0)
    return 0;

  /* The mkdir already failed, but we don't know why yet: it _could_ be
   * because the directory exists, but it could be for some other reason, too */
  if (stat(rc.dirfile, &stat_buf))
    berror(fatal, "cannot stat `%s'", rc.dirfile);

  /* stat worked, so rc.dirfile exists -- is it a directory? */
  if (!S_ISDIR(stat_buf.st_mode))
    bprintf(fatal, "cannot write to `%s': Not a directory\n", rc.dirfile);

  /* it is.  Look for a format file to double check that this is indeed ar
   * dirfile */
  strcpy(gpb, rc.dirfile);
  strcat(gpb, "/format");
  if (stat(gpb, &stat_buf)) {
    /* can't find a format file -- if we're trying to resume, give up here
     * otherwise ask for confirmation to overwrite the directory */
    if (rc.write_mode == 2)
      bprintf(fatal, "destination `%s' does not appear to be a "
          "dirfile\ncannot resume\n", rc.dirfile);

    snprintf(gpb, GPB_LEN,
        "defile: destination `%s' does not appear to be a dirfile\nContinue?",
        rc.dirfile);
    if(!YesNo(gpb, 0))
      exit(1);
  }

  /* it's a dirfile, so prepare for resume and/or overwrite */
  if (rc.write_mode == 1)
    bprintf(info, "\nOverwriting dirfile `%s'\n", rc.dirfile);

  if ((dir = opendir(rc.dirfile)) == NULL)
    berror(fatal, "cannot read directory `%s'", rc.dirfile);

  /* loop through the directory */
  strcpy(fullname, rc.dirfile);
  dirfile_end = fullname + strlen(fullname);
  if (*(dirfile_end - 1) != '/') {
    strcat(fullname, "/");
    dirfile_end++;
  }

  while ((lamb = readdir(dir)) != NULL) {
    strcpy(dirfile_end, lamb->d_name);

    if (stat(fullname, &stat_buf))
      berror(fatal, "cannot stat `%s'", fullname);

    /* is it a regular file? */
    if (S_ISREG(stat_buf.st_mode)) {
      if (rc.write_mode == 2) {
        found = 0;
        /* resume mode -- is this one of our fields? */
        if (strcmp(lamb->d_name, "format") == 0)
          continue; /* skip format file */
        if (strcmp(lamb->d_name, "FASTSAMP") == 0) {
          if ((n = GetNumFrames(stat_buf.st_size, 'U', "FASTSAMP")) < min_wrote)
            min_wrote = n;
          n_fast++;
          found = 1;
        } else if (strcmp(lamb->d_name, "DEFILE_FLAGS") == 0) {
          if ((n = GetNumFrames(stat_buf.st_size, 'u', "DEFILE_FLAGS"))
              < min_wrote)
            min_wrote = n;
          n_fast++;
          found = 1;
        } else
          for (i = 0; i < ccWideFast; ++i)
            if (strcmp(lamb->d_name, WideFastChannels[i].field) == 0) {
              if ((n = GetNumFrames(stat_buf.st_size, WideFastChannels[i].type,
                      lamb->d_name)) < min_wrote)
                min_wrote = n;
              n_fast++;
              found = 1;
              break;
            }
        if (!found)
          for (i = 0; i < ccNarrowFast; ++i)
            if (strcmp(lamb->d_name, FastChannels[i].field) == 0) {
              if ((n = GetNumFrames(stat_buf.st_size, FastChannels[i].type,
                      lamb->d_name)) < min_wrote)
                min_wrote = n;
              n_fast++;
              found = 1;
              break;
            }
#if DAS_CARDS > 0
        if (!found)
	  bolo_node = DAS_START;
          for (i = 0; i < DAS_CARDS; ++i) {
	    bolo_node++;
	    if (bolo_node%4 == 0) bolo_node++;
            for (j = 0; j < DAS_CHS; ++j) {
              sprintf(gpb, "n%02ic%02i", bolo_node, j);
              if (strcmp(lamb->d_name, gpb) == 0) {
                if ((n = GetNumFrames(stat_buf.st_size, 'U', gpb)) < min_wrote)
                  min_wrote = n;
                n_bolo++;
                found = 1;
                break;
              }
            }
	  }
#endif
        if (!found) {
          for (i = 0; i < ccNarrowSlow; ++i) {
            if (strcmp(lamb->d_name, SlowChannels[i].field) == 0) {
              if ((n = GetNumFrames(stat_buf.st_size * FAST_PER_SLOW,
                      SlowChannels[i].type, lamb->d_name)) < min_wrote)
                min_wrote = n;
              n_slow++;
              found = 1;
              break;
            }
          }
        }
        if (!found)
          for (i = 0; i < ccWideSlow; ++i)
            if (strcmp(lamb->d_name, WideSlowChannels[i].field) == 0) {
              if ((n = GetNumFrames(stat_buf.st_size * FAST_PER_SLOW,
                      WideSlowChannels[i].type, lamb->d_name)) < min_wrote)
                min_wrote = n;
              n_slow++;
              found = 1;
              break;
            }
        if (!found)
          for (i = 0; i < ccDecom; ++i)
            if (strcmp(lamb->d_name, DecomChannels[i].field) == 0) {
              if ((n = GetNumFrames(stat_buf.st_size * FAST_PER_SLOW,
                      DecomChannels[i].type, lamb->d_name)) < min_wrote)
                min_wrote = n;
              n_fast++;
              found = 1;
              break;
            }
        if (!found)
          bprintf(fatal, "error reading `%s': field `%s' is not in "
              "the channel list.\ncannot resume\n", rc.dirfile, lamb->d_name);
      } else
        /* overwrite mode - delete the file */
        if (unlink(fullname))
          berror(fatal, "cannot unlink `%s'", fullname);
    }
  }

  if (rc.write_mode == 2) {
    /* check to see if we've read all the channels */
    if ((n_fast != ccWideFast + ccNarrowFast + 2 + ccDecom) ||
        (n_slow != ccNarrowSlow + ccWideSlow) ||
        (n_bolo != DAS_CARDS * DAS_CHS))
      bprintf(fatal, "dirfile `%s' is missing field files\ncannot resume. n_fast=%d(%d), n_slow=%d(%d), n_bolo=%d(%d)\n",
	      rc.dirfile,n_fast,ccWideFast + ccNarrowFast + 1 + ccDecom,
	      n_slow,ccNarrowSlow + ccWideSlow,n_bolo,DAS_CARDS * DAS_CHS);

    /* Be safe -- go backwards a bit */
    if (min_wrote > 0) {
      min_wrote -= min_wrote % 20;
      min_wrote -= 20;
    }

    bprintf(info, "\nResuming dirfile `%s' at frame %li\n", rc.dirfile,
        min_wrote);
    ri.wrote = rc.resume_at = min_wrote;
  }

  if (closedir(dir))
    berror(fatal, "error closing directory `%s'", rc.dirfile);

  if (rc.write_mode == 2)
    /* We only allow resuming once per session */
    rc.write_mode = 0;

  return 1;
}

unsigned short* pre_buffer[FAST_PER_SLOW + 10];
void PreInitialiseDirFile(void)
{
  dtracevoid();

  int j;

  normal_fast = balloc(fatal, (ccFast + 1) * sizeof(struct FieldType));

  for (j = 0; j < FAST_PER_SLOW; ++j) {
    slow_fields[j] = balloc(fatal, slowsPerBi0Frame * sizeof(struct FieldType));
    slow_data[j] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned int));
  }
}

long OpenField(int fast, int size, const char* filename)
{
  long file;
#ifdef HAVE_LIBZ
  char gpb[GPB_LEN];
  int gzerrno;
  const char* gze;
#endif
  int offset;

  if (rc.resume_at >= 0) {
    offset = rc.resume_at * size * 2;
    if (!fast)
      offset /= FAST_PER_SLOW;
    /* append to file */
    if ((file = open(filename, O_WRONLY)) == -1)
      berror(fatal, "cannot open file `%s'", filename);

    if (lseek(file, offset, SEEK_SET) < 0)
      berror(fatal, "cannot lseek file `%s'", filename);
  } else {
    /* create new file */
#ifdef HAVE_LIBZ
    if (rc.gzip_output && (file = (long)gzdopen(creat(filename, 00644), "wb"))
        == 0) {
      snprintf(gpb, GPB_LEN, "cannot create file `%s'", filename);
      if (errno)
        berror(fatal, "%s\n", gpb);
      else  {
        gze = gzerror((gzFile)file, &gzerrno);
        bprintf(fatal, "%s: %s", gpb, gze);
      }
    } else if (!rc.gzip_output && (file = creat(filename, 00644)) == -1)
#else
    if ((file = creat(filename, 00644)) == -1)
#endif
      berror(fatal, "cannot create file `%s'", filename);
  }

  return file;
}

int CheckZeroes(int zeroes)
{
  if (zeroes > 0)
    bprintf(warning, "Frame %lli: discarded %i zeroed frame%s.\n", fc - 1,
        zeroes, (zeroes == 1) ? "" : "s");

  return 0;
}

/* looks for and tries to correct dropped/mangled frames */
int PreBuffer(unsigned short *frame)
{
  static int counter = 0;
  static int start = 2;
  static int zeroes = 0;
  int range;
  unsigned int last, this, next;
  int li, ti, ni;
  int lf, tf, nf;

  /* a reset */
  if (frame == NULL) {
    fc = 0;
    start = 2;
    counter = 0;
    zeroes = 0;
    return 0;
  }

  fc++; /* global frame counter */

  counter++;
  last = FAST_PER_SLOW + (counter + 0) % 3;
  this = FAST_PER_SLOW + (counter + 1) % 3;
  next = FAST_PER_SLOW + (counter + 2) % 3;

  memcpy(pre_buffer[next], frame, DiskFrameSize);

  if (start > 0) {
    start--;
    return -1;
  }

  li = pre_buffer[last][3];
  ti = pre_buffer[this][3];
  ni = pre_buffer[next][3];

  lf = pre_buffer[last][1];
  tf = pre_buffer[this][1];
  nf = pre_buffer[next][1];

  if ((li + 1) % FAST_PER_SLOW != ti) {
    if ((li + 2) % FAST_PER_SLOW == ni) {
      zeroes = CheckZeroes(zeroes);
      if (tf + 1 == nf && lf + 1 == tf) {
        bprintf(warning,
            "Frame %lli: corrected mangled multiplex index: (%i %i %i)\n",
            fc, lf, tf, nf);
        pre_buffer[this][3] = ti = (li + 1) % FAST_PER_SLOW;
        defile_flags |= DEFILE_FLAG_MANGLED_INDEX;
      } else {
        bprintf(warning, "Frame %lli: single frame sequencing error, eliding "
            "%i %i %i (%i %i %i)\n", fc, li, ni, ti, lf, tf, nf);
        ti = (li + 1) % FAST_PER_SLOW;
        memcpy(pre_buffer[this], pre_buffer[ti], DiskFrameSize);
        defile_flags |= DEFILE_FLAG_SINGLE_FRAME;
      }
    } else if (li == 0 && ti == 0 && (zeroes > 0 || ni == 0)) {
      zeroes++;
      defile_flags |= DEFILE_FLAG_ZEROED_FRAME;
      return 0;
    } else {
      zeroes = CheckZeroes(zeroes);
      if (ti >= FAST_PER_SLOW) {
        bprintf(warning, "Frame %lli: index out of range: %i\n", fc, ti);
        pre_buffer[this][3] = ti %= FAST_PER_SLOW;
      }
    }
  }

  if ((range = ti - li) <= 0)
    range += FAST_PER_SLOW;

  if (range > 1)
    bprintf(warning, "Frame %lli: inserted %i missing frame%s: %i -> %i\n", fc,
        range - 1, (range == 2) ?  "" : "s", li, ti);

  li = (li + 1) % FAST_PER_SLOW;

  memcpy(pre_buffer[ti], pre_buffer[this], DiskFrameSize);

  return (range << 8) + li;
}

/* Initialise dirfile */
void InitialiseDirFile(int reset, unsigned long offset)
{
  dtrace("%i, %lu", reset, offset);
  FILE* fp;
  int fd;
  int j, i, is_bolo = 0;
#if DAS_CARDS > 0
  char field[FIELD_LEN];
  int bolo_node;
  char first_bolo_buf[16];
#endif
  char gpb[GPB_LEN];
  char ext[4] = "";

  fc = 0;

#ifdef HAVE_LIBZ
  if (rc.gzip_output) {
    defileclose = &gzclose;
    strcpy(ext, ".gz");
  } else
#endif
    defileclose = &close;

  rc.resume_at = -1;

  if (mkdir(rc.dirfile, 00755) < 0)
    if (!CheckWriteAllow(errno))
      berror(fatal, "cannot create dirfile `%s'", rc.dirfile);

  bprintf(info, "\nWriting to dirfile `%s'\n", rc.dirfile);

  rc.dirname = strdup(rc.dirfile);
  snprintf(rc.dirname, strlen(rc.dirfile), "%s", basename(rc.dirfile));

  for (i = 0; i < FAST_PER_SLOW + 10; ++i) {
    pre_buffer[i] = balloc(fatal, DiskFrameSize);
    pre_buffer[i][3] = i;
  }

  /* Reset the PreBuffer */
  PreBuffer(NULL);

  for (i = 0; i < FAST_PER_SLOW; ++i)
    fast_frame[i] = balloc(fatal, DiskFrameSize);

  /***********************************
   * create and fill the format file *
   ***********************************/
  sprintf(gpb, "%s/format", rc.dirfile);

  if ((fd = creat(gpb, 0666)) < 0)
    berror(fatal, "cannot create format file `%s/format'", rc.dirfile);

  PathSplit_r(rc.dirfile, NULL, gpb);

  WriteFormatFile(fd, atoi(gpb), offset / FAST_PER_SLOW);

#ifdef __SPIDER__
  if (rc.extra_format) {
    sprintf(gpb, "/INCLUDE /data/etc/spider/format.mce_mplex\n"
        "/INCLUDE /data/etc/spider/format.bolo_stats\n");
    if (write(fd, gpb, strlen(gpb)) < 0)
      berror(err, "Error writing to format file\n");
  }
#else
  /* no extra format for BLAST */
#endif

  if (close(fd) < 0)
    berror(fatal, "Error while closing format file");

  /* DEFILE_FLAGS */
  sprintf(gpb, "%s/DEFILE_FLAGS%s", rc.dirfile, ext);
  defile_field.size = 1;
  defile_field.fp = OpenField(1, 1, gpb);
  defile_field.i0 = 1;
  if (reset) {
    if (rc.resume_at > 0)
      defile_field.nw = rc.resume_at;
    else
      defile_field.nw = 0;
  }
  defile_field.i_in = defile_field.i_out = 0;
  defile_field.b = balloc(fatal, MAXBUF * sizeof(unsigned short));

  /* FASTSAMP */
  sprintf(gpb, "%s/FASTSAMP%s", rc.dirfile, ext);
  normal_fast[0].size = 2;

  normal_fast[0].fp = OpenField(1, 2, gpb);
  normal_fast[0].i0 = 1;
  if (reset) {
    if (rc.resume_at > 0)
      normal_fast[0].nw = rc.resume_at;
    else
      normal_fast[0].nw = 0;
  }

  normal_fast[0].i_in = normal_fast[0].i_out = 0;
  normal_fast[0].b = balloc(fatal, MAXBUF * sizeof(unsigned int));

  n_fast = 1;  //original form

  /* slow chs */
  for (i = 0; i < slowsPerBi0Frame; i++) {
    for (j = 0; j < FAST_PER_SLOW; j++) {
      if (strlen(SlowChList[i][j].field) > 0) {
        slow_fields[j][i].size = FieldSize(SlowChList[i][j].type,
            SlowChList[i][j].field);

        sprintf(gpb, "%s/%s%s", rc.dirfile, SlowChList[i][j].field, ext);
        slow_fields[j][i].fp = OpenField(0, slow_fields[j][i].size, gpb);
      } else
        slow_fields[j][i].fp = -1;

      slow_fields[j][i].i_in = slow_fields[j][i].i_out = 0;
      if (reset) {
        if (rc.resume_at > 0)
          slow_fields[j][i].nw = rc.resume_at / FAST_PER_SLOW;
        else
          slow_fields[j][i].nw = 0;
      }

      slow_fields[j][i].b = balloc(fatal, 2 * MAXBUF);

      slow_fields[j][i].i0 = SLOW_OFFSET + i;
    }
  }

  /* normal fast chs */

#if DAS_CARDS > 0
  sprintf(first_bolo_buf, "n%02dc00lo", DAS_START+1);
#endif
  for (i = 0; i < ccFast + ccWideFast; i++) {
#if DAS_CARDS > 0
    if (strcmp(FastChList[i].field, first_bolo_buf) == 0) {
      bolo_i0 = i + SLOW_OFFSET + slowsPerBi0Frame;
      is_bolo = 1;
    } else if (ccDecom > 0 && strcmp(FastChList[i].field,
          DecomChannels[0].field) == 0) {
      is_bolo = 0;
    }
#endif

    if (!is_bolo && strlen(FastChList[i].field) > 0) {
      normal_fast[n_fast].size = FieldSize(FastChList[i].type,
          FastChList[i].field);
      sprintf(gpb, "%s/%s%s", rc.dirfile, FastChList[i].field, ext);

      normal_fast[n_fast].fp = OpenField(1, normal_fast[n_fast].size, gpb);
      normal_fast[n_fast].i0 = i + SLOW_OFFSET + slowsPerBi0Frame;
      normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
      if (reset) {
        if (rc.resume_at > 0)
          normal_fast[n_fast].nw = rc.resume_at;
        else
          normal_fast[n_fast].nw = 0;
      }

      normal_fast[n_fast].b = balloc(fatal, MAXBUF * 2 *
          normal_fast[n_fast].size);

      n_fast++;
    }
  }

#if DAS_CARDS > 0
  /* special (bolo) fast chs */
  bolo_node = DAS_START;
  for (i = 0; i < DAS_CARDS; i++) {
    bolo_node++;
    if (bolo_node%4 == 0) bolo_node++;
    for (j = 0; j < DAS_CHS; j++) {
      bolo_fields[i][j].size = 2;
      sprintf(field, "n%02dc%02d", bolo_node, j);
      sprintf(gpb, "%s/%s%s", rc.dirfile, field, ext);

      bolo_fields[i][j].fp = OpenField(1, 2, gpb);
      bolo_fields[i][j].i_in = bolo_fields[i][j].i_out = 0;

      if (reset) {
        if (rc.resume_at > 0)
          bolo_fields[i][j].nw = rc.resume_at;
        else
          bolo_fields[i][j].nw = 0;
      }

      bolo_fields[i][j].b = balloc(fatal, MAXBUF * 4);

      bolo_fields[i][j].i0 = bolo_i0 + i * (DAS_CARDS * 3 / 2) + j;
    }
  }
#endif

  if (rc.write_curfile) {
    // create the link file
    char lnkfile[1024];
    char curfile[1024];

    strncpy(curfile, rc.output_curfile, 1018);
    strcat(curfile, ".cur");

    strncpy(lnkfile, rc.output_curfile, 1018);
    strcat(lnkfile, ".lnk");
    unlink(lnkfile);
    if (symlink(rc.dirfile, lnkfile)<0) {
      berror(fatal, "could not create link from `%s' to `%s'",
             rc.dirfile, lnkfile);
    }
    // create the cur file
    if ((fp = fopen(curfile, "w")) == NULL)
      berror(fatal, "cannot create curfile `%s'", curfile);

    fprintf(fp, "%s\n", rc.dirfile);

    if (fclose(fp) < 0)
      berror(fatal, "cannot close curfile `%s'", curfile);
  }

  ri.dirfile_init = 1;
}

/* Clean up in preparation for re-initialising dirfile with a new name
 * Close open file descriptors and free allocated memory */
void CleanUp(void)
{
  dtracevoid();
  int j, i, is_bolo = 0;
#if DAS_CARDS > 0
  char first_bolo_buf[16];
#endif

  for (i = 0; i < FAST_PER_SLOW + 10; ++i)
    bfree(fatal, pre_buffer[i]);

  for (i = 0; i < FAST_PER_SLOW; ++i)
    bfree(fatal, fast_frame[i]);

  for(i = 0; i < slowsPerBi0Frame; i++)
    for(j = 0; j < FAST_PER_SLOW; j++) {
      if (slow_fields[j][i].fp != -1)
        defileclose(slow_fields[j][i].fp);
      if (slow_fields[j][i].b)
        bfree(fatal, slow_fields[j][i].b);
      slow_fields[j][i].b = NULL;
    }

#if DAS_CARDS > 0
  sprintf(first_bolo_buf, "n%02dc00lo", DAS_START+1);
#endif
  for(i = 0; i < ccFast; ++i) {
#if DAS_CARDS > 0
    if (strcmp(FastChList[i].field, first_bolo_buf) == 0)
      is_bolo = 1;
    else if (ccDecom > 0 && strcmp(FastChList[i].field,
          DecomChannels[0].field) == 0)
      is_bolo = 0;
#endif

    if (!is_bolo) {
      if (normal_fast[i].fp != -1)
        defileclose(normal_fast[i].fp);
      if (normal_fast[i].b)
        bfree(fatal, normal_fast[i].b);
      normal_fast[i].b = NULL;
    }
  }

  if (defile_field.fp != -1)
    defileclose(defile_field.fp);
  if (defile_field.b)
    bfree(fatal, defile_field.b);
  defile_field.b = NULL;

#if DAS_CARDS > 0
  for (i = 0; i < DAS_CARDS; i++)
    for (j = 0; j < DAS_CHS; j++) {
      if (bolo_fields[i][j].fp != -1)
        defileclose(bolo_fields[i][j].fp);
      if (bolo_fields[i][j].b)
        bfree(fatal, bolo_fields[i][j].b);
      bolo_fields[i][j].b = NULL;
    }
#endif
}

/* pushFrame: called from reader: puts rxframe into */
/* individual buffers                               */
void PushFrame(unsigned short* in_frame)
{
  static int discard_me = 1; /* we discard the first slow frame, since it's
                                probably a partial */
  unsigned int i_in, i_out;
  int i, j, k, curr_index;
  int write_ok;
#if DAS_CARDS > 0
  unsigned B0, B1;
#endif
  int range, first, c;
  unsigned short *frame;

  first = PreBuffer(in_frame);
  if (first == -1)
    return;

  range = first >> 8;
  first &= 0xff;

  /* Wait for dirfile initialisation */
  while (!ri.dirfile_init)
    usleep(10000);

  for (c = 0; c < range; ++c) {
    if (c != range - 1)
      defile_flags |= DEFILE_FLAG_INSERTED_FRAME;

    frame = pre_buffer[(first + c) % FAST_PER_SLOW];

#ifdef DEBUG_FASTSAMP
    printf("Fastsamp push: %lu\n", *(unsigned long*)(&frame[1]));
#endif

    /* slow data */
    curr_index = frame[3];
#ifdef DEBUG_SEQUENCING
    static int next_j = -1;
    if (next_j > -1 && next_j != curr_index)
      printf("frame %lli sequencing error: expected %i but got %i\n", fc,
          next_j, curr_index);
    next_j = (curr_index + 1) % FAST_PER_SLOW;
#endif

    if (curr_index < FAST_PER_SLOW)
      for (i = 0; i < slowsPerBi0Frame; i++)
        slow_data[curr_index][i] = frame[slow_fields[curr_index][i].i0];

    /* defile flags */
    defile_flag_buf[curr_index] = defile_flags;
    defile_flags = 0;

    /* fast data */
    memcpy(fast_frame[curr_index], frame, DiskFrameSize);

    /* do while loop blocks until sufficient buffers empty */
    do {
      write_ok = 1;
      /* ****************************************************************** */
      /* First make sure there is enough space in ALL the buffers           */
      /* We need to do it first and all at once to maintain synchronization */
      /* We discard the full frame id there is no space                     */
      /* ****************************************************************** */

      /************************************/
      /* Check buffer space in slow field */
      /************************************/
      for (i = 0; write_ok && i < slowsPerBi0Frame; i++)
        for (j = 0; write_ok && j < FAST_PER_SLOW; j++) {
          i_in = (slow_fields[j][i].i_in + 1) % MAXBUF;
          if(i_in == slow_fields[j][i].i_out)
            write_ok = 0;
        }

      /* Check buffer space in defile_field */
      if (write_ok) {
        i_out = defile_field.i_out;
        i_in = defile_field.i_in;

        if (i_out <= i_in)
          i_out += MAXBUF;

        if (i_in + FAST_PER_SLOW >= i_out)
          write_ok = 0;
      }

      /******************************************
       * Check buffer space in normal fast data *
       *****************************************/
      if (write_ok)
        for (j = 0; write_ok && j < n_fast; j++) {
          i_out = normal_fast[j].i_out;
          i_in = normal_fast[j].i_in;

          if (i_out <= i_in)
            i_out += MAXBUF;

          if(i_in + FAST_PER_SLOW >= i_out)
            write_ok = 0;
        }

#if DAS_CARDS > 0
      /*******************************************
       * Check buffer space in  fast bolo data   *
       *******************************************/
      if (write_ok)
        for (i = 0; write_ok && i < DAS_CARDS; i++)
          for (j = 0; write_ok && j < DAS_CHS; j += 2) {
            i_out = bolo_fields[i][j].i_out;
            i_in = bolo_fields[i][j].i_in;

            if (i_out <= i_in)
              i_out += MAXBUF;

            if(i_in + FAST_PER_SLOW >= i_out)
              write_ok = 0;
          }
#endif

      if (!write_ok)
        usleep(10000);
    } while (!write_ok);

    /*************************/
    /* PUSH RX FRAME TO DISK */
    /*************************/

    /* push if we're at the end of a frame... */
    if (curr_index == FAST_PER_SLOW - 1) {
      if (discard_me) /* discard the first frame */
        discard_me = 0;
      else {
        /* Slow Data */
        for (i = 0; i < slowsPerBi0Frame; i++) {
          for (j = 0; j < FAST_PER_SLOW; j++) {
            i_in = slow_fields[j][i].i_in;
            ((unsigned short*)slow_fields[j][i].b)[i_in] = slow_data[j][i];

            if (++i_in >= MAXBUF)
              i_in = 0;
            slow_fields[j][i].i_in = i_in;
          }
        }

        for (k = 0; k < FAST_PER_SLOW; ++k) {
          /* defile_field */
          i_in = defile_field.i_in;
          ((unsigned short*)defile_field.b)[i_in] = defile_flag_buf[k];

          defile_field.i_in = (i_in + 1) % MAXBUF;

          /********************/
          /* normal fast data */
          /********************/

          for (j = 0; j < n_fast; j++) {
            i_in = normal_fast[j].i_in;
            if (normal_fast[j].size == 2)
              ((unsigned*)normal_fast[j].b)[i_in] =
                *((unsigned*)(fast_frame[k] + normal_fast[j].i0));
            else 
              ((unsigned short*)normal_fast[j].b)[i_in] =
                fast_frame[k][normal_fast[j].i0];

            normal_fast[j].i_in = (i_in + 1) % MAXBUF;
          }

#if DAS_CARDS > 0
          /********************/
          /* fast bolo data   */
          /********************/
          for (i = 0; i < DAS_CARDS; i++) {
            for (j = 0; j < DAS_CHS; j += 2) {
              B0 = (unsigned)fast_frame[k][boloIndex[i][j][0] + BoloBaseIndex] +
                (((unsigned)fast_frame[k][boloIndex[i][j][1] + BoloBaseIndex] &
                  0xff00) << 8);
              B1 = fast_frame[k][boloIndex[i][j + 1][0] + BoloBaseIndex] +
                ((fast_frame[k][boloIndex[i][j + 1][1] + BoloBaseIndex] &
                  0x00ff) << 16);

              i_in = bolo_fields[i][j].i_in;
              ((unsigned*)bolo_fields[i][j].b)[i_in] = B0;
              ((unsigned*)bolo_fields[i][j + 1].b)[i_in] = B1;

              bolo_fields[i][j].i_in = bolo_fields[i][j + 1].i_in =
                (i_in + 1) % MAXBUF;
            }
          }
#endif
        }
      }
    }
  }
}

void WriteField(long file, int length, void *buffer)
{
#ifdef HAVE_LIBZ
  int gzerrno;
  const char *gze;

  if (rc.gzip_output && gzwrite((gzFile)file, buffer, length) == 0) {
    if (errno)
      berror(fatal, "Error on write");
    else {
      gze = gzerror((gzFile)file, &gzerrno);
      if (gzerrno)
        bprintf(fatal, "Error on write: %s (%i)", gze, gzerrno);
    }
  } else if (!rc.gzip_output && write(file, buffer, length) < 0)
#else
    if (write(file, buffer, length) < 0)
#endif
      berror(fatal, "Error on write");

#ifdef DEBUG_FASTSAMP
  int i;
  int seq_err = 0;
  static unsigned long last_fastsamp = 4000000000U;

  if (file == 5)
    for (i = 0; i < length / 4; ++i) {
      if (last_fastsamp < 4000000000U)
        if (last_fastsamp + 1 != ((unsigned long*)buffer)[i])
          seq_err = last_fastsamp + 1;
      last_fastsamp = ((unsigned long*)buffer)[i];
      printf("Fastsamp write: %lu\n", last_fastsamp);
    }

  if (seq_err)
    bprintf(fatal, "sequencing error (%i) detected.\n", seq_err);
#endif
}

/* DirFileWriter: separate thread: writes each field to disk */
void DirFileWriter(void)
{
  dtracevoid();
  unsigned short buffer[MAXBUF];
  unsigned int ibuffer[MAXBUF];
  int j, i;
  int i_in, i_out, i_buf;
  int i_in2, i_out2;
  int wrote_count = 0;
  int last_pass = 0;

  while (1) {
    /* slow data */
    for (i = 0; i < slowsPerBi0Frame; i++)
      for (j = 0; j < FAST_PER_SLOW; j++) {
        if (slow_fields[j][i].fp != -1) {
          i_in = slow_fields[j][i].i_in;
          i_out = slow_fields[j][i].i_out;
          i_buf = 0;
          if ((SlowChList[i][j].type == 'U') || (SlowChList[i][j].type == 'I')
              || (SlowChList[i][j].type == 'S'))
          {
            i_in2 = slow_fields[j][i + 1].i_in;
            i_out2 = slow_fields[j][i + 1].i_out;
            while (i_in != i_out && i_in2 != i_out2) {
              ibuffer[i_buf] = (unsigned)
                ((((unsigned short*)slow_fields[j][i + 1].b)[i_out2]) << 16)
                | (unsigned)
                (((unsigned short*)slow_fields[j][i].b)[i_out]);

              if (i == 0 && j == 0)
                wrote_count = ++slow_fields[j][i].nw * FAST_PER_SLOW;
              else if (wrote_count < ++slow_fields[j][i].nw * FAST_PER_SLOW)
                wrote_count = slow_fields[j][i].nw * FAST_PER_SLOW;

              i_out++;
              i_out2++;
              i_buf++;
              if (i_out >= MAXBUF)
                i_out = 0;
              if (i_out2 >= MAXBUF)
                i_out2 = 0;
            }

            WriteField(slow_fields[j][i].fp, i_buf * sizeof(unsigned), ibuffer);
            slow_fields[j][i].i_out = i_out;
            slow_fields[j][i + 1].i_out = i_out2;
          } else {
            while (i_in != i_out) {
              buffer[i_buf] = ((unsigned short*)slow_fields[j][i].b)[i_out];

              if (i == 0 && j == 0)
                wrote_count = ++slow_fields[j][i].nw * FAST_PER_SLOW;
              else if (wrote_count < ++slow_fields[j][i].nw * FAST_PER_SLOW)
                wrote_count = slow_fields[j][i].nw * FAST_PER_SLOW;

              i_out++;
              i_buf++;
              if (i_out >= MAXBUF)
                i_out = 0;
            }

            WriteField(slow_fields[j][i].fp, i_buf * sizeof(unsigned short),
                buffer);
            slow_fields[j][i].i_out = i_out;
          }
        } else if (i == 0 || ((SlowChList[i - 1][j].type != 'U')
              && (SlowChList[i - 1][j].type != 'I') &&
              (SlowChList[i - 1][j].type != 'S'))) {
          slow_fields[j][i].i_out = slow_fields[j][i].i_in;
        }
      }

    /* defile flags */
    i_in = defile_field.i_in;
    i_out = defile_field.i_out;
    i_buf = 0;

    while (i_in != i_out) {
      buffer[i_buf] = ((unsigned short*)defile_field.b)[i_out];

      if (wrote_count < ++defile_field.nw)
        wrote_count = defile_field.nw;

      i_out++;
      i_buf++;
      if (i_out >= MAXBUF)
        i_out = 0;
    }
    WriteField(defile_field.fp, i_buf * sizeof(unsigned short), buffer);
    defile_field.i_out = i_out;

    /**********************
     ** normal fast data **
     **********************/

    /* j = 0 (FASTSAMP) must be writen last if getdata is to return the proper
     * number of frames */
    for (j = 1; j < n_fast; j++) {
      i_in = normal_fast[j].i_in;
      i_out = normal_fast[j].i_out;
      i_buf = 0;
      if (normal_fast[j].size == 2) {
        while (i_in != i_out) {
          ibuffer[i_buf] = ((unsigned int*)normal_fast[j].b)[i_out];

          if (wrote_count < ++normal_fast[j].nw)
            wrote_count = normal_fast[j].nw;

          i_out++;
          i_buf++;
          if (i_out >= MAXBUF)
            i_out = 0;
        }

        WriteField(normal_fast[j].fp, i_buf * sizeof(unsigned), ibuffer);
      } else {
        while (i_in != i_out) {
          buffer[i_buf] = ((unsigned short*)normal_fast[j].b)[i_out];

          if (wrote_count < ++normal_fast[j].nw)
            wrote_count = normal_fast[j].nw;

          i_out++;
          i_buf++;
          if (i_out >= MAXBUF)
            i_out = 0;
        }
        WriteField(normal_fast[j].fp, i_buf * sizeof(unsigned short), buffer);
      }
      normal_fast[j].i_out = i_out;

    }

#if DAS_CARDS > 0
    /** Bolometer data */
    for (i = 0; i < DAS_CARDS; i++) {
      for (j = 0; j < DAS_CHS; j++) {
        i_in = bolo_fields[i][j].i_in;
        i_out = bolo_fields[i][j].i_out;

        i_buf = 0;
        while (i_in != i_out) {
          ibuffer[i_buf] =
            ((unsigned int*)bolo_fields[i][j].b)[i_out];

          if (wrote_count < bolo_fields[i][j].nw)
            wrote_count = bolo_fields[i][j].nw;

          i_out++;
          i_buf++;
          if (i_out >= MAXBUF)
            i_out = 0;
        }
        WriteField(bolo_fields[i][j].fp, i_buf * sizeof(unsigned), ibuffer);
        bolo_fields[i][j].i_out = i_out;
      }
    }
#endif

    /* Write FASTSAMP last */
    i_in = normal_fast[0].i_in;
    i_out = normal_fast[0].i_out;
    i_buf = 0;
    while (i_in != i_out) {
      ibuffer[i_buf] = ((unsigned int*)normal_fast[0].b)[i_out];

      if (wrote_count < ++normal_fast[0].nw)
        wrote_count = normal_fast[0].nw;

      i_out++;
      i_buf++;
      if (i_out >= MAXBUF)
        i_out = 0;
    }
    WriteField(normal_fast[0].fp, i_buf * sizeof(unsigned), ibuffer);
    normal_fast[0].i_out = i_out;

    /* Done writing */
    if (ri.wrote < wrote_count)
      ri.wrote = wrote_count;

    if (ri.reader_done) {
      if (ri.wrote == ri.read || last_pass) {
        ri.writer_done = 1;
        CleanUp();
        return;
      } else
        last_pass = 1;
    }

    usleep(10000);

    /* if dirfile_init is lowered, the reader has noticed a change in the
     * curfile: prepare for cycling the writer. */
    if (ri.dirfile_init == 0) {
      /* perform one last pass to ensure we've written all the data (the reader
       * has certainly stopped by now, so we should be fine with a single
       * last pass */
      if (!last_pass)
        last_pass = 1;
      else {
        /* Clean Up */
        CleanUp();

        bprintf(info, "Defiling `%s'\n    into `%s'\n", rc.chunk, rc.dirfile);
        if (rc.resume_at > 0)
          bprintf(info, "    starting at frame %li\n", rc.resume_at);
        bprintf(info, "\n");

        /* Re-initialise */
        wrote_count = 0;
        ri.read = ri.wrote = ri.old_total = 0;
        ri.frame_rate_reset = 1;
        InitialiseDirFile(1, 0);
        gettimeofday(&ri.last, &rc.tz);
        last_pass = 0;
      }
    }
  }
}
