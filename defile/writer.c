/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004 University of Toronto
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

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h> 
#include <time.h>
#include <zlib.h>

#include "blast.h"
#include "channels.h"
#include "defile.h"

extern struct ChannelStruct* WideSlowChannels;
extern struct ChannelStruct* SlowChannels;
extern struct ChannelStruct* WideFastChannels;
extern struct ChannelStruct* FastChannels;
extern struct ChannelStruct* DecomChannels;
void FPrintDerived(FILE* fp);

#define MAXBUF 3000 /* This is a 30 second buffer */

struct FieldType
{
  short i0;   /* start of field in rxframe, words */
  short size; /* size in words */
  int i_in;   /* index in elements */
  int i_out;  /* index in elements */
  int fp;
  void* b;    /* buffer */
  long int nw;
};

struct FieldType* normal_fast;
struct FieldType* slow_fields[FAST_PER_SLOW];
struct FieldType bolo_fields[DAS_CARDS][DAS_CHS];

unsigned short* slow_data[FAST_PER_SLOW];
int buf_overflow;
int n_fast, bolo_i0;

int (*defileclose) ();

extern unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

char* StringToLower(char* s)
{
  int i, len;
  static char ls[256];

  len = strlen(s);
  if (len > 255)
    len = 255;

  for (i = 0; i < len; i++) {
    ls[i] = tolower(s[i]);
  }
  ls[len] = '\0';
  return(ls);
}

char* StringToUpper(char* s)
{
  int i, len;
  static char us[256];

  len = strlen(s);

  for (i = 0; i < len; i++) {
    us[i] = toupper(s[i]);
  }

  us[len] = '\0';
  return(us);
}

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
  DIR* dir;
  char* dirfile_end;
  char fullname[FILENAME_LEN];
  char gpb[GPB_LEN];
  struct stat stat_buf;
  struct dirent* lamb;
  unsigned int i, j, found, n_fast = 0, n_slow = 0, n_bolo = 0;
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
        if (!found)
          for (i = 0; i < DAS_CARDS; ++i)
            for (j = 0; j < DAS_CHS; ++j) {
              sprintf(gpb, "n%ic%i", i + 5, j);
              if (strcmp(lamb->d_name, gpb) == 0) {
                if ((n = GetNumFrames(stat_buf.st_size, 'U', gpb)) < min_wrote)
                  min_wrote = n;
                n_bolo++;
                found = 1;
                break;
              }
            }
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
    if ((n_fast != ccWideFast + ccNarrowFast + 1 + ccDecom) ||
        (n_slow != ccNarrowSlow + ccWideSlow) ||
        (n_bolo != DAS_CARDS * DAS_CHS))
      bprintf(fatal, "dirfile `%s' is missing field files\ncannot resume.\n",
          rc.dirfile);

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

  if (rc.write_mode == 2) {
    /* We only allow resuming once per session */
    rc.write_mode = 0;
  }

  return 1;
}

void PreInitialiseDirFile(void)
{
  int j;

  if ((normal_fast = malloc(ccFast * sizeof(struct FieldType))) == NULL)
    berror(fatal, "cannot allocate heap");

  for (j = 0; j < FAST_PER_SLOW; ++j) {
    if ((slow_fields[j] = malloc(slowsPerBi0Frame * sizeof(struct FieldType)))
        == NULL)
      berror(fatal, "cannot allocate heap");

    if ((slow_data[j] = malloc(slowsPerBi0Frame * sizeof(unsigned int)))
        == NULL)
      berror(fatal, "cannot allocate heap");
  }
}

int OpenField(int fast, int size, const char* filename)
{
  char gpb[GPB_LEN];
  int file;
  int gzerrno;
  const char* gze;
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
    if (rc.gzip_output && (file = (int)gzdopen(creat(filename, 00644), "wb"))
        == 0) {
      snprintf(gpb, GPB_LEN, "cannot create file `%s'", filename);
      gze = gzerror((gzFile)file, &gzerrno);
      if (errno == Z_ERRNO)
        berror(fatal, gpb);
      else 
        bprintf(fatal, "%s: %s", gpb, gze);
    } else if ((file = creat(filename, 00644)) == -1)
      berror(fatal, "cannot create file `%s'", filename);
  }

  return file;
}

/* Initialise dirfile */
void InitialiseDirFile(int reset)
{
  FILE* fp;
  int j, i, is_bolo = 0;
  char field[FIELD_LEN];
  char gpb[GPB_LEN];
  char ext[4] = "";

  if (rc.gzip_output) {
    defileclose = &gzclose;
    strcpy(ext, ".gz");
  } else {
    defileclose = &close;
  }


  rc.resume_at = -1;

  if (mkdir(rc.dirfile, 00755) < 0)
    if (!CheckWriteAllow(errno))
      berror(fatal, "cannot create dirfile `%s'", rc.dirfile);

  bprintf(info, "\nWriting to dirfile `%s'\n", rc.dirfile);

  /*********************************** 
   * create and fill the format file * 
   ***********************************/
  sprintf(gpb, "%s/format", rc.dirfile);

  if ((fp = fopen(gpb, "w")) == NULL)
    berror(fatal, "cannot create format file `%s/format'", rc.dirfile);

  fprintf(fp, "FASTSAMP         RAW    U 20\n");
  n_fast = 0;
  sprintf(gpb, "%s/FASTSAMP%s", rc.dirfile, ext);
  normal_fast[n_fast].size = 2; 

  normal_fast[n_fast].fp = OpenField(1, 2, gpb);
  normal_fast[n_fast].i0 = 1;
  if (reset) {
    if (rc.resume_at > 0)
      normal_fast[n_fast].nw = rc.resume_at;
    else
      normal_fast[n_fast].nw = 0;
  }

  normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
  if ((normal_fast[n_fast].b = malloc(MAXBUF * sizeof(int))) == NULL)
    berror(fatal, "malloc");

  n_fast++;

  /* slow chs */
  fprintf(fp, "\n## SLOW CHANNELS:\n");
  for (i = 0; i < slowsPerBi0Frame; i++) {
    for (j = 0; j < FAST_PER_SLOW; j++) {
      if (strlen(SlowChList[i][j].field) > 0) {
        fprintf(fp, "%-16s RAW    %c 1\n",
            StringToLower(SlowChList[i][j].field),
            SlowChList[i][j].type);
        fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
            StringToUpper(SlowChList[i][j].field),
            StringToLower(SlowChList[i][j].field),
            SlowChList[i][j].m_c2e,
            SlowChList[i][j].b_e2e);
        if (fflush(fp) < 0)
          berror(fatal, "Error while flushing format file");

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

      if ((slow_fields[j][i].b = malloc( 2 * MAXBUF)) == NULL)
        berror(fatal, "malloc");

      slow_fields[j][i].i0 = SLOW_OFFSET + i;
    }
  }

  /* normal fast chs */
  fprintf(fp, "\n## FAST CHANNELS:\n");
  if (fflush(fp) < 0)
    berror(fatal, "Error while flushing format file");

  for (i = 0; i < ccFast + ccWideFast; i++) {
    if (strcmp(FastChList[i].field, "n5c0lo") == 0) {
      bolo_i0 = i + SLOW_OFFSET + slowsPerBi0Frame;
      is_bolo = 1;
    } else if (ccDecom > 0 && strcmp(FastChList[i].field,
          DecomChannels[0].field) == 0) {
      is_bolo = 0;
    }

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

      if ((normal_fast[n_fast].b = malloc(MAXBUF * 2
              * normal_fast[n_fast].size)) == NULL)
        berror(fatal, "malloc");

      n_fast++;
      fprintf(fp, "%-16s RAW    %c %d\n",
          StringToLower(FastChList[i].field),
          FastChList[i].type, FAST_PER_SLOW);
      fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
          StringToUpper(FastChList[i].field),
          StringToLower(FastChList[i].field),
          FastChList[i].m_c2e, FastChList[i].b_e2e);
      if (fflush(fp) < 0)
        berror(fatal, "Error while flushing format file");
    }
  }

  /* special (bolo) fast chs */
  fprintf(fp, "\n## BOLOMETERS:\n");
  if (fflush(fp) < 0)
    berror(fatal, "Error while flushing format file");

  for (i = 0; i < DAS_CARDS; i++) {
    for (j = 0; j < DAS_CHS; j++) {
      bolo_fields[i][j].size = 2;
      sprintf(field, "n%dc%d", i + 5, j);
      sprintf(gpb, "%s/%s%s", rc.dirfile, field, ext);

      bolo_fields[i][j].fp = OpenField(1, 2, gpb);
      bolo_fields[i][j].i_in = bolo_fields[i][j].i_out = 0;

      if (reset) {
        if (rc.resume_at > 0)
          bolo_fields[i][j].nw = rc.resume_at;
        else
          bolo_fields[i][j].nw = 0;
      }

      if ((bolo_fields[i][j].b = malloc(MAXBUF * 4)) == NULL)
        berror(fatal, "malloc");

      bolo_fields[i][j].i0 = bolo_i0 + i * (DAS_CARDS * 3 / 2)
        + j;
      fprintf(fp, "%-16s RAW    U %d\n",
          StringToLower(field), FAST_PER_SLOW);
      fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
          StringToUpper(field), StringToLower(field),
          LOCKIN_C2V, LOCKIN_OFFSET);
    }
  }

  /* derived channels */
  FPrintDerived(fp);

  if (fclose(fp) < 0)
    berror(fatal, "Error while closing format file");

  if (rc.write_curfile) {
    if ((fp = fopen(rc.output_curfile, "w")) == NULL)
      berror(fatal, "cannot create curfile `%s'", rc.output_curfile);

    fprintf(fp, rc.dirfile);
    fprintf(fp, "\n");

    if (fclose(fp) < 0)
      berror(fatal, "cannot close curfile `%s'", rc.output_curfile);
  }

  ri.dirfile_init = 1;
}

/* Clean up in preparation for re-initialising dirfile with a new name
 * Close open file descriptors and free allocated memory */
void CleanUp(void)
{
  int j, i, is_bolo = 0; 

  for(i = 0; i < slowsPerBi0Frame; i++)
    for(j = 0; j < FAST_PER_SLOW; j++) {
      if (slow_fields[j][i].fp != -1)
        defileclose(slow_fields[j][i].fp);
      if (slow_fields[j][i].b)
        free(slow_fields[j][i].b);
      slow_fields[j][i].b = NULL;
    }

  for(i = 0; i < ccFast; ++i) {
    if (strcmp(FastChList[i].field, "n5c0lo") == 0)
      is_bolo = 1;
    else if (ccDecom > 0 && strcmp(FastChList[i].field,
          DecomChannels[0].field) == 0)
      is_bolo = 0;

    if (!is_bolo) {
      if (normal_fast[i].fp != -1)
        defileclose(normal_fast[i].fp);
      if (normal_fast[i].b)
        free(normal_fast[i].b);
      normal_fast[i].b = NULL;
    }
  }

  for (i = 0; i < DAS_CARDS; i++)
    for (j = 0; j < DAS_CHS; j++) {
      if (bolo_fields[i][j].fp != -1)
        defileclose(bolo_fields[i][j].fp);
      if (bolo_fields[i][j].b)
        free(bolo_fields[i][j].b);
      bolo_fields[i][j].b = NULL;
    }
}

/* pushDiskFrame: called from reader: puts rxframe into */
/* individual buffers                                   */
void PushFrame(unsigned short* frame)
{
  unsigned int i_in;
  int i, j;
  int write_ok;
  static int i_count = 0;
  unsigned B0, B1;
  static int n_frames = 0;

  /* Wait for dirfile initialisation */
  while (!ri.dirfile_init) {
    usleep(10000);
  }
  /* do while loop blocks until sufficient buffers empty */
  do {
    write_ok = 1;
    /*************/
    /* slow data */
    j = frame[3];
    if (j < FAST_PER_SLOW)
      for (i = 0; i < slowsPerBi0Frame; i++)
        slow_data[j][i] = frame[slow_fields[j][i].i0];

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
        i_in = slow_fields[j][i].i_in;
        if(++i_in >= MAXBUF)
          i_in = 0;
        if(i_in == slow_fields[j][i].i_out)
          write_ok = 0;
      }

    /****************************************** 
     * Check buffer space in normal fast data * 
     *****************************************/
    for (j = 0; write_ok && j < n_fast; j++) {
      i_in = normal_fast[j].i_in;
      if (++i_in >= MAXBUF)
        i_in = 0;
      if(i_in == normal_fast[j].i_out)
        write_ok = 0;
    }

    /*******************************************
     * Check buffer space in  fast bolo data   *
     *******************************************/
    for (i = 0; write_ok && i < DAS_CARDS; i++)
      for (j = 0; write_ok && j < DAS_CHS; j += 2) {
        i_in = bolo_fields[i][j].i_in;
        if (++i_in >= MAXBUF)
          i_in = 0;
        if(i_in == bolo_fields[i][j].i_out)
          write_ok = 0;
      }

    if (!write_ok)
      usleep(10000);
  } while (!write_ok);

  /*************************/
  /* PUSH RX FRAME TO DISK */
  /*************************/

  /* push if FAST_PER_SLOW fasts are done... */
  if (++i_count == FAST_PER_SLOW) {
    i_count = 0;
    for (i = 0; i < slowsPerBi0Frame; i++) {
      for (j = 0; j < FAST_PER_SLOW; j++) {
        i_in = slow_fields[j][i].i_in;
        ((unsigned short*)slow_fields[j][i].b)[i_in] = slow_data[j][i];

        if (++i_in >= MAXBUF)
          i_in = 0;
        slow_fields[j][i].i_in = i_in;
      }
    }
    n_frames++;
  }

  /********************/
  /* normal fast data */
  /********************/

  for (j = 0; j < n_fast; j++) {
    i_in = normal_fast[j].i_in;
    if (normal_fast[j].size == 2) {
      ((unsigned*)normal_fast[j].b)[i_in] =
        *((unsigned*)(frame + normal_fast[j].i0));      
    } else {
      ((unsigned short*)normal_fast[j].b)[i_in] =
        ((unsigned short*)frame)[normal_fast[j].i0];      
    }

    if (++i_in >= MAXBUF)
      i_in = 0;
    normal_fast[j].i_in = i_in;
  };

  /********************/
  /* fast bolo data   */
  /********************/
  for (i = 0; i < DAS_CARDS; i++) {
    for (j = 0; j < DAS_CHS; j += 2) {
      B0 = (unsigned)frame[boloIndex[i][j][0] + BoloBaseIndex] +
        (((unsigned)frame[boloIndex[i][j][1] + BoloBaseIndex] & 0xff00) << 8);
      B1 = frame[boloIndex[i][j + 1][0] + BoloBaseIndex] +
        ((frame[boloIndex[i][j + 1][1] + BoloBaseIndex] & 0x00ff) << 16);

      i_in = bolo_fields[i][j].i_in;
      ((unsigned*)bolo_fields[i][j].b)[i_in] = B0;
      ((unsigned*)bolo_fields[i][j + 1].b)[i_in] = B1;

      if (++i_in >= MAXBUF)
        i_in = 0;
      bolo_fields[i][j].i_in = bolo_fields[i][j + 1].i_in = i_in;
    }
  }
}

void WriteField(int file, int length, void *buffer)
{
  int gzerrno;
  const char *gze;

  if (rc.gzip_output && gzwrite((gzFile)file, buffer, length) == 0) {
    gze = gzerror((gzFile)file, &gzerrno);
    if (errno == Z_ERRNO)
      berror(fatal, "Error on write");
    else 
      bprintf(fatal, "Error on write: %s", gze);
  } else if (write(file, buffer, length) < 0)
    berror(fatal, "Error on write");
}

/* DirFileWriter: separate thread: writes each field to disk */
void DirFileWriter(void)
{
  unsigned short buffer[MAXBUF];
  unsigned int ibuffer[MAXBUF];
  int j, i;
  int i_in, i_out, i_buf;
  int k;
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
          while (i_in != i_out) {
            if ((SlowChList[i][j].type == 'U') ||
                (SlowChList[i][j].type == 'I')) {
              ibuffer[i_buf] = (unsigned)
                ((((unsigned short*)slow_fields[j][i].b)[i_out]) << 16)
                | (unsigned)
                (((unsigned short*)slow_fields[j][i + 1].b)[i_out]);
            } else
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

          if ((SlowChList[i][j].type == 'U') || (SlowChList[i][j].type == 'I'))
            WriteField(slow_fields[j][i].fp, i_buf * sizeof(unsigned), ibuffer);
          else 
            WriteField(slow_fields[j][i].fp, i_buf * sizeof(unsigned short),
                buffer);
        }
        slow_fields[j][i].i_out = slow_fields[j][i].i_in;
      }

    /********************** 
     ** normal fast data ** 
     **********************/
    for(j = 0; j < n_fast; j++) {
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

    /** Bolometer data */
    for (i = 0; i < DAS_CARDS; i++) {
      for (j = 0; j < DAS_CHS; j++) {
        i_in = bolo_fields[i][j].i_in;
        i_out = bolo_fields[i][j].i_out;

        i_buf = 0;
        k = 0;
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

    if (ri.wrote < wrote_count)
      ri.wrote = wrote_count;

    if (ri.reader_done && ri.wrote == ri.read) {
      ri.writer_done = 1;
      CleanUp();
      return;
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

        /* Read the new Spec file */
        ReconstructChannelLists();

        /* Re-initialise */
        wrote_count = 0;
        ri.read = ri.wrote = ri.old_total = 0;
        InitialiseDirFile(1);
        gettimeofday(&rc.start, &rc.tz);
        last_pass = 0;
      }
    }
  }
}
