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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 
#include <time.h>

#include "tx_struct.h"
#include "defile.h"

void FPrintDerived(FILE* fp);

#define MAXBUF 3000 /* This is a 30 second buffer */

struct FieldType {
  short i0;   /* start of field in rxframe, words */
  short size; /* size in words */
  int i_in;   /* index in elements */
  int i_out;  /* index in elements */
  int fp;
  void* b;    /* buffer */
  long int nw;
};

struct FieldType normal_fast[N_FASTCHLIST];
struct FieldType slow_fields[N_SLOW][FAST_PER_SLOW];
struct FieldType bolo_fields[DAS_CARDS][DAS_CHS];

unsigned short slow_data[N_SLOW][FAST_PER_SLOW];
int buf_overflow;
int n_fast, bolo_i0;

extern unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

char* StringToLower(char* s)
{
  int i, len;
  static char ls[256];

  len = strlen(s);
  if (len > 255) len = 255;

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

/* Initialise dirfile */
void InitialiseDirFile(int reset)
{
  FILE* fp;
  int j, i;
  char field[FIELD_LEN];
  char gpb[GPB_LEN];


  if (mkdir(rc.dirfile, 00755) < 0) {
    snprintf(gpb, GPB_LEN, "defile: cannot create dirfile `%s'", rc.dirfile);
    perror(gpb);
    exit(0);
  }

  printf("\nWriting to dirfile %s\n", rc.dirfile);

  /*********************************** 
   * create and fill the format file * 
   ***********************************/
  sprintf(gpb, "%s/format", rc.dirfile);
  if ((fp = fopen(gpb, "w")) == NULL) {
    snprintf(gpb, GPB_LEN, "defile: cannot create format file `%s/format'",
        rc.dirfile);
    perror(gpb);
    exit(0);
  }
  fprintf(fp, "FASTSAMP         RAW    U 20\n");
  n_fast = 0;
  sprintf(gpb, "%s/FASTSAMP", rc.dirfile);
  if ((normal_fast[n_fast].fp = creat(gpb, 00644)) == -1) {
    snprintf(gpb, GPB_LEN, "defile: cannot create file `%s/FASTSAMP'",
        rc.dirfile);
    perror(gpb);
    exit(0);
  }
  normal_fast[n_fast].i0 = 1;
  if (reset)
    normal_fast[n_fast].nw = 0;
  normal_fast[n_fast].size = 2; 
  normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
  normal_fast[n_fast].b = malloc(MAXBUF*sizeof(int));
  n_fast++;

  /* slow chs */
  fprintf(fp, "\n## SLOW CHANNELS:\n");
  for (i = 0; i<N_SLOW; i++) {
    for (j = 0; j<FAST_PER_SLOW; j++) {
      if (strlen(SlowChList[i][j].field) > 0) {
        fprintf(fp, "%-16s RAW    %c 1\n",
            StringToLower(SlowChList[i][j].field),
            SlowChList[i][j].type);
        fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
            StringToUpper(SlowChList[i][j].field),
            StringToLower(SlowChList[i][j].field),
            SlowChList[i][j].m_c2e,
            SlowChList[i][j].b_e2e);
        if (fflush(fp) < 0) {
          perror("Error while flushing format file");
          exit(0);
        }
        sprintf(gpb, "%s/%s", rc.dirfile, SlowChList[i][j].field);
        if ((slow_fields[i][j].fp = creat(gpb, 00644)) == -1) {
          snprintf(gpb, GPB_LEN, "defile: cannot create file `%s/%s'",
              rc.dirfile, SlowChList[i][j].field);
          perror(gpb);
          exit(0);
        }
      } else {
        slow_fields[i][j].fp = -1;
      }
      slow_fields[i][j].i_in = slow_fields[i][j].i_out = 0;
      if (reset)
        slow_fields[i][j].nw = 0;
      slow_fields[i][j].b = malloc( 2 * MAXBUF);
      slow_fields[i][j].i0 = 4 + i;
    }
  }

  /* normal fast chs */
  fprintf(fp, "\n## FAST CHANNELS:\n");
  if (fflush(fp) < 0) {
    perror("Error while flushing format file");
    exit(0);
  }
  for (i = 0; i<N_FASTCHLIST; i++) {
    if (strcmp(FastChList[i].field, "n5c0lo") == 0) {
      bolo_i0 = i + FAST_OFFSET;
      break;
    } else if (strlen(FastChList[i].field) > 0) {
      switch (FastChList[i].type) {
        case 's': case 'u':
          normal_fast[n_fast].size = 1;
          break;
        case 'S': case 'U': case 'I': case 'f':
          normal_fast[n_fast].size = 2;
          break;
        default:
          printf("error: bad type in initdirfile: %s %c %d\n",
              FastChList[i].field, FastChList[i].type, i);
          exit(0);
      }
      sprintf(gpb, "%s/%s", rc.dirfile, FastChList[i].field);
      if ((normal_fast[n_fast].fp = creat(gpb, 00644)) == -1) {
        snprintf(gpb, GPB_LEN, "defile: cannot create file `%s/%s'",
            rc.dirfile, SlowChList[i][j].field);
        perror(gpb);
        exit(0);
      }
      normal_fast[n_fast].i0 = i + FAST_OFFSET;
      normal_fast[n_fast].i_in = normal_fast[n_fast].i_out = 0;
      if (reset)
        normal_fast[n_fast].nw = 0;
      normal_fast[n_fast].b = malloc(MAXBUF * 2 * normal_fast[n_fast].size);
      n_fast++;
      fprintf(fp, "%-16s RAW    %c %d\n",
          StringToLower(FastChList[i].field),
          FastChList[i].type, FAST_PER_SLOW);
      fprintf(fp, "%-16s LINCOM 1 %-16s %12g %12g\n",
          StringToUpper(FastChList[i].field),
          StringToLower(FastChList[i].field),
          FastChList[i].m_c2e, FastChList[i].b_e2e);
      if (fflush(fp) < 0) {
        perror("Error while flushing format file");
        exit(0);
      }
    }
  }

  /* special (bolo) fast chs */
  fprintf(fp, "\n## BOLOMETERS:\n");
  if (fflush(fp) < 0) {
    perror("Error while flushing format file");
    exit(0);
  }
  for (i = 0; i < DAS_CARDS; i++) {
    for (j = 0; j < DAS_CHS; j++) {
      sprintf(field, "n%dc%d", i + 5, j);
      sprintf(gpb, "%s/%s", rc.dirfile, field);
      if ((bolo_fields[i][j].fp = creat(gpb, 00644)) == -1) {
        snprintf(gpb, GPB_LEN, "defile: cannot create file `%s/%s'",
            rc.dirfile, SlowChList[i][j].field);
        perror(gpb);
        exit(0);
      }
      bolo_fields[i][j].size = 2;
      bolo_fields[i][j].i_in = bolo_fields[i][j].i_out = 0;
      if (reset)
        bolo_fields[i][j].nw = 0;
      bolo_fields[i][j].b = malloc(MAXBUF * 4);
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

  if (fclose(fp) < 0) {
    perror("Error while closing format file");
    exit(0);
  }

  if (rc.write_curfile) {
    if ((fp = fopen(rc.output_curfile, "w")) == NULL) {
      snprintf(gpb, GPB_LEN, "defile: cannot create curfile `%s'",
          rc.output_curfile);
      perror(gpb);
      exit(0);
    }

    fprintf(fp, rc.dirfile);

    if (fclose(fp) < 0) {
      snprintf(gpb, GPB_LEN, "defile: cannot close curfile `%s'",
          rc.output_curfile);
      perror(gpb);
      exit(0);
    }
  }

  ri.dirfile_init = 1;
}

/* Clean up in preparation for re-initialising dirfile with a new name
 * Close open file descriptors and free allocated memory */
void CleanUp(void)
{
  int j, i; 

  for(i = 0; i<N_SLOW; i++)
    for(j = 0; j<FAST_PER_SLOW; j++) {
      close(slow_fields[i][j].fp);
      free(slow_fields[i][j].b);
    }

  for(i = 0; i < N_FASTCHLIST; ++i) {
    if (strcmp(FastChList[i].field, "n5c0lo") == 0)
      break;
    close(normal_fast[n_fast].fp);
    free(normal_fast[n_fast].b);
  }

  for (i = 0; i < DAS_CARDS; i++)
    for (j = 0; j < DAS_CHS; j++) {
      close(bolo_fields[i][j].fp);
      free(bolo_fields[i][j].b);
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
    if (j < FAST_PER_SLOW) {
      for (i = 0; i<N_SLOW; i++) {
        slow_data[i][j] = frame[slow_fields[i][j].i0];
      }
    }

    /* ****************************************************************** */
    /* First make sure there is enough space in ALL the buffers           */
    /* We need to do it first and all at once to maintain synchronization */
    /* We discard the full frame id there is no space                     */
    /* ****************************************************************** */

    /************************************/   
    /* Check buffer space in slow field */
    /************************************/
    for (i = 0; write_ok && i < N_SLOW; i++)
      for (j = 0; write_ok && j < FAST_PER_SLOW; j++) {
        i_in = slow_fields[i][j].i_in;
        if(++i_in >= MAXBUF)  i_in = 0;
        if(i_in == slow_fields[i][j].i_out)
          write_ok = 0;
      }

    /****************************************** 
     * Check buffer space in normal fast data * 
     *****************************************/
    for (j = 0; write_ok && j < n_fast; j++) {
      i_in = normal_fast[j].i_in;
      if (++i_in>=MAXBUF)
        i_in = 0;
      if(i_in == normal_fast[j].i_out)
        write_ok = 0;
    }

    /*******************************************
     * Check buffer space in  fast bolo data   *
     *******************************************/
    for (i = 0; write_ok && i < DAS_CARDS; i++)
      for (j = 0; write_ok && j<DAS_CHS; j += 2) {
        i_in = bolo_fields[i][j].i_in;
        if (++i_in>=MAXBUF)
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
  if (++i_count==FAST_PER_SLOW) {
    i_count = 0;
    for (i = 0; i<N_SLOW; i++) {
      for (j = 0; j<FAST_PER_SLOW; j++) {
        i_in = slow_fields[i][j].i_in;
        ((unsigned short*)slow_fields[i][j].b)[i_in] = 
          slow_data[i][j];

        if (++i_in>=MAXBUF) i_in = 0;
        slow_fields[i][j].i_in = i_in;
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
        *((unsigned*)(frame+normal_fast[j].i0));      
    } else {
      ((unsigned short*)normal_fast[j].b)[i_in] =
        ((unsigned short*)frame)[normal_fast[j].i0];      
    }

    if (++i_in>=MAXBUF) i_in = 0;
    normal_fast[j].i_in = i_in;
  };

  /********************/
  /* fast bolo data   */
  /********************/
  for (i = 0; i<DAS_CARDS; i++) {
    for (j = 0; j<DAS_CHS; j += 2) {
      B0 = (unsigned)frame[boloIndex[i][j][0]] +
        (((unsigned)frame[boloIndex[i][j][1]] & 0xff00) << 8);
      B1 = frame[boloIndex[i][j + 1][0]] +
        ((frame[boloIndex[i][j + 1][1]] & 0x00ff) << 16);

      i_in = bolo_fields[i][j].i_in;
      ((unsigned*)bolo_fields[i][j].b)[i_in] = B0;
      ((unsigned*)bolo_fields[i][j + 1].b)[i_in] = B1;

      if (++i_in>=MAXBUF) i_in = 0;
      bolo_fields[i][j].i_in = bolo_fields[i][j+1].i_in = i_in;
    }
  }
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

  InitialiseDirFile(1);

  while (1) {
    /* slow data */
    for (i = 0; i<N_SLOW; i++) {
      for (j = 0; j<FAST_PER_SLOW; j++) {
        i_in = slow_fields[i][j].i_in;
        i_out = slow_fields[i][j].i_out;
        i_buf = 0;
        while (i_in != i_out) {
          if ((SlowChList[i][j].type=='U') ||
              (SlowChList[i][j].type=='I')) {
            ibuffer[i_buf] =
              (unsigned)
              ((((unsigned short*)slow_fields[i][j].b)[i_out]) << 16)
              | (unsigned)
              (((unsigned short*)slow_fields[i + 1][j].b)[i_out]);
          } else {
            buffer[i_buf] =
              ((unsigned short*)slow_fields[i][j].b)[i_out];
          }

          if (i == 0 && j == 0)
            wrote_count = ++slow_fields[i][j].nw * FAST_PER_SLOW;
          else if (wrote_count < ++slow_fields[i][j].nw * FAST_PER_SLOW)
            wrote_count = slow_fields[i][j].nw * FAST_PER_SLOW;

          i_out++;
          i_buf++;
          if (i_out >= MAXBUF)
            i_out = 0;
        }
        if ((SlowChList[i][j].type=='U') ||
            (SlowChList[i][j].type=='I')) {
          if ( (i_buf > 0) && (slow_fields[i][j].fp >= 0) ) 
            if (write(slow_fields[i][j].fp,
                  ibuffer, i_buf*sizeof(unsigned)) < 0) {
              perror("Error while writing slow channels");
            }
        } else {
          if ( (i_buf > 0)  && (slow_fields[i][j].fp >= 0) ) 
            if (write(slow_fields[i][j].fp,
                  buffer, i_buf*sizeof(unsigned short)) < 0) {
              perror("Error while writing slow channels");
            }
        }
        slow_fields[i][j].i_out = i_out;
      }
    }

    /********************** 
     ** normal fast data ** 
     **********************/
    for(j = 0; j<n_fast; j++) {
      i_in = normal_fast[j].i_in;
      i_out = normal_fast[j].i_out;
      i_buf = 0;
      if (normal_fast[j].size == 2) {
        while (i_in!=i_out) {
          ibuffer[i_buf] = ((unsigned int*)normal_fast[j].b)[i_out];

          if (wrote_count < ++normal_fast[j].nw)
            wrote_count = normal_fast[j].nw;

          i_out++;
          i_buf++;
          if (i_out>=MAXBUF) i_out = 0;
        }
        if (i_buf > 0) write(normal_fast[j].fp,
            ibuffer, i_buf*sizeof(unsigned int));
      } else {
        while (i_in!=i_out) {
          buffer[i_buf] = ((unsigned short*)normal_fast[j].b)[i_out];

          if (wrote_count < ++normal_fast[j].nw)
            wrote_count = normal_fast[j].nw;

          i_out++;
          i_buf++;
          if (i_out>=MAXBUF) i_out = 0;
        }
        if ( (i_buf > 0) && (normal_fast[j].fp >= 0) ) {
          if (write(normal_fast[j].fp, buffer, i_buf*sizeof(unsigned short))
              < 0) {
            perror("Error while writing fast channels");
          }
        }
      }
      normal_fast[j].i_out = i_out;

    }

    /** Bolometer data */
    for (i = 0; i<DAS_CARDS; i++) {
      for (j = 0; j<DAS_CHS; j++) {
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
          if (i_out>=MAXBUF) i_out = 0;
        }
        if ( (i_buf > 0) && (bolo_fields[i][j].fp >= 0) ) {
          if (write(bolo_fields[i][j].fp, 
                ibuffer, i_buf*sizeof(unsigned int)) < 0) {
            perror("Error while writing fast channels");
          }
        }
        bolo_fields[i][j].i_out = i_out;
      }
    }

    if (ri.wrote < wrote_count)
      ri.wrote = wrote_count;

    if (ri.reader_done && ri.wrote == ri.read) {
      ri.writer_done = 1;
      return;
    }

    usleep(1000);

    /* if dirfile_init is lowered, the reader has noticed a change in the
     * curfile prepare for cycling the writer. */
    if (ri.dirfile_init == 0) {
      /* perform one last pass to ensure we've written all the data (the reader
       * has certainly stopped by now, so we should be fine with a single
       * last pass */
      if (!last_pass)
        last_pass = 1;
      else {
        /* Clean Up and Reinitialise */
        CleanUp();
        InitialiseDirFile(0);
        wrote_count = 0;
        last_pass = 0;
      }
    }
  }
}
