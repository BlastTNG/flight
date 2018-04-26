/* 
 * linklist_writer.c: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of the SuperBIT project, modified and adapted for BLAST-TNG.
 *
 * linklist is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * linklist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: April 9, 2018 by Javier Romualdez
 */


#include <math.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "linklist_writer.h"

#ifdef __cplusplus

extern "C"{

#endif

extern superframe_entry_t block_entry;

void increment_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  if (!ll_rawfile) {
    linklist_err("Null linklist_rawfile\n");
    return;
  }

  // close old file if necessary
  if (ll_rawfile->fp) {
    fclose(ll_rawfile->fp);
    ll_rawfile->fp = NULL;
  }
  char filename[128];

  // set the framenum to zero
  ll_rawfile->framenum = 0;

  // open the binary file for raw linklist data
  sprintf(filename, "%s" LINKLIST_EXT ".%.2u", ll_rawfile->basename, ll_rawfile->filecount++);
  ll_rawfile->fp = fpreopenb(filename);
  if (!ll_rawfile->fp) {
    linklist_err("Could not open raw linklist binary file %s\n", filename);
    return;
  }
}
 
void make_linklist_rawfile_name(linklist_t * ll, char * filename) {
  // get the date string for file saving
  time_t now = time(0);
  char datestring[80] = {0};
  struct tm * tm_t = localtime(&now);
  strftime(datestring, sizeof(datestring)-1, "%Y-%m-%d-%H-%M-%S", tm_t);
  char tempname[80] = {0};

  int i;
  // strip possible extensions in name
  for (i = 0; i < strlen(ll->name); i++) {
    if (ll->name[i] == '.') break;
  }
  strncpy(tempname, ll->name, i);
  sprintf(filename, "%s/%s_%s", archive_dir, tempname, datestring);
}

linklist_rawfile_t * open_linklist_rawfile(linklist_t * ll, char * basename) {
  if (!ll) {
    linklist_err("Null linklist");
    return NULL;
  }
  if (!basename || (strlen(basename) == 0)) {
    linklist_err("Invalid rawfile basename");
    return NULL;
  }

  linklist_rawfile_t * ll_rawfile = calloc(1, sizeof(linklist_rawfile_t));
  strcpy(ll_rawfile->basename, basename);
  ll_rawfile->framenum = 0;
  ll_rawfile->filecount = 0;
  ll_rawfile->ll = ll;
  increment_linklist_rawfile(ll_rawfile);

  // free and return if could not open the file
  if (!ll_rawfile->fp) {
    free(ll_rawfile);
    return NULL;
  }

  // write the superframe and linklist format files
  char filename[128];
  sprintf(filename, "%s" SUPERFRAME_FORMAT_EXT, ll_rawfile->basename);
  write_superframe_format(ll->superframe, filename);
  sprintf(filename, "%s" LINKLIST_FORMAT_EXT, ll_rawfile->basename);
  write_linklist_format(ll_rawfile->ll, filename);

  return ll_rawfile;
}

void close_and_free_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  if (ll_rawfile->fp) fclose(ll_rawfile->fp);
  free(ll_rawfile); 
}
unsigned int write_linklist_rawfile(linklist_rawfile_t * ll_rawfile, uint8_t * buffer) {
  if (!ll_rawfile) { 
    linklist_err("Null rawfile linklist");
    return -1;
  }
  if (!buffer) { 
    linklist_err("Null buffer");
    return -1;
  }
  linklist_t * ll = ll_rawfile->ll;
  if (!ll) {
    linklist_err("Null linklist");
    return -1;
  }

  unsigned int writesize = ll->blk_size;
  unsigned int retval = 0;

  if (ll->flags & LL_INCLUDE_ALLFRAME) writesize += ll->superframe->allframe_size;

  if (ll_rawfile->fp) {
    fseek(ll_rawfile->fp, ll_rawfile->framenum*writesize, SEEK_SET);
    retval = fwrite(buffer, writesize, 1, ll_rawfile->fp);
    fflush(ll_rawfile->fp);
  }
  ll_rawfile->framenum++;
  return retval;
}

linklist_dirfile_t * open_linklist_dirfile(linklist_t * ll, char * dirname) {
  if (!ll) {
    linklist_err("Null linklist");
    return NULL;
  }
  if (!dirname || (strlen(dirname) == 0)) {
    linklist_err("Invalid dirfile name");
    return NULL;
  }

  // create linklist dirfile
  linklist_dirfile_t * ll_dirfile = calloc(1, sizeof(linklist_dirfile_t));
  if (dirname[strlen(dirname)-1] == '/') dirname[strlen(dirname)-1] = '\0';
  strcpy(ll_dirfile->filename, dirname);
  ll_dirfile->ll = ll;
  ll_dirfile->framenum = 0;

  // make the dir for the dirfile
	if (mkdir(dirname, 00755) < 0) {
		printf("%s directory exists\n", dirname);
		//perror("dirfile mkdir()");
		//exit(0);
	}

  // open formatfile
  char formatname[80] = {0};
  sprintf(formatname, "%s/format", ll_dirfile->filename);
  FILE * formatfile = fopen(formatname, "w");
  fprintf(formatfile,"# Linklist Dirfile Format File\n");
  fprintf(formatfile,"# Auto-generated by linklist_writer\n\n");

  fprintf(formatfile, "/VERSION 10\n");
  fprintf(formatfile, "/ENDIAN big\n");
  fprintf(formatfile, "/PROTECT none\n");
  fprintf(formatfile, "/ENCODING none\n");

  // generate binary files
  int i, j, k;
  char binname[80] = {0};
  linkentry_t * tlm_le = NULL; 

  ll_dirfile->bin = (FILE **) calloc(ll->n_entries, sizeof(FILE *));
  for (i = 0; i < ll->n_entries; i++) {
    tlm_le = &(ll->items[i]);
    if (tlm_le->tlm) {
      if (tlm_le->tlm == &block_entry) {
      } else {
        for (k = 0; k < i; k++) {
          if (ll->items[k].tlm && (tlm_le->tlm == ll->items[k].tlm)) break;
        }
        // comment out repeated entry 
        if (k != i) fprintf(formatfile, "# ");

        // add entry to format file
        fprintf(formatfile,"%s RAW %s %d\n", tlm_le->tlm->field, 
                                             get_sf_type_string(tlm_le->tlm->type), 
                                             tlm_le->tlm->spf);

        // comment out repeated entry 
        if (k != i) fprintf(formatfile, "# ");
        if (strlen(tlm_le->tlm->quantity) > 0)
        {
          fprintf(formatfile,"%s/quantity STRING \"", tlm_le->tlm->field);
          for (j = 0; j < strlen(tlm_le->tlm->quantity); j++)
          {
            if (tlm_le->tlm->quantity[j] == 92) fprintf(formatfile, "\\"); // fix getdata escape
            fprintf(formatfile, "%c", tlm_le->tlm->quantity[j]);
          }
          fprintf(formatfile,"\"\n");
        }

        // comment out repeated entry 
        if (k != i) fprintf(formatfile, "# ");

        if (strlen(tlm_le->tlm->units) > 0)
        {
          fprintf(formatfile,"%s/units STRING \"", tlm_le->tlm->field);
          for (j = 0; j < strlen(tlm_le->tlm->units); j++)
          {
            if (tlm_le->tlm->units[j] == 92) fprintf(formatfile, "\\"); // fix getdata escape
            fprintf(formatfile, "%c", tlm_le->tlm->units[j]);
          }
          fprintf(formatfile,"\"\n");
        }

        fflush(formatfile);

        // open the file if not already opened
        sprintf(binname, "%s/%s", ll_dirfile->filename, tlm_le->tlm->field);
        ll_dirfile->bin[i] = fpreopenb(binname);  

      } 
    } 
  }
  fclose(formatfile);

  return ll_dirfile;
}

void close_and_free_linklist_dirfile(linklist_dirfile_t * ll_dirfile) {
  int i;
  for (i = 0; i < ll_dirfile->ll->n_entries; i++) {
    if (ll_dirfile->bin[i]) fclose(ll_dirfile->bin[i]);
  }

  free(ll_dirfile->bin);
  free(ll_dirfile);
}

// writes a linklist buffer to a file
double write_linklist_dirfile(linklist_dirfile_t * ll_dirfile, uint8_t * buffer) {
  if (!ll_dirfile) { 
    linklist_err("Null dirfile linklist");
    return -1;
  }
  if (!buffer) { 
    linklist_err("Null buffer");
    return -1;
  }
  linklist_t * ll = ll_dirfile->ll;
  if (!ll) {
    linklist_err("Null linklist");
    return -1;
  }

  static uint8_t * superframe_buf = NULL;
  static int first_time = 1; 
  if (first_time) {
    superframe_buf = allocate_superframe(ll->superframe);
    first_time = 0;
  }

  // unpack the linklist
  double ret_val = decompress_linklist(superframe_buf, ll, buffer);

  // write the data to the dirfile
  linkentry_t * tlm_le = NULL;
  unsigned int dir_loc = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_out_size = 0;
  unsigned int tlm_out_skip = 0;
  unsigned int tlm_out_spf = 0;

  int i = 0;
  int j = 0;
  for (i = 0; i < ll->n_entries; i++) {
    tlm_le = &(ll->items[i]);
    if (tlm_le->tlm) {
      if (tlm_le->tlm == &block_entry) {
      } else if (ll_dirfile->bin[i]) { // just a normal field to be writting to the dirfile
        tlm_out_start = tlm_le->tlm->start;
        tlm_out_skip = tlm_le->tlm->skip;
        tlm_out_size = get_superframe_entry_size(tlm_le->tlm);
        tlm_out_spf = tlm_le->tlm->spf;

        dir_loc = ll_dirfile->framenum*tlm_out_size*tlm_out_spf;
        fseek(ll_dirfile->bin[i], dir_loc, SEEK_SET);

        for (j = 0; j < tlm_out_spf; j++) {
          fwrite(superframe_buf+tlm_out_start, tlm_out_size, 1, ll_dirfile->bin[i]);
          tlm_out_start += tlm_out_skip;
        }
        fflush(ll_dirfile->bin[i]);
      }
    }
  }  
  memset(superframe_buf, 0, ll->superframe->size);
  ll_dirfile->framenum++;

  return ret_val;
}

#ifdef __cplusplus

}

#endif
