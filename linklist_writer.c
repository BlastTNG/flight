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
#include <errno.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "linklist_writer.h"

#ifdef __cplusplus

extern "C"{

#endif

extern superframe_entry_t block_entry;
extern unsigned int ll_rawfile_default_fpf;

// creates a symlink for the rawfile with the new name pointing to the ll_rawfile basename 
void create_rawfile_symlinks(linklist_rawfile_t * ll_rawfile, char * newname) {
  char filename[LINKLIST_MAX_FILENAME_SIZE] = {0};
  char symname[LINKLIST_MAX_FILENAME_SIZE] = {0};

  snprintf(symname, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_EXT ".00", newname);
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_EXT ".00", ll_rawfile->basename);
  unlink(symname);
  symlink(filename, symname);

  snprintf(symname, LINKLIST_MAX_FILENAME_SIZE, "%s" SUPERFRAME_FORMAT_EXT, newname);
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" SUPERFRAME_FORMAT_EXT, ll_rawfile->basename);
  unlink(symname);
  symlink(filename, symname);

  snprintf(symname, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_FORMAT_EXT, newname);
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_FORMAT_EXT, ll_rawfile->basename);
  unlink(symname);
  symlink(filename, symname);

  snprintf(symname, LINKLIST_MAX_FILENAME_SIZE, "%s" CALSPECS_FORMAT_EXT, newname);
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" CALSPECS_FORMAT_EXT, ll_rawfile->basename);
  unlink(symname);
  symlink(filename, symname);
}

void make_linklist_rawfile_name(linklist_t * ll, char * filename) {
  // get the date string for file saving
  time_t now = time(0);
  char datestring[LINKLIST_MAX_FILENAME_SIZE] = {0};
  struct tm * tm_t = localtime(&now);
  strftime(datestring, sizeof(datestring)-1, "%Y-%m-%d-%H-%M-%S", tm_t);
  char tempname[LINKLIST_MAX_FILENAME_SIZE] = {0};

  int i;
  // strip possible extensions in name
  for (i = 0; i < strlen(ll->name); i++) {
    if (ll->name[i] == '.') break;
  }
  strncpy(tempname, ll->name, i);
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s/%s_%s", archive_dir, tempname, datestring);
}

int seekend_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  // seek to the current file and file location to be written to next
  int fileindex = ll_rawfile->isseekend;

  if (ll_rawfile->isseekend < 0) {
		// get the directory that the binary files are located
		int i, pos;
		char filename[LINKLIST_MAX_FILENAME_SIZE] = {0};
		strcpy(filename, ll_rawfile->basename);
		for (pos = strlen(filename)-1; pos >= 0; pos--) {
			if (filename[pos] == '/') {
				filename[pos] = '\0';
				break;
			}
		}

		// get list of the files in the directory
		struct dirent **dir;
		int n = scandir(filename, &dir, NULL, alphasort);

		// find the highest number binary fragment file with the matching name in the directory
		snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_EXT ".", ll_rawfile->basename+pos+1);
		for (i = 0; i < n; i++) {
			if (strncmp(filename, dir[i]->d_name, strlen(filename)) == 0) {
				int tmpindex = atoi(dir[i]->d_name+strlen(filename));
				fileindex = (tmpindex > fileindex) ? tmpindex : fileindex;
			}
		}
        if (fileindex < 0) fileindex = 0; // catch exception if no files exist
        ll_rawfile->isseekend = fileindex;
  }

  // printf("%d %d\n", fileindex);

  do {
    // seek to the beginning of the fragment files
    if (seek_linklist_rawfile(ll_rawfile, ll_rawfile->fpf*fileindex)) {
      return -1;
    }
    if (fseek(ll_rawfile->fp, 0, SEEK_END)) {
      return -1;
    }
    fileindex++;
  } while ((ll_rawfile->framenum = ftell(ll_rawfile->fp)/ll_rawfile->framesize) >= ll_rawfile->fpf);

  return 0; 
}

int seek_linklist_rawfile(linklist_rawfile_t * ll_rawfile, unsigned int framenum) {
  if (!ll_rawfile) {
    linklist_err("Null linklist_rawfile\n");
    return -1;
  }
  unsigned int fileindex = framenum / ll_rawfile->fpf;

  if (fileindex != ll_rawfile->fileindex || !ll_rawfile->fp) {
    ll_rawfile->fileindex = fileindex;
    if (fileindex > ll_rawfile->isseekend) ll_rawfile->isseekend = fileindex;

    // close the old file 
    if (ll_rawfile->fp) {
      fclose(ll_rawfile->fp);
      ll_rawfile->fp = NULL;
    }
    char filename[LINKLIST_MAX_FILENAME_SIZE];
    snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_EXT ".%.2u", ll_rawfile->basename, fileindex);
    ll_rawfile->fp = fpreopenb(filename);
    if (!ll_rawfile->fp) {
      linklist_err("Could not open raw linklist binary file %s\n", filename);
      return -2;
    }
  }

  ll_rawfile->framenum = framenum % ll_rawfile->fpf;

  return fseek(ll_rawfile->fp, ll_rawfile->framenum*ll_rawfile->framesize, SEEK_SET);
}

linklist_rawfile_t * open_linklist_rawfile(char * basename, linklist_t * ll) {
  return open_linklist_rawfile_opt(basename, ll, 0);
}

// flags are specific to linklist_rawfiles (separate from ll->flags)
linklist_rawfile_t * open_linklist_rawfile_opt(char * basename, linklist_t * ll, unsigned int flags) {
  if (!basename || (strlen(basename) == 0)) {
    linklist_err("Invalid rawfile basename");
    return NULL;
  }

  linklist_rawfile_t * ll_rawfile = calloc(1, sizeof(linklist_rawfile_t));
  strcpy(ll_rawfile->basename, basename);
  ll_rawfile->ll = ll;
  ll_rawfile->isseekend = -1;

  char filename[LINKLIST_MAX_FILENAME_SIZE];
  snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_FORMAT_EXT, ll_rawfile->basename);

  // get the number of frames per file (fpf)
  int fpf = read_linklist_formatfile_comment(filename, LINKLIST_FRAMES_PER_FILE_IND, "%d");
  if (fpf > 0) ll_rawfile->fpf = fpf;
  else ll_rawfile->fpf = ll_rawfile_default_fpf;

  int blk_size = read_linklist_formatfile_comment(filename, LINKLIST_FILE_SIZE_IND, "%d");
  if (blk_size > 0) {
    ll_rawfile->framesize = blk_size;
  } else {
    if (!ll_rawfile->ll) {
      linklist_err("Null linklist");
      free(ll_rawfile);
      return NULL;
    }
    ll_rawfile->framesize = ll->blk_size;
    if (ll->flags & LL_INCLUDE_ALLFRAME) ll_rawfile->framesize += ll->superframe->allframe_size;
  }

  if (flags & LL_RAWFILE_DUMMY) {
    // close the file descriptor
    if (ll_rawfile->fp) {
      fclose(ll_rawfile->fp);
      ll_rawfile->fp = NULL ;
    }
  } else {
    // open and seek to the beginning of the linklist rawfile
		if (seekend_linklist_rawfile(ll_rawfile) < 0) {
			free(ll_rawfile);
			return NULL;
		}
		if ((fpf < 0) || (blk_size < 0)) { // assume this is a new file, so write out the format files
			// write the superframe and linklist format files
			write_linklist_format(ll_rawfile->ll, filename);
			snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s" SUPERFRAME_FORMAT_EXT, ll_rawfile->basename);
			write_superframe_format(ll->superframe, filename);
		}
  }

  return ll_rawfile;
}

void close_and_free_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  if (!ll_rawfile) return;
  if (ll_rawfile->fp) fclose(ll_rawfile->fp);
  free(ll_rawfile); 
}

int tell_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  if (!ll_rawfile) { 
    linklist_err("Null rawfile linklist");
    return -1;
  }
  return ll_rawfile->fileindex*ll_rawfile->fpf+ll_rawfile->framenum;
}

int flush_linklist_rawfile(linklist_rawfile_t * ll_rawfile) {
  if (!ll_rawfile) { 
    linklist_err("Null rawfile linklist");
    return -1;
  }

  unsigned int retval = 0;
  if (ll_rawfile->fp) {
    retval = fflush(ll_rawfile->fp);
  }

  return retval;
}

int read_linklist_rawfile(linklist_rawfile_t * ll_rawfile, uint8_t * buffer) {
  if (!ll_rawfile) { 
    linklist_err("Null rawfile linklist");
    return -1;
  }
  if (!buffer) { 
    linklist_err("Null buffer");
    return -1;
  }

  unsigned int retval = 0;
  unsigned int framenum = ll_rawfile->fileindex*ll_rawfile->fpf+ll_rawfile->framenum;
  if (ll_rawfile->fp) {
    seek_linklist_rawfile(ll_rawfile, framenum); 
    retval = fread(buffer, ll_rawfile->framesize, 1, ll_rawfile->fp);
  }
  ll_rawfile->framenum++;
  return retval;
}

int write_linklist_rawfile(linklist_rawfile_t * ll_rawfile, uint8_t * buffer) {
  return write_linklist_rawfile_opt(ll_rawfile, buffer, 0);
}

int write_linklist_rawfile_opt(linklist_rawfile_t * ll_rawfile, uint8_t * buffer, unsigned int flags) {
  if (!ll_rawfile) { 
    linklist_err("Null rawfile linklist");
    return -1;
  }
  if (!buffer) { 
    linklist_err("Null buffer");
    return -1;
  }

  unsigned int retval = 0;
  unsigned int framenum = ll_rawfile->fileindex*ll_rawfile->fpf+ll_rawfile->framenum;
  if (ll_rawfile->fp) {
    seek_linklist_rawfile(ll_rawfile, framenum); 
    retval = fwrite(buffer, ll_rawfile->framesize, 1, ll_rawfile->fp);
  }
  ll_rawfile->framenum++;

  return retval;
}

#define N_BLOCK_DIRFILE_ENTRIES 6

linklist_dirfile_t * open_linklist_dirfile(char * dirname, linklist_t * ll) {
  return open_linklist_dirfile_opt(dirname, ll, 0);
}

linklist_dirfile_t * open_linklist_dirfile_opt(char * dirname, linklist_t * ll, unsigned int flags) {
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

  // create a map for superframe entries
  ll_dirfile->map = calloc(ll->superframe->n_entries, sizeof(uint8_t));

  // make the dir for the dirfile
  if (mkdir(ll_dirfile->filename, 00755) < 0) {
    if (errno == EEXIST) {
      linklist_info("%s dirfile exists. Appending data...\n", ll_dirfile->filename);
    } else {
      linklist_err("Could not create dirfile %s (errno %d: %s)\n", ll_dirfile->filename, errno, strerror(errno));
    }
  } else {
    linklist_info("New dirfile %s.\n", ll_dirfile->filename);
  }

  // open formatfile
  char formatname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  snprintf(formatname, LINKLIST_MAX_FILENAME_SIZE, "%s/format", ll_dirfile->filename);
  FILE * formatfile = fopen(formatname, "w");
  fprintf(formatfile,"# Linklist Dirfile Format File\n");
  fprintf(formatfile,"# Auto-generated by linklist_writer\n\n");

  fprintf(formatfile, "/VERSION 10\n");
  if (ll->superframe->flags & SF_USE_BIG_ENDIAN) {
    fprintf(formatfile, "/ENDIAN big\n");
  } else {
    fprintf(formatfile, "/ENDIAN little\n");
  }
  fprintf(formatfile, "/PROTECT none\n");
  fprintf(formatfile, "/ENCODING none\n");

  // generate binary files
  int i, j;
  char binname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  linkentry_t * tlm_le = NULL; 
  int tlm_index = 0;
  superframe_entry_t * sfe = ll->superframe->entries;

  // map out all the linklist entries within the superframe
  for (i = 0; i < ll->n_entries; i++) {
    tlm_le = &(ll->items[i]);
    if ((tlm_le->tlm) && (tlm_le->tlm != &block_entry)) {
			if ((tlm_index = superframe_entry_get_index(tlm_le->tlm, sfe)) == -1) {
				linklist_err("Could not get superframe index for \"%s\"\n", tlm_le->tlm->field);
				continue;
			}
      ll_dirfile->map[tlm_index] = 1; 
    }
  }

  // add all of the items from the superframe
  // those in the linklist are at the full rate
  // those not in the linklist are at 1 spf
  ll_dirfile->bin = (FILE **) calloc(ll->superframe->n_entries, sizeof(FILE *));
  for (i = 0; i < ll->superframe->n_entries; i++) {
    if (sfe[i].type == SF_NUM) continue; 

		// add entry to format file
		fprintf(formatfile,"%s RAW %s %d\n", sfe[i].field, 
																				 get_sf_type_string(sfe[i].type), 
																				 (ll_dirfile->map[i]) ? sfe[i].spf : 1);

    // add quantity
		if (strlen(sfe[i].quantity) > 0) {
			fprintf(formatfile,"%s/quantity STRING \"", sfe[i].field);
			for (j = 0; j < strlen(sfe[i].quantity); j++) {
				if (sfe[i].quantity[j] == 92) fprintf(formatfile, "\\"); // fix getdata escape
				fprintf(formatfile, "%c", sfe[i].quantity[j]);
			}
			fprintf(formatfile,"\"\n");
		}

    // add units
		if (strlen(sfe[i].units) > 0) {
			fprintf(formatfile,"%s/units STRING \"", sfe[i].field);
			for (j = 0; j < strlen(sfe[i].units); j++) {
				if (sfe[i].units[j] == 92) fprintf(formatfile, "\\"); // fix getdata escape
				fprintf(formatfile, "%c", sfe[i].units[j]);
			}
			fprintf(formatfile,"\"\n");
		}

		fflush(formatfile);

		// open the file if not already opened
		snprintf(binname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", ll_dirfile->filename, sfe[i].field);
		ll_dirfile->bin[i] = fpreopenb(binname);  
  }

  // add files for blocks
  ll_dirfile->blockbin = (FILE **) calloc(ll->num_blocks*N_BLOCK_DIRFILE_ENTRIES, sizeof(FILE *));
  for (i = 0; i < ll->num_blocks; i++) {
    #define WRITE_BLOCK_FORMAT_ENTRY(_FIELD)                                                   \
    ({                                                                                         \
			fprintf(formatfile, "%s_" #_FIELD " RAW %s %d\n", ll->blocks[i].name,                    \
                                                        get_sf_type_string(SF_UINT32),         \
                                                        1);                                    \
      fflush(formatfile);                                                                      \
      snprintf(binname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s_" #_FIELD, ll_dirfile->filename,    \
               ll->blocks[i].name);                                                            \
      ll_dirfile->blockbin[i*N_BLOCK_DIRFILE_ENTRIES+j] = fpreopenb(binname);                  \
      j++;                                                                                     \
    })
    j = 0;
    WRITE_BLOCK_FORMAT_ENTRY(id);
    WRITE_BLOCK_FORMAT_ENTRY(i);
    WRITE_BLOCK_FORMAT_ENTRY(n);
    WRITE_BLOCK_FORMAT_ENTRY(num);
    WRITE_BLOCK_FORMAT_ENTRY(alloc_size);
    WRITE_BLOCK_FORMAT_ENTRY(curr_size);
  }

  if (ll->superframe->calspecs[0]) {
    fprintf(formatfile, "\n####### Begin calspecs ######\n\n");
    FILE * calspecsfile = fopen(ll->superframe->calspecs, "rb");
    if (!calspecsfile) {
      linklist_err("Could not open calspecs file \"%s\"\n", ll->superframe->calspecs);
    }
    int a;
    while (1) {
			a = fgetc(calspecsfile); 
			if (!feof(calspecsfile)) fputc(a, formatfile);
			else break;
    }
    fclose(calspecsfile);
    fflush(formatfile);
  }

  ll_dirfile->format = formatfile;
  return ll_dirfile;
}

void close_and_free_linklist_dirfile(linklist_dirfile_t * ll_dirfile) {
  int i;
  for (i = 0; i < ll_dirfile->ll->superframe->n_entries; i++) {
    if (ll_dirfile->bin[i]) fclose(ll_dirfile->bin[i]);
  }
  for (i = 0; i < ll_dirfile->ll->num_blocks*N_BLOCK_DIRFILE_ENTRIES; i++) {
    if (ll_dirfile->blockbin[i]) fclose(ll_dirfile->blockbin[i]);
  }
  fclose(ll_dirfile->format);
  free(ll_dirfile->bin);
  free(ll_dirfile->blockbin);
  free(ll_dirfile->map);
  free(ll_dirfile);
}

int seek_linklist_dirfile(linklist_dirfile_t * ll_dirfile, unsigned int framenum) {
  int i = 0;
  unsigned int tlm_out_size = 0;
  unsigned int tlm_out_spf = 0;
  unsigned int dir_loc = 0;

  if (!ll_dirfile) { 
    linklist_err("Null dirfile linklist");
    return -1;
  }

  linklist_t * ll = ll_dirfile->ll;
  if (!ll) {
    linklist_err("Null linklist");
    return -1;
  }
  superframe_entry_t * sfe = ll->superframe->entries;

  // just a normal field to be writing to the dirfile
  for (i = 0; i < ll->superframe->n_entries; i++) {
		if (ll_dirfile->bin[i]) { 
			tlm_out_size = get_superframe_entry_size(&sfe[i]);
			tlm_out_spf = (ll_dirfile->map[i]) ? sfe[i].spf : 1;

			dir_loc = framenum*tlm_out_size*tlm_out_spf;
			fseek(ll_dirfile->bin[i], dir_loc, SEEK_SET);
		}
  }
  // block-specific fields to be writing to the dirfile
  for (i=0; i<(ll->num_blocks*N_BLOCK_DIRFILE_ENTRIES); i++) {
    if (ll_dirfile->blockbin[i]) {
      fseek(ll_dirfile->blockbin[i], framenum*sizeof(uint32_t), SEEK_SET);
    }
  } 
  ll_dirfile->framenum = framenum;
  return 0;
}

int flush_linklist_dirfile(linklist_dirfile_t * ll_dirfile) {
  int i = 0;

  if (!ll_dirfile) { 
    linklist_err("Null dirfile linklist");
    return -1;
  }

  linklist_t * ll = ll_dirfile->ll;
  if (!ll) {
    linklist_err("Null linklist");
    return -1;
  }

  // just a normal field to be writting to the dirfile
  for (i = 0; i < ll->superframe->n_entries; i++) {
		if (ll_dirfile->bin[i]) { 
			fflush(ll_dirfile->bin[i]);
		}
  }  
  // block-specific fields to be writing to the dirfile
  for (i=0; i<(ll->num_blocks*N_BLOCK_DIRFILE_ENTRIES); i++) {
    if (ll_dirfile->blockbin[i]) {
      fflush(ll_dirfile->blockbin[i]);
    }
  }

  return 0;
}

// writes a linklist buffer to a file
double write_linklist_dirfile(linklist_dirfile_t * ll_dirfile, uint8_t * buffer) {
  return write_linklist_dirfile_opt(ll_dirfile, buffer, 0);
}

double write_linklist_dirfile_opt(linklist_dirfile_t * ll_dirfile, uint8_t * buffer, unsigned int flags) {

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
  if (ll->flags & LL_INCLUDE_ALLFRAME) {
    read_allframe(superframe_buf, ll->superframe, buffer+ll->blk_size);
  }
  double ret_val = decompress_linklist_opt(superframe_buf, ll, buffer, UINT32_MAX, ll->flags);

  // write the data to the dirfile
  superframe_entry_t * sfe = ll->superframe->entries;
  // unsigned int dir_loc = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_out_size = 0;
  unsigned int tlm_out_skip = 0;
  unsigned int tlm_out_spf = 0;

  int i = 0;
  int j = 0;
  // just a normal field to be writing to the dirfile
  for (i = 0; i < ll->superframe->n_entries; i++) {
		if (ll_dirfile->bin[i]) { 
			tlm_out_start = sfe[i].start;
			tlm_out_skip = sfe[i].skip;
			tlm_out_size = get_superframe_entry_size(&sfe[i]);

      // entries in the linklist get the full rate
      // entries not in the linklist get 1 spf
			tlm_out_spf = (ll_dirfile->map[i]) ? sfe[i].spf : 1;

			// dir_loc = ll_dirfile->framenum*tlm_out_size*tlm_out_spf;

			for (j = 0; j < tlm_out_spf; j++) {
				fwrite(superframe_buf+tlm_out_start, tlm_out_size, 1, ll_dirfile->bin[i]);
				tlm_out_start += tlm_out_skip;
			}
		}
  }  
  // block-specific fields to be writing to the dirfile
  for (i = 0; i < ll->num_blocks; i++) {
    #define WRITE_BLOCK_ENTRY_DATA(_FIELD)                                                     \
    ({                                                                                         \
      FILE * blockfp = ll_dirfile->blockbin[i*N_BLOCK_DIRFILE_ENTRIES+j];                      \
      if (blockfp) fwrite(&ll->blocks[i]._FIELD, sizeof(uint32_t), 1, blockfp);                \
      j++;                                                                                     \
    })
    j = 0;
    WRITE_BLOCK_ENTRY_DATA(id);
    WRITE_BLOCK_ENTRY_DATA(i);
    WRITE_BLOCK_ENTRY_DATA(n);
    WRITE_BLOCK_ENTRY_DATA(num);
    WRITE_BLOCK_ENTRY_DATA(alloc_size);
    WRITE_BLOCK_ENTRY_DATA(curr_size);
  }
  memset(superframe_buf, 0, ll->superframe->size);
  
  ll_dirfile->framenum++;

  return ret_val;
}

#ifdef __cplusplus

}

#endif
