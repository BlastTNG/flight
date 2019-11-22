/* 
 * linklist.c: 
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
 * Created on: Jan 25, 2018 by Javier Romualdez
 */
/**
 * Description:
 *
 * This file contains functions for parsing and initializing linklist
 * files, which are used for the entry selection and compression of
 * BLAST frames over telemetry downlinks.
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
#include <ctype.h>

#include "linklist.h"
#include "linklist_compress.h"

#define COMM '#'
#define SPEC ':'
#define MULT '|'
#define DESC '"'

#ifdef __cplusplus

extern "C" {

#endif

// user override-able functions for printfs
int (*linklist_info)(const char *, ...) = printf;
int (*linklist_err)(const char *, ...) = printf;
int (*linklist_warn)(const char *, ...) = printf;
int (*linklist_fatal)(const char *, ...) = printf;

int num_compression_routines = 0; // number of compression routines available
superframe_entry_t block_entry; // a dummy entry for blocks
superframe_entry_t stream_entry; // a dummy entry for streams
unsigned int ll_rawfile_default_fpf = 900; // number of frames per file before incrementing linklist rawfiles
char archive_dir[LINKLIST_MAX_FILENAME_SIZE] = "/data/rawdir";

static char * def_ll_extensions[] = {".ll", LINKLIST_FORMAT_EXT, ""}; 

const char * SF_TYPES_STR[] = {
  "UINT8", "UINT16", "UINT32", "UINT64", 
  "INT8", "INT16", "INT32", "INT64", 
  "FLOAT32", "FLOAT64", "BLOCK", "STREAM"
};
const char * get_sf_type_string(uint8_t m_type)
{
  return SF_TYPES_STR[m_type];
}

uint8_t get_sf_type_int(char * str) {
  int i = 0;
  
  for (i = 0; SF_TYPES_STR[i][0]; i++) {
    if (strncmp(str, SF_TYPES_STR[i], strlen(SF_TYPES_STR[i])) == 0) break;
  }
  return i;
}

int hashkey = 5381;

//Implements the djb2 hashing algorithm for char*s 
unsigned int hash(const char* input){
  int returnVal = hashkey;
  int c;

  while ((c = *input++))
    returnVal = ((returnVal << 5) + returnVal) + c;

  return returnVal;
}

double def_datatodouble(uint8_t * data, uint8_t type)
{
  switch (type) {
    case SF_FLOAT64 : return (*((double *) data));
    case SF_FLOAT32 : return (*((float *) data));
    case SF_INT16 : return (int16_t) (*((int16_t *) data));
    case SF_UINT16 : return (*((uint16_t *) data));
    case SF_INT32 : return (int32_t) (*((int32_t *) data));
    case SF_UINT32 : return (*((uint32_t *) data));
    case SF_INT8 : return *((int8_t *) data);
    case SF_UINT8 : return *((uint8_t *) data);
    default : return 0;
  }
  return 0;
}
int def_doubletodata(uint8_t * data, double dub, uint8_t type)
{
  if (type == SF_FLOAT64)
  {
    *(double *) data = dub;
    return 8;
  } else if (type == SF_FLOAT32) {
    *(float *) data = dub;
    return 4;
  } else if (type == SF_INT16) {
    int16_t s = dub;
    *(int16_t*) data = s;
    return 2;
  } else if (type == SF_UINT16) {
    uint16_t u = dub;
    *(uint16_t*) data = u;
    return 2;
  } else if (type == SF_INT32) {
    int32_t i = dub;
    *(int32_t*) data = i;
    return 4;
  } else if (type == SF_UINT32) {
    uint32_t i = dub;
    *(uint32_t*) data = i;
    return 4;
  } else if (type == SF_INT8) {
    *(int8_t*) data = dub;
    return 1;
  } else if (type == SF_UINT8) {
    *(uint8_t*) data = dub;
    return 1;
  }
  return 0;
}

double def_datatodouble_be(uint8_t * data, uint8_t type)
{
  switch (type) {
    case SF_FLOAT64 : return bedtoh(*((double *) data));
    case SF_FLOAT32 : return beftoh(*((float *) data));
    case SF_INT16 : return (int16_t) be16toh(*((int16_t *) data));
    case SF_UINT16 : return be16toh(*((uint16_t *) data));
    case SF_INT32 : return (int32_t) be32toh(*((int32_t *) data));
    case SF_UINT32 : return be32toh(*((uint32_t *) data));
    case SF_INT8 : return *((int8_t *) data);
    case SF_UINT8 : return *((uint8_t *) data);
    default : return 0;
  }
  return 0;
}
int def_doubletodata_be(uint8_t * data, double dub, uint8_t type)
{
  if (type == SF_FLOAT64) {
    htobed(dub, *(uint64_t*) data);
    return 8;
  } else if (type == SF_FLOAT32) {
    htobef(dub, *(uint32_t*) data)
    return 4;
  } else if (type == SF_INT16) {
    int16_t s = dub;
    *(int16_t*) data = htobe16(s);
    return 2;
  } else if (type == SF_UINT16) {
    uint16_t u = dub;
    *(uint16_t*) data = htobe16(u);
    return 2;
  } else if (type == SF_INT32) {
    int32_t i = dub;
    *(int32_t*) data = htobe32(i);
    return 4;
  } else if (type == SF_UINT32) {
    uint32_t i = dub;
    *(uint32_t*) data = htobe32(i);
    return 4;
  } else if (type == SF_INT8) {
    *(int8_t*) data = dub;
    return 1;
  } else if (type == SF_UINT8) {
    *(uint8_t*) data = dub;
    return 1;
  }
  return 0;
}

void linklist_assign_datatodouble(superframe_t * superframe, double (*func)(uint8_t *, uint8_t)) {
  if (func) {
    superframe->datatodouble = func;
  } else {
    superframe->datatodouble = &def_datatodouble;
  }
}
void linklist_assign_doubletodata(superframe_t * superframe, int (*func)(uint8_t *, double, uint8_t)) {
  if (func) {
    superframe->doubletodata = func;
  } else {
    superframe->doubletodata = &def_doubletodata;
  }
}

superframe_t * linklist_build_superframe(superframe_entry_t* m_superframe_list, 
                                         double (*datatodouble)(uint8_t *, uint8_t), 
                                         int (*doubletodata)(uint8_t *, double, uint8_t),
                                         unsigned int flags) {

  superframe_t * superframe = (superframe_t *) calloc(1, sizeof(superframe_t));  

  superframe->entries = m_superframe_list;
  linklist_assign_datatodouble(superframe, datatodouble);
  linklist_assign_doubletodata(superframe, doubletodata);
  superframe->size = 0;
  superframe->n_entries = 0;
  superframe->flags = flags;

  int i = 0;
  for (i = 0; superframe->entries[i].field[0]; i++) {
    superframe->size += get_superframe_entry_size(&superframe->entries[i])*superframe->entries[i].spf;
    superframe->entries[i].superframe = superframe; // reference superframe
    superframe->n_entries++;
  }

  superframe->hash_table_size = superframe->n_entries*1000;
  superframe->hash_table = (superframe_entry_t **) calloc(superframe->hash_table_size, sizeof(superframe_entry_t *));
 
  for (i = 0; i < (int) superframe->n_entries; i++) {
    unsigned int hashloc = hash(superframe->entries[i].field)%superframe->hash_table_size;
    if (superframe->hash_table[hashloc]) {
      memset(superframe->hash_table, 0, sizeof(superframe_entry_t *)*superframe->hash_table_size);
      hashkey = (hashkey*137)&0xffff;
      linklist_err("Hash with entry \"%s\". Trying key %d\n", superframe->entries[i].field, hashkey);
      i = -1;
    } else {
      superframe->hash_table[hashloc] = &superframe->entries[i];
    }
  }

  superframe->serial = generate_superframe_serial(superframe);
  superframe->allframe_size = write_allframe(NULL, superframe, NULL);

  return superframe;
}

superframe_entry_t * superframe_find_by_name(superframe_t * sf, const char * name) {
  if (!name) return NULL;

  return sf->hash_table[hash(name)%sf->hash_table_size];
}

void realloc_list(linklist_t * ll, int * x)
{
  *x += 100;
  ll->items = (linkentry_t *) realloc(ll->items,(*x)*(sizeof(linkentry_t)));
}

void parse_line(char *line, char ** sinks, unsigned int nargs)
{
  int i, j;
  unsigned int n;
  int read = strlen(line);
  
  for (n=0;n<nargs;n++) sinks[n] = NULL;
  
  char * buffer = (char *) calloc(2*read,1);
  int quoteon = 0;  

  n = 0;
  j = 0;
  sinks[n++] = &line[j];
  
  for (i=0;i<read;i++)
  {
    while (((line[i] == ' ') || (line[i] == '\t')) && (n<nargs) && !quoteon)
    {
      i++;
      // concatenate args separated by MULT symbol
      if ((line[i] != ' ') && (line[i] != '\t') && 
        (line[i] != MULT) && (buffer[j-1] != MULT) &&
        (line[i] != '=') && (buffer[j-1] != '='))
      {
        buffer[j++] = '\0';
        sinks[n++] = &line[j];
      }
    }
    if (line[i] != DESC) buffer[j++] = line[i];
    else quoteon = 1-quoteon; 
  }
  for (i=n;((unsigned int)i)<nargs;i++) sinks[i] = &line[j];
  buffer[j++] = '\0';
  memcpy(line,buffer,j+1);
  free(buffer);
  
}

static int one (const struct dirent * unused)
{
  (void) unused;
  return 1;
}

int load_all_linklists(superframe_t * superframe, char * linklistdir, linklist_t ** ll_array, unsigned int flags) {
  return load_all_linklists_opt(superframe, linklistdir, ll_array, flags, NULL);
}
int load_all_linklists_opt(superframe_t * superframe, char * linklistdir, linklist_t ** ll_array, unsigned int flags, char ** exts) {
  if (!exts) exts = def_ll_extensions;

  struct dirent **dir;
  int n = scandir(linklistdir,&dir,one,alphasort);
  int i, j;
  int num = 0;

  if (n<0) { 
    linklist_fatal("Cannot open the linklists directory %s", linklistdir);
  } else if (n>=MAX_NUM_LINKLIST_FILES) { 
    linklist_fatal("Max linklists in %s\n",linklistdir);
  }
  
  // check to see if there are already linklists allocated
  i = 0;
  while (ll_array[i]) {
    delete_linklist(ll_array[i]);
    ll_array[i] = 0;
    i++;
  }

  char full_path_name[LINKLIST_MAX_FILENAME_SIZE] = {0};

  // load linklist names
  for (i = 0; i < n; i++) {
    int len = strlen(dir[i]->d_name);
    for (j=0; exts[j][0]; j++) {
      int extlen = strlen(exts[j]);
      if ((len >= extlen) && strcmp(&dir[i]->d_name[len-extlen], exts[j]) == 0) {
        snprintf(full_path_name, LINKLIST_MAX_FILENAME_SIZE, "%s%s",linklistdir, dir[i]->d_name);
        
        if ((ll_array[num] = parse_linklist_format_opt(superframe, full_path_name, flags)) == NULL) {
          linklist_fatal("Unable to load linklist at %s", full_path_name);
        }
        num++;
        break;
      }
    }
  }
  ll_array[num] = generate_superframe_linklist_opt(superframe, flags); // last linklist contains all the telemetry items
  ll_array[num+1] = NULL; // null terminate the list

  linklist_info("Total of %d linklists loaded from \"%s\"\n", num, linklistdir);

  return 1;
}

int linklist_find_id_by_name(char * name, linklist_t ** ll_array) {
  linklist_t * ll = ll_array[0];
  int i = 0;

  while (ll) {
    if (strcmp(ll->name, name) == 0) return i;
    ll = ll_array[++i];
  }
  // linklist_err("Linklist \"%s\" not found.\n", name);

  return -1;
}

linklist_t * linklist_find_by_name(char * name, linklist_t ** ll_array) {
  int i = linklist_find_id_by_name(name, ll_array);
  if (i >= 0) return ll_array[i];
  return NULL;
}

int set_checksum_field(linkentry_t * le, unsigned int byteloc)
{
  int blk_size = 2; // checksums are 16 bit

  le->tlm = NULL; // no tlm
  le->comp_type = 255; // arbitrary
  le->start = byteloc;
  le->num = 1; // only one 
  le->blk_size = blk_size;

  return blk_size;
}

int superframe_entry_get_index(superframe_entry_t * sfi, superframe_entry_t * sf) {
  long unsigned int item = (long unsigned int) sfi;
  long unsigned int base = (long unsigned int) sf;
  if (item < base) return -1;
  return (item-base)/sizeof(superframe_entry_t);
}

void update_linklist_hash(MD5_CTX *mdContext, linkentry_t * le)
{
  MD5_Update(mdContext, &le->start, sizeof(le->start));
  MD5_Update(mdContext, &le->comp_type, sizeof(le->comp_type));
  MD5_Update(mdContext, &le->blk_size, sizeof(le->blk_size));
  MD5_Update(mdContext, &le->num, sizeof(le->num));
}

void update_superframe_entry_hash(MD5_CTX *mdContext, superframe_entry_t * chan)
{
  MD5_Update(mdContext, chan->field, SF_FIELD_LEN);
  MD5_Update(mdContext, &chan->type, sizeof(chan->type));
  MD5_Update(mdContext, &chan->spf, sizeof(chan->spf));
  MD5_Update(mdContext, &chan->start, sizeof(chan->start));
  MD5_Update(mdContext, &chan->skip, sizeof(chan->skip));
}

int parse_block(linklist_t * ll, char * name)
{
  if (ll->num_blocks >= MAX_DATA_BLOCKS) {
    linklist_err("parse_block: cannot add \"%s\"; maximum data blocks (%d)\n", 
                  name, MAX_DATA_BLOCKS);
    return -1;
  }

  strcpy(ll->blocks[ll->num_blocks].name, name);
  ll->blocks[ll->num_blocks].id = 0; // use id as block send count
  ll->blocks[ll->num_blocks].buffer = (uint8_t *) calloc(1, DEF_BLOCK_ALLOC);
  ll->blocks[ll->num_blocks].alloc_size = DEF_BLOCK_ALLOC;
  ll->blocks[ll->num_blocks].le = &(ll->items[ll->n_entries]);
  ll->blocks[ll->num_blocks].i = 1; // inits
  ll->blocks[ll->num_blocks].n = 1; // inits
  ll->blocks[ll->num_blocks].num = 0; // inits

  ll->blocks[ll->num_blocks].filename[0] = 0; // inits
  ll->blocks[ll->num_blocks].fp = NULL; // inits

  return ll->num_blocks++;
}

int parse_stream(linklist_t * ll, char * name)
{
  if (ll->num_streams >= MAX_DATA_STREAMS) {
    linklist_err("parse_stream: cannot add \"%s\"; maximum data streams (%d)\n", 
                  name, MAX_DATA_STREAMS);
    return -1;
  }

  int ind = ll->num_streams;
  strcpy(ll->streams[ind].name, name);
  ll->streams[ind].le = &(ll->items[ll->n_entries]);

  int i;
  for (i=0; i<2; i++) {
    ll->streams[ind].buffers[i].data_size = 0;
    ll->streams[ind].buffers[i].loc = 0;
    ll->streams[ind].buffers[i].buffer = (uint8_t *) calloc(1, DEF_STREAM_ALLOC);
    ll->streams[ind].buffers[i].alloc_size = DEF_STREAM_ALLOC;
  }

  ll->streams[ind].curr = 0;
  ll->streams[ind].next = 0;

  return ll->num_streams++;
}

block_t * linklist_find_block_by_pointer(linklist_t * ll, linkentry_t * le)
{
  int i;
  for (i = 0; i < (int) ll->num_blocks; i++) {
    if (ll->blocks[i].le == le) return &ll->blocks[i];
  }
  return NULL;
}
stream_t * linklist_find_stream_by_pointer(linklist_t * ll, linkentry_t * le)
{
  int i;
  for (i = 0; i < (int) ll->num_streams; i++) {
    if (ll->streams[i].le == le) return &ll->streams[i];
  }
  return NULL;
}

uint32_t get_superframe_entry_size(superframe_entry_t * chan) {
  if (!chan) return 0;
  size_t retsize = 0;

  switch (chan->type) {
    case SF_INT8:
    case SF_UINT8:
    case SF_NUM:
    case SF_INF:
      retsize = 1;
      break;
    case SF_INT16:
    case SF_UINT16:
      retsize = 2;
      break;
    case SF_INT32:
    case SF_UINT32:
    case SF_FLOAT32:
      retsize = 4;
      break;
    case SF_INT64:
    case SF_UINT64:
    case SF_FLOAT64:
      retsize = 8;
      break;
    default:
      linklist_fatal("Invalid channel size for %s!\n", chan->field);
  }
  return retsize;
}

int linklist_get_comp_index(char * name) {
  int i;
  for (i=0; i<NUM_COMPRESS_TYPES; i++) {
    if (strcmp(name, compRoutine[i].name) == 0) break;
  }
  if (i != NUM_COMPRESS_TYPES) {
    return i;
  } else {
    linklist_err("Could not find compression type \"%s\"\n", name);
    return NO_COMP;
  }
}

int linklist_get_default_compression(superframe_entry_t * tlm, int flags) {
  // with the LL_AUTO_FLOAT_COMP flag, floats are automatically compressed to fixed point
  // 32 bit float => 16 bit fixed point
  // 64 bit float => 32 bit fixed point
  if ((flags & LL_AUTO_FLOAT_COMP) && tlm &&
      ((tlm->max != 0) || (tlm->min != 0))) {
      if (tlm->type == SF_FLOAT32) {
        return (enum dataCompressTypes) FIXED_PT_16BIT; 
      } else if (tlm->type == SF_FLOAT64) {
        return (enum dataCompressTypes) FIXED_PT_32BIT;
      }
  }
  // the default without special flags is no compression
  return NO_COMP;
}

char * strip_path(char * fname) {
  int i;
  for (i = strlen(fname)-1; i > 0; i--) {
    if (fname[i] == '/') { // got the filename
      i++;
      break;
    }
  }
  return (fname+i);
}
/**
 * parse_linklist_format_opt
 * 
 * Returns a pointer to a linklist parsed from file.
 * -> fname: path to linklist to be parsed
 */

linklist_t * parse_linklist_format(superframe_t * superframe, char *fname) {
  return parse_linklist_format_opt(superframe, fname, 0);
}

linklist_t * parse_linklist_format_opt(superframe_t * superframe, char *fname, int flags)
{
  // count the number of compression routines available
  if (num_compression_routines == 0) {
    int c = 0;
    while (strlen(compRoutine[c].name)) c++;
    num_compression_routines = c;
  }
  if (superframe == NULL) {
    linklist_err("parse_linklist_format: superframe is null\n");
    return NULL;
  }

  FILE * cf = fopen(fname,"r"); 
  if (cf == NULL) {
    linklist_err("parse_linklist_format: cannot find %s\n",fname);
    return NULL; 
  }

  int i, st;
  int read;
  char *line = NULL;
  size_t len = 0;

  uint8_t comp_type;
  uint32_t blk_size, num;
  uint32_t byteloc = 0;
  unsigned int chksm_count = 0;

  char *temps[20];
  int def_n_entries = 150;
  int optflag = 1;

  // allocate new linklist with entries and assign superframe
  linklist_t * ll = (linklist_t *) calloc(1,sizeof(linklist_t));
  ll->items = (linkentry_t *) calloc(def_n_entries,sizeof(linkentry_t));
  ll->superframe = superframe;

  // allocate extended types
  ll->blocks = (struct block_container *) calloc(MAX_DATA_BLOCKS, sizeof(struct block_container));
  ll->num_blocks = 0;
  ll->streams = (struct stream_container *) calloc(MAX_DATA_STREAMS, sizeof(struct stream_container));
  ll->num_streams = 0;

  // MD5 hash
  MD5_CTX mdContext;
  uint8_t md5hash[MD5_DIGEST_LENGTH] = {0};
  MD5_Init(&mdContext); // initialize hash

  // initial field for first checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
  update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  while ((read = getline(&line, &len, cf)) != -1) {
    // remove carriage return, trailing whitespace, etc
    line[read-1] = 0;
    read--;
    for (i = (read-1); i >= 0; i--) {
      if ((line[i] != ' ') && (line[i] != '\t')) break;
      else line[i] = 0;
    }
    read = i+1;
    
    if ((int) ll->n_entries >= (def_n_entries-1)) {
      realloc_list(ll,&def_n_entries);
    }

    // remove whitespace
    st = 0;
    while ((line[st] == '\t') || (line[st] == ' ')) st++;
    if ((line[st] != COMM) && (line[st] != '\n') && ((read-st) > 0)) { // skip comments and blank lines
      // check for options at the beginning of file
      if (optflag) {
        // check for auto checksum field
        if (strncmp(line+st, STR(LL_NO_AUTO_CHECKSUM), strlen(STR(LL_NO_AUTO_CHECKSUM))) == 0) {
          flags |= LL_NO_AUTO_CHECKSUM; 
          continue;
        }
      }
      optflag = 0;

      // add min checksum if necessary
      if ((chksm_count >= MIN_CHKSM_SPACING) && !(flags & LL_NO_AUTO_CHECKSUM)) {
        blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
        update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
        byteloc += blk_size;
        ll->n_entries++;
        chksm_count = 0;
      } 

      parse_line(line+st,temps,6);
      memset(&ll->items[ll->n_entries],0,sizeof(linkentry_t));

      if (strcmp(temps[0], LL_PARSE_CHECKSUM) == 0) { // special checksum field indicator
        blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
        chksm_count = 0;
      } else { // not a checksum field
        superframe_entry_t * chan = NULL;
        if (strcmp(temps[1], "B") == 0) { // blocks extended extended item
          chan = &block_entry; // dummy entry associated with block
          parse_block(ll, temps[0]);
          comp_type = NO_COMP;
        } else if (strcmp(temps[1], "S") == 0) { // streams extended item 
          chan = &stream_entry; // dummy entry associated with stream
          parse_stream(ll, temps[0]);
          comp_type = NO_COMP;
        } else { // just a normal field
          // find the entry in the superframe
          if (!(chan = superframe_find_by_name(superframe, temps[0]))) {
            linklist_err("**** parse_linklist_format (%s): unable to find telemetry entry %s ****\n",
                         fname, temps[0]);
            continue;
          }

          // compression can be:
          // - NONE: no compression
          // - ##: compression indexed by number in compRoutines
          // - name: compression indicated by name in compRoutines
          if ((strcmp(temps[1], "NONE") == 0) || (strlen(temps[1]) == 0)) {
            comp_type = linklist_get_default_compression(chan, flags);
          } else if (strspn(temps[1], "0123456789") == strlen(temps[1])) { // normal field, number
            comp_type = atoi(temps[1]); // get compression type
          } else { // normal field, string
            comp_type = linklist_get_comp_index(temps[1]);
          }
          // check that comp_type is within range
          if ((comp_type >= num_compression_routines) && (comp_type != NO_COMP)) {
            linklist_err("Invalid comp. type %d for \"%s\". Defaulting to uncompressed.\n",
                         comp_type, chan->field);
            comp_type = NO_COMP;
          }
        }

        // we know the superframe entry associated with the linklist entry, so update the hash
        update_superframe_entry_hash(&mdContext, chan);
        ll->items[ll->n_entries].tlm = chan;

        // get compressed samples per frame
        if ((strcmp(temps[2], "NONE") == 0) || (strlen(temps[2]) == 0)) {
          num = chan->spf; // no compression means same number as superframe entry
        } else {
          num = atoi(temps[2]); // get compressed samples per frame 
        }

        // fill the link entry structure
        ll->items[ll->n_entries].comp_type = comp_type;
        ll->items[ll->n_entries].start = byteloc;
        ll->items[ll->n_entries].num = num;
        ll->items[ll->n_entries].linklist = ll;

        // determine size of entry in the frame
        blk_size = 0;
        if (comp_type != NO_COMP) { // normal compressed field
          blk_size = (*compRoutine[comp_type].compressFunc)(NULL, &ll->items[ll->n_entries], NULL);
        } else { // no compression, so identical field to telemlist, but with decimation
          blk_size = num*get_superframe_entry_size(ll->items[ll->n_entries].tlm);
        }

        // assign blk_size bytes to entry in the frame
        if (blk_size > 0) {
          ll->items[ll->n_entries].blk_size = blk_size;
        } else {
          linklist_err("parse_linklist_format: zero compressed size for %s in %s\n",
                        ll->items[ll->n_entries].tlm->field,fname);
        }
      }

      // update the serial
      update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
      byteloc += blk_size;
      chksm_count += blk_size;
      ll->n_entries++;
    }
  }
  fclose(cf);

  // check memory allocation
  if ((int) ll->n_entries >= (def_n_entries-1)) {
    realloc_list(ll,&def_n_entries);
  }

  // add one last field for final checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
  update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  // fix linkentry pointers for blocks and streams
  int b_ind = 0;
  int s_ind = 0;
  for (i=0; i<(int) ll->n_entries; i++) {
    if (ll->items[i].tlm == &block_entry) {
      ll->blocks[b_ind].le = &ll->items[i];
      b_ind++;
    } else if (ll->items[i].tlm == &stream_entry) {
      ll->streams[s_ind].le = &ll->items[i];
      s_ind++;
    } 
  }
  if (b_ind != (int) ll->num_blocks) {
    linklist_err("Found inconsistent number of blocks (%d != %d)\n", b_ind, ll->num_blocks);
  }
  if (s_ind != (int) ll->num_streams) {
    linklist_err("Found inconsistent number of streams (%d != %d)\n", s_ind, ll->num_streams);
  }

  int file_blk_size = read_linklist_formatfile_comment(fname, LINKLIST_FILE_SIZE_IND, "%d");
  if ((file_blk_size > 0) && (file_blk_size != (int) byteloc)) {
    if (file_blk_size == (int) (byteloc+superframe->allframe_size)) {
      flags |= LL_INCLUDE_ALLFRAME;
    } else {
      linklist_err("File blksize %d inconsistent with parsed blksize %d\n", file_blk_size, byteloc);
    }
  }

  ll->blk_size = byteloc;
  ll->flags = flags;

  // add the linklist name
  memset(ll->name, 0, LINKLIST_SHORT_FILENAME_SIZE); // clear name completely
  strcpy(ll->name, strip_path(fname)); // copy the name

  // update the hash
  MD5_Update(&mdContext, &byteloc, sizeof(byteloc));
  MD5_Update(&mdContext, &ll->n_entries, sizeof(ll->n_entries));
  //MD5_Update(&mdContext, ll->name, strlen(ll->name));

  // generate serial
  MD5_Final(md5hash,&mdContext);
  memcpy(ll->serial,md5hash,MD5_DIGEST_LENGTH);

  // set defaults for the internal (de)compression routines
  ll->internal_buffer = NULL;
  ll->internal_id = 0;

  return ll;
}

// this should be the inverse of parse_linklist_format
void write_linklist_format(linklist_t * ll, char * fname)
{
  write_linklist_format_opt(ll, fname, ll->flags);
}

// this should be the inverse of parse_linklist_format
// flags are in addition to the ll->flags generated by parse_linklist_format
void write_linklist_format_opt(linklist_t * ll, char * fname, int flags)
{
  int i;
  FILE * formatfile = fopen(fname, "w");

  if (formatfile == NULL) {
    linklist_err("Unable to generate linklist file \"%s\"", fname);
    return;
  }
  flags |= ll->flags; // add the flags from linklist

  fprintf(formatfile, "#\n");
  fprintf(formatfile, "# Linklist \"%s\" Format File\n", fname);
  fprintf(formatfile, "# Auto-generated by linklist\n");
  fprintf(formatfile, "#");
  fprintf(formatfile, "\n");
  fprintf(formatfile, LINKLIST_FILE_SERIAL_IND "%.08x\n", *((uint32_t *) ll->serial)); // format specifier
  if (flags & LL_INCLUDE_ALLFRAME) {
    fprintf(formatfile, LINKLIST_FILE_SIZE_IND "%d\n", ll->blk_size+ll->superframe->allframe_size); 
  } else {
    fprintf(formatfile, LINKLIST_FILE_SIZE_IND "%d\n", ll->blk_size); // blk_size = bulk size
  }
  fprintf(formatfile, LINKLIST_FRAMES_PER_FILE_IND "%d\n", ll_rawfile_default_fpf); // number of frames per file
  fprintf(formatfile, "#\n");
  
  fprintf(formatfile, "%s\n\n", STR(LL_NO_AUTO_CHECKSUM));

  for (i = 0; i < (int) ll->n_entries; i++) { 
    if (ll->items[i].tlm == &block_entry) { // a block
      block_t * theblock = linklist_find_block_by_pointer(ll, &ll->items[i]);
      if (theblock) {
        fprintf(formatfile, "%s    ",  theblock->name); // block name
        fprintf(formatfile, "B    "); // block indicator
        fprintf(formatfile, "%u\n",  ll->items[i].num); // block size  
      } else {
        linklist_err("Could not find block in linklist\n");
      }
    } else if (ll->items[i].tlm == &stream_entry) { // a stream
      stream_t * thestream = linklist_find_stream_by_pointer(ll, &ll->items[i]);
      if (thestream) {
        fprintf(formatfile, "%s    ",  thestream->name); // stream name
        fprintf(formatfile, "S    "); // stream indicator
        fprintf(formatfile, "%u\n",  ll->items[i].num); // stream size  
      } else {
        linklist_err("Could not find stream in linklist\n");
      }
    } else if (ll->items[i].tlm) { // not a checksum field
      fprintf(formatfile, "%s    ", ll->items[i].tlm->field); // field name
      if (ll->items[i].comp_type == NO_COMP) {
        fprintf(formatfile, "%s    ", "NONE"); // compression type
      } else {
        fprintf(formatfile, "%s    ", compRoutine[ll->items[i].comp_type].name); // compression type
      }
      fprintf(formatfile, "%u\n", ll->items[i].num); // samples per frame (spf)
    } else if ((i != 0) && (i != (int) (ll->n_entries-1))){ // don't include first or last checksum
      fprintf(formatfile, "%s\n", LL_PARSE_CHECKSUM); // checksum indicator
    }

  }
  fflush(formatfile);
  fclose(formatfile);

}

void delete_superframe(superframe_t * sf) {
  free(sf->entries);
  free(sf->hash_table);
  free(sf);
}

void delete_linklist(linklist_t * ll)
{
  int i;
  for (i = 0; i < (int) ll->num_blocks; i++) {
    free(ll->blocks[i].buffer);
  }
  for (i = 0; i < (int) ll->num_streams; i++) {
    free(ll->streams[i].buffers[0].buffer);
    free(ll->streams[i].buffers[1].buffer);
  }
  free(ll->blocks);
  free(ll->streams);
  free(ll->items);
  free(ll);
}

linklist_t ** ll_list = NULL;
int8_t linktable[65536] = {0};

int linklist_generate_lookup(linklist_t ** lll) {
  uint16_t hash;
  int i = 0;

  memset(linktable, -1, 65536*sizeof(int8_t));
  if (!lll) {
    linklist_err("Linklist array is null\n");
    return 0;
  }
  ll_list = lll;
  linklist_t * ll = ll_list[0];

  while (ll) {
    hash = *((uint16_t *) ll->serial);
    if (linktable[hash] != -1)
    {
      linklist_err("Hash colliision for linklist %d and %d\n", i, linktable[hash]);
      return -1;
    }
    linktable[hash] = i;
    ll = ll_list[++i];
  }
  return 1;
}

// returns the a pointer to the linklist with the given serial number 
linklist_t * linklist_lookup_by_serial(uint16_t serial) {
  if (!ll_list) {
    linklist_err("linklist lookup is unallocated\n");
    return NULL;
  }
  int ind = linktable[serial];
  if (ind < 0) return NULL;
  return ll_list[ind];
}

/*
 * linklist_duplicate
 *
 * This function duplicates a linklist exactly, copying all entries and blocks.
 * References to the superframe are copied as well as the serial and flags.
*/
linklist_t * linklist_duplicate(linklist_t * ll) {
  if (!ll) {
    linklist_err("Cannot duplicate NULL linklist\n");
    return NULL;
  }
  int i, j;
  linklist_t * ll_copy = (linklist_t *) calloc(1, sizeof(linklist_t)); 
  
  // copy the structure directly
  memcpy(ll_copy, ll, sizeof(linklist_t));

  // copy the entries
  ll_copy->items = (linkentry_t *) calloc(ll->n_entries, sizeof(linkentry_t));
  memcpy(ll_copy->items, ll->items, ll->n_entries*sizeof(linkentry_t));

  // copy the blocks
  ll_copy->blocks = (struct block_container *) calloc(MAX_DATA_BLOCKS, sizeof(struct block_container));
  memcpy(ll_copy->blocks, ll->blocks, MAX_DATA_BLOCKS*sizeof(struct block_container));

  // copy the streams
  ll_copy->streams = (struct stream_container *) calloc(MAX_DATA_STREAMS, sizeof(struct stream_container));
  memcpy(ll_copy->streams, ll->streams, MAX_DATA_STREAMS*sizeof(struct stream_container));

  // change the reference to the copied linklist for each linkentry
  int b_ind = 0;
  int s_ind = 0;
  for (i=0; i<(int) ll_copy->n_entries; i++) {
    ll_copy->items[i].linklist = ll_copy;
    // allocate block memory and update reference to linkentry to the copied linklist
    if (ll_copy->items[i].tlm == &block_entry) {
      ll_copy->blocks[b_ind].buffer = (uint8_t *) calloc(1, ll_copy->blocks[b_ind].alloc_size);
      ll_copy->blocks[b_ind].fp = NULL; // only one block can own the file descriptor if non-NULL
      ll_copy->blocks[b_ind].le = &ll_copy->items[i];
      b_ind++;
    } else if (ll_copy->items[i].tlm == &stream_entry) {
      for (j=0; j<2; j++) {
				ll_copy->streams[s_ind].buffers[j].buffer = 
                             (uint8_t *) calloc(1, ll_copy->streams[s_ind].buffers[j].alloc_size);
        ll_copy->streams[s_ind].buffers[j].data_size = 0;
        ll_copy->streams[s_ind].buffers[j].loc = 0;
      }
      ll_copy->streams[s_ind].curr = 0;
      ll_copy->streams[s_ind].next = 0;
			ll_copy->streams[s_ind].fp = NULL; // only one block can own the file descriptor if non-NULL
      ll_copy->streams[s_ind].le = &ll_copy->items[i];
      s_ind++;
    }
  }

  // check consistency with the number of blocks found
  if (b_ind != (int) ll_copy->num_blocks) {
    linklist_err("linklist_duplicate: num_blocks mismatch (found %d, expected %d)\n", 
                 i, ll_copy->num_blocks);
  }

  return ll_copy;
}


linklist_t * generate_superframe_linklist(superframe_t * superframe) {
  return generate_superframe_linklist_opt(superframe, 0);
}

linklist_t * generate_superframe_linklist_opt(superframe_t * superframe, int flags)
{
  if (superframe== NULL) {
    linklist_err("Supeframe is NULL\n");
    return NULL;
  }
  superframe_entry_t * ll_superframe_list = superframe->entries;

  int i;
  unsigned int byteloc = 0;
  unsigned int blk_size = 0;
  unsigned int chksm_count = 0;
  unsigned int comp_type = 0;

  unsigned int extra_chksm = !(flags & LL_NO_AUTO_CHECKSUM) ? superframe->size/MIN_CHKSM_SPACING : 0;
  extra_chksm += 5;

  // MD5 hash
  MD5_CTX mdContext;
  uint8_t md5hash[MD5_DIGEST_LENGTH] = {0};
  MD5_Init(&mdContext); // initialize hash

  linklist_t * ll = (linklist_t *) calloc(1, sizeof(linklist_t));
  ll->items = (linkentry_t *) calloc(superframe->n_entries+extra_chksm, sizeof(linkentry_t));
  ll->flags = flags;
  ll->superframe = superframe;
  ll->blocks = (struct block_container *) calloc(MAX_DATA_BLOCKS,sizeof(struct block_container));
  ll->streams = (struct stream_container *) calloc(MAX_DATA_STREAMS,sizeof(struct stream_container));
  ll->n_entries = 0;

  // add initial checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]), byteloc);
  update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  // add all telemetry entries
  for (i = 0; i < (int) superframe->n_entries; i++) {
    if ((chksm_count >= MIN_CHKSM_SPACING) && !(flags & LL_NO_AUTO_CHECKSUM)) {
      blk_size = set_checksum_field(&(ll->items[ll->n_entries]), byteloc);
      update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
      byteloc += blk_size;
      ll->n_entries++;
      chksm_count = 0;
    }

    comp_type = linklist_get_default_compression(&ll_superframe_list[i], flags);

    ll->items[ll->n_entries].start = byteloc;
    ll->items[ll->n_entries].num = ll_superframe_list[i].spf;
    ll->items[ll->n_entries].comp_type = comp_type;

    superframe_entry_t * chan = NULL;
    if (ll_superframe_list[i].type == SF_NUM) {
      chan = &block_entry;
      parse_block(ll, ll_superframe_list[i].field);
    } else if (ll_superframe_list[i].type == SF_INF) {
      chan = &stream_entry;
      parse_stream(ll, ll_superframe_list[i].field);
    } else {
      chan = &ll_superframe_list[i];
    }
    ll->items[ll->n_entries].tlm = chan;
    ll->items[ll->n_entries].linklist = ll;

    if (comp_type != NO_COMP) { // normal compressed field
      blk_size = (*compRoutine[comp_type].compressFunc)(NULL, &ll->items[ll->n_entries], NULL);
    } else { // no compression, so identical field supeframe entry 
      blk_size = get_superframe_entry_size(&ll_superframe_list[i])*ll_superframe_list[i].spf; 
    }
    ll->items[ll->n_entries].blk_size = blk_size;

    update_superframe_entry_hash(&mdContext, chan);
    update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
    byteloc += blk_size;
    ll->n_entries++;
    chksm_count += blk_size;
  }

  // final checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]), byteloc);
  update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  ll->blk_size = byteloc;
  strcpy(ll->name, ALL_TELEMETRY_NAME);

  MD5_Update(&mdContext, &byteloc, sizeof(byteloc));
  MD5_Update(&mdContext, &ll->n_entries, sizeof(ll->n_entries));
  //MD5_Update(&mdContext, ll->name, (strlen(ll->name)/4)*4);

  // generate serial
  MD5_Final(md5hash, &mdContext);
  memcpy(ll->serial, md5hash, MD5_DIGEST_LENGTH);
 
  return ll;
}

uint64_t generate_superframe_serial(superframe_t * superframe) 
{
  superframe_entry_t * sf = superframe->entries;

  // MD5 hash variables
  MD5_CTX mdContext;
  uint8_t md5hash[MD5_DIGEST_LENGTH];
  MD5_Init(&mdContext); // initialize MD5 hash

  int i = 0;
  for (i = 0; sf[i].field[0]; i++) {
    MD5_Update(&mdContext, sf[i].field, strlen(sf[i].field));
    MD5_Update(&mdContext, &sf[i].type, sizeof(sf[i].type));
    MD5_Update(&mdContext, &sf[i].spf, sizeof(sf[i].spf));
    MD5_Update(&mdContext, &sf[i].start, sizeof(sf[i].start));
    MD5_Update(&mdContext, &sf[i].skip, sizeof(sf[i].skip));
    //if (strlen(sf[i].quantity)) MD5_Update(&mdContext, sf[i].quantity, strlen(sf[i].quantity));
    //if (strlen(sf[i].units)) MD5_Update(&mdContext, sf[i].units, strlen(sf[i].units));
  }

  // generate MD5 hash of command_list
  MD5_Final(md5hash,&mdContext);

  return *(uint64_t *) md5hash;
}

// get the number of frames per file (fpf) from the linklist formatfile
int read_linklist_formatfile_comment(char * fin, char * comment, const char * format) {
  FILE * f;

  if ((f = fopen(fin, "r")) == NULL) {
    return -2;
  }

  char * line = NULL;
  size_t len = 0; 
  int read = 0;
  int retval = -1;

  while ((read = getline(&line, &len, f)) != -1) {
    if (strncmp(line, comment, strlen(comment)) == 0) {
      fclose(f);
      sscanf(line+strlen(comment), format, &retval);
      return retval;
    }
  }
  fclose(f);

  return retval;
}

superframe_t * parse_superframe_format(char * fname) {
  // big endian is the default if the superframe format does not specify
  return parse_superframe_format_opt(fname, SF_USE_BIG_ENDIAN);
}

superframe_t * parse_superframe_format_opt(char * fname, int flags) {
  FILE * cf = fopen(fname, "r"); // open command file
  if (cf == NULL) {
    linklist_err("Cannot find %s\n",fname);
    return NULL;
  }

  char *line = NULL;
  size_t len = 0;
  int st, read;
  int i, j, r;
  char temp[100];
  char *temps[20];
  uint8_t * frame = NULL;
  int start, skip;
  int count = 1;
  uint64_t serial = 0;

  // flags
  uint8_t begin_f = 0, size_f = 0, serial_f = 0;

  int def_n_entries = 150;
  unsigned int blksize = 0;
  unsigned int n_entries = 0;

  superframe_entry_t * sf = (superframe_entry_t *) calloc(def_n_entries, sizeof(superframe_entry_t));

  while ((read = getline(&line, &len, cf)) != -1) {
     // remove the newline
    line[read-1] = '\0';
    read--;

    // remove end whitespace
    for (r=(read-1);r>=0;r--) {
      if ((line[r] != ' ') && (line[r] != '\t')) break;
      else line[r] = '\0';
    }
    read = r+1;

    // remove beginning whitespace
    st = 0;
    while ((line[st] == '\t') || (line[st] == ' ')) st++;

    if ((line[st] != COMM) && (line[st] != '\n') && ((read-st) > 0)) { // skip comments and blank lines
      strcpy(temp, line+st);

      if (!begin_f) { // ignore everything up to the BEGIN marker
        if (strncmp(temp,"BEGIN",5) == 0) begin_f = 1; // found beginning
      } else if (!serial_f) { // next is the serial number
        sscanf(temp, "%" PRIx64, &serial);
        serial_f = 1;
      } else if (!size_f) { // next is the frame size
        blksize = atoi(temp);
        size_f = 1;
        frame = (uint8_t *) calloc(blksize, sizeof(uint8_t));
      } else { // everything else is a telemetry entry
        parse_line(temp, temps, 8);        

        if (strncmp(temps[0],"END",3) == 0) {
          break;
        } else if (strncmp(temps[0], SUPERFRAME_ENDIAN_IND, strlen(SUPERFRAME_ENDIAN_IND)) == 0) {
          if (strncmp(temps[1], SUPERFRAME_ENDIAN_LITTLE, strlen(SUPERFRAME_ENDIAN_LITTLE)) == 0) {
            flags &= ~SF_USE_BIG_ENDIAN;
          } else if (strncmp(temps[1], SUPERFRAME_ENDIAN_BIG, strlen(SUPERFRAME_ENDIAN_BIG)) == 0) {
            flags |= SF_USE_BIG_ENDIAN;
          }
          continue;
        }
        if ((int) n_entries >= (def_n_entries-1)) {
          def_n_entries += 5;
          sf = (superframe_entry_t *) realloc(sf, def_n_entries*sizeof(superframe_entry_t));
        }

        memset(sf[n_entries].field, 0, SF_FIELD_LEN);
        strcpy(sf[n_entries].field, temps[0]); // name
        sf[n_entries].type = get_sf_type_int(temps[1]); // type int
        sf[n_entries].spf = atoi(temps[2]); // samples per frame

        unsigned int type_str_len = strlen(get_sf_type_string(sf[n_entries].type));
        if (strlen(temps[1]) > type_str_len) { // trailing characters are assumed to be min and max of format (min,max) 
          sscanf(temps[1]+type_str_len, "(%lf,%lf)%*s", &sf[n_entries].min, &sf[n_entries].max);
          //linklist_info("%s min=%f max=%f\n", sf[n_entries].field, sf[n_entries].min, sf[n_entries].max);
        }

        // determine start byte and skip
        start = atoi(temps[3]);
        skip = atoi(temps[4]);
        sf[n_entries].start = start; // start byte
        sf[n_entries].skip = skip; // skip
        memset(sf[n_entries].quantity, 0, SF_UNITS_LEN);
        memset(sf[n_entries].units, 0, SF_UNITS_LEN);
        if (strlen(temps[5])) strcpy(sf[n_entries].quantity, temps[5]);
        if (strlen(temps[6])) strcpy(sf[n_entries].units, temps[6]);
        sf[n_entries].var = NULL;

        // populate byte map
        for (i = 0; i < (int) sf[n_entries].spf; i++) {
          for (j = 0; j < (int) get_superframe_entry_size(&sf[n_entries]); j++) {
            if (frame[skip*i+start+j] == 1) {
              linklist_warn("Warning (line %d): byte %d overlap\n", count, skip*i+start+j);
            }
            frame[skip*i+start+j] = 1;
          }
        }
        n_entries++;
      }
    }
    count++;
  }
  fclose(cf);

  sf[n_entries].field[0] = '\0';

  superframe_t * superframe = NULL;
  if (flags & SF_USE_BIG_ENDIAN) {
    superframe = linklist_build_superframe(sf, &def_datatodouble_be, &def_doubletodata_be, flags);
  } else {
    superframe = linklist_build_superframe(sf, NULL, NULL, flags);
  }

  if (superframe->serial != serial) {
    linklist_err("Parsed serial 0x%" PRIx64 " does not match file serial 0x%" PRIx64 "\n", superframe->serial, serial);
  }
  if (superframe->size != blksize) {
    linklist_err("Parsed size %d does not match file size %d\n", superframe->size, blksize);
  }

  superframe->flags = flags;
  if (frame) free(frame);

  return superframe;
}

void write_superframe_format(superframe_t * superframe, const char * fname) {
  if (!superframe || !superframe->entries) return;
  superframe_entry_t * sf = superframe->entries;

  FILE * fp = fopen(fname, "w");
  if (!fp) {
    linklist_err("Unable to write format at \"%s\"\n", fname);
  }
  int i = 0;
  
  fprintf(fp, "BEGIN\n");
  fprintf(fp, "%" PRIx64 "\n", superframe->serial); 
  fprintf(fp, "%d\n", superframe->size); 
  if (!(superframe->flags & SF_USE_BIG_ENDIAN)) {
    fprintf(fp, "%s %s\n", SUPERFRAME_ENDIAN_IND, SUPERFRAME_ENDIAN_LITTLE); 
  }

  char type_string[128] = "";

  for (i = 0; sf[i].field[0]; i++) {
    if ((sf[i].max != 0) || (sf[i].min != 0)) sprintf(type_string, "%s(%lf,%lf)", get_sf_type_string(sf[i].type), sf[i].min, sf[i].max);
    else sprintf(type_string, "%s", get_sf_type_string(sf[i].type));
    fprintf(fp, "%s  %s  %u  %u  %u", sf[i].field, type_string, sf[i].spf, sf[i].start, sf[i].skip);
    if (strlen(sf[i].quantity)) fprintf(fp, "  \"%s\"  \"%s\"", sf[i].quantity, sf[i].units);
    fprintf(fp, "\n");
  }

  fprintf(fp, "END\n");

  fflush(fp);
  fclose(fp);
}

#ifdef __cplusplus
}

#endif
