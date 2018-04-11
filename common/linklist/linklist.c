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
#include <mosquitto.h>

#include "CRC_func.h"
#include "linklist.h"
#include "linklist_compress.h"

#define COMM '#'
#define SPEC ':'
#define MULT '|'
#define DESC '"'

#ifdef __cplusplus

extern "C" {

#endif

int tlm_no_checksum = 0; // flag to not use checksums in decompression routines
int no_auto_min_checksum = 0; // flag to not auto add checksums in compression routines
int num_compression_routines = 0; // number of compression routines available
superframe_entry_t block_entry = {{0}}; // a dummy entry for blocks

superframe_entry_t * ll_superframe_list = NULL;
unsigned int superframe_entry_count = 0;
superframe_entry_t ** superframe_hash_table = NULL;
unsigned int superframe_hash_table_size = 0;
unsigned int superframe_size = 0; 

double (*datatodouble)(uint8_t *, uint8_t) = NULL;
int (*doubletodata)(uint8_t *, double, uint8_t) = NULL;

void linklist_assign_datatodouble(double (*func)(uint8_t *, uint8_t)) {
  datatodouble = func;
}
void linklist_assign_doubletodata(int (*func)(uint8_t *, double, uint8_t)) {
  doubletodata = func;
}

//Implements the djb2 hashing algorithm for char*s 
unsigned int hash(const char* input){
  int returnVal = 5381;
  int c;

  while ((c = *input++))
    returnVal = ((returnVal << 5) + returnVal) + c;

  return returnVal;
}


void linklist_assign_superframe_list(superframe_entry_t* m_superframe_list) {
  ll_superframe_list = m_superframe_list;

  int i = 0;
  superframe_size = 0;
  superframe_entry_count = 0;
  for (i = 0; m_superframe_list[i].field[0]; i++) {
    superframe_size += get_superframe_entry_size(&m_superframe_list[i])*m_superframe_list[i].spf;
    superframe_entry_count++;
  }

  if (superframe_hash_table) free(superframe_hash_table);
  superframe_hash_table_size = superframe_entry_count*1000;
  superframe_hash_table = calloc(superframe_hash_table_size, sizeof(superframe_entry_t *));
  
  for (i = 0; i < superframe_entry_count; i++) {
    unsigned int hashloc = hash(m_superframe_list[i].field)%superframe_hash_table_size;
    if (superframe_hash_table[hashloc]) {
      linklist_err("Hash with entry \"%s\"", m_superframe_list[i].field);
    } else {
      superframe_hash_table[hashloc] = &m_superframe_list[i];
    }
  }
}

superframe_entry_t * superframe_find_by_name(char * name) {
  if (!name) return NULL;

  return superframe_hash_table[hash(name)%superframe_hash_table_size];
}

void realloc_list(struct link_list * ll, int * x)
{
  *x += 100;
  ll->items = (struct link_entry *) realloc(ll->items,(*x)*(sizeof(struct link_entry)));
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

static int one (const struct dirent *unused)
{
  return 1;
}

int load_all_linklists(char * linklistdir, linklist_t ** ll_array) {
  struct dirent **dir;
  int n = scandir(linklistdir,&dir,one,alphasort);
  int i;
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

  char full_path_name[128] = {0};

  // load linklist names
  for (i = 0; i < n; i++) {
    int len = strlen(dir[i]->d_name);
    if ((len >= 3) && strcmp(&dir[i]->d_name[len-3], ".ll") == 0) {
      sprintf(full_path_name, "%s%s",linklistdir, dir[i]->d_name);
      
      if ((ll_array[num] = parse_linklist(full_path_name)) == NULL) {
        linklist_fatal("Unable to load linklist at %s", full_path_name);
      }
      num++;
    }
  }
  ll_array[num] = linklist_all_telemetry(); // last linklist contains all the telemetry items
  ll_array[num+1] = NULL; // null terminate the list

  linklist_info("Total of %d linklists loaded from \"%s\"", num, linklistdir);

  return 1;
}

linklist_t * linklist_find_by_name(char * name, linklist_t ** ll_array)
{
  linklist_t * ll = ll_array[0];
  int i = 0;

  while (ll) {
    if (strcmp(ll->name, name) == 0) return ll;
    ll = ll_array[++i];
  }
  linklist_err("Linklist \"%s\" not found.\n", name);

  return NULL;
}

int set_checksum_field(struct link_entry * le, unsigned int byteloc)
{
  int blk_size = 2; // checksums are 16 bit

  le->tlm = NULL; // no tlm
  le->comp_type = 255; // arbitrary
  le->start = byteloc;
  le->num = 1; // only one 
  le->blk_size = blk_size;

  return blk_size;
}

void update_linklist_hash(MD5_CTX *mdContext, struct link_entry * le)
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
	MD5_Update(mdContext, &chan->spf, sizeof(chan->start));
	MD5_Update(mdContext, &chan->skip, sizeof(chan->skip));
}

void parse_block(linklist_t * ll, char * name)
{
  strcpy(ll->blocks[ll->num_blocks].name, name);
  ll->blocks[ll->num_blocks].id = 0; // use id as block send count
  ll->blocks[ll->num_blocks].buffer = (uint8_t *) calloc(1, DEF_BLOCK_ALLOC);
  ll->blocks[ll->num_blocks].alloc_size = DEF_BLOCK_ALLOC;
  ll->blocks[ll->num_blocks].le = &ll->items[ll->n_entries];
  ll->blocks[ll->num_blocks].i = 1; // inits
  ll->blocks[ll->num_blocks].n = 1; // inits
  ll->blocks[ll->num_blocks].num = 0; // inits

  ll->blocks[ll->num_blocks].filename[0] = 0; // inits
  ll->blocks[ll->num_blocks].fp = NULL; // inits

  ll->num_blocks++;
}

block_t * linklist_find_block_by_pointer(linklist_t * ll, linkentry_t * le)
{
  int i;
  for (i = 0; i < ll->num_blocks; i++) {
    if (ll->blocks[i].le == le) return &ll->blocks[i];
  }
  return NULL;
}

uint32_t get_superframe_entry_size(superframe_entry_t * chan) {
	size_t retsize = 0;
	switch (chan->type) {
		case SF_INT8:
		case SF_UINT8:
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
			linklist_fatal("Invalid Channel size!");
	}
	return retsize;
}

/**
 * parse_linklist
 * 
 * Returns a pointer to a linklist parsed from file.
 * -> fname: path to linklist to be parsed
 */

linklist_t * parse_linklist(char *fname)
{
  // allocate crc table if necessary
  if (crctable == NULL)
  {
    if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
    {
      linklist_fatal("mk_crctable() memory allocation failed\n");
    }
  } 

  // count the number of compression routines available
  if (num_compression_routines == 0)
  {
    int c = 0;
    while (strlen(compRoutine[c].name)) c++;
    num_compression_routines = c;
  }

  FILE * cf = fopen(fname,"r"); 
  if (cf == NULL)
  {
    linklist_err("parse_linklist: cannot find %s\n",fname);
    return NULL; 
  }

  if (ll_superframe_list == NULL)
  {
    linklist_err("parse_linklist: no ll_superframe_list is loaded\n");
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
  uint8_t isblock = 0;

  char *temps[20];
  int def_n_entries = 150;

  struct link_list * ll = (struct link_list *) calloc(1,sizeof(struct link_list));
  ll->items = (struct link_entry *) calloc(def_n_entries,sizeof(struct link_entry));
  ll->blocks = (struct block_container *) calloc(MAX_DATA_BLOCKS,sizeof(struct block_container));
  ll->num_blocks = 0;

  // MD5 hash
  MD5_CTX mdContext;
  uint8_t md5hash[MD5_DIGEST_LENGTH] = {0};
  MD5_Init(&mdContext); // initialize hash

  // initial field for first checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
  //MD5_Update(&mdContext, &ll->items[ll->n_entries], sizeof(struct link_entry)-sizeof(struct telem_entry *));
  update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  while ((read = getline(&line, &len, cf)) != -1)
  {
    line[read-1] = 0;
    read--;
    for (i=(read-1);i>=0;i--)
    {
      if ((line[i] != ' ') && (line[i] != '\t')) break;
      else line[i] = 0;
    }
    read = i+1;
    
    if (ll->n_entries >= def_n_entries) 
    {
      realloc_list(ll,&def_n_entries);
    }

    // remove whitespace
    st = 0;
    while ((line[st] == '\t') || (line[st] == ' ')) st++;
    if ((line[st] != COMM) && (line[st] != '\n') && ((read-st) > 0)) // skip comments and blank lines
    {
      // add min checksum if necessary
      if ((chksm_count >= MIN_CHKSM_SPACING) && !no_auto_min_checksum)
      {
        blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
        update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
        byteloc += blk_size;
        ll->n_entries++;
        chksm_count = 0;
      } 

      parse_line(line+st,temps,6);
      memset(&ll->items[ll->n_entries],0,sizeof(struct link_entry));

      // check for checksum field
      if (strcmp(temps[0],LL_PARSE_CHECKSUM) == 0) 
      { 
        blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
        chksm_count = 0;
      }
      else
      {
        superframe_entry_t * chan = NULL;
        if (strcmp(temps[1], "B") == 0) // blocks field 
        {
          chan = &block_entry; // dummy entry associated with block
          comp_type = NO_COMP;
          isblock = 1;
        } 
        else 
        {
          chan = superframe_find_by_name(temps[0]);
          isblock = 0;
          if ((strcmp(temps[1], "NONE") == 0) || (strlen(temps[1]) == 0))
          {
            comp_type = NO_COMP;
          }
          else if (strspn(temps[1], "0123456789") == strlen(temps[1])) // normal field, number
          {
            comp_type = atoi(temps[1]); // get compression type
          }
          else // normal field, string
          {
            for (i=0; i<NUM_COMPRESS_TYPES; i++)
            {
              if (strcmp(temps[1], compRoutine[i].name) == 0) break;
            }
            if (i != NUM_COMPRESS_TYPES)
            {
              comp_type = i;
            }
            else
            {
              linklist_err("Could not find compression type \"%s\"", temps[1]);
              comp_type = NO_COMP;
            }
          }
        }


        if (!chan)
        {
          linklist_err("parse_linklist: unable to find telemetry entry %s\n",temps[0]);
          continue;
        }

        // get compressed samples per frame
        if ((strcmp(temps[2], "NONE") == 0) || (strlen(temps[2]) == 0))
        {
          num = chan->spf;
        }
        else
        {
          num = atoi(temps[2]); // get compressed samples per frame 
        }

        update_superframe_entry_hash(&mdContext, chan);

        ll->items[ll->n_entries].tlm = chan; // connect entry to name

        // check that comp_type is within range
        if ((comp_type >= num_compression_routines) && (comp_type != NO_COMP))
        {
          linklist_err("Invalid comp. type %d for \"%s\". Defaulting to uncompressed.",comp_type,chan->field);
          comp_type = NO_COMP;
        }

        // fill the link entry structure
        ll->items[ll->n_entries].comp_type = comp_type;
        ll->items[ll->n_entries].start = byteloc;
        ll->items[ll->n_entries].num = num;

        if (isblock) // data block field
        {
          if (ll->num_blocks < MAX_DATA_BLOCKS)
          {
            parse_block(ll, temps[0]);
            blk_size = num; // blocks have size per sample of 1 byte
          }
          else
          {
            linklist_err("parse_linklist: max number of data blocks (%d) reached\n",MAX_DATA_BLOCKS);
          }
        }
        else if (comp_type != NO_COMP) // normal compressed field
        {
          blk_size = (*compRoutine[comp_type].compressFunc)(NULL,&ll->items[ll->n_entries],NULL);
        }
        else // no compression, so identical field to telemlist, but with decimation
        {
          blk_size = num*get_superframe_entry_size(ll->items[ll->n_entries].tlm);
        }

        if (blk_size > 0) ll->items[ll->n_entries].blk_size = blk_size;
        else
        {
          linklist_err("parse_linklist: zero compressed size for %s in %s\n",ll->items[ll->n_entries].tlm->field,fname);
        }
      }

      // update the serial
      update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
      byteloc += blk_size;
      chksm_count += blk_size;
      ll->n_entries++;

    }

  }

  // check memory allocation
  if (ll->n_entries >= def_n_entries) 
  {
    realloc_list(ll,&def_n_entries);
  }

  // add one last field for final checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]),byteloc);
  //MD5_Update(&mdContext, &ll->items[ll->n_entries], sizeof(struct link_entry)-sizeof(struct telem_entry *));
  update_linklist_hash(&mdContext,&ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  ll->blk_size = byteloc;

  // add the linklist name
  for (i = strlen(fname)-1; i > 0; i--) {
    if (fname[i] == '/') { // got the filename
      i++;
      break;
    }
  }
  memset(ll->name, 0, 64); // clear name completely
  strcpy(ll->name, fname+i); // copy the name

  // update the hash
  MD5_Update(&mdContext, &byteloc, sizeof(byteloc));
  MD5_Update(&mdContext, &ll->n_entries, sizeof(ll->n_entries));
  MD5_Update(&mdContext, ll->name, strlen(ll->name));

  // generate serial
  MD5_Final(md5hash,&mdContext);
  memcpy(ll->serial,md5hash,MD5_DIGEST_LENGTH);

  return ll;
}

void linklist_to_file(linklist_t * ll, char * fname)
{
  int i;
  FILE * formatfile = fopen(fname, "w");

  if (formatfile == NULL) {
    linklist_err("Unable to generate linklist file \"%s\"", fname);
    return;
  }

  fprintf(formatfile, "#\n");
  fprintf(formatfile, "# Linklist \"%s\" Format File\n", fname);
  fprintf(formatfile, "# Auto-generated by linklist\n");
  fprintf(formatfile, "#");
  fprintf(formatfile, "\n");
  fprintf(formatfile, "# Serial=%.04x\n", *((uint16_t *) ll->serial)); // format specifier
  fprintf(formatfile, "# Blk Size=%d\n", ll->blk_size); // blk_size = bulk size
  fprintf(formatfile, "#\n");

  for (i = 0; i < ll->n_entries; i++) { 
    if (ll->items[i].tlm == &block_entry) { // a block
      block_t * theblock = linklist_find_block_by_pointer(ll, &ll->items[i]);
      if (theblock) {
        fprintf(formatfile, "%s    ",  theblock->name); // block name
        fprintf(formatfile, "B    "); // block indicator
        fprintf(formatfile, "%u\n",  ll->items[i].num); // block size  
      } else {
        linklist_err("Could not find block in linklist");
      }
    } else if (ll->items[i].tlm) { // not a checksum field
      fprintf(formatfile, "%s    ", ll->items[i].tlm->field); // field name
      if (ll->items[i].comp_type == NO_COMP) {
        fprintf(formatfile, "%u    ", ll->items[i].comp_type); // compression type
      } else {
        fprintf(formatfile, "%s    ", compRoutine[ll->items[i].comp_type].name); // compression type
      }
      fprintf(formatfile, "%u\n", ll->items[i].num); // samples per frame (spf)
    } else { // don't include first or last checksum
      fprintf(formatfile, "%s\n", LL_PARSE_CHECKSUM); // checksum indicator
    }

  }
  fflush(formatfile);
  fclose(formatfile);

}

void delete_linklist(struct link_list * ll)
{
  int i;
  for (i=0;i<ll->n_entries;i++) free(&ll->items[i]);
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
linklist_t * linklist_lookup_by_serial(uint32_t serial) {
  if (!ll_list) {
    linklist_err("linklist lookup is unallocated\n");
    return NULL;
  }
  int ind = linktable[*((uint16_t *) &serial)];
  if (ind < 0) return NULL;
  return ll_list[ind];
}

linklist_t * linklist_all_telemetry()
{
  if (ll_superframe_list == NULL) {
    linklist_err("No supeframe list loaded");
    return NULL;
  }

  int i;
  unsigned int byteloc = 0;
  unsigned int blk_size = 0;
  unsigned int chksm_count = 0;

  unsigned int extra_chksm = (!no_auto_min_checksum) ? superframe_size/MIN_CHKSM_SPACING : 0;
  extra_chksm += 5;

  // MD5 hash
  MD5_CTX mdContext;
  uint8_t md5hash[MD5_DIGEST_LENGTH] = {0};
  MD5_Init(&mdContext); // initialize hash

  linklist_t * ll = (linklist_t *) calloc(1, sizeof(linklist_t));
  ll->items = (linkentry_t *) calloc(superframe_entry_count+extra_chksm, sizeof(linkentry_t));
  ll->blocks = NULL;
  ll->n_entries = 0;

  // add initial checksum
  blk_size = set_checksum_field(&(ll->items[ll->n_entries]), byteloc);
  update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
  byteloc += blk_size;
  ll->n_entries++;

  // add all telemetry entries
  for (i = 0; i < superframe_entry_count; i++) {
    if ((chksm_count >= MIN_CHKSM_SPACING) && !no_auto_min_checksum) {
      blk_size = set_checksum_field(&(ll->items[ll->n_entries]), byteloc);
      update_linklist_hash(&mdContext, &ll->items[ll->n_entries]);
      byteloc += blk_size;
      ll->n_entries++;
      chksm_count = 0;
    }

    blk_size = get_superframe_entry_size(&ll_superframe_list[i])*ll_superframe_list[i].spf; 

    ll->items[ll->n_entries].comp_type = NO_COMP; // uncompressed
    ll->items[ll->n_entries].num = ll_superframe_list[i].spf;
    ll->items[ll->n_entries].tlm = &ll_superframe_list[i];
    ll->items[ll->n_entries].blk_size = blk_size;
    ll->items[ll->n_entries].start = byteloc;

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
#ifdef __cplusplus
}

#endif
