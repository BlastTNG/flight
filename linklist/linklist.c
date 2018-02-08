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

#include "blast.h"
#include "channels_tng.h"
#include "linklist.h"
#include "linklist_crc.h"
#include "linklist_compress.h"

#define COMM '#'
#define SPEC ':'
#define MULT '|'
#define DESC '"'

int tlm_no_checksum = 0; // flag to not use checksums in decompression routines
int no_auto_min_checksum = 0; // flag to not auto add checksums in compression routines
int num_compression_routines = 0; // number of compression routines available

// TODO: this can be replaced by channel_size in "channels_tng.c", but it currently isn't publically available...
unsigned int get_channel_size(const channel_t * chan)
{
  if (!chan) blast_fatal("%s is NULL! Fix!",chan->field);
  switch (chan->type)
  {
    case TYPE_UINT8:
    case TYPE_INT8:
      return sizeof(int8_t);
    case TYPE_INT16:
    case TYPE_UINT16:
      return sizeof(int16_t);
    case TYPE_INT32:
    case TYPE_UINT32:
    case TYPE_FLOAT:
      return sizeof(int32_t);
    case TYPE_INT64:
    case TYPE_UINT64:
    case TYPE_DOUBLE:
      return sizeof(int64_t);
    default:
      blast_err("Invalid type %d for channel %s", chan->type, chan->field);
      return 0;
  }
}

unsigned int get_spf(unsigned int rate)
{
  switch (rate)
  {
    case RATE_1HZ:
      return 1;
    case RATE_5HZ:
      return 5;
    case RATE_100HZ:
      return 100;
    case RATE_200HZ:
      return 200;
    case RATE_244HZ:
      return 244;
    case RATE_488HZ:
      return 488;
    default:
      return 0;
  }
}

unsigned int get_channel_spf(const channel_t * chan)
{
  if (!chan) blast_fatal("%s is NULL! Fix!",chan->field);
  unsigned int rate = get_spf(chan->rate);
  if (!rate) blast_err("Invalid rate %d for channel %s", chan->rate, chan->field);
  return rate;
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

void update_channel_hash(MD5_CTX *mdContext, channel_t * chan)
{
  MD5_Update(mdContext, chan->field, FIELD_LEN);
	MD5_Update(mdContext, &chan->type, sizeof(chan->type));
	MD5_Update(mdContext, &chan->rate, sizeof(chan->rate));
}

/**
 * parse_linklist
 * 
 * Returns a pointer to a linklist parsed from file.
 * -> fname: path to linklist to be parsed
 */

struct link_list * parse_linklist(char *fname)
{
  // allocate crc table if necessary
  if (crctable == NULL)
  {
    if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
    {
      blast_fatal("mk_crctable() memory allocation failed\n");
    }
  } 

  // count the number of compression routines available
  if (num_compression_routines == 0)
  {
    int c = 0;
    int d = 0;
    while (compressFunc[c]) c++;
    while (decompressFunc[d]) d++;
    num_compression_routines = MIN(c,d);
  }

  // initialize the superframe structure if necessary
  if (!mainframe.data)
  {
    allocate_superframe(channel_list);
  }

  FILE * cf = fopen(fname,"r"); 
  if (cf == NULL)
  {
    blast_err("parse_linklist: cannot find %s\n",fname);
    return NULL; 
  }

  if (channel_list == NULL)
  {
    blast_err("parse_linklist: no channel_list is loaded\n");
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
        //MD5_Update(&mdContext, &ll->items[ll->n_entries], sizeof(struct link_entry)-sizeof(struct telem_entry *));
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
        channel_t * chan = channels_find_by_name(temps[0]);
        if (!chan)
        {
          blast_err("parse_linklist: unable to find telemetry entry %s\n",temps[0]);
          continue;
        }
        update_channel_hash(&mdContext, chan);

        ll->items[ll->n_entries].tlm = chan; // connect entry to name

        // read rest of the file to get compression type
        comp_type = atoi(temps[1]); // get compression type
        num = atoi(temps[2]); // get compressed samples per frame 

        // check that comp_type is within range
        if (comp_type >= num_compression_routines)
        {
          blast_err("Invalid comp. type %d for \"%s\". Defaulting to uncompressed.",comp_type,chan->field);
          comp_type = NO_COMP;
        }

        // fill the link entry structure
        ll->items[ll->n_entries].comp_type = comp_type;
        ll->items[ll->n_entries].start = byteloc;
        ll->items[ll->n_entries].num = num;

/*  TODO: blocks
        if (ll->items[ll->n_entries].tlm->type == 'B') // data block field
        {
          if (ll->num_blocks < MAX_DATA_BLOCKS)
          {
						parse_block(ll);
            blk_size = num; // blocks have size per sample of 1 byte
          }
          else
          {
            blast_err("parse_linklist: max number of data blocks (%d) reached\n",MAX_DATA_BLOCKS);
          }

        }

        else 
*/
        if (comp_type != NO_COMP) // normal compressed field
        {
          blk_size = (*compressFunc[comp_type])(NULL,&ll->items[ll->n_entries],NULL);
        }
        else // no compression, so identical field to telemlist, but with decimation
        {
          blk_size = num*get_channel_size(ll->items[ll->n_entries].tlm);
        }
        if (blk_size > 0) ll->items[ll->n_entries].blk_size = blk_size;
        else
        {
          blast_err("parse_linklist: zero compressed size for %s in %s\n",ll->items[ll->n_entries].tlm->field,fname);
        }
      }

      // update the serial
      //MD5_Update(&mdContext, &ll->items[ll->n_entries], sizeof(struct link_entry)-sizeof(struct telem_entry *));
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
  
  // update the hash
  MD5_Update(&mdContext, &byteloc, sizeof(byteloc));
  MD5_Update(&mdContext, &ll->n_entries, sizeof(ll->n_entries));

  // generate serial
  MD5_Final(md5hash,&mdContext);
  memcpy(ll->serial,md5hash,MD5_DIGEST_LENGTH);

	
  // print result
  for (i=0;i<ll->n_entries;i++)
  {
    if (ll->items[i].tlm != NULL)
    {
      printf("name = %s, start = %d, blk_size = %d, num = %d, comp_type = %d\n",
         ll->items[i].tlm->field, ll->items[i].start, ll->items[i].blk_size, ll->items[i].num,
         ll->items[i].comp_type);
    }
    else printf("CHECKSUM\n");
  }

  printf("Serial: ");
  for (i=0;i<MD5_DIGEST_LENGTH;i++) printf("%x",ll->serial[i]);
  printf("\n");
  printf("n_entries = %d, blk_size = %d\n",ll->n_entries,ll->blk_size);
	

  return ll;
}

void delete_linklist(struct link_list * ll)
{
  int i;
  for (i=0;i<ll->n_entries;i++) free(&ll->items[i]);
  free(ll);
}


#ifdef _TESTING

int main(int argc, char *argv[])
{
  channels_initialize(channel_list);
  linklist_t * test_ll = parse_linklist("test.ll");

/*
  channel_t * chan;

  for (chan = channel_list; chan->field[0]; chan++)
  {
    if (chan->rate == RATE_1HZ) printf("%s %d %d\n",chan->field,get_channel_start_in_superframe(chan),get_channel_skip_in_superframe(chan));
  }
*/
}

#endif
