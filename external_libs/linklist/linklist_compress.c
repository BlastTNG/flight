/* 
 * linklist_compress.c: 
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
#include <errno.h>
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>

#include "linklist.h"
#include "linklist_compress.h"

#ifdef __cplusplus

extern "C"{

#endif

int decimationCompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);
int decimationDecompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);

extern superframe_entry_t block_entry;

#define LL_CRC_POLY 0x1021
uint16_t *ll_crctable = NULL;

// generates the block header in the buffer
int make_block_header(uint8_t * buffer, uint16_t id, uint16_t size, uint16_t i, uint16_t n, uint32_t totalsize)
{
  /* block header (12 bytes)
   * ----------------
   * [0-1] = identifier
   * [2-3] = packet size [bytes]
   * [4-5] = packet number
   * [6-7] = total number of packets
   * [8-12] = total block size [bytes]
   */
  *((uint16_t *) (buffer+0)) = id; // ID
  memcpy(buffer+2,&size,2); // packet size
  memcpy(buffer+4,&i,2); // packet number
  memcpy(buffer+6,&n,2); // number of packets
  memcpy(buffer+8,&totalsize,4); // total size

  return PACKET_HEADER_SIZE;
}

// generates the block header in the buffer
int read_block_header(uint8_t * buffer, uint16_t *id, uint16_t *size, uint16_t *i, uint16_t *n, uint32_t *totalsize)
{
  *id = *(uint16_t *) (buffer+0);
  *size = *(uint16_t *) (buffer+2);
  *i = *(uint16_t *) (buffer+4);
  *n = *(uint16_t *) (buffer+6);
  *totalsize = *(uint32_t *) (buffer+8);

  return PACKET_HEADER_SIZE;
}

// generates and returns a CRC table for linlist checksum validation 
uint16_t *mk_ll_crctable(uint16_t poly, uint16_t (*crcfn)(uint16_t, uint16_t, uint16_t))
{
	uint16_t *ll_crctable;
	int i;
	if ((ll_crctable = (uint16_t *)malloc(256*sizeof(unsigned))) == NULL) {
		return NULL;
	}
	for (i = 0; i < 256; i++) {
		ll_crctable[i] = (*crcfn)(i, poly, 0);
	}
	return ll_crctable;
}

// generator for CRC table
uint16_t ll_crchware(uint16_t data, uint16_t genpoly, uint16_t accum)
{
	static int i;
	data <<= 8;
	for (i = 8; i > 0; i--) {
	  if ((data ^ accum) & 0x8000) {
      accum = (accum << 1) ^ genpoly;
	  } else {
      accum <<= 1;
    }
	  data <<= 1;
	}
	return accum;
}

// checks/generates a CRC value for received/sent message
void ll_crccheck(uint16_t data, uint16_t *accumulator, uint16_t *ll_crctable)
{
	*accumulator = (*accumulator << 8) ^ ll_crctable[(*accumulator >> 8) ^ data];
}

void send_file_to_linklist(linklist_t * ll, char * blockname, char * filename)
{
  if (!ll) {
    linklist_err("Invalid linklist given");
    return;
  }

  int i = 0;
  block_t * theblock = NULL;

  // find the block
  for (i = 0; i < ll->num_blocks; i++) {
    if (strcmp(blockname, ll->blocks[i].name) == 0) {
      theblock = &ll->blocks[i];
      break;
    }
  }
  if (!theblock) {
    linklist_err("Block \"%s\" not found in linklist \"%s\"", blockname, ll->name);
    return;
  }

  // check to see if the previous send is done
  if (theblock->i != theblock->n) { // previous transfer not done
    linklist_info("Previous transfer for block \"%s\" is incomplete.", blockname);
    return;
  }
 
  // open the file
  FILE * fp = fopen(filename, "rb+");
  if (!fp) {
    linklist_err("File \"%s\" not found", filename);
    return;
  }
  
  // get the size of the file
  unsigned int filesize = 0;
  fseek(fp, 0L, SEEK_END);
  filesize = ftell(fp);
  fseek(fp, 0L, SEEK_SET);

  // set the block variables to initialize transfer
  theblock->fp = fp;
  strcpy(theblock->filename, filename); // copy the filename stripped of path
  theblock->i = 0;
  theblock->num = 0;
  theblock->n = (filesize-1)/(theblock->le->blk_size-PACKET_HEADER_SIZE)+1;
  theblock->curr_size = filesize;

  theblock->id++; // increment the block counter

  linklist_info("File \"%s\" sent to linklist \"%s\"", filename, ll->name);

  return;
}

// randomizes/unrandomizes a buffer of a given size using a given seed
uint8_t randomized_buffer(uint8_t * buffer, unsigned int bufsize, unsigned int seed)
{
  srand(seed);
  int i = 0;
  unsigned int sum = 0;
  for (i = 0; i < bufsize; i++) {
    buffer[i] ^= rand();
    sum ^= buffer[i];
  }
  return sum;
}

// allocates the 1 Hz superframe needed for linklist compression
uint8_t * allocate_superframe(superframe_t * superframe)
{
  // allocate data and superframe offsets
  if (!superframe || !superframe->size) {
    linklist_err("Cannot allocate superframe");
    return NULL;
  }

  uint8_t * ptr = calloc(1, superframe->size);

  return ptr;
}


/**
 * compress_linklist
 * 
 * Uses default options for compress_linklist_opt (see below) 
 */
int compress_linklist(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in)
{
  return compress_linklist_opt(buffer_out, ll, buffer_in, UINT32_MAX, 0);
}

/**
 * compress_linklist_opt
 * 
 * Selects entries from and compresses a superframe according to the provide
 * linklist format.
 * -> buffer_out: buffer in which compressed frame will be written.
 * -> ll: pointer to linklist specifying compression and entry selection
 * -> buffer_in: pointer to the superframe to be compressed. 
 */
int compress_linklist_opt(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in, uint32_t maxsize, int flags)
{
  // allocate crc table if necessary
  if (ll_crctable == NULL)
  {
    if((ll_crctable = mk_ll_crctable((unsigned short)LL_CRC_POLY,ll_crchware)) == NULL)
    {
      linklist_fatal("mk_ll_crctable() memory allocation failed\n");
    }
  } 

  int i,j;

  unsigned int tlm_in_start = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_out_size = 0;
  uint8_t * tlm_in_buf, * tlm_out_buf;
  uint8_t tlm_comp_type = 0;
  struct link_entry * tlm_le;
  uint16_t checksum = 0;

  // check validity of buffers
  if (!buffer_in) {
    linklist_err("buffer_in is NULL");
    return 0;
  }
  if (!buffer_out) {
    linklist_err("buffer_out is NULL");
    return 0;
  }

  for (i=0;i<ll->n_entries;i++)
  {
    tlm_le = &(ll->items[i]);
    tlm_out_start = tlm_le->start;
    tlm_out_buf = buffer_out+tlm_out_start;

    if (tlm_le->tlm == NULL) // checksum field
    {
      if (!(flags & LL_IGNORE_CHECKSUM)) 
      {
        memcpy(tlm_out_buf+0,((uint8_t*)&checksum)+1,1);
        memcpy(tlm_out_buf+1,((uint8_t*)&checksum)+0,1);
        for (j=0;j<2;j++) ll_crccheck(tlm_out_buf[j],&checksum,ll_crctable); // check the checksum
        if (checksum != 0) 
        {
          linklist_err("compress_linklist: invalid checksum generated\n");
        }
        //printf("Compressed checksum result for %s: %d 0x%x\n",name,i,checksum);
      }
      checksum = 0; // reset checksum for next block
    }
    else // normal field
    {
      if (tlm_le->tlm == &block_entry) // data block field
      {
        block_t * theblock = linklist_find_block_by_pointer(ll, tlm_le);
        if (theblock) packetize_block_raw(theblock, tlm_out_buf);
        else linklist_err("Could not find block in linklist \"%s\"", ll->name);
      }
      else // just a normal field 
      {
        tlm_comp_type = tlm_le->comp_type;
        tlm_in_start = tlm_le->tlm->start;
        tlm_in_buf = buffer_in+tlm_in_start;
        if (tlm_comp_type != NO_COMP) // compression
        {
          (*compRoutine[tlm_comp_type].compressFunc)(tlm_out_buf,tlm_le,tlm_in_buf);
        }
        else
        {
          decimationCompress(tlm_out_buf,tlm_le,tlm_in_buf);
        }
      }

      // update checksum
			tlm_out_size = tlm_le->blk_size;
      for (j=0;j<tlm_out_size;j++) ll_crccheck(tlm_out_buf[j],&checksum,ll_crctable);
    }
  }
  return 1;
}


uint8_t * buffer_save = NULL;

int fill_linklist_with_saved(linklist_t * req_ll, int p_start, int p_end, uint8_t *buffer_out)
{
  int i, k;
  struct link_entry * tlm_le = NULL;
  unsigned int tlm_out_start;
  unsigned int loc1, loc2;
  unsigned int tlm_out_num;
  unsigned int tlm_size;
  unsigned int tlm_out_skip;

  for (i=p_start;i<p_end;i++)
  { 
    tlm_le = &(req_ll->items[i]);
    if (tlm_le->tlm != NULL)
    { 
      if (tlm_le->tlm != &block_entry)
      { 
        tlm_out_start = tlm_le->tlm->start;
        tlm_out_skip = tlm_le->tlm->skip;
        tlm_out_num = tlm_le->tlm->spf;
        tlm_size = get_superframe_entry_size(tlm_le->tlm);
        //printf("Fixing %s (start = %d)\n",tlm_le->tlm->name,tlm_out_start);
        for (k=0;k<tlm_out_num;k++)
        { 
          loc1 = tlm_out_skip*k;
          loc2 = tlm_out_skip*(tlm_out_num-1);
          memcpy(buffer_out+tlm_out_start+loc1, buffer_save+tlm_out_start+loc2, tlm_size);
        }
      }
      //memset(buffer_out+tlm_le->tlm->start,0,tlm_le->tlm->size*tlm_le->tlm->num);
    }
  }
  return i;
}

/**
 * decompress_linklist
 *
 * Uses defaults for decompress_linklist_opt (see below).
 */
double decompress_linklist(uint8_t * buffer_out, linklist_t * ll, uint8_t * buffer_in)
{
  return decompress_linklist_opt(buffer_out, ll, buffer_in, UINT32_MAX, LL_VERBOSE);
}

/**
 * decompress_linklist_opt
 * 
 * Selects entries from and compresses a superframe according to the provide
 * linklist format.
 * -> buffer_out: buffer in which decompressed superframe will be written.
                  If NULL, the buffer assigned to the linklist via
                  assign_superframe_to_linklist will be used.
 * -> ll: pointer to linklist specifying compression and entry selection
 * -> buffer_in: pointer to the compressed frame to be decompressed. 
 * -> maxsize: the maximum size of the input buffer which may be less than
 */
double decompress_linklist_opt(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in, uint32_t maxsize, int flags)
{
  // allocate crc table if necessary
  if (ll_crctable == NULL)
  {
    if((ll_crctable = mk_ll_crctable((unsigned short)LL_CRC_POLY,ll_crchware)) == NULL)
    {
      linklist_fatal("mk_ll_crctable() memory allocation failed\n");
    }
  } 

  superframe_t * superframe = ll->superframe;

  int i, j;

  unsigned int tlm_in_start = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_in_size = 0;
  uint8_t * tlm_in_buf, * tlm_out_buf;
  uint8_t tlm_comp_type = 0;
  struct link_entry * tlm_le;
  int p_start = 0, p_end = 0;
  uint16_t checksum = 0;
  uint16_t prechecksum = 0;
  uint16_t sumcount = 0;
  double ret = 0, tot = 0;

  // check validity of buffers
  if (!buffer_out) {
    linklist_err("buffer_out is NULL");
    return 0;
  }
  if (!buffer_in) {
    linklist_err("buffer_in is NULL");
    return 0;
  }

  if (buffer_save == NULL) buffer_save = calloc(1, superframe->size);

  // extract the data to the full buffer
  for (j=0;j<ll->n_entries;j++)
  {
    tlm_le = &(ll->items[j]);
    tlm_in_size = tlm_le->blk_size;
    tlm_in_start = tlm_le->start;
    tlm_in_buf = buffer_in+tlm_in_start;

    // update checksum
    for (i=0;i<tlm_in_size;i++) ll_crccheck(tlm_in_buf[i],&checksum,ll_crctable);

    p_end = j;

    if (tlm_le->tlm == NULL) // evaluating a checksum...
    {
      if (tlm_in_start > maxsize) { // reached the maximum input buffer size; the rest is assumed to be garbage
        fill_linklist_with_saved(ll, p_start, p_end, buffer_out);

        break;
        // linklist_info("Block %d is beyond the max size of %d", sumcount, maxsize);
      } else if ((checksum != 0) && !(flags & LL_IGNORE_CHECKSUM)) { // bad data block
        // clear/replace bad data from output buffer
        if (flags & LL_VERBOSE) linklist_info("decompress_linklist: checksum failed -> bad data (block %d)\n", sumcount);
        fill_linklist_with_saved(ll, p_start, p_end, buffer_out);
      }
      else ret++;

      // reset checksum
      prechecksum |= *(uint16_t *) tlm_in_buf;
      checksum = 0;
      sumcount++;
      p_start = j+1;
      tot++;

    }
    else
    {
      if (tlm_le->tlm == &block_entry) // data block entry 
      {
        block_t * theblock = linklist_find_block_by_pointer(ll, tlm_le);
        if (theblock) depacketize_block_raw(theblock, tlm_in_buf);
        else linklist_err("Could not find block in linklist \"%s\"", ll->name);
      }
      else // just a normal field
      {
        tlm_out_start = tlm_le->tlm->start;
        tlm_comp_type = tlm_le->comp_type;
        tlm_out_buf = buffer_out+tlm_out_start;

        if (tlm_comp_type != NO_COMP) // compression
        {
          (*compRoutine[tlm_comp_type].decompressFunc)(tlm_out_buf,tlm_le,tlm_in_buf);
        }
        else
        {
          decimationDecompress(tlm_out_buf,tlm_le,tlm_in_buf);
        }
      }
    }
  }
  if (!prechecksum && !(flags & LL_IGNORE_CHECKSUM)) // all the checksums were zero; must be blank frame
  {
    fill_linklist_with_saved(ll, 0, ll->n_entries, buffer_out);
    // ret = 0;
  }
  // save the data
  memcpy(buffer_save, buffer_out, superframe->size);

  ret = (tot == 0) ? ret : ret/tot;

  return ret;
}

void packetize_block_raw(struct block_container * block, uint8_t * buffer)
{
  if (block->i < block->n) { // packets left to be sent 
    unsigned int loc = (block->i*(block->le->blk_size-PACKET_HEADER_SIZE)); // location in data block
    unsigned int cpy = MIN(block->le->blk_size-PACKET_HEADER_SIZE, block->curr_size-loc);
    
    int fsize = make_block_header(buffer, block->id, cpy, block->i, block->n, block->curr_size);
 
    if (loc > block->curr_size) {
      linklist_err("Block location %d > total size %d", loc, block->curr_size);
      return;
    }

    if (block->fp) { // there is a file instead of data in a buffer
      *(uint16_t *) buffer |= BLOCK_FILE_MASK; // add the mask to indicate that the transfer is a file
      fseek(block->fp, loc, SEEK_SET); // go to the location in the file
      int retval = 0;
      if ((retval = fread(buffer+fsize, 1, cpy, block->fp)) != cpy) {
        linklist_err("Could only read %d/%d bytes at %d from file %s", retval, cpy, loc, block->filename);
      }
    } else { // there is data in the buffer
      memcpy(buffer+fsize, block->buffer+loc, cpy);
    }

    // printf("Sent block %d/%d (id = %d, size = %d, loc = %d)\n",block->i+1,block->n,block->id,cpy,loc);
    
    block->i++;
    block->num++;
  } else { // no blocks left
    memset(buffer, 0, block->le->blk_size);
    block->i = block->n;
    if (block->fp) { // file was open, so close it
      fclose(block->fp);
      block->fp = NULL;
      block->filename[0] = 0;
    }
    //printf("Nothing to send\n");
  }

}

// handle for openning and appending to files even if they don't exist
FILE * fpreopenb(char *fname)
{
  FILE * temp = fopen(fname,"ab");
  if (!temp) {
    printf("Cannot open %s (errno %d: %s) \n", fname, errno, strerror(errno));
    return NULL;
  }
  fclose(temp);
  return fopen(fname,"rb+");
}

void depacketize_block_raw(struct block_container * block, uint8_t * buffer)
{
  uint16_t id = 0;
  uint16_t i = 0, n = 0;
  uint16_t blksize = 0;
  uint32_t totalsize = 0;

  int fsize = read_block_header(buffer,&id,&blksize,&i,&n,&totalsize);

  // no data to read
  if (blksize == 0) {
    // close any dangling file pointers
    if (block->fp) {
      linklist_info("Closing \"%s\"\n\n", block->filename);
      fclose(block->fp);
      block->fp = NULL;
      block->filename[0] = 0;
    }
    return;
  }

  if (n == 0) { // special case of block fragment
    linklist_info("Received block fragment %d (size %d)\n",i,totalsize);
    
    totalsize = blksize;
    block->num = 1;
    
    i = 0;
    n = 0;
  } else if (i >= n) { 
    linklist_info("depacketize_block: index larger than total (%d > %d)\n",i,n);
    //memset(buffer,0,blksize+fsize); // clear the bad block
    return;
  }
 
  unsigned int loc = i*(block->le->blk_size-fsize);

  if (id & BLOCK_FILE_MASK) { // receiving a file
    // a different id packet was received, so close file if that packet is open
    id ^= BLOCK_FILE_MASK; // remove the mask
    if ((id != block->id) && block->fp) {
      fclose(block->fp);
      block->fp = NULL;
    }
    if (!block->fp) { // file not open yet
      sprintf(block->filename, "%s/%s_%d", LINKLIST_FILESAVE_DIR, block->name, id); // build filename
      if (!(block->fp = fpreopenb(block->filename))) {
        linklist_err("Cannot open file %s", block->filename);
        return;
      }
      linklist_info("New file \"%s\" opened\n", block->filename);
    }
    fseek(block->fp, loc, SEEK_SET); 
    int retval = 0;
    if ((retval = fwrite(buffer+fsize, 1, blksize, block->fp)) != blksize) {
      linklist_err("Could only write %d/%d bytes at %d to file %s", retval, blksize, loc, block->filename);
    }

  } else { // just a normal block to be stored in memory
    // expand the buffer if necessary
    if (totalsize > block->alloc_size)
    {  
      void * tp = realloc(block->buffer,totalsize);
      block->buffer = (uint8_t *) tp;
      block->alloc_size = totalsize;
    }
    memcpy(block->buffer+loc,buffer+fsize,blksize);
  }

  // set block variables 
  block->curr_size = totalsize;
  block->i = i;
  block->n = n;
  block->id = id;  

  block->i++;
  block->num++;

  linklist_info("Received \"%s\" %d/%d (%d/%d)\n",block->name,block->i,block->n,loc+blksize,totalsize);

}

// takes superframe at buffer in and creates an all frame in buffer out
// all frame is 1 sample per field uncompressed
int write_allframe(uint8_t * allframe, superframe_t * superframe, uint8_t * sf) {
  int i;
  int tlm_out_start = 4; // the first 4 bytes are the serial for allframes
  int tlm_in_start = 0; 
  unsigned int tlm_size = 0; 
  uint32_t test = ALLFRAME_SERIAL;

  int wd = 1;
  if ((allframe == NULL) || (sf == NULL)) wd = 0;

  superframe_entry_t * ll_superframe_list = superframe->entries;

  if (wd) memcpy(allframe, &test, 4);

  for (i = 0; i < superframe->n_entries; i++) {
    tlm_in_start = ll_superframe_list[i].start;
    tlm_size = get_superframe_entry_size(&ll_superframe_list[i]);

    if (wd) memcpy(allframe+tlm_out_start, sf+tlm_in_start, tlm_size);
    tlm_out_start += tlm_size;
  }
  return tlm_out_start;

}

// takes an allframe at buffer in at 1 sample per frame and writes repeated
// samples in the superframe
int read_allframe(uint8_t * sf, superframe_t * superframe, uint8_t * allframe) {
  int i,j;
  int tlm_out_start = 0;
  int tlm_in_start = 4;
  int tlm_num = 0;
  int tlm_skip = 0;
  unsigned int tlm_size = 0;
  
  superframe_entry_t * ll_superframe_list = superframe->entries;

  // check to see if this is actually an allframe
  if (*((uint32_t *) allframe) != ALLFRAME_SERIAL) { 
    return 0;
  }
  
  for (i = 0; i < superframe->n_entries; i++) { 
    tlm_out_start = ll_superframe_list[i].start;
    tlm_size = get_superframe_entry_size(&ll_superframe_list[i]);
    tlm_num = ll_superframe_list[i].spf;
    tlm_skip = ll_superframe_list[i].skip;

    for (j = 0; j < tlm_num; j++) {
      memcpy(sf+tlm_out_start, allframe+tlm_in_start, tlm_size);
      tlm_out_start += tlm_skip;
    }
    tlm_in_start += tlm_size;
  } 
  return tlm_in_start;

}

double antiAlias(uint8_t * data_in, char type, unsigned int num, unsigned int skip,
                   double (*datatodouble)(uint8_t *, uint8_t))
{
  int i;
  double halfsum = 0;
  double ret = 0;  

  if (num == 1) return (*datatodouble)(data_in,type);

  for (i=0;i<num;i++) 
  {
    halfsum += (*datatodouble)(data_in+i*skip,type); 
  }
  ret = halfsum/num;

  return ret;
}


int stream32bitFixedPtComp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int wd = 1;
  int blk_size = 0; 
  double offset = 0.0, gain = 1.0;
  double temp1;

  char type = le->tlm->type;
  //unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int inputskip = le->tlm->skip;
  unsigned int inputnum = le->tlm->spf;
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  int i;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd)
  {
    type = '0';
  }

/* TODO MAXMIN
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT32_MAX);
  } 
*/
  offset = 0;
  gain = 1; 

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type ,decim, inputskip, datatodouble);
      temp1 = (temp1-offset)/(gain);
			if (temp1 > UINT32_MAX) temp1 = UINT32_MAX;
      else if (temp1 < 0.0) temp1 = 0.0;
      *((uint32_t*) (data_out+blk_size)) = temp1;
    }
    blk_size+=4;
  }

  return blk_size;
}

int stream32bitFixedPtDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int i,j;
  int wd = 1;
  int blk_size = 0;
  double gain = 1.0, offset = 0.0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  int outputnum = le->tlm->spf;
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

/* TODO MAXMIN
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT32_MAX);
  }
*/
  offset = 0;
  gain = 1;

  double dataout = 0;
  uint32_t value = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) 
    {
      value = *((uint32_t *) &data_in[i*4]);
      dataout = ((double) value)*gain+offset;
    }
    //printf("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //printf("\n\n");

  return blk_size;
}

int stream16bitFixedPtComp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int wd = 1;
  int blk_size = 0; 
  double offset = 0.0, gain = 1.0;
  double temp1;

  char type = le->tlm->type;
  //unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int inputskip = le->tlm->skip;
  unsigned int inputnum = le->tlm->spf;
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  int i;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd)
  {
    type = '0';
  }

  offset = 0;
  gain = 1;
/* TODO MAXMIN
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT16_MAX);
  }  
*/

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type, decim, inputskip, datatodouble);
      temp1 = (temp1-offset)/(gain);
			if (temp1 > UINT16_MAX) temp1 = UINT16_MAX;
      else if (temp1 < 0.0) temp1 = 0.0;
      *((uint16_t*) (data_out+blk_size)) = temp1;
    }
    blk_size+=2;
  }

  return blk_size;
}

int stream16bitFixedPtDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int i,j;
  int wd = 1;
  int blk_size = 0;
  double gain = 1.0, offset = 0.0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  int outputnum = le->tlm->spf;
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

/* TODO MAXMIN
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT16_MAX);
  }
*/
  offset = 0;
  gain = 1;

  double dataout = 0;
  uint16_t value = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) 
    {
      value = *((uint16_t *) &data_in[i*2]);
      dataout = ((double) value)*gain+offset;
    }
    //printf("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //printf("\n\n");

  return blk_size;
}
int stream8bitComp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int wd = 1;
  int blk_size = 8; // gain and offset are two floats
  float offset, gain;

  char type = le->tlm->type;
  //unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int inputskip = le->tlm->skip;
  unsigned int inputnum = le->tlm->spf;
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  int i;
  double min = 1.0e30, max = -1.0e30, temp1 = 0;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd)
  {
    type = '0';
  }

  // get gain and offset from first 10% of the input data  
  for (i=0;i<(inputnum);i+=decim)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*inputskip), type, decim, inputskip, datatodouble);
      if (temp1 < min) min = temp1;
      if (temp1 > max) max = temp1;
    }
  }
  offset = min;
  gain = (max-min)/((double) (UINT8_MAX-1)); // scale from 0x0 to 0xfe

  if (wd)
  {
    memcpy(data_out+0,&gain,4);
    memcpy(data_out+4,&offset,4);
  }

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type, decim, inputskip, datatodouble);
      temp1 = (temp1-offset)/(gain);
      data_out[blk_size] = temp1;
    }
    blk_size++;
  }

  return blk_size;
}

int stream8bitDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int i,j;
  int wd = 1;
  int blk_size = 0;
  float gain = 0;
  float offset = 0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  int outputnum = le->tlm->spf;
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (wd)
  {
    gain = *((float *) (data_in+0));
    offset = *((float *) (data_in+4));
  }

  double dataout = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) dataout = ((double) data_in[i+8])*gain+offset;
    //printf("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //printf("\n\n");

  return blk_size;
}


int stream8bitDeltaComp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int wd = 1;
  int blk_size = 8; // gain and offset are two floats
  double remainder = 0;
  float offset, gain;

  char type = le->tlm->type;
  //unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int inputskip = le->tlm->skip;
  unsigned int inputnum = le->tlm->spf;
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;  

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  int i;
  double temp1 = 0, temp2 = 0, dif = 0;
  double min = INT32_MAX, max= -INT32_MAX;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd) 
  {
    type = '0';
  }

  // get gain and offset from first 10% of the input data  
  temp1 = 0;
  for (i=0;i<(inputnum);i+=decim)
  {
    if (wd) 
    {
      temp2 = antiAlias(data_in+(i*inputskip), type, decim, inputskip, datatodouble);
      dif = temp2-temp1;
      if (dif < min) min = dif;
      if (dif > max) max = dif;
      temp1 = temp2;
    }
  }

  offset = min;
  gain = ((double) (max-min))/((double) (UINT8_MAX-1)); // scale from 0x0 to 0xfe

  if (wd)
  {
    memcpy(data_out+0,&gain,4);
    memcpy(data_out+4,&offset,4);
  }

  temp1 = 0;
  remainder = 0;
  for (i=0;i<outputnum;i++)
  {
    if (wd) 
    {
      temp2 = antiAlias(data_in+(i*decim*inputskip), type, decim, inputskip, datatodouble);
      dif = (temp2-temp1-offset)/(gain);
      data_out[blk_size] = dif+remainder;
      remainder += dif-data_out[blk_size];
      temp1 = temp2;
    }
    blk_size++;
  }
  remainder = 0;
 
  return blk_size;
}

int stream8bitDeltaDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int i,j;
  int wd = 1;
  int blk_size = 0;
  float gain = 0;
  float offset = 0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  int outputnum = le->tlm->spf;
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (wd)
  {
    gain = *((float *) (data_in+0));
    offset = *((float *) (data_in+4));
  }

  double dataout = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) dataout += ((double) data_in[i+8])*gain+offset;
    //printf("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;  
      blk_size += outputsize;
    }
  }
  //printf("\n\n");

  return blk_size;
}


int decimationCompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  int wd = 1;
  int blk_size = 0;

  //unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int inputskip = le->tlm->skip;
  unsigned int inputnum = le->tlm->spf;
  unsigned int inputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  unsigned int size = get_superframe_entry_size(le->tlm);  

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  if (decim == 0) return 0;
  int i;
  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  uint64_t dout = 0;
  char type = le->tlm->type;
  double temp1 = 0;

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type, decim, inputskip, datatodouble);
      (*doubletodata)((uint8_t *) &dout,temp1,type);
      memcpy(data_out+(i*inputsize),&dout,size);
    }
    blk_size += size;
  }
  return blk_size;

}

int decimationDecompress(uint8_t * data_out, struct link_entry *le, uint8_t * data_in)
{
  int i,j;
  int wd = 1;
  int blk_size = 0;

  unsigned int outputskip = le->tlm->skip;
  int outputnum = le->tlm->spf;
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;

  unsigned int size = get_superframe_entry_size(le->tlm);

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  for (i=0;i<inputnum;i++)
  {
    for (j=0;j<decim;j++)
    {
      if (wd) 
      {
        memcpy(data_out+((i*decim+j)*outputskip),data_in+(i*size),size);
      }
      blk_size += size;
    }
  }

  return blk_size;

}


struct dataCompressor compRoutine[NUM_COMPRESS_TYPES+1] = {
  {COMPRESS(FIXED_PT_16BIT), stream16bitFixedPtComp, stream16bitFixedPtDecomp},
  {COMPRESS(FIXED_PT_32BIT), stream32bitFixedPtComp, stream32bitFixedPtDecomp},
  {COMPRESS(MEAN_FLOAT_DELTA_8BIT), stream8bitDeltaComp, stream8bitDeltaDecomp},
  {COMPRESS(MEAN_FLOAT_8BIT), stream8bitComp, stream8bitDecomp},
 
  // terminator
  {0}
};

#ifdef __cplusplus

}

#endif
