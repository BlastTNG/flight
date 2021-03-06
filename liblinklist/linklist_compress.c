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

extern int (*linklist_info)(const char *, ...);
extern int (*linklist_err)(const char *, ...);
extern int (*linklist_warn)(const char *, ...);
extern int (*linklist_fatal)(const char *, ...);

extern superframe_entry_t block_entry;
extern superframe_entry_t stream_entry;

#define LL_CRC_POLY 0x1021
uint16_t *ll_crctable = NULL;

/* block header (12 bytes)
 * ----------------
 * [0-3] = identifier (0x80000000 => file)
 * [4-5] = size of data in packet [bytes]
 * [6-8] = packet number (i)
 * [9-11] = total number of packets (n)
 */
// generates the block header in the buffer
int make_block_header(uint8_t * buffer, uint32_t id, uint16_t size, uint32_t i, uint32_t n)
{
  i &= 0x00ffffff;
  n &= 0x00ffffff;

  memcpy(buffer+0, &id,   4); // ID
  memcpy(buffer+4, &size, 2); // size of data in packet
  memcpy(buffer+6, &i,    3); // packet number
  memcpy(buffer+9, &n,    3); // number of packets

  return PACKET_HEADER_SIZE;
}

// generates the block header in the buffer
int read_block_header(uint8_t * buffer, uint32_t *id, uint16_t *size, uint32_t *i, uint32_t *n)
{
  *i &= 0x00ffffff;
  *n &= 0x00ffffff;

  memcpy(id,   buffer+0, 4); // ID
  memcpy(size, buffer+4, 2); // size of data in packet
  memcpy(i,    buffer+6, 3); // packet number
  memcpy(n,    buffer+9, 3); // number of packets

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

block_t * block_find_by_name(linklist_t * ll, char * blockname) {
  unsigned int j;
  // find the block
  for (j = 0; j < ll->num_blocks; j++) {
    if (strcmp(blockname, ll->blocks[j].name) == 0) {
      return &ll->blocks[j];
    }
  }
  return NULL;
}
stream_t * stream_find_by_name(linklist_t * ll, char * streamname) {
  unsigned int j;
  // find the stream
  for (j = 0; j < ll->num_streams; j++) {
    if (strcmp(streamname, ll->streams[j].name) == 0) {
      return &ll->streams[j];
    }
  }
  return NULL;
}

// get a list of pointers to all the streams with a given name (null terminated)
stream_t ** linklist_get_streamlist(linklist_t ** ll_array, char * streamname) {
  int i;
  int num = 0;
  stream_t ** streamlist = (stream_t **) calloc(MAX_NUM_LINKLIST_FILES, sizeof(stream_t *));
  for (i=0; ll_array[i]; i++) {
    if ((streamlist[num] = stream_find_by_name(ll_array[i], streamname))) num++;
  } 
  streamlist[num] = NULL;
  return streamlist;
}


void close_block_fp(block_t * block) {
  if (!block->fp) return;
  FILE *fp = block->fp;
  block->fp = NULL;
  fclose(fp);
}

void close_stream_fp(stream_t * stream) {
  if (!stream->fp) return;
  FILE *fp = stream->fp;
  stream->fp = NULL;
  fclose(fp);
}

#pragma pack(push, 1) // structure packing aligns at 1 byte (no padding)
struct preempt_data {
  uint32_t serial;
  char filename[LINKLIST_SHORT_FILENAME_SIZE];
};
#pragma pack(pop) // undo custom structure packing

int linklist_send_file_by_block(linklist_t * ll, char * blockname, char * filename, 
                               int32_t id, int flags) {
  return linklist_send_file_by_block_ind(ll, blockname, filename, id, flags, 0, 0);
}

int linklist_send_file_by_block_ind(linklist_t * ll, char * blockname, char * filename, 
                                    int32_t id, int flags, unsigned int i, unsigned int n) {
  if (!ll) {
    linklist_err("linklist_send_file_by_block_ind: Invalid linklist given\n");
    return 0;
  }

  // find the block in the linklist
  block_t * theblock = block_find_by_name(ll, blockname);
  if (!theblock) {
    linklist_err("linklist_send_file_by_block_ind: Block \"%s\" not found in linklist \"%s\"\n", blockname, ll->name);
    return 0;
  }

  // check to see if the previous send is done
  if (!(flags & BLOCK_OVERRIDE_CURRENT) && (theblock->i < theblock->n)) {
    linklist_info("linklist_send_file_by_block_ind: Previous transfer for block \"%s\" is incomplete.\n", blockname);
    return 0;
  }

  // cancel any current block packetization (n < i sentinel condition)
  theblock->n = 0;
  theblock->i = 1;

  // check for any dangling file pointers
  if (theblock->fp) {
    close_block_fp(theblock);
  }
 
  // open the file
  FILE * fp = fopen(filename, "rb+");
  if (!fp) {
    linklist_err("linklist_send_file_by_block_ind: File \"%s\" not found\n", filename);
    return 0;
  }
  
  // get the size of the file
  unsigned int filesize = 0;
  fseek(fp, 0L, SEEK_END);
  filesize = ftell(fp);
  fseek(fp, 0L, SEEK_SET);

  // deal with empty files
  if (!filesize) {
    linklist_err("linklist_send_file_by_block_ind: File \"%s\" is empty\n", filename);
    return 0;
  }

  uint32_t n_total = (filesize-1)/(theblock->le->blk_size-PACKET_HEADER_SIZE)+1;
  if (n_total & 0xff000000) {
    linklist_err("linklist_send_file_by_block_ind: File \"%s\" is too large\n", filename);
    return 0;
  }

  // check the preempt flag for send aux data before the file
  if (flags & BLOCK_PREEMPT_FILE) {
    struct preempt_data * pd = (struct preempt_data *) theblock->buffer;
    memset(pd, 0, sizeof(struct preempt_data));

    pd->serial = BLOCK_PREEMPT_ID; 
    int i;
    for (i=strlen(filename)-1; i>=0; i--) {
      if (filename[i] == '/') break;
    }
    strncpy(pd->filename, filename+i+1, LINKLIST_SHORT_FILENAME_SIZE-1);
  }

  // set the block variables to initialize transfer
  theblock->fp = fp; // non-null file desc indicates that we want to read data from file
  strcpy(theblock->filename, filename); 
  theblock->num = 0;
  theblock->id = id;
  if (!(flags & BLOCK_NO_DOWNSTREAM_FILE)) {
    theblock->id |= BLOCK_FILE_MASK; // the flag/mask indicates that we want a file on the ground
  }
  theblock->curr_size = filesize;
  theblock->flags = flags;
  if (!n && !i) { // set the whole transfer
    theblock->i = 0;
    theblock->n = n_total;
  } else { // set the potential partial transfer
    theblock->i = MIN(MIN(i, n), n_total);
    theblock->n = MIN(n, n_total);
  }

  linklist_info("File \"%s\" (%d B == %d pkts) sent to linklist \"%s\" (i=%d, n=%d)\n", filename, filesize, n_total, ll->name, theblock->i, theblock->n);

  return 1;
}

int assign_file_to_streamlist(stream_t ** streamlist, char * filename, int offset, int whence) {
  int i;
  int retval = 0;
  for (i=0; streamlist[i]; i++) {
    retval += assign_file_to_stream(streamlist[i], filename, offset, whence);
  }
  return retval;
}

int remove_file_from_streamlist(stream_t ** streamlist) {
  int i;
  int retval = 0;
  for (i=0; streamlist[i]; i++) {
    retval += remove_file_from_stream(streamlist[i]);
  }
  return retval;
}

int seek_file_in_streamlist(stream_t ** streamlist, int offset, int whence) {
  int i;
  int retval = 0;
  for (i=0; streamlist[i]; i++) {
    retval += seek_file_in_stream(streamlist[i], offset, whence);
  }
  return retval;
}

void write_next_streamlist(stream_t ** streamlist, uint8_t * buffer, unsigned int bsize, unsigned int flags) {
  int i;
  for (i=0; streamlist[i]; i++) {
    write_next_stream(streamlist[i], buffer, bsize, flags);
  }
}

void stop_current_streamlist(stream_t ** streamlist) {
  int i;
  for (i=0; streamlist[i]; i++) {
    stop_current_stream(streamlist[i]);
  }
}

int linklist_assign_file_to_stream(linklist_t * ll, char * streamname, char * filename, 
                                   int offset, int whence) {
  if (!ll) {
    linklist_err("linklist_assign_file_to_stream: Invalid linklist given\n");
    return 0;
  }

  // find the block in the linklist
  stream_t * thestream = stream_find_by_name(ll, streamname);
  if (!thestream) {
    linklist_err("linklist_assign_file_to_stream: Stream \"%s\" not found in linklist \"%s\"\n", streamname, ll->name);
    return 0;
  }

  return assign_file_to_stream(thestream, filename, offset, whence);
}

int assign_file_to_stream(stream_t * thestream, char * filename, int offset, int whence) {
  if (thestream->fp) {
    close_stream_fp(thestream);
  }
 
  // open the file
  FILE * fp = fopen(filename, "rb+");
  if (!fp) {
    linklist_err("linklist_assign_file_to_stream: File \"%s\" not found\n", filename);
    return 0;
  }
 
  // seek to specified location 
  fseek(fp, offset, whence);

  // assign the stream the file descriptor
  strcpy(thestream->filename, filename); // copy the filename stripped of path
  thestream->fp = fp; 

  return 1;
}

int linklist_remove_file_from_stream(linklist_t * ll, char * streamname) {
  if (!ll) {
    linklist_err("linklist_assign_file_to_stream: Invalid linklist given\n");
    return 0;
  }

  // find the block in the linklist
  stream_t * thestream = stream_find_by_name(ll, streamname);
  if (!thestream) {
    linklist_err("linklist_assign_file_to_stream: Stream \"%s\" not found in linklist \"%s\"\n", streamname, ll->name);
    return 0;
  }
  return remove_file_from_stream(thestream);
}

int remove_file_from_stream(stream_t * thestream) {
  if (thestream->fp) {
    close_stream_fp(thestream);
    thestream->filename[0] = '\0';
  }
  return 1;
}

int linklist_seek_file_in_stream(linklist_t * ll, char * streamname, int offset, int whence) {
  if (!ll) {
    linklist_err("linklist_seek_file_in_stream: Invalid linklist given\n");
    return 0;
  }

  // find the block in the linklist
  stream_t * thestream = stream_find_by_name(ll, streamname);
  if (!thestream) {
    linklist_err("linklist_seek_file_in_stream: Stream \"%s\" not found in linklist \"%s\"\n", streamname, ll->name);
    return 0;
  }
  return seek_file_in_stream(thestream, offset, whence);
}

int seek_file_in_stream(stream_t * thestream, int offset, int whence) {
  int retval = 0;
  if (thestream->fp) {
    retval = fseek(thestream->fp, offset, whence);
  }
  return retval;
}

void linklist_write_next_stream(linklist_t * ll, char * streamname, uint8_t * buffer, unsigned int bsize, unsigned int flags) {
  if (!ll) {
    linklist_err("linklist_write_next_stream: Invalid linklist given\n");
    return;
  }

  // find the block in the linklist
  stream_t * thestream = stream_find_by_name(ll, streamname);
  if (!thestream) {
    linklist_err("linklist_write_next_stream: Stream \"%s\" not found in linklist \"%s\"\n", streamname, ll->name);
    return;
  }
  write_next_stream(thestream, buffer, bsize, flags);
}

void write_next_stream(stream_t * stream, uint8_t * buffer, unsigned int bsize, unsigned int flags) {
  unsigned int newnext = 1-stream->curr;
  substream_t * ss = &stream->buffers[newnext]; 

  // do not overwrite data that must be sent
  if ((ss->flags & STREAM_MUST_SEND) && !(flags & STREAM_MUST_SEND)) return;
 
  // expand the buffer if necessary
  if (ss->alloc_size < bsize) {
    ss->alloc_size = bsize;
    ss->buffer = (uint8_t *) realloc(ss->buffer, ss->alloc_size);
  }

  // copy the data to the buffer
  memcpy(ss->buffer, buffer, bsize);
  ss->data_size = bsize;
  ss->flags = flags;
  ss->loc = 0;

  // queue the next buffer for writing
  stream->next = newnext; // only writing functions can modify stream->next
}

void linklist_stop_current_stream(linklist_t * ll, char * streamname) {
  if (!ll) {
    linklist_err("linklist_stop_current_stream: Invalid linklist given\n");
    return;
  }

  // find the block in the linklist
  stream_t * thestream = stream_find_by_name(ll, streamname);
  if (!thestream) {
    linklist_err("linklist_stop_current_stream: Stream \"%s\" not found in linklist \"%s\"\n", streamname, ll->name);
    return;
  }
  stop_current_stream(thestream);
}

// sets the current location to the data size to start the next stream (cancelling the current one)
void stop_current_stream(stream_t * thestream) {
  int curr = thestream->curr;
  substream_t * ss = &thestream->buffers[curr];
  ss->loc = ss->data_size;
}

// randomizes/unrandomizes a buffer of a given size using a given seed
uint8_t randomized_buffer(uint8_t * buffer, unsigned int bufsize, unsigned int seed)
{
  srand(seed);
  unsigned int i = 0;
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

  uint8_t * ptr = (uint8_t *) calloc(1, superframe->size);

  return ptr;
}

/*
 * compress_linklist_internal
 *
 * An implementation of compress_linklist that uses an internal buffer to store
 * compressed data. Compression only occurs the data id (arg 1) is different
 * from the last time the function is called. This is useful in cases where 
 * multiple routines are compressing the same linklist so that the same linklist 
 * isn't compressed multiple times unnecessarily for the same data.
 */
int compress_linklist_internal(uint64_t id, linklist_t * ll, uint8_t *buffer_in) {
  return compress_linklist_internal_opt(id, ll, buffer_in, UINT32_MAX, 0);
}

int compress_linklist_internal_opt(uint64_t id, linklist_t * ll, uint8_t *buffer_in, uint32_t maxsize, int flags) {
  // allocate the buffer if necessary
  if (!ll->internal_buffer) {
    ll->internal_buffer = (uint8_t *) calloc(1, ll->superframe->size);
  }

  // only compress data for new id 
  if (ll->internal_id == id) return 0;
  ll->internal_id = id;

  return compress_linklist_opt(ll->internal_buffer, ll, buffer_in, maxsize, flags);
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
 * linklist format. Return positive number if non-zero data was written.
 * -> buffer_out: buffer in which compressed frame will be written.
 * -> ll: pointer to linklist specifying compression and entry selection
 * -> buffer_in: pointer to the superframe to be compressed. 
 */
int compress_linklist_opt(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in, uint32_t maxsize, int flags)
{

  (void) maxsize; // unsued for now
 
  // allocate crc table if necessary
  if (ll_crctable == NULL)
  {
    if((ll_crctable = mk_ll_crctable((unsigned short)LL_CRC_POLY,ll_crchware)) == NULL)
    {
      linklist_fatal("mk_ll_crctable() memory allocation failed\n");
    }
  } 

  unsigned int i,j;

  unsigned int tlm_in_start = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_out_size = 0;
  uint8_t * tlm_in_buf, * tlm_out_buf;
  uint8_t tlm_comp_type = 0;
  struct link_entry * tlm_le;
  uint16_t checksum = 0;
  uint8_t retval = 0;

  // check validity of buffers
  if (!buffer_in) {
    linklist_err("buffer_in is NULL");
    return 0;
  }
  if (!buffer_out) {
    linklist_err("buffer_out is NULL");
    return 0;
  }

  for (i=0;i<ll->n_entries;i++) {
    tlm_le = &(ll->items[i]);
    tlm_out_start = tlm_le->start;
    tlm_out_buf = buffer_out+tlm_out_start;

    if (tlm_le->tlm == NULL) { // checksum field
      if (!(flags & LL_IGNORE_CHECKSUM)) {
        memcpy(tlm_out_buf+0,((uint8_t*)&checksum)+1,1);
        memcpy(tlm_out_buf+1,((uint8_t*)&checksum)+0,1);
        for (j=0;j<2;j++) ll_crccheck(tlm_out_buf[j],&checksum,ll_crctable); // check the checksum
        if (checksum != 0) {
          linklist_err("compress_linklist: invalid checksum generated\n");
        }
        //linklist_info("Compressed checksum result for %s: %d 0x%x\n",name,i,checksum);
      }
      checksum = 0; // reset checksum for next block
    } else { // normal field
      if (tlm_le->tlm == &block_entry) { // data block field
        block_t * theblock = linklist_find_block_by_pointer(ll, tlm_le);
        if (theblock) packetize_block(theblock, tlm_out_buf);
        else linklist_err("Could not find block in linklist \"%s\"\n", ll->name);
      } else if (tlm_le->tlm == &stream_entry) { // data stream field
        stream_t * thestream = linklist_find_stream_by_pointer(ll, tlm_le);
        if (thestream) packetize_stream(thestream, tlm_out_buf);
        else linklist_err("Could not find stream in linklist \"%s\"\n", ll->name);
      } else { // just a normal field 
        tlm_comp_type = tlm_le->comp_type;
        tlm_in_start = tlm_le->tlm->start;
        tlm_in_buf = buffer_in+tlm_in_start;
        if (tlm_comp_type != NO_COMP) { // compression
          (*compRoutine[tlm_comp_type].compressFunc)(tlm_out_buf,tlm_le,tlm_in_buf);
        } else {
          decimationCompress(tlm_out_buf,tlm_le,tlm_in_buf);
        }
      }

      // update checksum
      tlm_out_size = tlm_le->blk_size;
      for (j=0;j<tlm_out_size;j++) {
        ll_crccheck(tlm_out_buf[j],&checksum,ll_crctable);
        retval |= tlm_out_buf[j]; // retval is whether or not there is nonzero data
      }
    }
  }
  return retval;
}


uint8_t * buffer_save = NULL;

int fill_linklist_with_saved(linklist_t * req_ll, unsigned int p_start, unsigned int p_end, uint8_t *buffer_out)
{
  unsigned int i, k;
  struct link_entry * tlm_le = NULL;
  unsigned int tlm_out_start;
  unsigned int loc1, loc2;
  unsigned int tlm_out_num;
  unsigned int tlm_size;
  unsigned int tlm_out_skip;

  for (i=p_start;i<p_end;i++) { 
    tlm_le = &(req_ll->items[i]);
    if (tlm_le->tlm != NULL) { 
      if ((tlm_le->tlm != &block_entry) && (tlm_le->tlm != &stream_entry)) { // do not fill extended items (blocks, streams)
        tlm_out_start = tlm_le->tlm->start;
        tlm_out_skip = tlm_le->tlm->skip;
        tlm_out_num = tlm_le->tlm->spf;
        tlm_size = get_superframe_entry_size(tlm_le->tlm);
        // linklist_info("Fixing %s (start = %d)\n",tlm_le->tlm->field,tlm_out_start);
        for (k=0;k<tlm_out_num;k++) { 
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

/*
 * decompress_linklist_internal
 *
 * An implementation of decompress_linklist that uses an internal buffer to store
 * decompressed data. Decompression only occurs the data id (arg 1) is different
 * from the last time the function is called. This is useful in cases where 
 * multiple routines are decompressing the same linklist so that the same linklist 
 * isn't decompressed multiple times unnecessarily for the same data.
 */
double decompress_linklist_internal(uint64_t id, linklist_t * ll, uint8_t *buffer_in) {
  return decompress_linklist_internal_opt(id, ll, buffer_in, UINT32_MAX, 0);
}

double decompress_linklist_internal_opt(uint64_t id, linklist_t * ll, uint8_t *buffer_in, 
                                        uint32_t maxsize, int flags) {
  // allocate the buffer if necessary
  if (!ll->internal_buffer) {
    ll->internal_buffer = (uint8_t *) calloc(1, ll->superframe->size);
  }

  // only decompress data for new id 
  if (ll->internal_id == id) return 0;
  ll->internal_id = id;

  return decompress_linklist_opt(ll->internal_buffer, ll, buffer_in, maxsize, flags);
}

/**
 * decompress_linklist
 *
 * Uses defaults for decompress_linklist_opt (see below).
 */
double decompress_linklist(uint8_t * buffer_out, linklist_t * ll, uint8_t * buffer_in)
{
  return decompress_linklist_opt(buffer_out, ll, buffer_in, UINT32_MAX, 0);
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

  unsigned int i, j;

  unsigned int tlm_in_start = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_in_size = 0;
  uint8_t * tlm_in_buf, * tlm_out_buf;
  uint8_t tlm_comp_type = 0;
  struct link_entry * tlm_le;
  unsigned int p_start = 0, p_end = 0;
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

  if (buffer_save == NULL) buffer_save = (uint8_t *) calloc(1, superframe->size);

  // extract the data to the full buffer
  for (j=0; j<ll->n_entries; j++) {
    tlm_le = &(ll->items[j]);
    tlm_in_size = tlm_le->blk_size;
    tlm_in_start = tlm_le->start;
    tlm_in_buf = buffer_in+tlm_in_start;

    // update checksum
    for (i=0; i<tlm_in_size; i++) {
      ll_crccheck(tlm_in_buf[i],&checksum,ll_crctable);
      prechecksum |= *(uint16_t *) &tlm_in_buf[i];
    }

    p_end = j;

    if (tlm_le->tlm == NULL) { // found a checksum field
      if (tlm_in_start > maxsize) { // max. input buffer size; the rest assumed to be garbage
        fill_linklist_with_saved(ll, p_start, p_end, buffer_out);
        break;
      } else if (checksum && !(flags & LL_IGNORE_CHECKSUM)) { // bad data block
        // clear/replace bad data from output buffer
        if (flags & LL_VERBOSE) {
          linklist_info("decompress_linklist: checksum failed -> bad data (block %d)\n", sumcount);
        }
        fill_linklist_with_saved(ll, p_start, p_end, buffer_out);
        ret++;
      } else if (!checksum && !prechecksum && !(flags & LL_IGNORE_CHECKSUM)) { // had a block of all zeros
        fill_linklist_with_saved(ll, p_start, p_end, buffer_out);
        if (flags & LL_VERBOSE) {
          linklist_info("decompress_linklist: block of zeros (block %d)\n", sumcount);
        }
      } else { // after all that, the checksum is good
        for (i=p_start; i<p_end; i++) { // decompress everything in that block
          tlm_le = &(ll->items[i]);
          tlm_in_size = tlm_le->blk_size;
          tlm_in_start = tlm_le->start;
          tlm_in_buf = buffer_in+tlm_in_start;

          if (tlm_le->tlm == &block_entry) { // data block entry 
            block_t * theblock = linklist_find_block_by_pointer(ll, tlm_le);
            if (theblock) depacketize_block(theblock, tlm_in_buf);
            else linklist_err("Could not find block in linklist \"%s\"\n", ll->name);
          } else if (tlm_le->tlm == &stream_entry) { // data stream entry 
            stream_t * thestream = linklist_find_stream_by_pointer(ll, tlm_le);
            if (thestream) depacketize_stream(thestream, tlm_in_buf);
            else linklist_err("Could not find stream in linklist \"%s\"\n", ll->name);
          } else { // just a normal field
            tlm_out_start = tlm_le->tlm->start;
            tlm_comp_type = tlm_le->comp_type;
            tlm_out_buf = buffer_out+tlm_out_start;

            if (tlm_comp_type != NO_COMP) { // compression
              (*compRoutine[tlm_comp_type].decompressFunc)(tlm_out_buf, tlm_le, tlm_in_buf);
            } else {
              decimationDecompress(tlm_out_buf, tlm_le, tlm_in_buf);
            }
          }
        }
      }

      // reset checksum
      prechecksum = 0;
      checksum = 0;
      sumcount++;
      p_start = j+1;
      tot++;

    }
  }
  // save the data
  memcpy(buffer_save, buffer_out, superframe->size);
  ret = (tot == 0) ? NAN : (tot-ret)/tot;

  return ret;
}

unsigned int linklist_blocks_queued(linklist_t * ll) {
  unsigned int i;
  unsigned int retval = 0;
  if (!ll) return retval;

  for (i=0; i<ll->num_blocks; i++) {
    if (ll->blocks[i].i < ll->blocks[i].n) retval++;
  }
  return retval;
}

void packetize_block(block_t * block, uint8_t * buffer)
{
  if (block->fp && (block->flags & BLOCK_PREEMPT_FILE)) { // send preempt file data before the rest of the data
    int fsize = make_block_header(buffer, block->id & ~BLOCK_FILE_MASK, 
                                  block->le->blk_size-PACKET_HEADER_SIZE, 0, 1);

    memcpy(buffer+fsize, block->buffer, block->le->blk_size);
    block->flags &= ~BLOCK_PREEMPT_FILE; // clear the flag so next time, actual data can be sent
  } else if (block->i < block->n) { // packets left to be sent 
    unsigned int loc = (block->i*(block->le->blk_size-PACKET_HEADER_SIZE)); // location in data block
    unsigned int cpy = MIN(block->le->blk_size-PACKET_HEADER_SIZE, block->curr_size-loc);

    if (cpy > UINT16_MAX) {
      linklist_err("Block packet size %d is too large\n", cpy);
      return;
    }
    
    // make the header
    int fsize = make_block_header(buffer, block->id, cpy, block->i, block->n);
 
    if (loc > block->curr_size) {
      linklist_err("Block location %d > total size %d", loc, block->curr_size);
      return;
    }

    if (block->fp) { // there is a file instead of data in a buffer
      fseek(block->fp, loc, SEEK_SET); // go to the location in the file
      int retval = 0;
      if ((retval = fread(buffer+fsize, 1, cpy, block->fp)) != (int) cpy) {
        linklist_err("Could only read %d/%d bytes at %d from file %s", retval, cpy, loc, block->filename);
      }
    } else { // there is data in the buffer
      memcpy(buffer+fsize, block->buffer+loc, cpy);
    }

    block->i++;
    block->num++;
  } else { // no blocks left
    memset(buffer, 0, block->le->blk_size);
    if (block->fp) { // file was open, so close it
      close_block_fp(block);
      block->filename[0] = '\0';
    }
  }

}

// handle for openning and appending to files even if they don't exist
FILE * fpreopenb(char *fname)
{
  FILE * temp = fopen(fname,"ab");
  if (!temp) {
    linklist_err("Cannot open %s (errno %d: %s) \n", fname, errno, strerror(errno));
    return NULL;
  }
  fclose(temp);
  return fopen(fname,"rb+");
}

void depacketize_block(block_t * block, uint8_t * buffer)
{
  uint32_t id = 0;
  uint32_t i = 0, n = 0;
  uint16_t blksize = 0;
  static FILE * missed_pkts_fp = NULL;

  int fsize = read_block_header(buffer, &id, &blksize, &i, &n);

  // no data to read
  if (blksize == 0) {
    // close any dangling file pointers
    if (block->fp) {
      close_block_fp(block);
      block->filename[0] = '\0';
    }
    return;
  }
  if (i >= n) { 
    linklist_info("depacketize_block: index larger than total (%d > %d)\n", i, n);
    return;
  }
  // report missing block for already opened files
  if (block->i < i) {
    if (!missed_pkts_fp) {
      missed_pkts_fp = fpreopenb(LINKLIST_MISSED_PKTS_FILE);
      time_t now;
      time(&now);
      fprintf(missed_pkts_fp, "\n--------\n%s--------\n", ctime(&now));
    } 
    if (missed_pkts_fp) {
      fprintf(missed_pkts_fp, "%s %d %d\n", 
              (strlen(block->filename) > 0) ? block->filename : "(missing)", 
              block->i, 
              i+1);
      fflush(missed_pkts_fp);
    }
  }
  // ... and print them to terminal
  while ((block->i < i) && block->fp) {
    linklist_info("Missing block %d/%d\n", block->i+1, n);
    block->i++;
  }
 
  unsigned int loc = i*(block->le->blk_size-fsize);
  unsigned int expected_size = n*(block->le->blk_size-fsize);

  if (id & BLOCK_FILE_MASK) { // receiving a file
    id &= ~BLOCK_FILE_MASK; // remove the mask

    // a different id packet was received, so close file if that packet is open
    if ((id != block->id) || (i < block->i)  || (n != block->n)) {
      close_block_fp(block);
    }
    
    // file not open yet
    if (!block->fp) {
      struct preempt_data * pd = (struct preempt_data *) block->buffer;
      if (pd->serial == BLOCK_PREEMPT_ID) { // auxiliary data has been preempted, so override file defaults
        sprintf(block->filename, "%s/%s", LINKLIST_FILESAVE_DIR, pd->filename); // build filename
        memset(pd, 0, sizeof(struct preempt_data));
      } else { // defaults
        sprintf(block->filename, "%s/%s_%d", LINKLIST_FILESAVE_DIR, block->name, id); // build filename
      }
      if (!(block->fp = fpreopenb(block->filename))) {
        linklist_err("Cannot open file %s", block->filename);
        return;
      }
      linklist_info("File \"%s\" opened\n", block->filename);
      // make symlink to newest opened file
      char symname[LINKLIST_MAX_FILENAME_SIZE] = "";
      snprintf(symname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s.lnk", LINKLIST_FILESAVE_DIR, block->name);
      unlink(symname);
      if (symlink(block->filename, symname) < 0) {
        linklist_err("Unable to form symlink %s -> %s\n", block->filename, symname);
      }
    }
    fseek(block->fp, loc, SEEK_SET); 
    int retval = 0;
    if ((retval = fwrite(buffer+fsize, 1, blksize, block->fp)) != blksize) {
      linklist_err("Wrote %d != %d bytes at %d to file %s", retval, blksize, loc, block->filename);
    }
    
    // got last packet, so close
    if ((i+1) == n) {
      linklist_info("Completed \"%s\"\n\n", block->filename);
      close_block_fp(block);
    }

  } else { // just a normal block to be stored in memory
    // expand the buffer if necessary
    if (expected_size > block->alloc_size) {  
      void * tp = realloc(block->buffer, expected_size);
      block->buffer = (uint8_t *) tp;
      block->alloc_size = expected_size;
    }
    memcpy(block->buffer+loc, buffer+fsize, blksize);
  }

  // set block variables 
  block->curr_size = expected_size;
  block->i = i;
  block->n = n;
  block->id = id;  

  block->i++;
  block->num++;
}

void packetize_stream(stream_t * stream, uint8_t * buffer) {
  unsigned int blk_size = stream->le->blk_size;
  unsigned int cpy = 0;
  unsigned int total = 0;
  substream_t * ss = &stream->buffers[stream->curr];
 
  memset(buffer, 0, blk_size); // always zero out the buffer

  while (total < blk_size) {
    // reached the end of the current buffer, so go to next one
    if (ss->loc >= ss->data_size) { 
      // reset this buffer
      ss->loc = 0; 

      // clear the data
      // if data is not cleared, it will be repeated, unless there is newer data to send
      if (!(ss->flags & STREAM_DONT_CLEAR)) {
        memset(ss->buffer, 0, ss->data_size);
        ss->data_size = 0;
        ss->flags = 0;
      }

      // update current read buffer
      stream->curr = stream->next; // only a reading function can modify stream->curr
      ss = &stream->buffers[stream->curr];

      // this buffer is the current, so it is being sent (don't need to force send again)
      ss->flags &= ~STREAM_MUST_SEND;

      // for file streams, overwrite buffer with file data
      if (stream->fp) {
        memset(ss->buffer, 0, ss->data_size);
        fflush(stream->fp);
        int ret = fread(ss->buffer, 1, ss->alloc_size, stream->fp);
        ss->data_size = (ret > 0) ? ret : 0;
        ss->flags = 0;
      }

      // escape if the next buffer has no data; otherwise continue
      if (!ss->data_size) break;
      continue;
    } 

    // copy from the current buffer
    cpy = MIN(blk_size-total, ss->data_size-ss->loc);
    memcpy(buffer+total, ss->buffer+ss->loc, cpy); // cpy is always non-zero
    // linklist_info("%s", (char *) buffer);

    // increment total bytes and buffer location
    ss->loc += cpy;
    total += cpy;
  }
}

void depacketize_stream(stream_t * stream, uint8_t * buffer) {
  write_next_stream(stream, buffer, stream->le->blk_size, 0);
}

// takes superframe at buffer in and creates an all frame in buffer out
// all frame is 1 sample per field uncompressed
int write_allframe(uint8_t * allframe, superframe_t * superframe, uint8_t * sf) {
  unsigned int i, j;
  int tlm_out_start = 4; // the first 4 bytes are the serial for allframes
  int tlm_in_start = 0; 
  unsigned int tlm_size = 0; 
  uint32_t test = ALLFRAME_SERIAL;
  uint16_t crc = 0;

  int wd = 1;
  if ((allframe == NULL) || (sf == NULL)) wd = 0;

  superframe_entry_t * ll_superframe_list = superframe->entries;

  // allocate crc table if necessary
  if (ll_crctable == NULL)
  {
    if ((ll_crctable = mk_ll_crctable((unsigned short)LL_CRC_POLY,ll_crchware)) == NULL)
    {
      linklist_fatal("mk_ll_crctable() memory allocation failed\n");
      ll_crctable = NULL;
      return -2;
    }
  } 

  if (wd) memcpy(allframe, &test, 4);

  for (i = 0; i < superframe->n_entries; i++) {
    tlm_in_start = ll_superframe_list[i].start;
    tlm_size = get_superframe_entry_size(&ll_superframe_list[i]);

    if (wd) {
      memcpy(allframe+tlm_out_start, sf+tlm_in_start, tlm_size);
      for (j=0;j<tlm_size;j++) ll_crccheck(allframe[tlm_out_start+j],&crc,ll_crctable);
    }
    tlm_out_start += tlm_size;
  }

  if (wd) {
    // write the crc 
    memcpy(allframe+2,((uint8_t*)&crc)+1,1);
    memcpy(allframe+3,((uint8_t*)&crc)+0,1);
  }

  return tlm_out_start;

}

// takes an allframe at buffer in at 1 sample per frame and writes repeated
// samples in the superframe
int read_allframe(uint8_t * sf, superframe_t * superframe, uint8_t * allframe) {
  unsigned int i,j;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_in_start = 4;
  unsigned int tlm_num = 0;
  unsigned int tlm_skip = 0;
  unsigned int tlm_size = 0;
  int check_crc = 1;
  
  superframe_entry_t * ll_superframe_list = superframe->entries;

  // allocate crc table if necessary
  if (ll_crctable == NULL)
  {
    if ((ll_crctable = mk_ll_crctable((unsigned short)LL_CRC_POLY,ll_crchware)) == NULL)
    {
      linklist_fatal("mk_ll_crctable() memory allocation failed\n");
      ll_crctable = NULL;
      return -2;
    }
  } 

  // check to see if this is actually an allframe
  if (*((uint16_t *) allframe) != (uint16_t) ALLFRAME_SERIAL) { 
    return 0;
  }
  if (*((uint32_t *) allframe) == (uint32_t) ALLFRAME_SERIAL) {
    check_crc = 0;
  }

  uint16_t crc = 0;
  
  for (i = 0; i < superframe->n_entries; i++) { 
    tlm_out_start = ll_superframe_list[i].start;
    tlm_size = get_superframe_entry_size(&ll_superframe_list[i]);
    tlm_num = ll_superframe_list[i].spf;
    tlm_skip = ll_superframe_list[i].skip;

    for (j=0;j<tlm_size;j++) ll_crccheck(allframe[tlm_in_start+j],&crc,ll_crctable);
    for (j = 0; j < tlm_num; j++) {
      if (sf) memcpy(sf+tlm_out_start, allframe+tlm_in_start, tlm_size);
      tlm_out_start += tlm_skip;
    }
    tlm_in_start += tlm_size;
  } 

  // check the crc
  ll_crccheck(allframe[2], &crc, ll_crctable);
  ll_crccheck(allframe[3], &crc, ll_crctable);

  if (check_crc && crc)
  {
    linklist_info("Bad all_frame checksum\n");
    // give an all frame that is saved from the previous good buffer
    if (buffer_save && sf) memcpy(sf, buffer_save, superframe->size);
    return -1;
  }

  // set the last good buffer to the allframe
  if (buffer_save && sf) memcpy(buffer_save, sf, superframe->size);
  return (int) tlm_in_start;

}

double antiAlias(uint8_t * data_in, char type, unsigned int num, unsigned int skip,
                   double (*datatodouble)(uint8_t *, uint8_t))
{
  unsigned int i;
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
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  unsigned int i;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd)
  {
    type = '0';
  }

  offset = 0;
  gain = 1; 
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT32_MAX);
  } 

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type ,decim, inputskip, datatodouble);
      temp1 = (temp1-offset)/(gain);
      if (temp1 > UINT32_MAX) temp1 = UINT32_MAX;
      else if (temp1 < 0.0) temp1 = 0.0;
      (*doubletodata)(data_out+blk_size, temp1, SF_UINT32);
    }
    blk_size+=4;
  }

  return blk_size;
}

int stream32bitFixedPtDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  unsigned int i,j;
  int wd = 1;
  int blk_size = 0;
  double gain = 1.0, offset = 0.0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  unsigned int outputnum = le->tlm->spf;
  unsigned int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  offset = 0;
  gain = 1;
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT32_MAX);
  }

  double dataout = 0;
  uint32_t value = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) 
    {
      value = (*datatodouble)(&data_in[i*4], SF_UINT32);
      dataout = ((double) value)*gain+offset;
    }
    //linklist_info("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //linklist_info("\n\n");

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
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  unsigned int i;

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (!wd)
  {
    type = '0';
  }

  offset = 0;
  gain = 1;
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT16_MAX);
  }  

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip), type, decim, inputskip, datatodouble);
      temp1 = (temp1-offset)/(gain);
      if (temp1 > UINT16_MAX) temp1 = UINT16_MAX;
      else if (temp1 < 0.0) temp1 = 0.0;
      (*doubletodata)(data_out+blk_size, temp1, SF_UINT16);
    }
    blk_size+=2;
  }

  return blk_size;
}

int stream16bitFixedPtDecomp(uint8_t * data_out, struct link_entry * le, uint8_t * data_in)
{
  unsigned int i,j;
  int wd = 1;
  int blk_size = 0;
  double gain = 1.0, offset = 0.0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  unsigned int outputnum = le->tlm->spf;
  unsigned int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  offset = 0;
  gain = 1;
  if ((le->tlm->max != 0) || (le->tlm->min != 0))
  {
    offset = le->tlm->min;
    gain = (le->tlm->max-le->tlm->min)/((double) UINT16_MAX);
  }

  double dataout = 0;
  uint16_t value = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) 
    {
      value = (*datatodouble)(&data_in[i*2], SF_UINT16);
      dataout = ((double) value)*gain+offset;
    }
    //linklist_info("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //linklist_info("\n\n");

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
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  unsigned int i;
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
    (*doubletodata)(data_out+0, (double) gain, SF_FLOAT32);
    (*doubletodata)(data_out+4, (double) offset, SF_FLOAT32);
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
  unsigned int i,j;
  int wd = 1;
  int blk_size = 0;
  float gain = 0;
  float offset = 0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  unsigned int outputnum = le->tlm->spf;
  unsigned int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (wd)
  {
    gain = (*datatodouble)(data_in+0, SF_FLOAT32);
    offset = (*datatodouble)(data_in+4, SF_FLOAT32);
  }

  double dataout = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) dataout = ((double) data_in[i+8])*gain+offset;
    //linklist_info("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;
      blk_size += outputsize;
    }
  }
  //linklist_info("\n\n");

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
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 

  unsigned int i;
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
    (*doubletodata)(data_out+0, (double) gain, SF_FLOAT32);
    (*doubletodata)(data_out+4, (double) offset, SF_FLOAT32);
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
  unsigned int i,j;
  int wd = 1;
  int blk_size = 0;
  float gain = 0;
  float offset = 0;

  char type = le->tlm->type;
  unsigned int outputsize = get_superframe_entry_size(le->tlm);
  unsigned int outputskip = le->tlm->skip;
  unsigned int outputnum = le->tlm->spf;
  unsigned int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

  // reference the custom conversion function
  int (*doubletodata)(uint8_t *, double, uint8_t) = le->linklist->superframe->doubletodata; 
  double (*datatodouble)(uint8_t *, uint8_t) = le->linklist->superframe->datatodouble; 

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  if (wd)
  {
    gain = (*datatodouble)(data_in+0, SF_FLOAT32);
    offset = (*datatodouble)(data_in+4, SF_FLOAT32);
  }

  double dataout = 0;

  for (i=0;i<inputnum;i++)
  {
    if (wd) dataout += ((double) data_in[i+8])*gain+offset;
    //linklist_info("%f ",dataout);
    for (j=0;j<decim;j++)
    {
      if (wd) (*doubletodata)(data_out+outputstart,dataout,type);
      outputstart += outputskip;  
      blk_size += outputsize;
    }
  }
  //linklist_info("\n\n");

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
  unsigned int i;
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
  unsigned int i,j;
  int wd = 1;
  int blk_size = 0;

  unsigned int outputskip = le->tlm->skip;
  unsigned int outputnum = le->tlm->spf;
  unsigned int inputnum = le->num;
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
