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
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>

#include "blast.h"
#include "channels_tng.h"
#include "channel_macros.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "linklist_crc.h"

#ifdef __cplusplus

extern "C"{

#endif

int decimationCompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);
int decimationDecompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);

uint32_t superframe_offset[RATE_END] = {0};
uint32_t superframe_skip[RATE_END] = {0};
uint32_t superframe_size = 0;

void define_superframe()
{
  int rate = 0;
  for (rate = 0; rate<RATE_END; rate++) 
  {
    superframe_offset[rate] = superframe_size;
    superframe_skip[rate] = frame_size[rate];
    superframe_size += frame_size[rate]*get_spf(rate);
  }
  blast_info("Superframe skip and offsets allocated\n");
  
}

// allocates the 1 Hz superframe needed for linklist compression
uint8_t * allocate_superframe()
{
  // allocate data and superframe offsets
  if (!superframe_size) define_superframe();

  uint8_t * ptr = calloc(1,superframe_size);

  return ptr;
}

uint32_t get_channel_start_in_superframe(const channel_t * chan)
{
  if (!superframe_size) define_superframe();

  if (channel_data[chan->rate] > chan->var)
  {
    blast_err("get_channel_start_in_superframe: channel is not in channel_list");
    return 0;
  }
  return ((long unsigned int) (chan->var-channel_data[chan->rate])) + superframe_offset[chan->rate];
}

uint32_t get_channel_skip_in_superframe(const channel_t * chan)
{
  if (!superframe_size) define_superframe();

  if (channel_data[chan->rate] > chan->var)
  {
    blast_err("get_channel_start_in_superframe: channel is not in channel_list");
    return 0;
  }
  return superframe_skip[chan->rate];
}

/**
 * add_frame_to_superframe
 * 
 * Takes a BLAST frame at a given rate and copies it to the superframe.
 * -> frame: BLAST frame to be copied to the superframe
 * -> rate: the rate type for the BLAST frame
 */
unsigned int add_frame_to_superframe(void * frame, E_RATE rate, void * superframe)
{
  static unsigned int frame_location[RATE_END] = {0};
  if (!superframe)
	{
    blast_err("Superframe is not allocated. Fix!");
    return 0;
  }
  if (!frame)
  {
    blast_err("Frame pointer is NULL. Fix!");
    return 0;
  }
  
  // clear the frame if wrapping has occurred (ensures no split data)
  if (frame_location[rate] == 0)
  {
    memset(superframe+superframe_offset[rate],0,frame_size[rate]*get_spf(rate));
  }

  // copy the frame to the superframe
  memcpy(superframe+superframe_offset[rate]+superframe_skip[rate]*frame_location[rate],frame,frame_size[rate]);

  // update the frame location
  frame_location[rate] = (frame_location[rate]+1)%get_spf(rate);

  // return the next frame location in the superframe 
  // (0 indicates that frame will wrap on next function call)
  return frame_location[rate];
}

/**
 * extract_frame_from_superframe
 * 
 * Extracts a BLAST frame at a given rate from superframe and copies it to a given buffer.
 * -> frame: BLAST frame to be copied from the superframe
 * -> rate: the rate type for the BLAST frame
 */
unsigned int extract_frame_from_superframe(void * frame, E_RATE rate, void * superframe)
{
  static unsigned int frame_location[RATE_END] = {0};
  if (!superframe)
	{
    blast_err("Superframe is not allocated. Fix!");
    return 0;
  }
  if (!frame)
  {
    blast_err("Frame pointer is NULL. Fix!");
    return 0;
  }

  // copy the frame from the superframe
  memcpy(frame,superframe+superframe_offset[rate]+superframe_skip[rate]*frame_location[rate],frame_size[rate]);

  // update the frame location
  frame_location[rate] = (frame_location[rate]+1)%get_spf(rate);

  // return the next frame location in the superframe 
  // (0 indicates that frame will wrap on next function call)
  return frame_location[rate];
}

/**
 * compress_linklist
 * 
 * Selects channels from and compresses a superframe according to the provide
 * linklist format.
 * -> buffer_out: buffer in which compressed frame will be written
 * -> ll: pointer to linklist specifying compression and channel selection
 * -> buffer_in: pointer to the superframe to be compressed
 */
int compress_linklist(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in)
{
  int i,j;

  unsigned int tlm_in_start = 0;
  unsigned int tlm_out_start = 0;
  unsigned int tlm_out_size = 0;
  uint8_t * tlm_in_buf, * tlm_out_buf;
  uint8_t tlm_comp_type = 0;
  struct link_entry * tlm_le;
  uint16_t checksum = 0;

  for (i=0;i<ll->n_entries;i++)
  {
    tlm_le = &(ll->items[i]);
    tlm_out_start = tlm_le->start;
    tlm_out_buf = buffer_out+tlm_out_start;

    if (tlm_le->tlm == NULL) // checksum field
    {
      //if (!tlm_no_checksum) // TODO: OPTION FOR IGNORING CHECKSUMS
      {
        memcpy(tlm_out_buf+0,((uint8_t*)&checksum)+1,1);
        memcpy(tlm_out_buf+1,((uint8_t*)&checksum)+0,1);
        for (j=0;j<2;j++) crccheck(tlm_out_buf[j],&checksum,crctable); // check the checksum
        if (checksum != 0) 
        {
          printf("compress_linklist: invalid checksum generated\n");
        }
        //printf("Compressed checksum result for %s: %d 0x%x\n",name,i,checksum);
      }
      checksum = 0; // reset checksum for next block
    }
    else // normal field
    {
      tlm_out_size = tlm_le->blk_size;
      tlm_in_start = get_channel_start_in_superframe(tlm_le->tlm);
      tlm_comp_type = tlm_le->comp_type;
      tlm_in_buf = buffer_in+tlm_in_start;

/* TODO: BLOCKS
      if (tlm_le->tlm->type == 'B') // block types for images
      {
        packetize_block(downlinklist[linkfile_ind],tlm_le->tlm->name,tlm_out_buf);
        //printf("intname = %d\n",*(uint16_t *) tlm_out_buf);
      }
      else 
*/
      if (tlm_comp_type != NO_COMP) // compression
      {
        (*compressFunc[tlm_comp_type])(tlm_out_buf,tlm_le,tlm_in_buf);
      }
      else
      {
        decimationCompress(tlm_out_buf,tlm_le,tlm_in_buf);
      }

      // update checksum
      for (j=0;j<tlm_out_size;j++) crccheck(tlm_out_buf[j],&checksum,crctable);
    }
  }
  return 1;
}


/* TODO: PERSISTENT DATA
uint8_t * buffer_save = NULL;

int fill_linklist_with_saved(struct link_list * req_ll, int p_start, int p_end, uint8_t *buffer_out)
{
	int i, k;
	struct link_entry * tlm_le = NULL;
	unsigned int tlm_out_start;
	unsigned int loc1, loc2;

  for (i=p_start;i<p_end;i++)
  {
    tlm_le = &(req_ll->items[i]);
		if (tlm_le->tlm != NULL)
		{
			if (tlm_le->tlm->type != 'B')
			{
				tlm_out_start = tlm_le->tlm->start;
				//printf("Fixing %s (start = %d)\n",tlm_le->tlm->name,tlm_out_start);
				for (k=0;k<tlm_le->tlm->num;k++)
				{
					loc1 = tlm_le->tlm->skip*k;
					loc2 = tlm_le->tlm->skip*(tlm_le->tlm->num-1);
					memcpy(buffer_out+tlm_out_start+loc1,buffer_save+tlm_out_start+loc2,tlm_le->tlm->size);
				}
			}
    	//memset(buffer_out+tlm_le->tlm->start,0,tlm_le->tlm->size*tlm_le->tlm->num);
		}
  }
	return i;
}
*/

/**
 * decompress_linklist
 * 
 * Selects channels from and compresses a superframe according to the provide
 * linklist format.
 * -> buffer_out: buffer in which decompressed superframe will be written
 * -> ll: pointer to linklist specifying compression and channel selection
 * -> buffer_in: pointer to the compressed frame to be decompressed
 */
double decompress_linklist(uint8_t *buffer_out, linklist_t * ll, uint8_t *buffer_in)
{
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

  /* TODO PERSISENT DATA
	if (buffer_save == NULL) buffer_save = calloc(1,frame_size*2+all_frame_size);
	//memset(buffer_out,0,frame_size);
  */

  // extract the data to the full buffer
  for (j=0;j<ll->n_entries;j++)
  {
    tlm_le = &(ll->items[j]);
    tlm_in_size = tlm_le->blk_size;
    tlm_in_start = tlm_le->start;
    tlm_in_buf = buffer_in+tlm_in_start;

    // update checksum
    for (i=0;i<tlm_in_size;i++) crccheck(tlm_in_buf[i],&checksum,crctable);

    p_end = j;

    if (tlm_le->tlm == NULL) // evaluating a checksum...
    {
      if ((checksum != 0)) // TODO: OPTION FOR IGNORING CHECKSUM && !tlm_no_checksum) // bad data block
      {
        // clear/replace bad data from output buffer
        printf("decompress_linklist: checksum failed -> bad data (block %d)\n",sumcount);
/* TODO: PERSISTENT DATA
        fill_linklist_with_saved(req_ll,p_start,p_end,buffer_out);
*/
      }
			else ret++;
      //if (!tlm_no_checksum) printf("Checksum result: 0x%x\n",checksum);
      // reset checksum
			prechecksum |= *(uint16_t *) tlm_in_buf;
      checksum = 0;
			sumcount++;
      p_start = j+1;
			tot++;

    }
    else
    {
      tlm_out_start = get_channel_start_in_superframe(tlm_le->tlm);
      tlm_comp_type = tlm_le->comp_type;
      tlm_out_buf = buffer_out+tlm_out_start;

/* TODO BLOCKS
      if (tlm_le->tlm->type == 'B') // block types for images
      {
        depacketize_block(downlinklist[linkfile_ind],tlm_le->tlm->name,tlm_in_buf);
        //printf("intname = %d\n",*(uint16_t *) tlm_out_buf);
      }
      else 
*/
      if (tlm_comp_type != NO_COMP) // compression
      {
        (*decompressFunc[tlm_comp_type])(tlm_out_buf,tlm_le,tlm_in_buf);
      }
      else
      {
        decimationDecompress(tlm_out_buf,tlm_le,tlm_in_buf);
      }
    }
  }

/* TODO: PERSISTENT DATA
	if (!prechecksum) // all the checksums were zero; must be blank frame
	{
		fill_linklist_with_saved(req_ll,0,req_ll->n_entries,buffer_out);
		ret = 0;
	}

	// save the data
	memcpy(buffer_save,buffer_out,frame_size);
*/

	ret = (tot == 0) ? ret : ret/tot;

  return ret;
}

double datatodouble(uint8_t * data, char type)
{
  switch (type) 
  {
    case TYPE_DOUBLE : return bedtoh(*((double *) data));
    case TYPE_FLOAT : return beftoh(*((float *) data));
    case TYPE_INT16 : return (int16_t) be16toh(*((int16_t *) data)); // TODO: DOUBLE CHECK IS THIS A TYPO WITH GET_INT16!!!
    case TYPE_UINT16 : return be16toh(*((uint16_t *) data));
    case TYPE_INT32 : return (int32_t) be32toh(*((int32_t *) data));
    case TYPE_UINT32 : return be32toh(*((uint32_t *) data));
    case TYPE_INT8 : return *((int8_t *) data);
    case TYPE_UINT8 : return *((uint8_t *) data);
    default : return 0;
  }
  return 0;
}
int doubletodata(uint8_t * data, double dub, char type)
{
  if (type == TYPE_DOUBLE)
  {
    htobed(dub,*(uint64_t*) data);
    return 8;
  }
  else if (type == TYPE_FLOAT)
  {
    htobef(dub,*(uint32_t*) data)
    return 4;
  }
  else if (type == TYPE_INT16)
  {
    int16_t s = dub;
    *(int16_t*) data = htobe16(s);
    return 2;
  }
  else if (type == TYPE_UINT16)
  {
    uint16_t u = dub;
    *(uint16_t*) data = htobe16(u);
    return 2;
  }
  else if (type == TYPE_INT32)
  {
    int32_t i = dub;
    *(int32_t*) data = htobe32(i);
    return 4;
  }
  else if (type == TYPE_UINT32)
  {
    uint32_t i = dub;
    *(uint32_t*) data = htobe32(i);
    return 4;
  }
  else if (type == TYPE_INT8)
  {
    *(int8_t*) data = dub;
    return 1;
  }
  else if (type == TYPE_UINT8)
  {
    *(uint8_t*) data = dub;
    return 1;
  }
  return 0;
}

double antiAlias(uint8_t * data_in, char type, unsigned int num, unsigned int skip, double * store)
{
  int i;
  double halfsum = 0;
  double ret = 0;  

  if (num == 1) return datatodouble(data_in,type);

  for (i=0;i<num;i++) 
  {
    halfsum += datatodouble(data_in+i*skip,type); 
  }
  //ret = (halfsum+(*store))/2.0/num;
  //*store = halfsum;
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
  //unsigned int inputsize = get_channel_size(le->tlm);
  unsigned int inputskip = get_channel_skip_in_superframe(le->tlm);
  unsigned int inputnum = get_channel_spf(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

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
      //temp1 = datatodouble(data_in+(i*decim*inputskip),type);
      temp1 = antiAlias(data_in+(i*decim*inputskip),type,decim,inputskip,&le->compvars[0]);
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
  unsigned int outputsize = get_channel_size(le->tlm);
  unsigned int outputskip = get_channel_skip_in_superframe(le->tlm);
  int outputnum = get_channel_spf(le->tlm);
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

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
      if (wd) doubletodata(data_out+outputstart,dataout,type);
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
  //unsigned int inputsize = get_channel_size(le->tlm);
  unsigned int inputskip = get_channel_skip_in_superframe(le->tlm);
  unsigned int inputnum = get_channel_spf(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

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
    gain = (le->tlm->max-le->tlm->min)/((double) UINT16_MAX);
  }  
*/
  offset = 0;
  gain = 1;

  for (i=0;i<outputnum;i++)
  {
    if (wd)
    {
      temp1 = antiAlias(data_in+(i*decim*inputskip),type,decim,inputskip,&le->compvars[0]);
      //temp1 = datatodouble(data_in+(i*decim*inputskip),type);
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
  unsigned int outputsize = get_channel_size(le->tlm);
  unsigned int outputskip = get_channel_skip_in_superframe(le->tlm);
  int outputnum = get_channel_spf(le->tlm);
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

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
      if (wd) doubletodata(data_out+outputstart,dataout,type);
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
  //unsigned int inputsize = get_channel_size(le->tlm);
  unsigned int inputskip = get_channel_skip_in_superframe(le->tlm);
  unsigned int inputnum = get_channel_spf(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;

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
      temp1 = antiAlias(data_in+(i*inputskip),type,decim,inputskip,&le->compvars[0]);
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
      //temp1 = datatodouble(data_in+(i*decim*inputskip),type);
      temp1 = antiAlias(data_in+(i*decim*inputskip),type,decim,inputskip,&le->compvars[0]);
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
  unsigned int outputsize = get_channel_size(le->tlm);
  unsigned int outputskip = get_channel_skip_in_superframe(le->tlm);
  int outputnum = get_channel_spf(le->tlm);
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

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
      if (wd) doubletodata(data_out+outputstart,dataout,type);
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
  //unsigned int inputsize = get_channel_size(le->tlm);
  unsigned int inputskip = get_channel_skip_in_superframe(le->tlm);
  unsigned int inputnum = get_channel_spf(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  if (decim == 0) return 0;  

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
      temp2 = antiAlias(data_in+(i*inputskip),type,decim,inputskip,&le->compvars[0]);
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
      //temp2 = datatodouble(data_in+(i*decim*inputskip),type);
      temp2 = antiAlias(data_in+(i*decim*inputskip),type,decim,inputskip,&le->compvars[0]);
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
  unsigned int outputsize = get_channel_size(le->tlm);
  unsigned int outputskip = get_channel_skip_in_superframe(le->tlm);
  int outputnum = get_channel_spf(le->tlm);
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;
  unsigned int outputstart = 0;

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
      if (wd) doubletodata(data_out+outputstart,dataout,type);
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

  //unsigned int inputsize = get_channel_size(le->tlm);
  unsigned int inputskip = get_channel_skip_in_superframe(le->tlm);
  unsigned int inputnum = get_channel_spf(le->tlm);
  unsigned int outputnum = le->num;
  unsigned int decim = inputnum/outputnum;
  unsigned int size = get_channel_size(le->tlm);  


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
      temp1 = antiAlias(data_in+(i*decim*inputskip),type,decim,inputskip,&le->compvars[0]);
      doubletodata((uint8_t *) &dout,temp1,type);
      memcpy(data_out+(i*inputskip),&dout,size);
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

  unsigned int outputskip = get_channel_skip_in_superframe(le->tlm);
  int outputnum = get_channel_spf(le->tlm);
  int inputnum = le->num;
  unsigned int decim = outputnum/inputnum;

  unsigned int size = get_channel_size(le->tlm);

  if ((data_out == NULL) || (data_in == NULL)) wd = 0; // don't read/write data

  for (i=0;i<inputnum;i++)
  {
    for (j=0;j<decim;j++)
    {
      if (wd) 
      {
        memcpy(data_out+((i*decim+j)*outputskip),data_in+(i*outputskip),size);
      }
      blk_size += size;
    }
  }

  return blk_size;

}

int (*compressFunc[]) (uint8_t *, struct link_entry *, uint8_t *) = {
  // ** THESE TWO FUNCTIONS MUST BE FIRSt **/
  stream16bitFixedPtComp,   stream32bitFixedPtComp,   
  stream8bitDeltaComp,     stream8bitComp,
  NULL    
};

int (*decompressFunc[]) (uint8_t *, struct link_entry *, uint8_t *) = {
  // ** THESE TWO FUNCTIONS MUST BE FIRSt **/
  stream16bitFixedPtDecomp,  stream32bitFixedPtDecomp,  
  stream8bitDeltaDecomp,   stream8bitDecomp,
  NULL
};

#ifdef __cplusplus

}

#endif
