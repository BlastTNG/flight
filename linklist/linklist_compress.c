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

int decimationCompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);
int decimationDecompress(uint8_t * data_out, struct link_entry * le, uint8_t * data_in);

superframe_t mainframe = {0};

// allocates the 1 Hz superframe needed for linklist compression
uint32_t allocate_superframe(const channel_t * const m_channel_list)
{
  uint32_t byte_loc = 0;
  const channel_t *channel;

  int n_channels = 1024;
  int i = 0;
  uint32_t blk_size = 0;

  mainframe.byte_map = calloc(sizeof(uint32_t), n_channels);

  for (channel = m_channel_list; channel->field[0]; channel++) 
  {
    if (i > n_channels) mainframe.byte_map = realloc(mainframe.byte_map,n_channels += 32);

    mainframe.byte_map[i] = byte_loc;
    blk_size = get_channel_size(channel)*get_channel_spf(channel);
    byte_loc += blk_size;
    i++;
  }

  blast_info("%d bytes allocated for superframe\n", byte_loc);
  mainframe.data = calloc(1,byte_loc);

  return byte_loc;
}

uint32_t get_channel_start_in_superframe(const channel_t * chan)
{
  if (!mainframe.byte_map)
  {
    blast_err("get_channel_start_in_superframe: superframe is not allocated. Fix!");
    return 0;
  }
  else if (chan < channel_list)
  {
    blast_err("get_channel_start_in_superframe: channel is not in channel_list");
    return 0;
  }
  return mainframe.byte_map[((long unsigned int) (chan-channel_list))];
}

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
    case TYPE_DOUBLE : return *((double *) data);
    case TYPE_FLOAT : return *((float *) data);
    case TYPE_INT16 : return *((int16_t *) data);
    case TYPE_UINT16 : return *((uint16_t *) data);
    case TYPE_INT32 : return *((int32_t *) data);
    case TYPE_UINT32 : return *((uint32_t *) data);
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
    double d = dub;
    memcpy(data,&d,8);
    return 8;
  }
  else if (type == TYPE_FLOAT)
  {
    float f = dub;
    memcpy(data,&f,4);
    return 4;
  }
  else if (type == TYPE_INT16)
  {
    int16_t s = dub;
    memcpy(data,&s,2);
    return 2;
  }
  else if (type == TYPE_UINT16)
  {
    uint16_t u = dub;
    memcpy(data,&u,2);
    return 2;
  }
  else if (type == TYPE_INT32)
  {
    int32_t i = dub;
    memcpy(data,&i,4);
    return 4;
  }
  else if (type == TYPE_UINT32)
  {
    uint32_t i = dub;
    memcpy(data,&i,4);
    return 4;
  }
  else if (type == TYPE_INT8)
  {
    int8_t u = dub;
    memcpy(data,&u,1);
    return 1;
  }
  else if (type == TYPE_UINT8)
  {
    uint8_t u = dub;
    memcpy(data,&u,1);
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

  for (i=0;i<num;i++) halfsum += datatodouble(data_in+i*skip,type);

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
  unsigned int inputskip = get_channel_size(le->tlm);
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
  unsigned int outputskip = get_channel_size(le->tlm);
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
  unsigned int inputskip = get_channel_size(le->tlm);
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
  unsigned int outputskip = get_channel_size(le->tlm);
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
  unsigned int inputskip = get_channel_size(le->tlm);
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
  unsigned int outputskip = get_channel_size(le->tlm);
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
  unsigned int inputskip = get_channel_size(le->tlm);
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
  unsigned int outputskip = get_channel_size(le->tlm);
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
  unsigned int inputskip = get_channel_size(le->tlm);
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

  unsigned int outputskip = get_channel_size(le->tlm);
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
