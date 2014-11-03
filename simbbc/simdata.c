/* simbbc: kernel driver simulating the BLAST Bus and controller
 *
 * simbbc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * simbbc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with bbc_pci; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA 
 * Or visit http://www.gnu.org/licenses/
 *
 * Written by Enzo Pascale, Sept. 2 2009
 * Thoroughly hacked by University of Toronto, 2011
 */

#include "bbc_pci.h"
#include "simdata.h"

static struct {
  unsigned framecounter;
  unsigned bbc_frame[2 * BBCPCI_MAX_FRAME_SIZE];
  unsigned *bb1_frame;
  unsigned *bb1_ptr;
  unsigned short bb1_data[BI0_TABLE_SIZE];

  unsigned *bb2_frame;
  unsigned *bb2_ptr;
  unsigned short bb2_data[BI0_TABLE_SIZE];
  
} bbc_data;


void InitBBCData(void)
{
  int k;
  bbc_data.bb1_frame = bbc_data.bbc_frame;
  bbc_data.bb2_frame = bbc_data.bbc_frame + BBCPCI_MAX_FRAME_SIZE;
  bbc_data.bb1_ptr = bbc_data.bb1_frame;
  bbc_data.bb2_ptr = bbc_data.bb2_frame;
  bbc_data.framecounter = 0;
  
  for (k = 0; k < 2 * BBCPCI_MAX_FRAME_SIZE; k++) {
    bbc_data.bbc_frame[k] = 0;
  }

  for (k = 0; k < BI0_TABLE_SIZE; k++) {
    bbc_data.bb1_data[k] = 0;
    bbc_data.bb2_data[k] = 0;
  }
}

void InitAuxData()
{
  //for constant but non-zero reads from bus, populate here
  bbc_data.bb1_data[BI0_MAGIC(GYRO1)] = GYRO_DATA;
  bbc_data.bb1_data[BI0_MAGIC(GYRO2)] = GYRO_DATA;
  bbc_data.bb1_data[BI0_MAGIC(GYRO3)] = GYRO_DATA;
}

unsigned int GetFrameCount()
{
  return bbc_data.framecounter;
}

void WriteToFrame(unsigned addr, unsigned data)
{
  if (addr > BBCPCI_MAX_FRAME_SIZE)
    bbc_data.bb2_frame[addr - BBCPCI_MAX_FRAME_SIZE] = data;
  else
    bbc_data.bb1_frame[addr] = data;
}

int GetFrameNextWord(unsigned* out_data)
{
  unsigned data;
  static unsigned wordcount = 0;
  static int bus = 0;

  data = bus ? *bbc_data.bb2_ptr : *bbc_data.bb1_ptr;

  //force serial number into first two words
  if (wordcount == 1) {
    data = (data & 0xffff0000) | BBC_WRITE
      | (bbc_data.framecounter & 0x0000ffff);
  } else if (wordcount == 2) {
    data = (data & 0xffff0000) | BBC_WRITE
      | ((bbc_data.framecounter >> 16) & 0x0000ffff);
  }

  //update the bb[12]_data tables on writes, make response to reads
  out_data[0] = data;
  if (wordcount < 3) {	//fsync and serial number
    out_data[1] = 0;
  } else if (data & BBC_WRITE) {
    if (bus)
      bbc_data.bb2_data[BI0_MAGIC(data)] = BBC_DATA(data);
    else
      bbc_data.bb1_data[BI0_MAGIC(data)] = BBC_DATA(data);
    out_data[1] = 0;
  } else if (data & BBC_READ) {
    if (bus)
      out_data[1] = (data & 0xffff0000) | BBC_WRITE
        | (bbc_data.bb2_data[BI0_MAGIC(data)] & 0x0000ffff);
    else
      out_data[1] = (data & 0xffff0000) | BBC_WRITE
        | (bbc_data.bb1_data[BI0_MAGIC(data)] & 0x0000ffff);
  }

  //move to next word, and return done if on an FSYNC
  wordcount++;
  if (bus) {
    bbc_data.bb2_ptr++;
    if (*bbc_data.bb2_ptr == BBC_ENDWORD)
      bbc_data.bb2_ptr = bbc_data.bb2_frame;

    if (*bbc_data.bb2_ptr & BBC_FSYNC) {
      bus = 0;
      wordcount = 0;
      return 1;
    }
  } else {
    bbc_data.bb1_ptr++;
    if (*bbc_data.bb1_ptr == BBC_ENDWORD)
      bbc_data.bb1_ptr = bbc_data.bb1_frame;

    if (*bbc_data.bb1_ptr & BBC_FSYNC) {
      bus = 1;
      return 0;
    }
  }

  return 0;
}

void FrameSyncLogic(void);
int StarcamTriggerLogic(void);

/* frame synchronous logic, and digital outputs via parallel port */
void HandleFrameLogic()
{
  unsigned char bout;
  bbc_data.framecounter++;

  FrameSyncLogic();
  bout = StarcamTriggerLogic();

#if PARALLEL_BASE
  outb(bout, PARALLEL_BASE);
  wmb();
#endif
}

void FrameSyncLogic()
{
  //perform the adc_sync handshake with every node
  int i;
  for (i=0; i<64; i++) {
    bbc_data.bb1_data[BI0_MAGIC(BBC_NODE(i) | BBC_STAT_CH)] =
      bbc_data.bb1_data[BI0_MAGIC(BBC_NODE(i) | BBC_SYNC_CH)] | 0x1;
    bbc_data.bb2_data[BI0_MAGIC(BBC_NODE(i) | BBC_STAT_CH)] =
      bbc_data.bb2_data[BI0_MAGIC(BBC_NODE(i) | BBC_SYNC_CH)] | 0x1;
  }
}

int StarcamTriggerLogic()
{
  unsigned char bout = 0;
  unsigned short trig;
  static unsigned int cam0_counter=0, cam1_counter=0;
  static unsigned short cam0_index=0, cam1_index=0;

  //read trigger fields to chech for new pulses
  trig = bbc_data.bb1_data[BI0_MAGIC(CAM0_TRIGGER)];
  if ((trig & 0xc000) != cam0_index) {
    cam0_index = trig & 0xc000;
    cam0_counter = trig & 0x3fff;
    if (cam0_counter & 1) cam0_counter++;
  }
  trig = bbc_data.bb1_data[BI0_MAGIC(CAM1_TRIGGER)];
  if ((trig & 0xc000) != cam1_index) {
    cam1_index = trig & 0xc000;
    cam1_counter = trig & 0x3fff;
    if (cam1_counter & 1) cam1_counter++;
  }

  //pulse logic
  if(cam0_counter>0) {
    cam0_counter--;
    bout |= 0x1;
  }
  if(cam1_counter>0) {
    cam1_counter--;
    bout |= 0x2;
  }

  //write pulses to bus
#ifdef BLAST
  bbc_data.bb1_data[BI0_MAGIC(CAM0_PULSE)] = (bout & 0x1) ? 1 : 0;
  bbc_data.bb1_data[BI0_MAGIC(CAM1_PULSE)] = (bout & 0x2) ? 1 : 0;
#else  //BLASTPOL
  bbc_data.bb1_data[BI0_MAGIC(CAM_PULSES)] = bout;
#endif

  bout = ~bout & 0x03;	    //parallel port with switches inverts logic?
  return bout;
}
