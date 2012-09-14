/* mcp: the master control program
 *
 * tx.c writes data from mcp to the nios (bbc) to include it in frames
 * 
 * This software is copyright (C) 2002-2011 University of Toronto
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "bbc_pci.h"

#include "channels.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"

#define NIOS_BUFFER_SIZE 100

void ReadWriteAzEl(int index);	// az-el.c

void PhaseControl(void);	//hk.c
void BiasControl(void);

extern int bbc_fp;

extern unsigned int az_enc; // az-el.c
extern unsigned int el_enc; // az_el.c

/* this is provided to let the various controls know that we're doing our
 * initial control writes -- there's no input data yet */
int mcp_initial_controls = 0;

/************************************************************************/
/*                                                                      */
/*  WriteAux: write aux data, like cpu time, temperature                */
/*                                                                      */
/************************************************************************/
static void WriteAux(void)
{
  static struct NiosStruct* timeAddr;
  static struct NiosStruct* timeUSecAddr;
  static struct NiosStruct* diskFreeAddr;
  static struct NiosStruct* bbcFifoSizeAddr;
  static struct NiosStruct* ploverAddr;
  
  struct timeval tv;
  struct timezone tz;
  
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    timeAddr = GetNiosAddr("time");
    timeUSecAddr = GetNiosAddr("time_usec");
    diskFreeAddr = GetNiosAddr("disk_free");
    bbcFifoSizeAddr = GetNiosAddr("bbc_fifo_size");
    ploverAddr = GetNiosAddr("plover");
  }

  gettimeofday(&tv, &tz);

  WriteData(timeAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  WriteData(timeUSecAddr, tv.tv_usec, NIOS_QUEUE);
  WriteData(diskFreeAddr, CommandData.df, NIOS_QUEUE);
  WriteData(bbcFifoSizeAddr, CommandData.bbcFifoSize, NIOS_QUEUE);
  WriteData(ploverAddr, CommandData.plover, NIOS_QUEUE);
}

/*****************************************************************/
/* SyncADC: check to see if any boards need to be synced and     */
/* send the sync bit if they do.  Only one board can be synced   */
/* in each superframe.                                           */
/*****************************************************************/
//list node numbers for sync, which may have gaps
#define NUM_SYNC 7
const unsigned short sync_nums[NUM_SYNC] = {0,1,2,3,4,5,6};
#define REBOOT_TIMEOUT 50 /* 10 sec -- in 5Hz Frames */
static void SyncADC (void)
{
  static struct NiosStruct* syncAddr[NUM_SYNC];
  static struct BiPhaseStruct* statusAddr[NUM_SYNC];
  static int doingSync[NUM_SYNC];
  static unsigned short int serial[NUM_SYNC];
  char buffer[9];

  int k, l, m;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;

    for (k = 0; k < NUM_SYNC; ++k) {
      doingSync[k] = 0;
      serial[k] = 0xeb90;
      l = sync_nums[k];
      sprintf(buffer, "sync%02i", l);
      syncAddr[k] = GetNiosAddr(buffer);
      sprintf(buffer, "status%02i", l);
      statusAddr[k] = GetBiPhaseAddr(buffer);
    }
  }

  for (m = 0; m < NUM_SYNC; ++m) {
    l = sync_nums[m];	    //node number
    k = ReadData(statusAddr[m]);

    if ((k & 0x3) == 0x1 && CommandData.power.adc_reset[l/4] == 0) {
      /* board is up, but needs to be synced */
      if (!doingSync[m])
        bprintf(info, "ADC Sync: node %i asserted\n", l);
      doingSync[m] = BBC_ADC_SYNC | 0x3;
    } else if ((k & 0x3) == 0x3) {
      /* board is up and synced */
      if (doingSync[m] & BBC_ADC_SYNC) {
        bprintf(info, "ADC Sync: node %i deasserted\n", l);
      }
      doingSync[m] = 0x3;
    } else {
      /* board is not yet alive */
      doingSync[m] = 0;
    }

    /* count down reset pulse, if asserted, and force resync */
    if (CommandData.power.adc_reset[l/4] > 0) {
      if (l%4 == 0) CommandData.power.adc_reset[l/4]--;
      doingSync[m] = 0x0;
    }

    /* update the serial if we got a good response last time */
    /* only toggle while not trying to reset card */
    if (CommandData.power.adc_reset[l/4] == 0 && (k & 0xfffc) == serial[m]) {
      if (serial[m] == 0xeb90)
        serial[m] = (~0xeb90) & 0xfffc;
      else
        serial[m] = 0xeb90;
    }

    RawNiosWrite(syncAddr[m]->niosAddr, BBC_WRITE | BBC_NODE(l) | BBC_CH(63)
        | doingSync[m] | (serial[m] & 0xfffc), NIOS_FLUSH);
  }
}

//Most of this file consisted of frame writing functions calling WriteData
//for data needed in frames that was not obtained from the BLAST bus
//All fields written here must have an entry in tx_struct

void InitTxFrame()
{
  int bus, m, i, j, niosAddr, m0addr;

  bprintf(info, "Frame Control: Writing Initial Tx Frame.\n");

  for (bus = 0; bus < 2; ++bus) {
    for (m = 0; m < FAST_PER_SLOW; ++m) {
      for (i = 0; i < TxFrameWords[bus]; ++i) {
        niosAddr = i + bus * BBCPCI_MAX_FRAME_SIZE + m * TxFrameWords[bus];
        m0addr = i + bus * BBCPCI_MAX_FRAME_SIZE;
        if (i == 0) {  /* framesync */
          if (bus)
            RawNiosWrite(niosAddr, BBC_FSYNC | BBC_WRITE | BBC_NODE(63)
                | BBC_CH(4) | 0xB008, NIOS_QUEUE);
          else if (m == 0)
            /* this is address 0 in the NiosFrame; what _should_ go here is the
             * Bus 0 framesync.  Instead we write BBC_ENDWORD which effectively
             * traps Nios here, preventing it from trying to send out our frame
             * while we're constructing it.  Later, once everything is composed,
             * we'll write the framesync here and Nios will start sending out
             * the frame -- we flush this so it takes effect immediately. */
            RawNiosWrite(niosAddr, BBC_ENDWORD, NIOS_FLUSH);
          else
            RawNiosWrite(niosAddr, BBC_FSYNC | BBC_WRITE | BBC_NODE(63)
                | BBC_CH(0) | 0xEB90, NIOS_QUEUE);
        } else if (i == 1 && bus == 0) /* fastsamp lsb */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(1),
              NIOS_QUEUE);
        else if (i == 2 && bus == 0) /* fastsamp msb */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(2),
              NIOS_QUEUE);
        else if (i == 3 && bus == 0) /* multiplex index */
          RawNiosWrite(niosAddr, BBC_WRITE | BBC_NODE(63) | BBC_CH(3) | m,
              NIOS_QUEUE);
        else
          for (j = 0; j < ccTotal; ++j)
            if (NiosLookup[j].niosAddr == niosAddr)
              RawNiosWrite(niosAddr, NiosLookup[j].bbcAddr, NIOS_QUEUE);
            else if (NiosLookup[j].niosAddr == niosAddr - 1
                && NiosLookup[j].wide)
              RawNiosWrite(niosAddr, BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  NIOS_QUEUE);
            else if (NiosLookup[j].fast && NiosLookup[j].niosAddr == m0addr)
              RawNiosWrite(niosAddr, NiosLookup[j].bbcAddr, NIOS_QUEUE);
            else if (NiosLookup[j].fast && NiosLookup[j].niosAddr
                == m0addr - 1 && NiosLookup[j].wide)
              RawNiosWrite(niosAddr, BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  NIOS_QUEUE);

        for (j = 0; j < 2 * FAST_PER_SLOW; ++j)
          if (NiosSpares[j] == niosAddr)
            RawNiosWrite(niosAddr, BBCSpares[j], NIOS_QUEUE);
      }
    }
  }

  /* force flush of write buffer */
  RawNiosWrite(-1, -1, NIOS_FLUSH);

  /* do initial controls */
  bprintf(info, "Frame Control: Running Initial Controls.\n");
  mcp_initial_controls = 1;
  UpdateBBCFrame();
  mcp_initial_controls = 0;

  /* write the framesync to address 0 to get things going... */
  bprintf(info, "Frame Control: Frame Composition Complete.  Starting NIOS.\n");
  RawNiosWrite(0, BBC_FSYNC | BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | 0xEB90,
      NIOS_FLUSH);
}

void RawNiosWrite(unsigned int addr, unsigned int data, int flush_flag)
{
  int n;
  static int counter = 0;
  static unsigned int niosData[2 * NIOS_BUFFER_SIZE];

  if (addr != -1) {
    niosData[counter++] = addr;
    niosData[counter++] = data;
  }

  if (flush_flag || counter == 2 * NIOS_BUFFER_SIZE) {
    n = write(bbc_fp, niosData, counter * sizeof(unsigned int));
    if (n < counter * sizeof(unsigned int)) {
      bprintf(warning, "Frame Control: Short write to Nios.");
    } 
    counter = 0;
  }
}

/* write to the nios (bbc) */
void WriteData(struct NiosStruct* addr, unsigned int data, int flush_flag)
{
  int i;

  if (addr->fast)
    for (i = 0; i < FAST_PER_SLOW; ++i) {
      RawNiosWrite(addr->niosAddr + i * TxFrameWords[addr->bus],
          addr->bbcAddr | (data & 0xffff),
          flush_flag && !addr->wide && i == FAST_PER_SLOW - 1);
      if (addr->wide)
        RawNiosWrite(addr->niosAddr + 1 + i * TxFrameWords[addr->bus],
            BBC_NEXT_CHANNEL(addr->bbcAddr) | (data >> 16),
            flush_flag && i == FAST_PER_SLOW - 1);
    }
  else {
    /* slow data */
    RawNiosWrite(addr->niosAddr, addr->bbcAddr | (data & 0xffff),
        flush_flag && !addr->wide);
    if (addr->wide)
      RawNiosWrite(addr->niosAddr + 1,
          BBC_NEXT_CHANNEL(addr->bbcAddr) | (data >> 16), flush_flag);
  }
}

//TODO convert suitable WriteData calls to WriteCalData
void WriteCalData(struct NiosStruct* addr, unsigned int data, int flush_flag)
{
  WriteData(addr, (unsigned int)(((double)data - addr->b)/addr->m), flush_flag);
}

unsigned int ReadData(struct BiPhaseStruct* addr)
{
  unsigned int result;
  if (addr->nios->fast) {
    result = RxFrame[addr->channel];
    if (addr->nios->wide)
      result |= (RxFrame[addr->channel+1] << 16);
  } else {
    result = slow_data[addr->index][addr->channel];
    if (addr->nios->wide)
      result |= (slow_data[addr->index][addr->channel+1] << 16);
  }
  return result;
}

//TODO convert slow_data and RxFrame accesses to ReadData or ReadCalData
double ReadCalData(struct BiPhaseStruct* addr)
{
  if (addr->nios->sign) {   //signed
    return ((double)(int)ReadData(addr) * addr->nios->m + addr->nios->b);
  } else {		    //unsigned
    return ((double)ReadData(addr) * addr->nios->m + addr->nios->b);
  }
}

/* called from mcp, should call all nios writing functions */
void UpdateBBCFrame()
{
  static int index = 0;
 
  /*** do fast Controls ***/

  ReadWriteAzEl(index);

  /*** do slow Controls ***/
  if (index == 0) {
    if (!mcp_initial_controls) SyncADC();
    WriteAux();
    PhaseControl();
    BiasControl();
  }

  if (!mcp_initial_controls) index = (index + 1) % FAST_PER_SLOW;

  //make sure frame is flushed
  RawNiosWrite(-1,-1,NIOS_FLUSH);
}
