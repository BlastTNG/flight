/*
 * mcplib:
 * Interface to communicate with mcp and BLAST bus
 *
 * This software is copyright (C) 2007 University of Toronto
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

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include "channels.h"
#include "tx.h"

//external declarations
void* startmcp(void*);   //mcp.c
void  closemcp();
extern unsigned int BBFrameIndex;
extern FILE* mcp_outfile;
extern int mcp_use_framefile;

//globals
struct BBOutStruct {
  unsigned char digital[6];         //one entry per digital group on card
  unsigned short dac[32];	    //16-bit SPI DAC bus output
};

struct BBInStruct {
  unsigned int analog[50];          //analog inputs
  unsigned char digital[6];         //one entry per group
};

static unsigned short i_bb;                 //read index in input buffer
static unsigned short i_bbout;              //write index in output buffer
static struct BBInStruct BBInData[3][3];    //buffered data from bbc
static struct BBOutStruct BBOutData[3][3];  //buffered data to write to bbc
static struct BBInStruct BBInCurr[3];       //snapshot of input buffer
static struct BBOutStruct BBOutCurr[3];     //snapshot of output buffer

static pthread_t mcp_id;

//implementation of interface
void BB_start()
{
  memset((void*)BBInCurr, 0, sizeof(struct BBInStruct)*3);
  memset((void*)BBOutCurr, 0, sizeof(struct BBOutStruct)*3);
  pthread_create(&mcp_id, NULL, &startmcp, NULL);
}

void BB_stop()
{
  closemcp();
}

void BB_receive()
{
  memcpy((void*)BBInCurr, (void*)BBInData[i_bb], 
      sizeof(struct BBInStruct)*3);
}

void BB_send()
{
  memcpy((void*)BBOutData[i_bbout], (void*)BBOutCurr,
      sizeof(struct BBOutStruct)*3);
  i_bbout = (i_bbout+1)%3;
}

unsigned char BB_getDigital(short card, short group)
{
  return BBInCurr[card].digital[group];
}

unsigned int BB_getAnalog(short card, short analog)
{
  return BBInCurr[card].analog[analog];
}

unsigned int BB_getIndex()
{
  return BBFrameIndex;
}

void BB_setDigital(short card, short group, unsigned char val)
{
  BBOutCurr[card].digital[group] = val;
}

void BB_setDAC(short card, short dac, unsigned short val)
{
  BBOutCurr[card].dac[dac] = val;
}

void BB_setOutfile(FILE* file)
{
  mcp_outfile = file;
}

void BB_useFramefile(int flag)
{
  mcp_use_framefile = flag;
}


//handle I/O between buffers and BLAST bus, used by tx.c
void BB_handleGlobals()
{
  int i,j;
  static int firsttime = 1;
  char buf[16];
  int i_in = (i_bb+1)%3;     //write index in input buffer
  int i_out = (i_bbout+2)%3;   //read index in output buffer
  unsigned int temp;

  static struct NiosStruct* digOutAddrs[3][6];
  static struct NiosStruct* dacOutAddrs[3][32];
  static struct BiPhaseStruct* digInAddrs[3][6];
  static struct BiPhaseStruct* anaInAddrs[3][50];

  //find Nios/BiPhase read/write addrs
  if (firsttime)
  {
    for (i=0; i<3; i++)
    {
      for (j=0; j<3; j++) {
	sprintf(buf, "adc%1d_wd%1d", i+1, j+1);
	digOutAddrs[i][j] = GetNiosAddr(buf);
	sprintf(buf, "adc%1d_d%1d", i+1, j+1);
	digInAddrs[i][j] = GetBiPhaseAddr(buf);
      }
      for (j=0; j<32; j++) {
	sprintf(buf, "adc%1d_dac%02d", i+1, j);
	dacOutAddrs[i][j] = GetNiosAddr(buf);
      }
      for (j=0; j<25; j++) {
	sprintf(buf, "adc%1d_a1_%02d", i+1, j);
	anaInAddrs[i][j] = GetBiPhaseAddr(buf);
	sprintf(buf, "adc%1d_a2_%02d", i+1, j);
	anaInAddrs[i][j+25] = GetBiPhaseAddr(buf);
      }
    }
    firsttime = 0;
  }

  for (i=0; i<3; i++)
  {
    for (j=0; j<3; j++) {
      temp = ReadData(digInAddrs[i][j]);
      BBInData[i_in][i].digital[j] = (temp & 0x00ff);
      BBInData[i_in][i].digital[j+1] = (temp & 0xff00) >> 8;
      temp = BBOutData[i_out][i].digital[j];
      temp |= BBOutData[i_out][i].digital[j+1] << 8;
      WriteData(digOutAddrs[i][j], temp, NIOS_QUEUE);
    }
    for (j=0; j<32; j++) {
      WriteData(dacOutAddrs[i][j], BBOutData[i_out][i].dac[j], NIOS_QUEUE);
    }
    for (j=0; j<50; j++) {
      BBInData[i_in][i].analog[j] = ReadData(anaInAddrs[i][j]);
    }
  }

  i_bb = i_in;      //allow user to read data just written
}

