/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
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

/* tx.c */

/* NB: As of 7 Sep 2003 this file has been split up into four pieces:
 *
 * auxiliary.c: Auxiliary controls: Lock Motor, Pumps, Electronics Heat, ISC
 * das.c:       DAS, Bias and Cryo controls
 * motors.c:    Motor commanding and Scan modes
 * tx.c:        Pointing data writeback, ADC sync, and standard Tx frame control
 *
 * -dvw */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "bbc_pci.h"

#include "channels.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"
#include "sss_struct.h"
#include "chrgctrl.h"

#define NIOS_BUFFER_SIZE 100

extern short int SouthIAm;

extern int StartupVeto;

short int InCharge = 0;

int EthernetSun = 3;
int EthernetIsc = 3;
int EthernetOsc = 3;

extern struct AxesModeStruct axes_mode; /* motors.c */

extern struct ISCStatusStruct ISCSentState[2];  /* isc.c */

extern unsigned int sched_lst; /* sched_lst */

extern int bbc_fp;

extern struct chat_buf chatter_buffer;  /* mcp.c */

double round(double x);

/* in actuators.c */
void StoreActBus(void);
void SecondaryMirror(void);

/* in hwpr.c */
void StoreHWPRBus(void);

/* in auxiliary.c */
void ChargeController(void);
void ControlAuxMotors(unsigned short *RxFrame);
void ControlGyroHeat(unsigned short *RxFrame);
void CameraTrigger(int which);
void ControlPower(void);

/* in das.c */
void BiasControl(unsigned short* RxFrame);
void CryoControl(int index);
void PhaseControl(void);

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex, unsigned short *RxFrame);

/* in sched.c */
void DoSched();

/* in starpos.c */
double getlst(time_t t, double lon);

/* in xystage.c */
void StoreStageBus(int index);

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
  static struct NiosStruct* timeoutAddr;
  static struct NiosStruct* tChipFlcAddr;
  static struct NiosStruct* tCpuFlcAddr;
  static struct NiosStruct* tXFlcAddr;
  static struct NiosStruct* statusMCCAddr;
  static struct BiPhaseStruct* statusMCCReadAddr;
  static struct NiosStruct* bi0FifoSizeAddr;
  static struct NiosStruct* bbcFifoSizeAddr;
  static struct NiosStruct* ploverAddr;
  static struct NiosStruct* he4LevOldAddr;
  static struct NiosStruct* statusNetAddr;
  static struct BiPhaseStruct* he4LevReadAddr;
  static int incharge = -1;
  time_t t;
  int i_point;
  struct timeval tv;
  struct timezone tz;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    statusMCCAddr = GetNiosAddr("status_mcc");
    statusMCCReadAddr = ExtractBiPhaseAddr(statusMCCAddr);

    he4LevOldAddr = GetNiosAddr("he4_lev_old");
    he4LevReadAddr = GetBiPhaseAddr("he4_lev");

    tChipFlcAddr = GetNiosAddr("t_chip_flc");
    tCpuFlcAddr = GetNiosAddr("t_cpu_flc");
    tXFlcAddr = GetNiosAddr("t_x_flc");
    timeAddr = GetNiosAddr("time");
    timeUSecAddr = GetNiosAddr("time_usec");
    diskFreeAddr = GetNiosAddr("disk_free");
    timeoutAddr = GetNiosAddr("timeout");
    bi0FifoSizeAddr = GetNiosAddr("bi0_fifo_size");
    bbcFifoSizeAddr = GetNiosAddr("bbc_fifo_size");
    ploverAddr = GetNiosAddr("plover");
    statusNetAddr = GetNiosAddr("status_net");
  }

  if (StartupVeto>0) {
    InCharge = 0;
  } else {
    InCharge = !(SouthIAm
	^ (slow_data[statusMCCReadAddr->index][statusMCCReadAddr->channel] & 0x1));
  }
  if (InCharge != incharge && InCharge) {
    bprintf(info, "System: I, %s, have gained control.\n", SouthIAm ? "South" : "North");
    CommandData.actbus.force_repoll = 1;
  } else if (InCharge != incharge) {
    bprintf(info, "System: I, %s, have lost control.\n", SouthIAm ? "South" : "North");
  }

  if (CommandData.Cryo.heliumLevel)
    CommandData.Cryo.he4_lev_old
      = slow_data[he4LevReadAddr->index][he4LevReadAddr->channel];

  WriteData(he4LevOldAddr, CommandData.Cryo.he4_lev_old, NIOS_QUEUE);

  incharge = InCharge;

  gettimeofday(&tv, &tz);

  WriteData(timeAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  WriteData(timeUSecAddr, tv.tv_usec, NIOS_QUEUE);

  WriteData(tChipFlcAddr, CommandData.temp1, NIOS_QUEUE);
  WriteData(tCpuFlcAddr, CommandData.temp2, NIOS_QUEUE);
  WriteData(tXFlcAddr, CommandData.temp3, NIOS_QUEUE);

  WriteData(diskFreeAddr, CommandData.df, NIOS_QUEUE);

  i_point = GETREADINDEX(point_index);

#ifdef BOLOTEST
  t = mcp_systime(NULL);
#else
  t = PointingData[i_point].t;
#endif

  WriteData(timeoutAddr, CommandData.pointing_mode.t - t, NIOS_QUEUE);
  WriteData(bi0FifoSizeAddr, CommandData.bi0FifoSize, NIOS_QUEUE);
  WriteData(bbcFifoSizeAddr, CommandData.bbcFifoSize, NIOS_QUEUE);
  WriteData(ploverAddr, CommandData.plover, NIOS_QUEUE);

  WriteData(statusNetAddr, 
       (EthernetSun & 0x3) + 
       ((EthernetIsc & 0x3) << 2) + 
       ((EthernetOsc & 0x3) << 4), 
       NIOS_QUEUE);

  WriteData(statusMCCAddr, 
       (SouthIAm ? 0x1 : 0x0) +                 //0x01
       (CommandData.at_float ? 0x2 : 0x0) +     //0x02
       (CommandData.sucks ? 0x10 : 0x00) +      //0x10
       ((CommandData.lat_range & 0x3) << 5) +   //0x60
       ((CommandData.alice_file & 0xFF) << 8),  //0xFF00
       NIOS_FLUSH);
}

void WriteChatter (int index)
{
  static struct NiosStruct* chatterAddr;
  static int firsttime = 1;
  unsigned int chat;

  if (firsttime)
  {
    firsttime = 0;
    chatterAddr = GetNiosAddr("chatter");
  }

  switch (index & 0x03)
  {
    case 0x00:
      chat = 0x0000;
      break;
    case 0x01:
      chat = 0x8000;
      break;
    case 0x02:
      chat = 0x0080;
      break;
    case 0x03:
      chat = 0x8080;
      break;
    default:
      chat = 0x0000;
      break;
  }

  chat += (unsigned int)(chatter_buffer.msg[chatter_buffer.reading][index * 2] & 0x7F);
  chat += (unsigned int)((chatter_buffer.msg[chatter_buffer.reading][(index * 2) + 1]) & 0x7F) << 8;

  if (index == (FAST_PER_SLOW - 1))
  {
    chatter_buffer.reading = (chatter_buffer.reading + 1) & 0x3;
  }

  WriteData(chatterAddr, chat, NIOS_FLUSH);
}

/***************************************************************/
/* SetGyroMask:                                                */
/* mask gyros - automatic masking (if the gyro is faulty)      */
/*           or- commanded masking                             */
/* power cycle gyros - if masked for 1s                        */
/*                and- hasn't been power cycled in the last 25s */
/***************************************************************/
#define MASK_TIMEOUT 5 /* 1 sec -- in 5Hz Slow Frames */ 
#define GYRO_PCYCLE_TIMEOUT 125 /* 25 sec */
void SetGyroMask (void) {
  static struct NiosStruct* maskGyAddr;
  unsigned int GyroMask = 0x3f; //all gyros enabled
  int convert[6] = {1,5,0,2,3,4};//order of gyros in power switching
  static int t_mask[6] = {0,0,0,0,0,0};
  static int wait[6] = {0,0,0,0,0,0};
  static struct BiPhaseStruct* faultGyAddr;
  static int firsttime = 1;
  unsigned int GyroFault;

  if (firsttime) {
    firsttime = 0;
    maskGyAddr = GetNiosAddr("mask_gy");
    faultGyAddr = GetBiPhaseAddr("fault_gy");
  }

  GyroFault = slow_data[faultGyAddr->index][faultGyAddr->channel];
  int i;
  for (i=0; i<6; i++) {
    int j = convert[i];
    if (GyroFault & (0x01 << i)) {
      GyroMask &= ~(0x01 << i);
      t_mask[i] +=1;
      if (t_mask[i] > MASK_TIMEOUT) {
	if (wait[i] == 0) {
	  CommandData.power.gyro_off_auto[j] = PCYCLE_HOLD_LEN;
	  wait[i] = GYRO_PCYCLE_TIMEOUT;
	}
      }
    }
    else if ((CommandData.gymask & (0x01 << i)) == 0 ) {
      GyroMask &= ~(0x01 << i);
    } else {
      GyroMask |= 0x01 << i;	
      t_mask[i] = 0;
    }

    if (wait[i] > 0) wait[i]--;
  }

  WriteData(maskGyAddr, GyroMask, NIOS_QUEUE);
}

/*****************************************************************/
/* SyncADC: check to see if any boards need to be synced and     */
/* send the sync bit if they do.  Only one board can be synced   */
/* in each superframe.                                           */
/*****************************************************************/
//list node numbers for sync, which may have gaps
#define NUM_SYNC 29
const unsigned short sync_nums[NUM_SYNC] = {0,1,2,3,4,5,6,8,9,10,12,13,14,\
			16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
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
    k = slow_data[statusAddr[m]->index][statusAddr[m]->channel];

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

#ifndef BOLOTEST
static struct NiosStruct* GetSCNiosAddr(char* field, int which)
{
  char buffer[FIELD_LEN];
  sprintf(buffer, "%s_%s", field, which ? "osc" : "isc");

  return GetNiosAddr(buffer);
}

static void StoreStarCameraData(int index, int which)
{
  static int firsttime[2] = {1, 1};
  static int blob_index[2] = {0, 0};
  static int blob_data[2][15][4];

  int i, i_isc = 0;

  /** isc fields **/
  static struct NiosStruct* Blob0XAddr[2];
  static struct NiosStruct* Blob1XAddr[2];
  static struct NiosStruct* Blob2XAddr[2];
  static struct NiosStruct* Blob0YAddr[2];
  static struct NiosStruct* Blob1YAddr[2];
  static struct NiosStruct* Blob2YAddr[2];
  static struct NiosStruct* Blob0FAddr[2];
  static struct NiosStruct* Blob1FAddr[2];
  static struct NiosStruct* Blob2FAddr[2];
  static struct NiosStruct* Blob0SAddr[2];
  static struct NiosStruct* Blob1SAddr[2];
  static struct NiosStruct* Blob2SAddr[2];
  static struct NiosStruct* ErrorAddr[2];
  static struct NiosStruct* MapmeanAddr[2];
  static struct NiosStruct* FramenumAddr[2];
  static struct NiosStruct* RdSigmaAddr[2];
  static struct NiosStruct* RaAddr[2];
  static struct NiosStruct* DecAddr[2];
  static struct NiosStruct* HxFlagAddr[2];
  static struct NiosStruct* McpnumAddr[2];
  static struct NiosStruct* ApertAddr[2];
  static struct NiosStruct* MdistAddr[2];
  static struct NiosStruct* NblobsAddr[2];
  static struct NiosStruct* FocusAddr[2];
  static struct NiosStruct* FocOffAddr[2];
  static struct NiosStruct* ThreshAddr[2];
  static struct NiosStruct* GridAddr[2];
  static struct NiosStruct* StateAddr[2];
  static struct NiosStruct* MinblobsAddr[2];
  static struct NiosStruct* MaxblobsAddr[2];
  static struct NiosStruct* MaglimitAddr[2];
  static struct NiosStruct* NradAddr[2];
  static struct NiosStruct* LradAddr[2];
  static struct NiosStruct* TolAddr[2];
  static struct NiosStruct* MtolAddr[2];
  static struct NiosStruct* QtolAddr[2];
  static struct NiosStruct* RtolAddr[2];
  static struct NiosStruct* FpulseAddr[2];
  static struct NiosStruct* SpulseAddr[2];
  static struct NiosStruct* XOffAddr[2];
  static struct NiosStruct* YOffAddr[2];
  static struct NiosStruct* IHoldAddr[2];
  static struct NiosStruct* SavePrdAddr[2];
  static struct NiosStruct* Temp1Addr[2];
  static struct NiosStruct* Temp2Addr[2];
  static struct NiosStruct* Temp3Addr[2];
  static struct NiosStruct* Temp4Addr[2];
  static struct NiosStruct* PressureAddr[2];
  static struct NiosStruct* GainAddr[2];
  static struct NiosStruct* OffsetAddr[2];
  static struct NiosStruct* ExposureAddr[2];
  static struct NiosStruct* TrigTypeAddr[2];
  static struct NiosStruct* RealTrigAddr[2];
  static struct NiosStruct* BlobIdxAddr[2];
  static struct NiosStruct* FieldrotAddr[2];
  static struct NiosStruct* DiskfreeAddr[2];
  static struct NiosStruct* MaxslewAddr[2];
  static struct NiosStruct* MaxAgeAddr[2];
  static struct NiosStruct* AgeAddr[2];

  if (firsttime[which]) {
    firsttime[which] = 0;
    Blob0XAddr[which] = GetSCNiosAddr("blob00_x", which);
    Blob1XAddr[which] = GetSCNiosAddr("blob01_x", which);
    Blob2XAddr[which] = GetSCNiosAddr("blob02_x", which);
    Blob0YAddr[which] = GetSCNiosAddr("blob00_y", which);
    Blob1YAddr[which] = GetSCNiosAddr("blob01_y", which);
    Blob2YAddr[which] = GetSCNiosAddr("blob02_y", which);
    Blob0FAddr[which] = GetSCNiosAddr("blob00_f", which);
    Blob1FAddr[which] = GetSCNiosAddr("blob01_f", which);
    Blob2FAddr[which] = GetSCNiosAddr("blob02_f", which);
    Blob0SAddr[which] = GetSCNiosAddr("blob00_s", which);
    Blob1SAddr[which] = GetSCNiosAddr("blob01_s", which);
    Blob2SAddr[which] = GetSCNiosAddr("blob02_s", which);
    ErrorAddr[which] = GetSCNiosAddr("error", which);
    MapmeanAddr[which] = GetSCNiosAddr("mapmean", which);
    RdSigmaAddr[which] = GetSCNiosAddr("rd_sigma", which);
    FramenumAddr[which] = GetSCNiosAddr("framenum", which);
    RaAddr[which] = GetSCNiosAddr("ra", which);
    DecAddr[which] = GetSCNiosAddr("dec", which);
    NblobsAddr[which] = GetSCNiosAddr("nblobs", which);
    HxFlagAddr[which] = GetSCNiosAddr("hx_flag", which);
    McpnumAddr[which] = GetSCNiosAddr("mcpnum", which);

    StateAddr[which] = GetSCNiosAddr("state", which);
    FocusAddr[which] = GetSCNiosAddr("focus", which);
    FocOffAddr[which] = GetSCNiosAddr("foc_off", which);
    ApertAddr[which] = GetSCNiosAddr("apert", which);
    ThreshAddr[which] = GetSCNiosAddr("thresh", which);
    GridAddr[which] = GetSCNiosAddr("grid", which);
    MdistAddr[which] = GetSCNiosAddr("mdist", which);
    MinblobsAddr[which] = GetSCNiosAddr("minblobs", which);
    MaxblobsAddr[which] = GetSCNiosAddr("maxblobs", which);
    MaglimitAddr[which] = GetSCNiosAddr("maglimit", which);
    NradAddr[which] = GetSCNiosAddr("nrad", which);
    LradAddr[which] = GetSCNiosAddr("lrad", which);
    TolAddr[which] = GetSCNiosAddr("tol", which);
    MtolAddr[which] = GetSCNiosAddr("mtol", which);
    QtolAddr[which] = GetSCNiosAddr("qtol", which);
    RtolAddr[which] = GetSCNiosAddr("rtol", which);
    FpulseAddr[which] = GetSCNiosAddr("fpulse", which);
    SpulseAddr[which] = GetSCNiosAddr("spulse", which);
    XOffAddr[which] = GetSCNiosAddr("x_off", which);
    YOffAddr[which] = GetSCNiosAddr("y_off", which);
    IHoldAddr[which] = GetSCNiosAddr("i_hold", which);
    SavePrdAddr[which] = GetSCNiosAddr("save_prd", which);
    PressureAddr[which] = GetSCNiosAddr("pressure1", which);
    GainAddr[which] = GetSCNiosAddr("gain", which);
    OffsetAddr[which] = GetSCNiosAddr("offset", which);
    ExposureAddr[which] = GetSCNiosAddr("exposure", which);
    TrigTypeAddr[which] = GetSCNiosAddr("trig_type", which);
    FieldrotAddr[which] = GetSCNiosAddr("fieldrot", which);
    RealTrigAddr[which] = GetSCNiosAddr("real_trig", which);
    BlobIdxAddr[which] = GetSCNiosAddr("blob_idx", which);
    DiskfreeAddr[which] = GetSCNiosAddr("diskfree", which);
    MaxslewAddr[which] = GetSCNiosAddr("maxslew", which);
    MaxAgeAddr[which] = GetSCNiosAddr("max_age", which);
    AgeAddr[which] = GetSCNiosAddr("age", which);

    Temp1Addr[0] = GetNiosAddr("t_flange_isc");
    Temp2Addr[0] = GetNiosAddr("t_heat_isc");
    Temp3Addr[0] = GetNiosAddr("t_lens_isc");
    Temp4Addr[0] = GetNiosAddr("t_comp_isc");
    Temp1Addr[1] = GetNiosAddr("t_flange_osc");
    Temp2Addr[1] = GetNiosAddr("t_heat_osc");
    Temp3Addr[1] = GetNiosAddr("t_lens_osc");
    Temp4Addr[1] = GetNiosAddr("t_comp_osc");
  }

  /** Increment isc index -- this only happens once per slow frame */
  if (index == 0)
    if (((iscread_index[which] + 1) % 5) != iscwrite_index[which]) {
      iscread_index[which] = (iscread_index[which] + 1) % 5;
      /* reset blob multiplexing if this is a pointing packet */
      if (ISCSolution[which][iscread_index[which]].flag == 1)
        blob_index[which] = 0;
    }

  i_isc = iscread_index[which];

  /*** State Info ***/
  WriteData(StateAddr[which], (unsigned int)(ISCSentState[which].save * 0x0001
        + ISCSentState[which].pause * 0x0002
        + ISCSentState[which].abort * 0x0004
        + ISCSentState[which].autofocus * 0x0008
        + ISCSentState[which].shutdown * 0x0010 /* 2 bits */
        + ISCSentState[which].eyeOn * 0x0040
        + ISCSolution[which][i_isc].heaterOn * 0x0080
        + ISCSentState[which].useLost * 0x0100
        ), NIOS_QUEUE);
  WriteData(FocusAddr[which], (unsigned int)ISCSentState[which].focus_pos,
      NIOS_QUEUE);
  WriteData(FocOffAddr[which], (unsigned int)ISCSentState[which].focusOffset,
      NIOS_QUEUE);
  WriteData(ApertAddr[which], (unsigned int)ISCSentState[which].ap_pos,
      NIOS_QUEUE);
  WriteData(ThreshAddr[which], (unsigned int)(ISCSentState[which].sn_threshold
        * 10.), NIOS_QUEUE);
  WriteData(GridAddr[which], (unsigned int)ISCSentState[which].grid,
      NIOS_QUEUE);
  WriteData(MdistAddr[which], (unsigned int)ISCSentState[which].mult_dist,
      NIOS_QUEUE);
  WriteData(MinblobsAddr[which], (unsigned int)ISCSentState[which].minBlobMatch,
      NIOS_QUEUE);
  WriteData(MaxblobsAddr[which], (unsigned int)ISCSentState[which].maxBlobMatch,
      NIOS_QUEUE);
  WriteData(MaglimitAddr[which], (unsigned int)(ISCSentState[which].mag_limit
        * 1000.), NIOS_QUEUE);
  WriteData(NradAddr[which], (unsigned int)(ISCSentState[which].norm_radius
        * RAD2I), NIOS_QUEUE);
  WriteData(LradAddr[which], (unsigned int)(ISCSentState[which].lost_radius
        * RAD2I), NIOS_QUEUE);
  WriteData(TolAddr[which], (unsigned int)(ISCSentState[which].tolerance
        * RAD2ARCSEC), NIOS_QUEUE);
  WriteData(MtolAddr[which], (unsigned int)(ISCSentState[which].match_tol
        * 65535.), NIOS_QUEUE);
  WriteData(QtolAddr[which], (unsigned int)(ISCSentState[which].quit_tol
        * 65535.), NIOS_QUEUE);
  WriteData(RtolAddr[which], (unsigned int)(ISCSentState[which].rot_tol
        * RAD2I), NIOS_QUEUE);
  WriteData(XOffAddr[which], (unsigned int)(ISCSentState[which].azBDA * RAD2I),
      NIOS_QUEUE);
  WriteData(YOffAddr[which], (unsigned int)(ISCSentState[which].elBDA * RAD2I),
      NIOS_QUEUE);
  WriteData(IHoldAddr[which], (unsigned int)(ISCSentState[which].hold_current),
      NIOS_QUEUE);
  WriteData(Temp1Addr[which],
      (unsigned int)(ISCSolution[which][i_isc].temp1 * 200.), NIOS_QUEUE);
  WriteData(Temp2Addr[which],
      (unsigned int)(ISCSolution[which][i_isc].temp2 * 200.), NIOS_QUEUE);
  WriteData(Temp3Addr[which],
      (unsigned int)(ISCSolution[which][i_isc].temp3 * 200.), NIOS_QUEUE);
  WriteData(Temp4Addr[which],
      (unsigned int)(ISCSolution[which][i_isc].temp4 * 200.), NIOS_QUEUE);
  WriteData(PressureAddr[which],
      (unsigned int)(ISCSolution[which][i_isc].pressure1 * 2000.), NIOS_QUEUE);
  WriteData(GainAddr[which], (unsigned int)(ISCSentState[which].gain * 655.36),
      NIOS_QUEUE);
  WriteData(OffsetAddr[which], ISCSentState[which].offset, NIOS_QUEUE);
  WriteData(ExposureAddr[which], ISCSentState[which].exposure / 100,
      NIOS_QUEUE);
  WriteData(TrigTypeAddr[which], ISCSentState[which].triggertype, NIOS_QUEUE);
  WriteData(MaxslewAddr[which],
      (unsigned int)ISCSentState[which].maxSlew / RAD2I, NIOS_QUEUE);

  WriteData(FpulseAddr[which],
      (unsigned int)(CommandData.ISCControl[which].fast_pulse_width),
      NIOS_QUEUE);
  WriteData(SpulseAddr[which],
      (unsigned int)(CommandData.ISCControl[which].pulse_width), NIOS_QUEUE);
  WriteData(MaxAgeAddr[which], 
      (unsigned int)(CommandData.ISCControl[which].max_age*10), NIOS_QUEUE);
  WriteData(AgeAddr[which], 
      (unsigned int)(CommandData.ISCControl[which].age*10), NIOS_QUEUE);
  WriteData(SavePrdAddr[which],
      (unsigned int)(CommandData.ISCControl[which].save_period), NIOS_FLUSH);

  /* The handshake flag -- for handshakes, we only write this. */
  WriteData(HxFlagAddr[which], (unsigned int)ISCSolution[which][i_isc].flag,
      NIOS_QUEUE);

  /*** Blobs ***/
  /* Save current blob data if the current frame is a pointing solution;
   * we only do this once per slow frame */
  if (index == 0 && ISCSolution[which][i_isc].flag)
    for (i = 0; i < 15; ++i) {
      blob_data[which][i][0] = (int)(ISCSolution[which][i_isc].blob_x[i] * 40.);
      blob_data[which][i][1] = (int)(ISCSolution[which][i_isc].blob_y[i] * 40.);
      blob_data[which][i][2] = ISCSolution[which][i_isc].blob_flux[i];
      blob_data[which][i][3] = (int)(ISCSolution[which][i_isc].blob_sn[i]
          * 65.536);
    }

  if (index == 0) {
    /* When we're writing a handshake packet, these blobs are still from the
     * previous pointing packet */
    WriteData(Blob0XAddr[which], blob_data[which][blob_index[which] * 3 + 0][0],
        NIOS_QUEUE);
    WriteData(Blob1XAddr[which], blob_data[which][blob_index[which] * 3 + 1][0],
        NIOS_QUEUE);
    WriteData(Blob2XAddr[which], blob_data[which][blob_index[which] * 3 + 2][0],
        NIOS_QUEUE);

    WriteData(Blob0YAddr[which], blob_data[which][blob_index[which] * 3 + 0][1],
        NIOS_QUEUE);
    WriteData(Blob1YAddr[which], blob_data[which][blob_index[which] * 3 + 1][1],
        NIOS_QUEUE);
    WriteData(Blob2YAddr[which], blob_data[which][blob_index[which] * 3 + 2][1],
        NIOS_QUEUE);

    WriteData(Blob0FAddr[which], blob_data[which][blob_index[which] * 3 + 0][2],
        NIOS_QUEUE);
    WriteData(Blob1FAddr[which], blob_data[which][blob_index[which] * 3 + 1][2],
        NIOS_QUEUE);
    WriteData(Blob2FAddr[which], blob_data[which][blob_index[which] * 3 + 2][2],
        NIOS_QUEUE);

    WriteData(Blob0SAddr[which], blob_data[which][blob_index[which] * 3 + 0][3],
        NIOS_QUEUE);
    WriteData(Blob1SAddr[which], blob_data[which][blob_index[which] * 3 + 1][3],
        NIOS_QUEUE);
    WriteData(Blob2SAddr[which], blob_data[which][blob_index[which] * 3 + 2][3],
        NIOS_QUEUE);

    WriteData(BlobIdxAddr[which], blob_index[which], NIOS_QUEUE);

    /* increment blob index once per slow frame */
    blob_index[which] = (blob_index[which] + 1) % 5;
  }

  if (!ISCSolution[which][i_isc].flag)
    return;

  /* Everything after this happens only for pointing packets */

  /*** Solution Info ***/
  WriteData(FramenumAddr[which],
      (unsigned int)ISCSolution[which][i_isc].framenum, NIOS_QUEUE);
  WriteData(RaAddr[which],
      (unsigned int)(ISCSolution[which][i_isc].ra * RAD2LI), NIOS_QUEUE);
  WriteData(DecAddr[which], (unsigned int)((ISCSolution[which][i_isc].dec
          + M_PI / 2) * 2. * RAD2LI), NIOS_QUEUE);
  WriteData(NblobsAddr[which], (unsigned int)ISCSolution[which][i_isc].n_blobs,
      NIOS_QUEUE);

  if (ISCSolution[which][i_isc].sigma * RAD2ARCSEC > 65535)
    WriteData(RdSigmaAddr[which], 65535, NIOS_QUEUE);
  else
    WriteData(RdSigmaAddr[which], (unsigned int)(ISCSolution[which][i_isc].sigma
          * RAD2ARCSEC), NIOS_QUEUE);

  WriteData(FieldrotAddr[which], (unsigned int)(ISCSolution[which][i_isc].rot
        * RAD2I), NIOS_QUEUE);

  WriteData(McpnumAddr[which],
      (unsigned int)ISCSolution[which][i_isc].MCPFrameNum, NIOS_QUEUE);
  WriteData(RealTrigAddr[which],
      (unsigned int)ISCSolution[which][i_isc].triggertype, NIOS_QUEUE);
  WriteData(ErrorAddr[which], (unsigned int)ISCSolution[which][i_isc].cameraerr,
      NIOS_QUEUE);
  WriteData(MapmeanAddr[which], (unsigned int)ISCSolution[which][i_isc].mapMean,
      NIOS_QUEUE);
  WriteData(DiskfreeAddr[which],
      (unsigned int)ISCSolution[which][i_isc].diskspace / 5, NIOS_QUEUE);
}

/************************************************************************/
/*                                                                      */
/*    Store derived acs and pointing data in frame                      */
/*                                                                      */
/************************************************************************/
static void StoreData(int index)
{
  static int firsttime = 1;

  static struct NiosStruct* azRelSunSsAddr;
  static struct NiosStruct* phaseSsAddr;
  static struct NiosStruct* snrSsAddr;
  static struct NiosStruct* sunTimeSsAddr;
  static struct NiosStruct* tCpuSsAddr;
  static struct NiosStruct* tHddSsAddr;
  static struct NiosStruct* tCaseSsAddr;
  static struct NiosStruct* tPortSsAddr;
  static struct NiosStruct* tStarSsAddr;
  static struct NiosStruct* v5SsAddr;
  static struct NiosStruct* v12SsAddr;
  static struct NiosStruct* vBattSsAddr;
  static struct NiosStruct* Raw01SsAddr;
  static struct NiosStruct* Raw02SsAddr;
  static struct NiosStruct* Raw03SsAddr;
  static struct NiosStruct* Raw04SsAddr;
  static struct NiosStruct* Raw05SsAddr;
  static struct NiosStruct* Raw06SsAddr;
  static struct NiosStruct* Raw07SsAddr;
  static struct NiosStruct* Raw08SsAddr;
  static struct NiosStruct* Raw09SsAddr;
  static struct NiosStruct* Raw10SsAddr;
  static struct NiosStruct* Raw11SsAddr;
  static struct NiosStruct* Raw12SsAddr;

  static struct NiosStruct* latSipAddr;
  static struct NiosStruct* lonSipAddr;
  static struct NiosStruct* altSipAddr;
  static struct NiosStruct* timeSipAddr;
  static struct NiosStruct* mksLoSipAddr;
  static struct NiosStruct* mksMedSipAddr;
  static struct NiosStruct* mksHiSipAddr;

  /** pointing mode indexes **/
  static struct NiosStruct* svetoLenAddr;
  static struct NiosStruct* slewVetoAddr;
  static struct NiosStruct* modePAddr;
  static struct NiosStruct* xPAddr, *yPAddr;
  static struct NiosStruct* velAzPAddr, *delPAddr;
  static struct NiosStruct* wPAddr, *hPAddr;
  static struct NiosStruct* ra1PAddr, *dec1PAddr;
  static struct NiosStruct* ra2PAddr, *dec2PAddr;
  static struct NiosStruct* ra3PAddr, *dec3PAddr;
  static struct NiosStruct* ra4PAddr, *dec4PAddr;

  static struct NiosStruct* vetoSensorAddr;

  /** derived pointing data */
  static struct NiosStruct* mcpFrameAddr;
  static struct NiosStruct* OffsetIFelGYAddr;
  static struct NiosStruct* OffsetIFelGYiscAddr;
  static struct NiosStruct* OffsetIFrollGYiscAddr;
  static struct NiosStruct* OffsetIFyawGYiscAddr;
  static struct NiosStruct* OffsetIFelGYoscAddr;
  static struct NiosStruct* OffsetIFrollGYoscAddr;
  static struct NiosStruct* OffsetIFyawGYoscAddr;
  static struct NiosStruct* OffsetIFrollGYAddr;
  static struct NiosStruct* OffsetIFyawGYAddr;
  static struct NiosStruct* azAddr;
  static struct NiosStruct* elAddr;
  static struct NiosStruct* raAddr;
  static struct NiosStruct* decAddr;
  static struct NiosStruct* altAddr;
  static struct NiosStruct* latAddr;
  static struct NiosStruct* lonAddr;
  static struct NiosStruct* lstAddr;
  static struct NiosStruct* azMagAddr;
  static struct NiosStruct* pitchMagAddr;
  static struct NiosStruct* declinationMagAddr;
  static struct NiosStruct* sigmaMagAddr;
  static struct NiosStruct* dgpsAzAddr;
  static struct NiosStruct* dgpsPitchAddr;
  static struct NiosStruct* dgpsRollAddr;
  static struct NiosStruct* dgpsSigmaAddr;
  static struct NiosStruct* azSsAddr;
  static struct NiosStruct* azrawPss1Addr;
  static struct NiosStruct* elrawPss1Addr;
  static struct NiosStruct* snrPss1Addr;
  static struct NiosStruct* azPss1Addr;
  //static struct NiosStruct* azrawPss2Addr;
  //static struct NiosStruct* elrawPss2Addr;
  //static struct NiosStruct* snrPss2Addr;
  //static struct NiosStruct* azPss2Addr;
  static struct NiosStruct* sigmaSsAddr;
  static struct NiosStruct* azSunAddr;
  static struct NiosStruct* elSunAddr;
  static struct NiosStruct* azIscAddr;
  static struct NiosStruct* elIscAddr;
  static struct NiosStruct* sigmaIscAddr;
  static struct NiosStruct* azOscAddr;
  static struct NiosStruct* elOscAddr;
  static struct NiosStruct* sigmaOscAddr;
  static struct NiosStruct* elEncAddr;
  static struct NiosStruct* sigmaEncAddr;
  static struct NiosStruct* elClinAddr;
  static struct NiosStruct* sigmaClinAddr;

  /** dgps fields **/
  static struct NiosStruct* dgpsTimeAddr;
  static struct NiosStruct* dgpsLatAddr;
  static struct NiosStruct* dgpsLonAddr;
  static struct NiosStruct* dgpsAltAddr;
  static struct NiosStruct* dgpsSpeedAddr;
  static struct NiosStruct* dgpsDirAddr;
  static struct NiosStruct* dgpsClimbAddr;
  static struct NiosStruct* dgpsAttOkAddr;
  static struct NiosStruct* dgpsAzRawAddr;
  static struct NiosStruct* dgpsPitchRawAddr;
  static struct NiosStruct* dgpsRollRawAddr;
  static struct NiosStruct* dgpsNSatAddr;

  /* trim fields */
  static struct NiosStruct *trimClinAddr;
  static struct NiosStruct *trimEncAddr;
  static struct NiosStruct *trimNullAddr;
  static struct NiosStruct *trimMagAddr;
  static struct NiosStruct *dgpsTrimAddr;
  static struct NiosStruct *trimSsAddr;

  static struct NiosStruct *modeCalAddr;
  static struct NiosStruct *periodCalAddr;
  static struct NiosStruct *lstSchedAddr;

  /* low level scan mode diagnostics */
  static struct NiosStruct *modeAzMcAddr;
  static struct NiosStruct *modeElMcAddr;
  static struct NiosStruct *dirAzMcAddr;
  static struct NiosStruct *dirElMcAddr;
  static struct NiosStruct *destAzMcAddr;
  static struct NiosStruct *destElMcAddr;
  static struct NiosStruct *velAzMcAddr;
  static struct NiosStruct *velElMcAddr;

  /* Motor data read out over serial threads in motors.c */
  static struct NiosStruct *velRWAddr;
  static struct NiosStruct *tMCRWAddr;
  static struct NiosStruct *iSerRWAddr;
  static struct NiosStruct *stat1RWAddr;
  static struct NiosStruct *stat2RWAddr;
  static struct NiosStruct *faultRWAddr;
  static struct NiosStruct *infoRWAddr;
  static struct NiosStruct *driveErrCtsRWAddr;
  static struct NiosStruct *elRawEncAddr;
  static struct NiosStruct *tMCElAddr;
  static struct NiosStruct *iSerElAddr;
  static struct NiosStruct *stat1ElAddr;
  static struct NiosStruct *stat2ElAddr;
  static struct NiosStruct *faultElAddr;
  static struct NiosStruct *infoElAddr;
  static struct NiosStruct *driveErrCtsElAddr;
  static struct NiosStruct *resPivAddr;
  static struct NiosStruct *iSerPivAddr;
  static struct NiosStruct *statDrPivAddr;
  static struct NiosStruct *statS1PivAddr;
  static struct NiosStruct *azGyAddr;
  static struct NiosStruct *velSerPivAddr;
  static struct NiosStruct *infoPivAddr;
  static struct NiosStruct *driveErrCtsPivAddr;
  static struct NiosStruct *verboseRWAddr;
  static struct NiosStruct *verboseElAddr;
  static struct NiosStruct *verbosePivAddr;

  int i_rw_motors;
  int i_elev_motors;
  int i_pivot_motors;
  int i_ss;
  int i_point;
  int i_dgps;
  int sensor_veto;

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    azAddr = GetNiosAddr("az");
    elAddr = GetNiosAddr("el");
    mcpFrameAddr = GetNiosAddr("mcp_frame");

    azRelSunSsAddr = GetNiosAddr("az_rel_sun_ss");
    phaseSsAddr = GetNiosAddr("phase_ss");
    snrSsAddr = GetNiosAddr("snr_ss");
    sunTimeSsAddr = GetNiosAddr("sun_time_ss");
    tCpuSsAddr = GetNiosAddr("t_cpu_ss");
    tHddSsAddr = GetNiosAddr("t_hdd_ss");
    tCaseSsAddr = GetNiosAddr("t_case_ss");
    tPortSsAddr = GetNiosAddr("t_port_ss");
    tStarSsAddr = GetNiosAddr("t_star_ss");
    v5SsAddr = GetNiosAddr("v_5_ss");
    v12SsAddr = GetNiosAddr("v_12_ss");
    vBattSsAddr = GetNiosAddr("v_batt_ss");
    Raw01SsAddr = GetNiosAddr("raw_01_ss");
    Raw02SsAddr = GetNiosAddr("raw_02_ss");
    Raw03SsAddr = GetNiosAddr("raw_03_ss");
    Raw04SsAddr = GetNiosAddr("raw_04_ss");
    Raw05SsAddr = GetNiosAddr("raw_05_ss");
    Raw06SsAddr = GetNiosAddr("raw_06_ss");
    Raw07SsAddr = GetNiosAddr("raw_07_ss");
    Raw08SsAddr = GetNiosAddr("raw_08_ss");
    Raw09SsAddr = GetNiosAddr("raw_09_ss");
    Raw10SsAddr = GetNiosAddr("raw_10_ss");
    Raw11SsAddr = GetNiosAddr("raw_11_ss");
    Raw12SsAddr = GetNiosAddr("raw_12_ss");

    latSipAddr = GetNiosAddr("lat_sip");
    lonSipAddr = GetNiosAddr("lon_sip");
    altSipAddr = GetNiosAddr("alt_sip");
    timeSipAddr = GetNiosAddr("time_sip");

    mksLoSipAddr = GetNiosAddr("mks_lo_sip");
    mksMedSipAddr = GetNiosAddr("mks_med_sip");
    mksHiSipAddr = GetNiosAddr("mks_hi_sip");

    OffsetIFelGYAddr = GetNiosAddr("offset_ifel_gy");
    OffsetIFelGYiscAddr = GetNiosAddr("off_ifel_gy_isc");
    OffsetIFrollGYiscAddr = GetNiosAddr("off_ifroll_gy_isc");
    OffsetIFyawGYiscAddr = GetNiosAddr("off_ifyaw_gy_isc");
    OffsetIFelGYoscAddr = GetNiosAddr("off_ifel_gy_osc");
    OffsetIFrollGYoscAddr = GetNiosAddr("off_ifroll_gy_osc");
    OffsetIFyawGYoscAddr = GetNiosAddr("off_ifyaw_gy_osc");
    OffsetIFrollGYAddr = GetNiosAddr("offset_ifroll_gy");
    OffsetIFyawGYAddr = GetNiosAddr("offset_ifyaw_gy");
    raAddr = GetNiosAddr("ra");
    decAddr = GetNiosAddr("dec");
    latAddr = GetNiosAddr("lat");
    altAddr = GetNiosAddr("alt");
    lonAddr = GetNiosAddr("lon");
    lstAddr = GetNiosAddr("lst");
    azMagAddr = GetNiosAddr("az_mag");
    pitchMagAddr = GetNiosAddr("pitch_mag");
    declinationMagAddr = GetNiosAddr("declination_mag");
    sigmaMagAddr = GetNiosAddr("sigma_mag");
    dgpsAzAddr = GetNiosAddr("az_dgps");
    dgpsPitchAddr = GetNiosAddr("pitch_dgps");
    dgpsRollAddr = GetNiosAddr("roll_dgps");
    dgpsSigmaAddr = GetNiosAddr("sigma_dgps");
    azSsAddr = GetNiosAddr("az_ss");
    sigmaSsAddr = GetNiosAddr("sigma_ss");
    azSunAddr = GetNiosAddr("az_sun");
    elSunAddr = GetNiosAddr("el_sun");
    trimSsAddr = GetNiosAddr("trim_ss");
    azrawPss1Addr = GetNiosAddr("azraw_pss1");
    elrawPss1Addr = GetNiosAddr("elraw_pss1");
    snrPss1Addr = GetNiosAddr("snr_pss1");
    azPss1Addr = GetNiosAddr("az_pss1");  // evolved az
    //azrawPss2Addr = GetNiodAddr("azraw_pss2");
    //elrawPss2Addr = GetNiodAddr("elraw_pss2");
    //snrPss2Addr = GetNiosAddr("snr_pss2");
    //azPss1Addr = GetNiosAdd("az_pss2");  // evolved az
    modeCalAddr = GetNiosAddr("mode_cal");
    periodCalAddr = GetNiosAddr("period_cal");
    azIscAddr = GetNiosAddr("az_isc");
    elIscAddr = GetNiosAddr("el_isc");
    sigmaIscAddr = GetNiosAddr("sigma_isc");
    azOscAddr = GetNiosAddr("az_osc");
    elOscAddr = GetNiosAddr("el_osc");
    sigmaOscAddr = GetNiosAddr("sigma_osc");
    elEncAddr = GetNiosAddr("el_enc");
    sigmaEncAddr = GetNiosAddr("sigma_enc");
    elClinAddr = GetNiosAddr("el_clin");
    sigmaClinAddr = GetNiosAddr("sigma_clin");

    svetoLenAddr = GetNiosAddr("sveto_len");
    slewVetoAddr = GetNiosAddr("slew_veto");
    modePAddr = GetNiosAddr("mode_p");
    xPAddr = GetNiosAddr("x_p");
    yPAddr = GetNiosAddr("y_p");
    velAzPAddr = GetNiosAddr("vel_az_p");
    delPAddr = GetNiosAddr("del_p");
    wPAddr = GetNiosAddr("w_p");
    hPAddr = GetNiosAddr("h_p");
    ra1PAddr = GetNiosAddr("ra_1_p");
    dec1PAddr = GetNiosAddr("dec_1_p");
    ra2PAddr = GetNiosAddr("ra_2_p");
    dec2PAddr = GetNiosAddr("dec_2_p");
    ra3PAddr = GetNiosAddr("ra_3_p");
    dec3PAddr = GetNiosAddr("dec_3_p");
    ra4PAddr = GetNiosAddr("ra_4_p");
    dec4PAddr = GetNiosAddr("dec_4_p");

    vetoSensorAddr = GetNiosAddr("veto_sensor");

    dgpsTimeAddr = GetNiosAddr("time_dgps");
    dgpsLatAddr = GetNiosAddr("lat_dgps");
    dgpsLonAddr = GetNiosAddr("lon_dgps");
    dgpsAltAddr = GetNiosAddr("alt_dgps");
    dgpsSpeedAddr = GetNiosAddr("speed_dgps");
    dgpsDirAddr = GetNiosAddr("dir_dgps");
    dgpsClimbAddr = GetNiosAddr("climb_dgps");
    dgpsNSatAddr = GetNiosAddr("n_sat_dgps");
    dgpsAttOkAddr = GetNiosAddr("att_ok_dgps");
    dgpsAzRawAddr = GetNiosAddr("az_raw_dgps");
    dgpsPitchRawAddr = GetNiosAddr("pitch_raw_dgps");
    dgpsRollRawAddr = GetNiosAddr("roll_raw_dgps");

    lstSchedAddr = GetNiosAddr("lst_sched");

    trimClinAddr = GetNiosAddr("trim_clin");
    trimEncAddr = GetNiosAddr("trim_enc");
    trimNullAddr = GetNiosAddr("trim_null");
    trimMagAddr = GetNiosAddr("trim_mag");
    dgpsTrimAddr = GetNiosAddr("trim_dgps");

    modeAzMcAddr = GetNiosAddr("mode_az_mc");
    modeElMcAddr = GetNiosAddr("mode_el_mc");
    destAzMcAddr = GetNiosAddr("dest_az_mc");
    destElMcAddr = GetNiosAddr("dest_el_mc");
    velAzMcAddr = GetNiosAddr("vel_az_mc");
    velElMcAddr = GetNiosAddr("vel_el_mc");
    dirAzMcAddr = GetNiosAddr("dir_az_mc");
    dirElMcAddr = GetNiosAddr("dir_el_mc");

    velRWAddr = GetNiosAddr("vel_rw");
    elRawEncAddr = GetNiosAddr("el_raw_enc");
    tMCRWAddr = GetNiosAddr("t_mc_rw");
    iSerRWAddr = GetNiosAddr("i_ser_rw");
    stat1RWAddr = GetNiosAddr("stat_1_rw");
    stat2RWAddr = GetNiosAddr("stat_2_rw");
    faultRWAddr = GetNiosAddr("fault_rw");
    infoRWAddr = GetNiosAddr("drive_info_rw");
    driveErrCtsRWAddr = GetNiosAddr("drive_err_cts_rw");
    tMCElAddr = GetNiosAddr("t_mc_el");
    iSerElAddr = GetNiosAddr("i_ser_el");
    stat1ElAddr = GetNiosAddr("stat_1_el");
    stat2ElAddr = GetNiosAddr("stat_2_el");
    faultElAddr = GetNiosAddr("fault_el");
    resPivAddr = GetNiosAddr("res_piv");
    iSerPivAddr = GetNiosAddr("i_ser_piv");
    statDrPivAddr = GetNiosAddr("stat_dr_piv");
    statS1PivAddr = GetNiosAddr("stat_s1_piv");
    azGyAddr = GetNiosAddr("az_gy");
    velSerPivAddr = GetNiosAddr("vel_ser_piv");
    infoPivAddr = GetNiosAddr("drive_info_piv");
    driveErrCtsPivAddr = GetNiosAddr("drive_err_cts_piv");
    infoElAddr = GetNiosAddr("drive_info_el");
    driveErrCtsElAddr = GetNiosAddr("drive_err_cts_el");
    verboseRWAddr = GetNiosAddr("verbose_rw");
    verboseElAddr = GetNiosAddr("verbose_el");
    verbosePivAddr = GetNiosAddr("verbose_piv");
  }

  /*************************************************
   *             Fast Controls                     *
   ************************************************/
  i_point = GETREADINDEX(point_index);
  i_ss = GETREADINDEX(ss_index);
  i_rw_motors = GETREADINDEX(rw_motor_index);
  i_elev_motors = GETREADINDEX(elev_motor_index);
  i_pivot_motors = GETREADINDEX(pivot_motor_index);

  WriteData(azAddr, (unsigned int)(PointingData[i_point].az * DEG2LI),
      NIOS_QUEUE);
  WriteData(elAddr, (unsigned int)(PointingData[i_point].el * DEG2LI),
      NIOS_QUEUE);

  WriteData(elEncAddr, (unsigned int)((PointingData[i_point].enc_el
                      + CommandData.enc_el_trim)* DEG2I), NIOS_QUEUE);
  WriteData(sigmaEncAddr,
      (unsigned int)(PointingData[i_point].enc_sigma * DEG2I), NIOS_QUEUE);

  WriteData(velRWAddr,
      ((long int)(RWMotorData[i_rw_motors].vel_rw/4.0*DEG2I)), NIOS_QUEUE);
  WriteData(elRawEncAddr,
      ((long int)(ElevMotorData[i_elev_motors].enc_raw_el*DEG2I)), NIOS_QUEUE);

  WriteData(resPivAddr,
      PivotMotorData[i_pivot_motors].res_piv*DEG2I, NIOS_QUEUE);

  /*************************************************
   *             Slow Controls                     *
   ************************************************/
  if (index != 0) return;

  /************ star cameras ************************/
  StoreStarCameraData(index, 0); /* write ISC data */
  StoreStarCameraData(index, 1); /* write OSC data */

  /* scan modes */
  WriteData(modeAzMcAddr, axes_mode.az_mode, NIOS_QUEUE);
  WriteData(modeElMcAddr, axes_mode.el_mode, NIOS_QUEUE);
  WriteData(dirAzMcAddr, axes_mode.az_dir, NIOS_QUEUE);
  WriteData(dirElMcAddr, axes_mode.el_dir, NIOS_QUEUE);
  WriteData(destAzMcAddr, axes_mode.az_dest * DEG2I, NIOS_QUEUE);
  WriteData(destElMcAddr, axes_mode.el_dest * DEG2I, NIOS_QUEUE);
  WriteData(velAzMcAddr, axes_mode.az_vel * 6000., NIOS_QUEUE);
  WriteData(velElMcAddr, axes_mode.el_vel * 6000., NIOS_QUEUE);

  /********** Sun Sensor Data **********/
  WriteData(phaseSsAddr, PointingData[i_point].ss_phase * DEG2I, NIOS_QUEUE);
  WriteData(snrSsAddr, PointingData[i_point].ss_snr * 1000, NIOS_QUEUE);
  WriteData(sunTimeSsAddr, SunSensorData[i_ss].sun_time, NIOS_QUEUE);
  WriteData(tCpuSsAddr, SunSensorData[i_ss].t_cpu * 100, NIOS_QUEUE);
  WriteData(tHddSsAddr, SunSensorData[i_ss].t_hdd * 100, NIOS_QUEUE);
  WriteData(tCaseSsAddr, SunSensorData[i_ss].t_case * 100, NIOS_QUEUE);
  WriteData(tPortSsAddr, SunSensorData[i_ss].t_port * 100, NIOS_QUEUE);
  WriteData(tStarSsAddr, SunSensorData[i_ss].t_starboard * 100, NIOS_QUEUE);
  WriteData(v5SsAddr, SunSensorData[i_ss].v5 * 100, NIOS_QUEUE);
  WriteData(v12SsAddr, SunSensorData[i_ss].v12 * 100, NIOS_QUEUE);
  WriteData(vBattSsAddr, SunSensorData[i_ss].vbatt * 100, NIOS_QUEUE);
  WriteData(Raw01SsAddr, SunSensorData[i_ss].m01, NIOS_QUEUE);
  WriteData(Raw02SsAddr, SunSensorData[i_ss].m02, NIOS_QUEUE);
  WriteData(Raw03SsAddr, SunSensorData[i_ss].m03, NIOS_QUEUE);
  WriteData(Raw04SsAddr, SunSensorData[i_ss].m04, NIOS_QUEUE);
  WriteData(Raw05SsAddr, SunSensorData[i_ss].m05, NIOS_QUEUE);
  WriteData(Raw06SsAddr, SunSensorData[i_ss].m06, NIOS_QUEUE);
  WriteData(Raw07SsAddr, SunSensorData[i_ss].m07, NIOS_QUEUE);
  WriteData(Raw08SsAddr, SunSensorData[i_ss].m08, NIOS_QUEUE);
  WriteData(Raw09SsAddr, SunSensorData[i_ss].m09, NIOS_QUEUE);
  WriteData(Raw10SsAddr, SunSensorData[i_ss].m10, NIOS_QUEUE);
  WriteData(Raw11SsAddr, SunSensorData[i_ss].m11, NIOS_QUEUE);
  WriteData(Raw12SsAddr, SunSensorData[i_ss].m12, NIOS_QUEUE);
  WriteData(azRelSunSsAddr, PointingData[i_point].ss_az_rel_sun * DEG2I,
      NIOS_QUEUE);
  /********* PSS data *************/
  WriteData(azrawPss1Addr, PointingData[i_point].pss1_azraw * DEG2I, NIOS_QUEUE);
  WriteData(elrawPss1Addr, PointingData[i_point].pss1_elraw * DEG2I, NIOS_QUEUE);
  WriteData(snrPss1Addr, PointingData[i_point].pss1_snr * 1000., NIOS_QUEUE);
  WriteData(azPss1Addr, PointingData[i_point].pss1_az * DEG2I, NIOS_QUEUE);
  //WriteData(azrawPss2Addr, PointingData[i_point].pss2_azraw * DEG2I, NIOS_QUEUE);
  //WriteData(elrawPss2Addr, PointingData[i_point].pss2_elraw * DEG2I, NIOS_QUEUE);
  //WriteData(snrPss1Addr, PointingData[i_point].pss2_snr * 1000., NIOS_QUEUE);
  //WriteData(azPss1Addr, PointingData[i_point].pss2_snr * DEG2I, NIOS_QUEUE);
  /********** SIP GPS Data **********/
  WriteData(latSipAddr, (int)(SIPData.GPSpos.lat*DEG2I), NIOS_QUEUE);
  WriteData(lonSipAddr, (int)(SIPData.GPSpos.lon*DEG2I), NIOS_QUEUE);
  WriteData(altSipAddr, (int)(SIPData.GPSpos.alt), NIOS_QUEUE);
  WriteData(timeSipAddr, SIPData.GPStime.UTC, NIOS_QUEUE);

  /********** SIP MKS Altitude ************/
  WriteData(mksLoSipAddr, (int)(SIPData.MKSalt.lo), NIOS_QUEUE);
  WriteData(mksMedSipAddr, (int)(SIPData.MKSalt.med), NIOS_QUEUE);
  WriteData(mksHiSipAddr, (int)(SIPData.MKSalt.hi), NIOS_QUEUE);

  /************* processed pointing data *************/
  WriteData(raAddr, (unsigned int)(PointingData[i_point].ra * H2LI),
      NIOS_QUEUE);
  WriteData(decAddr, (unsigned int)(PointingData[i_point].dec * DEG2LI),
      NIOS_QUEUE);

  WriteData(OffsetIFelGYAddr,
      (signed int)(PointingData[i_point].offset_ifel_gy * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFelGYiscAddr,
      (signed int)(PointingData[i_point].offset_ifel_gy_isc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFrollGYiscAddr,
      (signed int)(PointingData[i_point].offset_ifroll_gy_isc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFyawGYiscAddr,
      (signed int)(PointingData[i_point].offset_ifyaw_gy_isc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFelGYoscAddr,
      (signed int)(PointingData[i_point].offset_ifel_gy_osc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFrollGYoscAddr,
      (signed int)(PointingData[i_point].offset_ifroll_gy_osc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFyawGYoscAddr,
      (signed int)(PointingData[i_point].offset_ifyaw_gy_osc * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFrollGYAddr,
      (signed int)(PointingData[i_point].offset_ifroll_gy * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFyawGYAddr,
      (signed int)(PointingData[i_point].offset_ifyaw_gy * 32768.), NIOS_QUEUE);
  WriteData(latAddr, (unsigned int)(PointingData[i_point].lat * DEG2LI),
      NIOS_QUEUE);
  WriteData(lonAddr, (unsigned int)(PointingData[i_point].lon * DEG2LI),
      NIOS_QUEUE);
  WriteData(altAddr, (unsigned int)(PointingData[i_point].alt), NIOS_QUEUE);

  WriteData(mcpFrameAddr, PointingData[i_point].mcp_frame, NIOS_QUEUE);
  WriteData(lstAddr, PointingData[i_point].lst, NIOS_QUEUE);

  WriteData(azMagAddr,
      (unsigned int)((PointingData[i_point].mag_az +
                      CommandData.mag_az_trim) * DEG2I), NIOS_QUEUE);
  WriteData(pitchMagAddr,
      (unsigned int)(ACSData.mag_pitch * DEG2I), NIOS_QUEUE);
  WriteData(declinationMagAddr,
      (unsigned int)(PointingData[i_point].mag_model * DEG2I), NIOS_QUEUE);
  WriteData(sigmaMagAddr,
      (unsigned int)(PointingData[i_point].mag_sigma * DEG2I), NIOS_QUEUE);
  WriteData(trimMagAddr, CommandData.mag_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(dgpsAzAddr,
      (unsigned int)((PointingData[i_point].dgps_az  +
                      CommandData.dgps_az_trim) * DEG2I), NIOS_QUEUE);
  WriteData(dgpsPitchAddr,
      (unsigned int)(PointingData[i_point].dgps_pitch * DEG2I), NIOS_QUEUE);
  WriteData(dgpsRollAddr,
      (unsigned int)(PointingData[i_point].dgps_roll * DEG2I), NIOS_QUEUE);
  WriteData(dgpsSigmaAddr,
      (unsigned int)(PointingData[i_point].dgps_sigma * DEG2I), NIOS_QUEUE);
  WriteData(dgpsTrimAddr, CommandData.dgps_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(azSsAddr, (unsigned int)((PointingData[i_point].ss_az +
          CommandData.ss_az_trim) * DEG2I),
      NIOS_QUEUE);
  WriteData(sigmaSsAddr,
      (unsigned int)(PointingData[i_point].ss_sigma * DEG2I), NIOS_QUEUE);
  WriteData(azSunAddr, (unsigned int)(PointingData[i_point].sun_az*DEG2I),
      NIOS_QUEUE);
  WriteData(elSunAddr, (int)(PointingData[i_point].sun_el*DEG2I), NIOS_QUEUE);
  WriteData(trimSsAddr, CommandData.ss_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(modeCalAddr, CommandData.Cryo.calibrator, NIOS_QUEUE);
  WriteData(periodCalAddr, CommandData.Cryo.calib_period, NIOS_QUEUE);

  WriteData(azIscAddr,
      (unsigned int)(PointingData[i_point].isc_az * DEG2I), NIOS_QUEUE);
  WriteData(elIscAddr,
      (unsigned int)(PointingData[i_point].isc_el * DEG2I), NIOS_QUEUE);
  WriteData(sigmaIscAddr,
      (unsigned int)(PointingData[i_point].isc_sigma * DEG2I), NIOS_QUEUE);

  WriteData(azOscAddr,
      (unsigned int)(PointingData[i_point].osc_az * DEG2I), NIOS_QUEUE);
  WriteData(elOscAddr,
      (unsigned int)(PointingData[i_point].osc_el * DEG2I), NIOS_QUEUE);
  WriteData(sigmaOscAddr,
      (unsigned int)(PointingData[i_point].osc_sigma * DEG2I), NIOS_QUEUE);

  WriteData(trimEncAddr, CommandData.enc_el_trim * DEG2I, NIOS_QUEUE);

  WriteData(elClinAddr,
      (unsigned int)((PointingData[i_point].clin_el +
                      CommandData.clin_el_trim) * DEG2I), NIOS_QUEUE);
  WriteData(sigmaClinAddr,
      (unsigned int)(PointingData[i_point].clin_sigma * DEG2I), NIOS_QUEUE);
  WriteData(trimClinAddr, CommandData.clin_el_trim * DEG2I, NIOS_QUEUE);

  WriteData(trimNullAddr, CommandData.null_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(azGyAddr,
      (int)(PointingData[i_point].v_az * 32768.0/20.0), NIOS_QUEUE);

  /************* Pointing mode fields *************/
  WriteData(slewVetoAddr, (int)(CommandData.pointing_mode.nw) / 4.,
      NIOS_QUEUE);
  WriteData(svetoLenAddr, (int)(CommandData.slew_veto) / 4., NIOS_QUEUE);
  WriteData(modePAddr, (int)(CommandData.pointing_mode.mode), NIOS_QUEUE);
  if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) ||
      (CommandData.pointing_mode.mode == P_AZ_SCAN))
    WriteData(xPAddr, (int)(CommandData.pointing_mode.X * DEG2I),
        NIOS_QUEUE);
  else
    WriteData(xPAddr, (int)(CommandData.pointing_mode.X * H2I), NIOS_QUEUE);

  WriteData(yPAddr, (int)(CommandData.pointing_mode.Y * DEG2I), NIOS_QUEUE);
  WriteData(velAzPAddr, (int)(CommandData.pointing_mode.vaz*VEL2I), NIOS_QUEUE);
  WriteData(delPAddr, (int)(CommandData.pointing_mode.del * VEL2I), NIOS_QUEUE);
  WriteData(wPAddr, (int)(CommandData.pointing_mode.w * DEG2I), NIOS_QUEUE);
  WriteData(hPAddr, (int)(CommandData.pointing_mode.h * DEG2I), NIOS_QUEUE);
  WriteData(ra1PAddr, (int)(CommandData.pointing_mode.ra[0] * H2I), NIOS_QUEUE);
  WriteData(dec1PAddr, (int)(CommandData.pointing_mode.dec[0] * DEG2I),
      NIOS_QUEUE);
  WriteData(ra2PAddr, (int)(CommandData.pointing_mode.ra[1] * H2I), NIOS_QUEUE);
  WriteData(dec2PAddr, (int)(CommandData.pointing_mode.dec[1] * DEG2I),
      NIOS_QUEUE);
  WriteData(ra3PAddr, (int)(CommandData.pointing_mode.ra[2] * H2I), NIOS_QUEUE);
  WriteData(dec3PAddr, (int)(CommandData.pointing_mode.dec[2] * DEG2I),
      NIOS_QUEUE);
  WriteData(ra4PAddr, (int)(CommandData.pointing_mode.ra[3] * H2I), NIOS_QUEUE);
  WriteData(dec4PAddr, (int)(CommandData.pointing_mode.dec[3] * DEG2I),
      NIOS_QUEUE);

  sensor_veto = (!CommandData.use_sun) | ((!CommandData.use_isc) << 1) |
    ((!CommandData.use_elenc) << 2) |
    ((!CommandData.use_mag) << 3) |
    ((!CommandData.use_gps) << 4) |
    ((!CommandData.use_elclin) << 5) |
    ((!CommandData.use_osc) << 6) |
    ((CommandData.disable_el) << 10) |
    ((CommandData.disable_az) << 11) |
    ((CommandData.force_el) << 12);

  if (PointingData[i_point].t >= CommandData.pointing_mode.t)
    sensor_veto |= (1 << 7);

  sensor_veto |= (CommandData.az_autogyro << 8);
  sensor_veto |= (CommandData.el_autogyro << 9);

  WriteData(vetoSensorAddr, sensor_veto, NIOS_QUEUE);

  /************* dgps fields *************/
  WriteData(dgpsTimeAddr, DGPSTime, NIOS_QUEUE);

  /** Pos fields **/
  i_dgps = GETREADINDEX(dgpspos_index);
  WriteData(dgpsLatAddr, (int)(DGPSPos[i_dgps].lat * DEG2I), NIOS_QUEUE);
  WriteData(dgpsLonAddr, (int)(DGPSPos[i_dgps].lon * DEG2I), NIOS_QUEUE);
  WriteData(dgpsAltAddr, (int)(DGPSPos[i_dgps].alt), NIOS_QUEUE);
  WriteData(dgpsSpeedAddr, (int)(DGPSPos[i_dgps].speed * DEG2I), NIOS_QUEUE);
  WriteData(dgpsDirAddr, (int)(DGPSPos[i_dgps].direction * DEG2I), NIOS_QUEUE);
  WriteData(dgpsClimbAddr, (int)(DGPSPos[i_dgps].climb * DEG2I), NIOS_QUEUE);
  WriteData(dgpsNSatAddr, DGPSPos[i_dgps].n_sat, NIOS_QUEUE);

  WriteData(lstSchedAddr, sched_lst, NIOS_QUEUE);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);
  WriteData(dgpsAzRawAddr, DGPSAtt[i_dgps].az * DEG2I, NIOS_QUEUE);
  WriteData(dgpsPitchRawAddr, DGPSAtt[i_dgps].pitch * DEG2I, NIOS_QUEUE);
  WriteData(dgpsRollRawAddr, DGPSAtt[i_dgps].roll * DEG2I, NIOS_QUEUE);
  WriteData(dgpsAttOkAddr, DGPSAtt[i_dgps].att_ok, NIOS_QUEUE);
  WriteData(tMCRWAddr,RWMotorData[i_rw_motors].temp,NIOS_QUEUE);
  WriteData(iSerRWAddr,((int)(RWMotorData[i_rw_motors].current/30.0*32768.0)),NIOS_QUEUE);
  WriteData(stat1RWAddr,(RWMotorData[i_rw_motors].status & 0xffff),NIOS_QUEUE);
  WriteData(stat2RWAddr,((RWMotorData[i_rw_motors].status & 0xffff0000)>> 16),NIOS_QUEUE);
  WriteData(faultRWAddr,RWMotorData[i_rw_motors].fault_reg,NIOS_QUEUE);
  WriteData(infoRWAddr,RWMotorData[i_rw_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsRWAddr,RWMotorData[i_rw_motors].err_count,NIOS_QUEUE);
  WriteData(tMCElAddr,ElevMotorData[i_elev_motors].temp,NIOS_QUEUE);
  WriteData(iSerElAddr,((int)(ElevMotorData[i_elev_motors].current/30.0*32768.0)),NIOS_QUEUE);
  WriteData(stat1ElAddr,(ElevMotorData[i_elev_motors].status & 0xffff),NIOS_QUEUE);
  WriteData(stat2ElAddr,((ElevMotorData[i_elev_motors].status & 0xffff0000)>> 16),NIOS_QUEUE);
  WriteData(faultElAddr,ElevMotorData[i_elev_motors].fault_reg,NIOS_QUEUE);
  WriteData(iSerPivAddr,PivotMotorData[i_pivot_motors].current*32768.0/20.0,NIOS_QUEUE);
  WriteData(statDrPivAddr,(PivotMotorData[i_pivot_motors].db_stat & 0xff)
                 +((PivotMotorData[i_pivot_motors].dp_stat & 0xff)<< 8),NIOS_QUEUE);
  WriteData(statS1PivAddr,PivotMotorData[i_pivot_motors].ds1_stat,NIOS_QUEUE);
  WriteData(velSerPivAddr,PivotMotorData[i_pivot_motors].dps_piv,NIOS_QUEUE);

  WriteData(infoElAddr,ElevMotorData[i_elev_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsElAddr,ElevMotorData[i_elev_motors].err_count,NIOS_QUEUE);
  WriteData(infoPivAddr,PivotMotorData[i_pivot_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsPivAddr,PivotMotorData[i_pivot_motors].err_count,NIOS_QUEUE);
  WriteData(verboseRWAddr,CommandData.verbose_rw,NIOS_QUEUE);
  WriteData(verboseElAddr,CommandData.verbose_el,NIOS_QUEUE);
  WriteData(verbosePivAddr,CommandData.verbose_piv,NIOS_QUEUE);

}
#endif

void InitTxFrame(unsigned short *RxFrame)
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
  UpdateBBCFrame(RxFrame);
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

void UpdateBBCFrame(unsigned short *RxFrame)
{
  static int index = 0;

  /*** do Controls ***/
#ifndef BOLOTEST
  if (!mcp_initial_controls)
    DoSched();
  UpdateAxesMode();
  StoreData(index);
  ControlGyroHeat(RxFrame);
  WriteMot(index, RxFrame);
#endif
#ifdef USE_XY_THREAD
  StoreStageBus(index);
#endif
  CryoControl(index);
  BiasControl(RxFrame);
  WriteChatter(index);

  /*** do slow Controls ***/
  if (index == 0) {
    if (!mcp_initial_controls)
      SyncADC();
    WriteAux();
    StoreActBus();
    SecondaryMirror();
    PhaseControl();
    StoreHWPRBus();
#ifndef BOLOTEST
    SetGyroMask();
    ChargeController();
    ControlPower();
#endif
  }

  if (!mcp_initial_controls)
    index = (index + 1) % FAST_PER_SLOW;

#ifndef BOLOTEST
  ControlAuxMotors(RxFrame);
  CameraTrigger(0); /* isc */
  CameraTrigger(1); /* osc */
#endif
}
