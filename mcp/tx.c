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

#define NIOS_BUFFER_SIZE 100

extern short int SamIAm;
short int InCharge;

extern struct AxesModeStruct axes_mode; /* motors.c */

extern struct ISCStatusStruct ISCSentState[2];  /* isc.c */

extern unsigned int sched_lst; /* sched_lst */

extern int bbc_fp;

double round(double x);

/* in actuators.c */
void StoreActBus(void);
void SecondaryMirror(void);

/* in auxiliary.c */
void ChargeController(void);
void ControlAuxMotors(unsigned short *RxFrame);
void ControlGyroHeat(unsigned short *RxFrame);
void CameraTrigger(int which);
void ControlPower(void);

/* in das.c */
void BiasControl(unsigned short* RxFrame);
void CryoControl(void);
void ForceBiasCheck(void);
void PhaseControl(void);

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex, unsigned short *RxFrame);

/* in sched.c */
void DoSched();

/* in starpos.c */
double getlst(time_t t, double lon);

/* in xystage.c */
void StoreStageBus(void);

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
  static struct NiosStruct* cpuTimeAddr;
  static struct NiosStruct* cpuTimeuSAddr;
  static struct NiosStruct* diskFreeAddr;
  static struct NiosStruct* aliceFileAddr;
  static struct NiosStruct* timeoutAddr;
  static struct NiosStruct* cpuTemp1Addr;
  static struct NiosStruct* cpuTemp2Addr;
  static struct NiosStruct* cpuTemp3Addr;
  static struct NiosStruct* samIAmAddr;
  static struct NiosStruct* bi0FifoSizeAddr;
  static struct NiosStruct* bbcFifoSizeAddr;
  static struct NiosStruct* ploverAddr;
  static struct NiosStruct* atFloatAddr;
  static struct NiosStruct* scheduleAddr;
  static struct NiosStruct* he4LevOldAddr;
  static struct BiPhaseStruct* samIAmReadAddr;
  static struct BiPhaseStruct* he4LevReadAddr;
  static int incharge = -1;
  time_t t;
  int i_point;
  struct timeval tv;
  struct timezone tz;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    samIAmAddr = GetNiosAddr("sam_i_am");
    samIAmReadAddr = ExtractBiPhaseAddr(samIAmAddr);

    he4LevOldAddr = GetNiosAddr("he4_lev_old");
    he4LevReadAddr = GetBiPhaseAddr("he4_lev");

    cpuTemp1Addr = GetNiosAddr("cpu_temp1");
    cpuTemp2Addr = GetNiosAddr("cpu_temp2");
    cpuTemp3Addr = GetNiosAddr("cpu_temp3");
    cpuTimeAddr = GetNiosAddr("cpu_time");
    cpuTimeuSAddr = GetNiosAddr("cpu_usec");
    diskFreeAddr = GetNiosAddr("disk_free");
    aliceFileAddr = GetNiosAddr("alice_file");
    timeoutAddr = GetNiosAddr("timeout");
    bi0FifoSizeAddr = GetNiosAddr("bi0_fifo_size");
    bbcFifoSizeAddr = GetNiosAddr("bbc_fifo_size");
    ploverAddr = GetNiosAddr("plover");
    atFloatAddr = GetNiosAddr("at_float");
    scheduleAddr = GetNiosAddr("schedule");
  }

  InCharge = !(SamIAm
      ^ slow_data[samIAmReadAddr->index][samIAmReadAddr->channel]);
  if (InCharge != incharge && InCharge) {
    bputs(info, "System: I have gained control.\n");
    CommandData.actbus.force_repoll = 1;
  } else if (InCharge != incharge)
    bputs(info, "System: I have lost control.\n");

  if (CommandData.Cryo.heliumLevel)
    CommandData.Cryo.he4_lev_old
      = slow_data[he4LevReadAddr->index][he4LevReadAddr->channel];

  WriteData(he4LevOldAddr, CommandData.Cryo.he4_lev_old, NIOS_QUEUE);

  incharge = InCharge;

  gettimeofday(&tv, &tz);

  WriteData(cpuTimeAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  WriteData(cpuTimeuSAddr, tv.tv_usec, NIOS_QUEUE);

  WriteData(cpuTemp1Addr, CommandData.temp1, NIOS_QUEUE);
  WriteData(cpuTemp2Addr, CommandData.temp2, NIOS_QUEUE);
  WriteData(cpuTemp3Addr, CommandData.temp3, NIOS_QUEUE);

  WriteData(samIAmAddr, SamIAm, NIOS_QUEUE);
  WriteData(diskFreeAddr, CommandData.df, NIOS_QUEUE);

  i_point = GETREADINDEX(point_index);

#ifdef BOLOTEST
  t = mcp_systime(NULL);
#else
  t = PointingData[i_point].t;
#endif

  WriteData(aliceFileAddr, CommandData.alice_file, NIOS_QUEUE);
  WriteData(timeoutAddr, CommandData.pointing_mode.t - t, NIOS_QUEUE);
  WriteData(bi0FifoSizeAddr, CommandData.bi0FifoSize, NIOS_QUEUE);
  WriteData(bbcFifoSizeAddr, CommandData.bbcFifoSize, NIOS_QUEUE);
  WriteData(ploverAddr, CommandData.plover, NIOS_QUEUE);
  WriteData(atFloatAddr, CommandData.at_float, NIOS_QUEUE);
  WriteData(scheduleAddr, CommandData.sucks + CommandData.lat_range * 2,
      NIOS_FLUSH);
}

/***************************************************************/
/* SetGyroMask:                                                */
/* mask gyros - automatic masking (if the gyro is faulty)      */
/*           or- commanded masking                             */
/* power cycle gyros - if masked for 1s                        */
/*                and- hasn't been power cycled in the last 5s */
/***************************************************************/
#define MASK_TIMEOUT 5 /* 1 sec -- in 5Hz Frames */ 
#define GYRO_ON 25 /* 5 sec */
#define PCYCLE_TIMEOUT 125 /* 25 sec */
void SetGyroMask (void)
{
static struct NiosStruct* gymaskAddr;
gymaskAddr = GetNiosAddr("gyro_mask");
unsigned int GyroMask = 0x3f; //all gyros enabled
int convert[6] = {1,5,0,6,3,4};//order of gyros in power switching
static int t_mask[6] = {0,0,0,0,0,0};
static int wait[6] = {0,0,0,0,0,0};
static int off[6] = {0,0,0,0,0,0};//1=gyro is off, 0=gyro is on
static struct BiPhaseStruct* gyfaultAddr;
gyfaultAddr = GetBiPhaseAddr("gyro_fault");;
unsigned int GyroFault;
GyroFault = slow_data[gyfaultAddr->index][gyfaultAddr->channel];
int i;
for (i=0; i<6; i++) {
  int j = convert[i];
  if (GyroFault & (0x01 << i)) {
    GyroMask &= ~(0x01 << i);
    t_mask[i] +=1;
    if (t_mask[i] > MASK_TIMEOUT) {
      if (wait[i] == 0) {
	CommandData.power.gyro_off[j] |= 0x01;
	off[i] = 1;
      }
    }
  }
  else if ((CommandData.gymask & (0x01 << i)) == 0 ) {
    GyroMask &= ~(0x01 << i);
  }
  else {
    GyroMask |= 0x01 << i;	
    t_mask[i] = 0;
  }
  
  if (off[i]) wait[i] +=1;
  if (wait[i] == GYRO_ON) CommandData.power.gyro_off[j] &= ~0x01;
  if (wait[i] > PCYCLE_TIMEOUT) {
    wait[i] = 0;
    off[i] = 0;
  }
}

WriteData(gymaskAddr, GyroMask, NIOS_QUEUE);
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
    l = sync_nums[m];
    k = slow_data[statusAddr[m]->index][statusAddr[m]->channel];

    if ((k & 0x3) == 0x1) {
      /* board is up, but needs to be synced */
      if (!doingSync[m])
        bprintf(info, "ADC Sync: node %i asserted\n", l);
      doingSync[m] = BBC_ADC_SYNC;
    } else {
      if (doingSync[m]) {
        bprintf(info, "ADC Sync: node %i deasserted\n", l);
        if (l == 8)
          ForceBiasCheck();
      }
      doingSync[m] = 0;
    }

    /* update the serial if we got a good response last time */
    if ((k & 0xfffc) == serial[m]) {
      if (serial[m] == 0xeb90)
        serial[m] = (~0xeb90) & 0xfffc;
      else
        serial[m] = 0xeb90;
    }

    RawNiosWrite(syncAddr[m]->niosAddr, BBC_WRITE | BBC_NODE(l) | BBC_CH(63)
        | doingSync[m] | (serial[m] & 0xfffc) | 0x3, NIOS_FLUSH);
  }
}

#ifndef BOLOTEST
static struct NiosStruct* GetSCNiosAddr(char* field, int which)
{
  char buffer[FIELD_LEN];
  sprintf(buffer, "%s_%s", which ? "osc" : "isc", field);

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
  static struct NiosStruct* HoldIAddr[2];
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
    HoldIAddr[which] = GetSCNiosAddr("hold_i", which);
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

    Temp1Addr[0] = GetNiosAddr("t_isc_flange");
    Temp2Addr[0] = GetNiosAddr("t_isc_heat");
    Temp3Addr[0] = GetNiosAddr("t_isc_lens");
    Temp4Addr[0] = GetNiosAddr("t_isc_comp");
    Temp1Addr[1] = GetNiosAddr("t_osc_flange");
    Temp2Addr[1] = GetNiosAddr("t_osc_heat");
    Temp3Addr[1] = GetNiosAddr("t_osc_lens");
    Temp4Addr[1] = GetNiosAddr("t_osc_comp");
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
  WriteData(HoldIAddr[which], (unsigned int)(ISCSentState[which].hold_current),
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

  static struct NiosStruct* ssAzRelSunAddr;
  static struct NiosStruct* ssPhaseAddr;
  static struct NiosStruct* ssSnrAddr;
  static struct NiosStruct* ssSunTimeAddr;
  static struct NiosStruct* ssCpuTempAddr;
  static struct NiosStruct* ssHddTempAddr;
  static struct NiosStruct* ssCaseTempAddr;
  static struct NiosStruct* ssPortTempAddr;
  static struct NiosStruct* ssStarTempAddr;
  static struct NiosStruct* ssV5Addr;
  static struct NiosStruct* ssV12Addr;
  static struct NiosStruct* ssVBattAddr;
  static struct NiosStruct* ssRaw01Addr;
  static struct NiosStruct* ssRaw02Addr;
  static struct NiosStruct* ssRaw03Addr;
  static struct NiosStruct* ssRaw04Addr;
  static struct NiosStruct* ssRaw05Addr;
  static struct NiosStruct* ssRaw06Addr;
  static struct NiosStruct* ssRaw07Addr;
  static struct NiosStruct* ssRaw08Addr;
  static struct NiosStruct* ssRaw09Addr;
  static struct NiosStruct* ssRaw10Addr;
  static struct NiosStruct* ssRaw11Addr;
  static struct NiosStruct* ssRaw12Addr;

  static struct NiosStruct* sipLatAddr;
  static struct NiosStruct* sipLonAddr;
  static struct NiosStruct* sipAltAddr;
  static struct NiosStruct* sipTimeAddr;
  static struct NiosStruct* sipMksLoAddr;
  static struct NiosStruct* sipMksMedAddr;
  static struct NiosStruct* sipMksHiAddr;

  /** pointing mode indexes **/
  static struct NiosStruct* svetoLenAddr;
  static struct NiosStruct* slewVetoAddr;
  static struct NiosStruct* pModeAddr;
  static struct NiosStruct* pXDegAddr, *pYAddr;
  static struct NiosStruct* pVazAddr, *pDelAddr;
  static struct NiosStruct* pWAddr, *pHAddr;
  static struct NiosStruct* pRa1Addr, *pDec1Addr;
  static struct NiosStruct* pRa2Addr, *pDec2Addr;
  static struct NiosStruct* pRa3Addr, *pDec3Addr;
  static struct NiosStruct* pRa4Addr, *pDec4Addr;

  static struct NiosStruct* sensorVetoAddr;

  /** derived pointing data */
  static struct NiosStruct* mcpFrameAddr;
  static struct NiosStruct* gyIFelOffsetAddr;
  static struct NiosStruct* iscGyIFelOffsetAddr;
  static struct NiosStruct* iscGyIFrollOffsetAddr;
  static struct NiosStruct* iscGyIFyawOffsetAddr;
  static struct NiosStruct* oscGyIFelOffsetAddr;
  static struct NiosStruct* oscGyIFrollOffsetAddr;
  static struct NiosStruct* oscGyIFyawOffsetAddr;
  static struct NiosStruct* gyIFrollOffsetAddr;
  static struct NiosStruct* gyIFyawOffsetAddr;
  static struct NiosStruct* azAddr;
  static struct NiosStruct* elAddr;
  static struct NiosStruct* raAddr;
  static struct NiosStruct* decAddr;
  static struct NiosStruct* altAddr;
  static struct NiosStruct* latAddr;
  static struct NiosStruct* lonAddr;
  static struct NiosStruct* timeAddr;
  static struct NiosStruct* lstAddr;
  static struct NiosStruct* magAzAddr;
  static struct NiosStruct* magPitchAddr;
  static struct NiosStruct* magModelAddr;
  static struct NiosStruct* magSigmaAddr;
  static struct NiosStruct* dgpsAzAddr;
  static struct NiosStruct* dgpsPitchAddr;
  static struct NiosStruct* dgpsRollAddr;
  static struct NiosStruct* dgpsSigmaAddr;
  static struct NiosStruct* ssAzAddr;
  static struct NiosStruct* ssSigmaAddr;
  static struct NiosStruct* sunAzAddr;
  static struct NiosStruct* sunElAddr;
  static struct NiosStruct* iscAzAddr;
  static struct NiosStruct* iscElAddr;
  static struct NiosStruct* iscSigmaAddr;
  static struct NiosStruct* oscAzAddr;
  static struct NiosStruct* oscElAddr;
  static struct NiosStruct* oscSigmaAddr;
  static struct NiosStruct* encElAddr;
  static struct NiosStruct* encSigmaAddr;
  static struct NiosStruct* clinElAddr;
  static struct NiosStruct* clinSigmaAddr;

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
  static struct NiosStruct* dgpsAttIndexAddr;
  static struct NiosStruct* dgpsPosIndexAddr;
  static struct NiosStruct* dgpsNSatAddr;

  /* trim fields */
  static struct NiosStruct *clinTrimAddr;
  static struct NiosStruct *encTrimAddr;
  static struct NiosStruct *nullTrimAddr;
  static struct NiosStruct *magTrimAddr;
  static struct NiosStruct *dgpsTrimAddr;
  static struct NiosStruct *ssTrimAddr;

  static struct NiosStruct *calModeAddr;
  static struct NiosStruct *schedLstAddr;

  /* low level scan mode diagnostics */
  static struct NiosStruct *azModeAddr;
  static struct NiosStruct *elModeAddr;
  static struct NiosStruct *azDirAddr;
  static struct NiosStruct *elDirAddr;
  static struct NiosStruct *azDestAddr;
  static struct NiosStruct *elDestAddr;
  static struct NiosStruct *azVelAddr;
  static struct NiosStruct *elVelAddr;

  /* Motor data read out over serial threads in motors.c */
  static struct NiosStruct *rwEncVel;
  static struct NiosStruct *rwTempAddr;
  static struct NiosStruct *rwIRawAddr;
  static struct NiosStruct *rwStat1Addr;
  static struct NiosStruct *rwStat2Addr;
  static struct NiosStruct *rwFaultAddr;
  static struct NiosStruct *elevEncPos;
  static struct NiosStruct *elTempAddr;
  static struct NiosStruct *elIRawAddr;
  static struct NiosStruct *elStat1Addr;
  static struct NiosStruct *elStat2Addr;
  static struct NiosStruct *elFaultAddr;
  static struct NiosStruct *resPivRawAddr;
  static struct NiosStruct *pivIRawAddr;
  static struct NiosStruct *pivDStatAddr;
  static struct NiosStruct *pivS1StatAddr;


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

    ssAzRelSunAddr = GetNiosAddr("ss_az_rel_sun");
    ssPhaseAddr = GetNiosAddr("ss_phase");
    ssSnrAddr = GetNiosAddr("ss_snr");
    ssSunTimeAddr = GetNiosAddr("ss_sun_time");
    ssCpuTempAddr = GetNiosAddr("ss_cpu_temp");
    ssHddTempAddr = GetNiosAddr("ss_hdd_temp");
    ssCaseTempAddr = GetNiosAddr("ss_case_temp");
    ssPortTempAddr = GetNiosAddr("ss_port_temp");
    ssStarTempAddr = GetNiosAddr("ss_star_temp");
    ssV5Addr = GetNiosAddr("ss_v_5");
    ssV12Addr = GetNiosAddr("ss_v_12");
    ssVBattAddr = GetNiosAddr("ss_v_batt");
    ssRaw01Addr = GetNiosAddr("ss_raw_01");
    ssRaw02Addr = GetNiosAddr("ss_raw_02");
    ssRaw03Addr = GetNiosAddr("ss_raw_03");
    ssRaw04Addr = GetNiosAddr("ss_raw_04");
    ssRaw05Addr = GetNiosAddr("ss_raw_05");
    ssRaw06Addr = GetNiosAddr("ss_raw_06");
    ssRaw07Addr = GetNiosAddr("ss_raw_07");
    ssRaw08Addr = GetNiosAddr("ss_raw_08");
    ssRaw09Addr = GetNiosAddr("ss_raw_09");
    ssRaw10Addr = GetNiosAddr("ss_raw_10");
    ssRaw11Addr = GetNiosAddr("ss_raw_11");
    ssRaw12Addr = GetNiosAddr("ss_raw_12");

    sipLatAddr = GetNiosAddr("sip_lat");
    sipLonAddr = GetNiosAddr("sip_lon");
    sipAltAddr = GetNiosAddr("sip_alt");
    sipTimeAddr = GetNiosAddr("sip_time");

    sipMksLoAddr = GetNiosAddr("sip_mks_lo");
    sipMksMedAddr = GetNiosAddr("sip_mks_med");
    sipMksHiAddr = GetNiosAddr("sip_mks_hi");

    gyIFelOffsetAddr = GetNiosAddr("gy_ifel_offset");
    iscGyIFelOffsetAddr = GetNiosAddr("isc_gy_ifel_off");
    iscGyIFrollOffsetAddr = GetNiosAddr("isc_gy_ifroll_off");
    iscGyIFyawOffsetAddr = GetNiosAddr("isc_gy_ifyaw_off");
    oscGyIFelOffsetAddr = GetNiosAddr("osc_gy_ifel_off");
    oscGyIFrollOffsetAddr = GetNiosAddr("osc_gy_ifroll_off");
    oscGyIFyawOffsetAddr = GetNiosAddr("osc_gy_ifyaw_off");
    gyIFrollOffsetAddr = GetNiosAddr("gy_ifroll_offset");
    gyIFyawOffsetAddr = GetNiosAddr("gy_ifyaw_offset");
    raAddr = GetNiosAddr("ra");
    decAddr = GetNiosAddr("dec");
    latAddr = GetNiosAddr("lat");
    altAddr = GetNiosAddr("alt");
    lonAddr = GetNiosAddr("lon");
    timeAddr = GetNiosAddr("time");
    lstAddr = GetNiosAddr("lst");
    magAzAddr = GetNiosAddr("mag_az");
    magPitchAddr = GetNiosAddr("mag_pitch");
    magModelAddr = GetNiosAddr("mag_model");
    magSigmaAddr = GetNiosAddr("mag_sigma");
    dgpsAzAddr = GetNiosAddr("dgps_az");
    dgpsPitchAddr = GetNiosAddr("dgps_pitch");
    dgpsRollAddr = GetNiosAddr("dgps_roll");
    dgpsSigmaAddr = GetNiosAddr("dgps_sigma");
    ssAzAddr = GetNiosAddr("ss_az");
    ssSigmaAddr = GetNiosAddr("ss_sigma");
    sunAzAddr = GetNiosAddr("sun_az");
    sunElAddr = GetNiosAddr("sun_el");
    ssTrimAddr = GetNiosAddr("ss_trim");
    calModeAddr = GetNiosAddr("cal_mode");
    iscAzAddr = GetNiosAddr("isc_az");
    iscElAddr = GetNiosAddr("isc_el");
    iscSigmaAddr = GetNiosAddr("isc_sigma");
    oscAzAddr = GetNiosAddr("osc_az");
    oscElAddr = GetNiosAddr("osc_el");
    oscSigmaAddr = GetNiosAddr("osc_sigma");
    encElAddr = GetNiosAddr("enc_el");
    encSigmaAddr = GetNiosAddr("enc_sigma");
    clinElAddr = GetNiosAddr("clin_el");
    clinSigmaAddr = GetNiosAddr("clin_sigma");

    svetoLenAddr = GetNiosAddr("sveto_len");
    slewVetoAddr = GetNiosAddr("slew_veto");
    pModeAddr = GetNiosAddr("p_mode");
    pXDegAddr = GetNiosAddr("p_x_deg");
    pYAddr = GetNiosAddr("p_y");
    pVazAddr = GetNiosAddr("p_vaz");
    pDelAddr = GetNiosAddr("p_del");
    pWAddr = GetNiosAddr("p_w");
    pHAddr = GetNiosAddr("p_h");
    pRa1Addr = GetNiosAddr("p_ra_1");
    pDec1Addr = GetNiosAddr("p_dec_1");
    pRa2Addr = GetNiosAddr("p_ra_2");
    pDec2Addr = GetNiosAddr("p_dec_2");
    pRa3Addr = GetNiosAddr("p_ra_3");
    pDec3Addr = GetNiosAddr("p_dec_3");
    pRa4Addr = GetNiosAddr("p_ra_4");
    pDec4Addr = GetNiosAddr("p_dec_4");

    sensorVetoAddr = GetNiosAddr("sensor_veto");

    dgpsTimeAddr = GetNiosAddr("dgps_time");
    dgpsLatAddr = GetNiosAddr("dgps_lat");
    dgpsLonAddr = GetNiosAddr("dgps_lon");
    dgpsAltAddr = GetNiosAddr("dgps_alt");
    dgpsSpeedAddr = GetNiosAddr("dgps_speed");
    dgpsDirAddr = GetNiosAddr("dgps_dir");
    dgpsClimbAddr = GetNiosAddr("dgps_climb");
    dgpsNSatAddr = GetNiosAddr("dgps_n_sat");
    dgpsPosIndexAddr = GetNiosAddr("dgps_pos_index");
    dgpsAttOkAddr = GetNiosAddr("dgps_att_ok");
    dgpsAzRawAddr = GetNiosAddr("dgps_az_raw");
    dgpsAttIndexAddr = GetNiosAddr("dgps_att_index");

    schedLstAddr = GetNiosAddr("sched_lst");

    clinTrimAddr = GetNiosAddr("clin_trim");
    encTrimAddr = GetNiosAddr("enc_trim");
    nullTrimAddr = GetNiosAddr("null_trim");
    magTrimAddr = GetNiosAddr("mag_trim");
    dgpsTrimAddr = GetNiosAddr("dgps_trim");

    azModeAddr = GetNiosAddr("az_mode");
    elModeAddr = GetNiosAddr("el_mode");
    azDestAddr = GetNiosAddr("az_dest");
    elDestAddr = GetNiosAddr("el_dest");
    azVelAddr = GetNiosAddr("az_vel");
    elVelAddr = GetNiosAddr("el_vel");
    azDirAddr = GetNiosAddr("az_dir");
    elDirAddr = GetNiosAddr("el_dir");

    rwEncVel = GetNiosAddr("rw_vel_raw");
    elevEncPos = GetNiosAddr("enc_el_raw");
    rwTempAddr = GetNiosAddr("rw_temp");
    rwIRawAddr = GetNiosAddr("rw_i_raw");
    rwStat1Addr = GetNiosAddr("rw_stat_1");
    rwStat2Addr = GetNiosAddr("rw_stat_2");
    rwFaultAddr = GetNiosAddr("rw_fault");
    elTempAddr = GetNiosAddr("el_temp");
    elIRawAddr = GetNiosAddr("el_i_raw");
    elStat1Addr = GetNiosAddr("el_stat_1");
    elStat2Addr = GetNiosAddr("el_stat_2");
    elFaultAddr = GetNiosAddr("el_fault");
    resPivRawAddr = GetNiosAddr("res_piv_raw");
    pivIRawAddr = GetNiosAddr("piv_i_raw");
    pivDStatAddr = GetNiosAddr("piv_d_stat");
    pivS1StatAddr = GetNiosAddr("piv_s1_stat");

  }

  i_point = GETREADINDEX(point_index);
  i_ss = GETREADINDEX(ss_index);
  i_rw_motors = GETREADINDEX(rw_motor_index);
  i_elev_motors = GETREADINDEX(elev_motor_index);
  i_pivot_motors = GETREADINDEX(pivot_motor_index);
  /* scan modes */
  WriteData(azModeAddr, axes_mode.az_mode, NIOS_QUEUE);
  WriteData(elModeAddr, axes_mode.el_mode, NIOS_QUEUE);
  WriteData(azDirAddr, axes_mode.az_dir, NIOS_QUEUE);
  WriteData(elDirAddr, axes_mode.el_dir, NIOS_QUEUE);
  WriteData(azDestAddr, axes_mode.az_dest * DEG2I, NIOS_QUEUE);
  WriteData(elDestAddr, axes_mode.el_dest * DEG2I, NIOS_QUEUE);
  WriteData(azVelAddr, axes_mode.az_vel * 6000., NIOS_QUEUE);
  WriteData(elVelAddr, axes_mode.el_vel * 6000., NIOS_QUEUE);

  /********** Sun Sensor Data **********/
  WriteData(ssPhaseAddr, PointingData[i_point].ss_phase * DEG2I, NIOS_QUEUE);
  WriteData(ssSnrAddr, PointingData[i_point].ss_snr * 1000, NIOS_QUEUE);
  WriteData(ssSunTimeAddr, SunSensorData[i_ss].sun_time, NIOS_QUEUE);
  WriteData(ssCpuTempAddr, SunSensorData[i_ss].t_cpu * 100, NIOS_QUEUE);
  WriteData(ssHddTempAddr, SunSensorData[i_ss].t_hdd * 100, NIOS_QUEUE);
  WriteData(ssCaseTempAddr, SunSensorData[i_ss].t_case * 100, NIOS_QUEUE);
  WriteData(ssPortTempAddr, SunSensorData[i_ss].t_port * 100, NIOS_QUEUE);
  WriteData(ssStarTempAddr, SunSensorData[i_ss].t_starboard * 100, NIOS_QUEUE);
  WriteData(ssV5Addr, SunSensorData[i_ss].v5 * 100, NIOS_QUEUE);
  WriteData(ssV12Addr, SunSensorData[i_ss].v12 * 100, NIOS_QUEUE);
  WriteData(ssVBattAddr, SunSensorData[i_ss].vbatt * 100, NIOS_QUEUE);
  WriteData(ssRaw01Addr, SunSensorData[i_ss].m01, NIOS_QUEUE);
  WriteData(ssRaw02Addr, SunSensorData[i_ss].m02, NIOS_QUEUE);
  WriteData(ssRaw03Addr, SunSensorData[i_ss].m03, NIOS_QUEUE);
  WriteData(ssRaw04Addr, SunSensorData[i_ss].m04, NIOS_QUEUE);
  WriteData(ssRaw05Addr, SunSensorData[i_ss].m05, NIOS_QUEUE);
  WriteData(ssRaw06Addr, SunSensorData[i_ss].m06, NIOS_QUEUE);
  WriteData(ssRaw07Addr, SunSensorData[i_ss].m07, NIOS_QUEUE);
  WriteData(ssRaw08Addr, SunSensorData[i_ss].m08, NIOS_QUEUE);
  WriteData(ssRaw09Addr, SunSensorData[i_ss].m09, NIOS_QUEUE);
  WriteData(ssRaw10Addr, SunSensorData[i_ss].m10, NIOS_QUEUE);
  WriteData(ssRaw11Addr, SunSensorData[i_ss].m11, NIOS_QUEUE);
  WriteData(ssRaw12Addr, SunSensorData[i_ss].m12, NIOS_QUEUE);
  WriteData(ssAzRelSunAddr, PointingData[i_point].ss_az_rel_sun * DEG2I,
      NIOS_QUEUE);
  /********** SIP GPS Data **********/
  WriteData(sipLatAddr, (int)(SIPData.GPSpos.lat*DEG2I), NIOS_QUEUE);
  WriteData(sipLonAddr, (int)(SIPData.GPSpos.lon*DEG2I), NIOS_QUEUE);
  WriteData(sipAltAddr, (int)(SIPData.GPSpos.alt), NIOS_QUEUE);
  WriteData(sipTimeAddr, SIPData.GPStime.UTC, NIOS_QUEUE);

  /********** SIP MKS Altitude ************/
  WriteData(sipMksLoAddr, (int)(SIPData.MKSalt.lo), NIOS_QUEUE);
  WriteData(sipMksMedAddr, (int)(SIPData.MKSalt.med), NIOS_QUEUE);
  WriteData(sipMksHiAddr, (int)(SIPData.MKSalt.hi), NIOS_QUEUE);

  /************* processed pointing data *************/

  // TODO
  WriteData(azAddr, (unsigned int)(PointingData[i_point].az * DEG2LI),
      NIOS_QUEUE);
  WriteData(elAddr, (unsigned int)(PointingData[i_point].el * DEG2LI),
      NIOS_QUEUE);

  WriteData(raAddr, (unsigned int)(PointingData[i_point].ra * H2LI),
      NIOS_QUEUE);
  WriteData(decAddr, (unsigned int)(PointingData[i_point].dec * DEG2LI),
      NIOS_QUEUE);

  WriteData(gyIFelOffsetAddr,
      (signed int)(PointingData[i_point].gy_ifel_offset * 32768.), NIOS_QUEUE);
  WriteData(iscGyIFelOffsetAddr,
      (signed int)(PointingData[i_point].isc_gy_ifel_offset * 32768.), NIOS_QUEUE);
  WriteData(iscGyIFrollOffsetAddr,
      (signed int)(PointingData[i_point].isc_gy_ifroll_offset * 32768.), NIOS_QUEUE);
  WriteData(iscGyIFyawOffsetAddr,
      (signed int)(PointingData[i_point].isc_gy_ifyaw_offset * 32768.), NIOS_QUEUE);
  WriteData(oscGyIFelOffsetAddr,
      (signed int)(PointingData[i_point].osc_gy_ifel_offset * 32768.), NIOS_QUEUE);
  WriteData(oscGyIFrollOffsetAddr,
      (signed int)(PointingData[i_point].osc_gy_ifroll_offset * 32768.), NIOS_QUEUE);
  WriteData(oscGyIFyawOffsetAddr,
      (signed int)(PointingData[i_point].osc_gy_ifyaw_offset * 32768.), NIOS_QUEUE);
  WriteData(gyIFrollOffsetAddr,
      (signed int)(PointingData[i_point].gy_ifroll_offset * 32768.), NIOS_QUEUE);
  WriteData(gyIFyawOffsetAddr,
      (signed int)(PointingData[i_point].gy_ifyaw_offset * 32768.), NIOS_QUEUE);
  WriteData(latAddr, (unsigned int)(PointingData[i_point].lat * DEG2LI),
      NIOS_QUEUE);
  WriteData(lonAddr, (unsigned int)(PointingData[i_point].lon * DEG2LI),
      NIOS_QUEUE);
  WriteData(altAddr, (unsigned int)(PointingData[i_point].alt), NIOS_QUEUE);

  WriteData(mcpFrameAddr, PointingData[i_point].mcp_frame, NIOS_QUEUE);
  WriteData(timeAddr, PointingData[i_point].t, NIOS_QUEUE);
  WriteData(lstAddr, PointingData[i_point].lst, NIOS_QUEUE);

  WriteData(magAzAddr,
      (unsigned int)((PointingData[i_point].mag_az +
                      CommandData.mag_az_trim) * DEG2I), NIOS_QUEUE);
  WriteData(magPitchAddr,
      (unsigned int)(ACSData.mag_pitch * DEG2I), NIOS_QUEUE);
  WriteData(magModelAddr,
      (unsigned int)(PointingData[i_point].mag_model * DEG2I), NIOS_QUEUE);
  WriteData(magSigmaAddr,
      (unsigned int)(PointingData[i_point].mag_sigma * DEG2I), NIOS_QUEUE);
  WriteData(magTrimAddr, CommandData.mag_az_trim * DEG2I, NIOS_QUEUE);

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

  WriteData(ssAzAddr, (unsigned int)((PointingData[i_point].ss_az +
          CommandData.ss_az_trim) * DEG2I),
      NIOS_QUEUE);
  WriteData(ssSigmaAddr,
      (unsigned int)(PointingData[i_point].ss_sigma * DEG2I), NIOS_QUEUE);
  WriteData(sunAzAddr, (unsigned int)(PointingData[i_point].sun_az*DEG2I),
      NIOS_QUEUE);
  WriteData(sunElAddr, (int)(PointingData[i_point].sun_el*DEG2I), NIOS_QUEUE);
  WriteData(ssTrimAddr, CommandData.ss_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(calModeAddr, CommandData.Cryo.calibrator, NIOS_QUEUE);

  WriteData(iscAzAddr,
      (unsigned int)(PointingData[i_point].isc_az * DEG2I), NIOS_QUEUE);
  WriteData(iscElAddr,
      (unsigned int)(PointingData[i_point].isc_el * DEG2I), NIOS_QUEUE);
  WriteData(iscSigmaAddr,
      (unsigned int)(PointingData[i_point].isc_sigma * DEG2I), NIOS_QUEUE);

  WriteData(oscAzAddr,
      (unsigned int)(PointingData[i_point].osc_az * DEG2I), NIOS_QUEUE);
  WriteData(oscElAddr,
      (unsigned int)(PointingData[i_point].osc_el * DEG2I), NIOS_QUEUE);
  WriteData(oscSigmaAddr,
      (unsigned int)(PointingData[i_point].osc_sigma * DEG2I), NIOS_QUEUE);

  WriteData(encElAddr,
      (unsigned int)((PointingData[i_point].enc_el
                      +CommandData.enc_el_trim)* DEG2I), NIOS_QUEUE);
  WriteData(encSigmaAddr,
      (unsigned int)(PointingData[i_point].enc_sigma * DEG2I), NIOS_QUEUE);
  WriteData(encTrimAddr, CommandData.enc_el_trim * DEG2I, NIOS_QUEUE);

  WriteData(clinElAddr,
      (unsigned int)((PointingData[i_point].clin_el +
                      CommandData.clin_el_trim) * DEG2I), NIOS_QUEUE);
  WriteData(clinSigmaAddr,
      (unsigned int)(PointingData[i_point].clin_sigma * DEG2I), NIOS_QUEUE);
  WriteData(clinTrimAddr, CommandData.clin_el_trim * DEG2I, NIOS_QUEUE);

  WriteData(nullTrimAddr, CommandData.null_az_trim * DEG2I, NIOS_QUEUE);

  /************* Pointing mode fields *************/
  WriteData(slewVetoAddr, (int)(CommandData.pointing_mode.nw) / 4.,
      NIOS_QUEUE);
  WriteData(svetoLenAddr, (int)(CommandData.slew_veto) / 4., NIOS_QUEUE);
  WriteData(pModeAddr, (int)(CommandData.pointing_mode.mode), NIOS_QUEUE);
  if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) ||
      (CommandData.pointing_mode.mode == P_AZ_SCAN))
    WriteData(pXDegAddr, (int)(CommandData.pointing_mode.X * DEG2I),
        NIOS_QUEUE);
  else
    WriteData(pXDegAddr, (int)(CommandData.pointing_mode.X * H2I), NIOS_QUEUE);

  WriteData(pYAddr, (int)(CommandData.pointing_mode.Y * DEG2I), NIOS_QUEUE);
  WriteData(pVazAddr, (int)(CommandData.pointing_mode.vaz * VEL2I), NIOS_QUEUE);
  WriteData(pDelAddr, (int)(CommandData.pointing_mode.del * VEL2I), NIOS_QUEUE);
  WriteData(pWAddr, (int)(CommandData.pointing_mode.w * DEG2I), NIOS_QUEUE);
  WriteData(pHAddr, (int)(CommandData.pointing_mode.h * DEG2I), NIOS_QUEUE);
  WriteData(pRa1Addr, (int)(CommandData.pointing_mode.ra[0] * H2I), NIOS_QUEUE);
  WriteData(pDec1Addr, (int)(CommandData.pointing_mode.dec[0] * DEG2I),
      NIOS_QUEUE);
  WriteData(pRa2Addr, (int)(CommandData.pointing_mode.ra[1] * H2I), NIOS_QUEUE);
  WriteData(pDec2Addr, (int)(CommandData.pointing_mode.dec[1] * DEG2I),
      NIOS_QUEUE);
  WriteData(pRa3Addr, (int)(CommandData.pointing_mode.ra[2] * H2I), NIOS_QUEUE);
  WriteData(pDec3Addr, (int)(CommandData.pointing_mode.dec[2] * DEG2I),
      NIOS_QUEUE);
  WriteData(pRa4Addr, (int)(CommandData.pointing_mode.ra[3] * H2I), NIOS_QUEUE);
  WriteData(pDec4Addr, (int)(CommandData.pointing_mode.dec[3] * DEG2I),
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

  WriteData(sensorVetoAddr, sensor_veto, NIOS_QUEUE);

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
  WriteData(dgpsPosIndexAddr, i_dgps, NIOS_QUEUE);

  WriteData(schedLstAddr, sched_lst, NIOS_QUEUE);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);
  WriteData(dgpsAzRawAddr, DGPSAtt[i_dgps].az * DEG2I, NIOS_QUEUE);
  WriteData(dgpsAttOkAddr, DGPSAtt[i_dgps].att_ok, NIOS_QUEUE);
  WriteData(dgpsAttIndexAddr, i_dgps, NIOS_QUEUE);
  WriteData(rwEncVel,((long int)(RWMotorData[i_rw_motors].rw_vel_raw/4.0*DEG2I)), NIOS_QUEUE);
  WriteData(elevEncPos,((long int)(ElevMotorData[i_elev_motors].enc_el_raw*DEG2I)), NIOS_QUEUE);
  WriteData(rwTempAddr,RWMotorData[i_rw_motors].temp,NIOS_QUEUE);
  WriteData(rwIRawAddr,((int)(RWMotorData[i_rw_motors].current/30.0*32768.0)),NIOS_QUEUE);
  WriteData(rwStat1Addr,(RWMotorData[i_rw_motors].status & 0xffff),NIOS_QUEUE);
  WriteData(rwStat2Addr,((RWMotorData[i_rw_motors].status & 0xffff0000)>> 16),NIOS_QUEUE);
  WriteData(rwFaultAddr,RWMotorData[i_rw_motors].fault_reg,NIOS_QUEUE);
  WriteData(elTempAddr,ElevMotorData[i_elev_motors].temp,NIOS_QUEUE);
  WriteData(elIRawAddr,((int)(ElevMotorData[i_elev_motors].current/30.0*32768.0)),NIOS_QUEUE);
  WriteData(elStat1Addr,(ElevMotorData[i_elev_motors].status & 0xffff),NIOS_QUEUE);
  WriteData(elStat2Addr,((ElevMotorData[i_elev_motors].status & 0xffff0000)>> 16),NIOS_QUEUE);
  WriteData(elFaultAddr,ElevMotorData[i_elev_motors].fault_reg,NIOS_QUEUE);
  WriteData(resPivRawAddr,PivotMotorData[i_pivot_motors].res_piv_raw*DEG2I,NIOS_QUEUE);
  WriteData(pivIRawAddr,PivotMotorData[i_pivot_motors].current*32768.0/20.0,NIOS_QUEUE);
  WriteData(pivDStatAddr,(PivotMotorData[i_pivot_motors].db_stat & 0xff)
                 +((PivotMotorData[i_pivot_motors].dp_stat & 0xff)<< 8),NIOS_QUEUE);
  WriteData(pivS1StatAddr,PivotMotorData[i_pivot_motors].ds1_stat,NIOS_QUEUE);
  StoreStarCameraData(index, 0); /* write ISC data */
  StoreStarCameraData(index, 1); /* write OSC data */
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
    if (n < counter * sizeof(unsigned int))
      bprintf(warning, "Frame Control: Short write to Nios");
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
  ControlPower();
  StoreActBus();
#endif
#ifdef USE_XY_THREAD
  StoreStageBus();
#endif
  BiasControl(RxFrame);

  /*** do slow Controls ***/
  if (index == 0) {
    if (!mcp_initial_controls)
      SyncADC();
    WriteAux();
    SecondaryMirror();
    CryoControl();
    PhaseControl();
#ifndef BOLOTEST
    SetGyroMask();
    ChargeController();
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
