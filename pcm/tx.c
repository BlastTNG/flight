/* mcp: the Spider master control program
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

/* tx.c */

/* NB: As of 7 Sep 2003 this file has been split up into four pieces:
 *
 * auxiliary.c: Auxiliary controls: Lock Motor, Pumps, Electronics Heat, ISC
 * hk.c:        Housekeeping, Bias, and Cryo controls
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
#include <limits.h>

#include "share/bbc_pci.h"

#include "share/channels.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"
#include "chrgctrl.h"

#define NIOS_BUFFER_SIZE 100

extern short int BitsyIAm;

extern int StartupVeto;

short int InCharge = 0;

int EthernetSun = 3;
int EthernetIsc = 3;
int EthernetOsc = 3;
int EthernetSBSC = 3;

extern struct AxesModeStruct axes_mode; /* motors.c */

extern unsigned int sched_lst; /* sched_lst */

extern int bbc_fp;

extern struct chat_buf chatter_buffer;  /* mcp.c */

double round(double x);

/* in actuators.c */
void StoreActBus(void);
void SecondaryMirror(void);

/* in hk.c */
void HouseKeeping(int);

/* in hwpr.c */
void StoreHWPRBus(void);

/* in auxiliary.c */
void ChargeController(void);
void ControlAuxMotors();
void ControlGyroHeat();
void CameraTrigger(int which);
void ControlPower(void);
void VideoTx(void);

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex);

/* in sbsc.cpp */
void cameraFields();        

/* in table.cpp */
void updateTableSpeed();

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
  static struct NiosStruct* rateTdrssAddr;
  static struct NiosStruct* rateIridiumAddr;
  static struct NiosStruct* tChipFlcAddr;
  static struct NiosStruct* tCpuFlcAddr;
  static struct NiosStruct* tMbFlcAddr;
  static struct NiosStruct* statusMCCAddr;
  static struct BiPhaseStruct* statusMCCReadAddr;
  static struct NiosStruct* bi0FifoSizeAddr;
  static struct NiosStruct* bbcFifoSizeAddr;
  static struct NiosStruct* ploverAddr;
  static struct NiosStruct* statusEthAddr;
  static struct NiosStruct* partsSchedAddr;
  static struct NiosStruct* upslotSchedAddr;
  
  static int incharge = -1;
  time_t t;
  int i_point;
  struct timeval tv;
  struct timezone tz;
  
  unsigned short mccstatus;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    statusMCCAddr = GetNiosAddr("status_mcc");
    statusMCCReadAddr = ExtractBiPhaseAddr(statusMCCAddr);

    tChipFlcAddr = GetNiosAddr("t_chip_flc");
    tCpuFlcAddr = GetNiosAddr("t_cpu_flc");
    tMbFlcAddr = GetNiosAddr("t_mb_flc");
    timeAddr = GetNiosAddr("time");
    timeUSecAddr = GetNiosAddr("time_usec");
    diskFreeAddr = GetNiosAddr("disk_free");
    timeoutAddr = GetNiosAddr("timeout");
    rateTdrssAddr = GetNiosAddr("rate_tdrss");
    rateIridiumAddr= GetNiosAddr("rate_iridium");
    bi0FifoSizeAddr = GetNiosAddr("bi0_fifo_size");
    bbcFifoSizeAddr = GetNiosAddr("bbc_fifo_size");
    ploverAddr = GetNiosAddr("plover");
    statusEthAddr = GetNiosAddr("status_eth");
    partsSchedAddr = GetNiosAddr("parts_sched");
    upslotSchedAddr = GetNiosAddr("upslot_sched");
  }

  if (StartupVeto>0) {
    InCharge = 0;
  } else {
    InCharge = !(BitsyIAm
	^ (slow_data[statusMCCReadAddr->index][statusMCCReadAddr->channel] & 0x1));
  //bprintf(info, "BitsyIAm = %d", BitsyIAm);
  //bprintf(info, "mcc status bit 1 = %d", (slow_data[statusMCCReadAddr->index][statusMCCReadAddr->channel] & 0x1));
  //bprintf(info, "InCharge = %d", InCharge);
  }
  if (InCharge != incharge && InCharge) {
    bprintf(info, "System: I, %s, have gained control.\n", BitsyIAm ? "Bitsy" : "Itsy");
    CommandData.actbus.force_repoll = 1;
  } else if (InCharge != incharge) {
    bprintf(info, "System: I, %s, have lost control.\n", BitsyIAm ? "Bitsy" : "Itsy");
  }

  incharge = InCharge;

  gettimeofday(&tv, &tz);

  WriteData(timeAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  WriteData(timeUSecAddr, tv.tv_usec, NIOS_QUEUE);

  WriteData(tChipFlcAddr, CommandData.temp1, NIOS_QUEUE);
  WriteData(tCpuFlcAddr, CommandData.temp2, NIOS_QUEUE);
  WriteData(tMbFlcAddr, CommandData.temp3, NIOS_QUEUE);

  WriteData(diskFreeAddr, CommandData.df, NIOS_QUEUE);
  
  WriteData(partsSchedAddr, CommandData.parts_sched&0xffffff, NIOS_QUEUE);
  WriteData(upslotSchedAddr, CommandData.upslot_sched, NIOS_QUEUE);

  i_point = GETREADINDEX(point_index);

#ifdef BOLOTEST
  t = mcp_systime(NULL);
#else
  t = PointingData[i_point].t;
#endif

  if (CommandData.pointing_mode.t > t) {
    WriteData(timeoutAddr, CommandData.pointing_mode.t - t, NIOS_QUEUE);
  } else {
    WriteData(timeoutAddr, 0, NIOS_QUEUE);
  }
    
  WriteData(bi0FifoSizeAddr, CommandData.bi0FifoSize, NIOS_QUEUE);
  WriteData(bbcFifoSizeAddr, CommandData.bbcFifoSize, NIOS_QUEUE);
  WriteData(ploverAddr, CommandData.plover, NIOS_QUEUE);
  WriteData(rateTdrssAddr, CommandData.tdrss_bw, NIOS_QUEUE);
  WriteData(rateIridiumAddr, CommandData.iridium_bw, NIOS_QUEUE);

  WriteData(statusEthAddr, 
       (EthernetSun & 0x3) + 
       ((EthernetIsc & 0x3) << 2) + 
       ((EthernetOsc & 0x3) << 4) +
       ((EthernetSBSC & 0x3) << 6),  
       NIOS_QUEUE);

  mccstatus =        
    (BitsyIAm ? 0x1 : 0x0) +                 //0x01
    (CommandData.at_float ? 0x2 : 0x0) +     //0x02
    (CommandData.uplink_sched ? 0x08 : 0x00) + //0x08
    (CommandData.sucks ? 0x10 : 0x00) +      //0x10
       //((CommandData.lat_range & 0x3) << 5) +   //0x60
    ((CommandData.slot_sched & 0xFF) << 8);  //0xFF00

  if (CommandData.uplink_sched) {
    mccstatus |= 0x60;
  } else {
    mccstatus |= ((CommandData.lat_range & 0x3) << 5);
  }
  
  WriteData(statusMCCAddr, mccstatus,
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
#define NUM_SYNC 19
const unsigned short sync_nums[NUM_SYNC] = {0,1,2,3,4,5,6,8,9,10,11,12,13,14,\
			15,16,17,18,19};
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

#ifndef BOLOTEST
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
  static struct NiosStruct* dithStepPAddr;

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
  static struct NiosStruct* azRawMagAddr;
  static struct NiosStruct* pitchMagAddr;
  static struct NiosStruct* declinationMagAddr;
  static struct NiosStruct* sigmaMagAddr;
  static struct NiosStruct* dgpsAzAddr;
  static struct NiosStruct* dgpsSigmaAddr;
  static struct NiosStruct* azSsAddr;
  static struct NiosStruct* azrawPss1Addr;
  static struct NiosStruct* elrawPss1Addr;
  static struct NiosStruct* snrPss1Addr;
  static struct NiosStruct* azPss1Addr;
  static struct NiosStruct* azrawPss2Addr;
  static struct NiosStruct* elrawPss2Addr;
  static struct NiosStruct* snrPss2Addr;
  static struct NiosStruct* azPss2Addr;
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
  static struct NiosStruct* elLutClinAddr;
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
  static struct NiosStruct* dgpsAzCovAddr;
  static struct NiosStruct* dgpsPitchCovAddr;
  static struct NiosStruct* dgpsRollCovAddr;
  static struct NiosStruct* dgpsAntEAddr;
  static struct NiosStruct* dgpsAntNAddr;
  static struct NiosStruct* dgpsAntUAddr;
  static struct NiosStruct *dgpsCovLimAddr;
  static struct NiosStruct *dgpsAntsLimAddr;
  static struct NiosStruct* dgpsNSatAddr;

  /* trim fields */
  static struct NiosStruct *trimClinAddr;
  static struct NiosStruct *trimEncAddr;
  static struct NiosStruct *trimNullAddr;
  static struct NiosStruct *trimMagAddr;
  static struct NiosStruct *trimPss1Addr;
  static struct NiosStruct *trimPss2Addr;
  static struct NiosStruct *dgpsTrimAddr;
  static struct NiosStruct *trimSsAddr;

  static struct NiosStruct *lstSchedAddr;

  /* low level scan mode diagnostics */
  static struct NiosStruct *modeAzMcAddr;
  static struct NiosStruct *modeElMcAddr;
  static struct NiosStruct *dirAzMcAddr;
  static struct NiosStruct *dirElMcAddr;
  static struct NiosStruct *destAzMcAddr;
  static struct NiosStruct *destElMcAddr;
  static struct NiosStruct *dithElAddr;
  static struct NiosStruct *velAzMcAddr;
  static struct NiosStruct *velElMcAddr;

  /* Motor data read out over serial threads in motors.c */
  static struct NiosStruct *velSerRWAddr;
//  static struct NiosStruct *tMCRWAddr; // JAS--afaik, no temp. reading avail. 
                                         // from AMC controller
  static struct NiosStruct *resRWAddr;
  static struct NiosStruct *iSerRWAddr;
//  static struct NiosStruct *stat1RWAddr;// JAS--Old Copley controller data
//  static struct NiosStruct *stat2RWAddr;
//  static struct NiosStruct *faultRWAddr;
  static struct NiosStruct *infoRWAddr;
  static struct NiosStruct *statDrRWAddr;
  static struct NiosStruct *statS1RWAddr;
  static struct NiosStruct *driveErrCtsRWAddr;
  //static struct NiosStruct *elRawEncAddr;
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
  //int i_elev_motors;
  int i_pivot_motors;
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
    azRawMagAddr = GetNiosAddr("az_raw_mag");
    pitchMagAddr = GetNiosAddr("pitch_mag");
    declinationMagAddr = GetNiosAddr("declination_mag");
    sigmaMagAddr = GetNiosAddr("sigma_mag");
    dgpsAzAddr = GetNiosAddr("az_dgps");
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
    azrawPss2Addr = GetNiosAddr("azraw_pss2");
    elrawPss2Addr = GetNiosAddr("elraw_pss2");
    snrPss2Addr = GetNiosAddr("snr_pss2");
    azPss2Addr = GetNiosAddr("az_pss2");  // evolved az
    azIscAddr = GetNiosAddr("az_isc");
    elIscAddr = GetNiosAddr("el_isc");
    sigmaIscAddr = GetNiosAddr("sigma_isc");
    azOscAddr = GetNiosAddr("az_osc");
    elOscAddr = GetNiosAddr("el_osc");
    sigmaOscAddr = GetNiosAddr("sigma_osc");
    elEncAddr = GetNiosAddr("el_enc");
    sigmaEncAddr = GetNiosAddr("sigma_enc");
    elClinAddr = GetNiosAddr("el_clin");
    elLutClinAddr = GetNiosAddr("el_lut_clin");
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
    dithStepPAddr = GetNiosAddr("dith_step_p");

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
    dgpsAzCovAddr = GetNiosAddr("az_cov_dgps");
    dgpsPitchCovAddr = GetNiosAddr("pitch_cov_dgps");
    dgpsRollCovAddr = GetNiosAddr("roll_cov_dgps");
    dgpsAntEAddr = GetNiosAddr("ant_e_dgps");
    dgpsAntNAddr = GetNiosAddr("ant_n_dgps");
    dgpsAntUAddr = GetNiosAddr("ant_u_dgps");

    lstSchedAddr = GetNiosAddr("lst_sched");

    trimClinAddr = GetNiosAddr("trim_clin");
    trimEncAddr = GetNiosAddr("trim_enc");
    trimNullAddr = GetNiosAddr("trim_null");
    trimMagAddr = GetNiosAddr("trim_mag");
    trimPss1Addr = GetNiosAddr("trim_pss1");
    trimPss2Addr = GetNiosAddr("trim_pss2");
    dgpsTrimAddr = GetNiosAddr("trim_dgps");
    dgpsCovLimAddr = GetNiosAddr("cov_lim_dgps");
    dgpsAntsLimAddr = GetNiosAddr("ants_lim_dgps");

    modeAzMcAddr = GetNiosAddr("mode_az_mc");
    modeElMcAddr = GetNiosAddr("mode_el_mc");
    destAzMcAddr = GetNiosAddr("dest_az_mc");
    destElMcAddr = GetNiosAddr("dest_el_mc");
    velAzMcAddr = GetNiosAddr("vel_az_mc");
    velElMcAddr = GetNiosAddr("vel_el_mc");
    dirAzMcAddr = GetNiosAddr("dir_az_mc");
    dirElMcAddr = GetNiosAddr("dir_el_mc");
    dithElAddr = GetNiosAddr("dith_el");
    /* JAS--comment out irrelevant fields from old RW Copley controller*/
    velSerRWAddr = GetNiosAddr("vel_ser_rw");
    //elRawEncAddr = GetNiosAddr("el_raw_enc");
//  tMCRWAddr = GetNiosAddr("t_mc_rw");
    resRWAddr = GetNiosAddr("res_rw");
    iSerRWAddr = GetNiosAddr("i_ser_rw");
//  stat1RWAddr = GetNiosAddr("stat_1_rw");
//  stat2RWAddr = GetNiosAddr("stat_2_rw");
//  faultRWAddr = GetNiosAddr("fault_rw");i
    statDrRWAddr = GetNiosAddr("stat_dr_rw");
    statS1RWAddr = GetNiosAddr("stat_s1_rw");
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
  i_rw_motors = GETREADINDEX(rw_motor_index);
  //i_elev_motors = GETREADINDEX(elev_motor_index);
  i_pivot_motors = GETREADINDEX(pivot_motor_index);

  WriteData(azAddr, (unsigned int)(PointingData[i_point].az * DEG2LI),
      NIOS_QUEUE);
  WriteData(elAddr, (unsigned int)(PointingData[i_point].el * DEG2LI),
      NIOS_QUEUE);

  WriteData(elEncAddr, (unsigned int)((PointingData[i_point].enc_el
                      + CommandData.enc_el_trim)* DEG2I), NIOS_QUEUE);
  WriteData(sigmaEncAddr,
      (unsigned int)(PointingData[i_point].enc_sigma * DEG2I), NIOS_QUEUE);

  //WriteData(velRWAddr,
  //    ((long int)(RWMotorData[i_rw_motors].vel_rw/4.0*DEG2I)), NIOS_QUEUE);
 
  WriteCalData(velSerRWAddr, RWMotorData[i_rw_motors].dps_rw, NIOS_QUEUE);
// WriteData(elRawEncAddr,
   //   ((long int)(ElevMotorData[i_elev_motors].enc_raw_el*DEG2I)), NIOS_QUEUE);
  WriteData(resRWAddr, RWMotorData[i_rw_motors].res_rw*DEG2I, NIOS_QUEUE);

  /*************************************************
   *             Slow Controls                     *
   ************************************************/
  if (index != 0) return;

  /* scan modes */
  WriteData(modeAzMcAddr, axes_mode.az_mode, NIOS_QUEUE);
  WriteData(modeElMcAddr, axes_mode.el_mode, NIOS_QUEUE);
  WriteData(dirAzMcAddr, axes_mode.az_dir, NIOS_QUEUE);
  WriteData(dirElMcAddr, axes_mode.el_dir, NIOS_QUEUE);
  WriteData(dithElAddr, axes_mode.el_dith * 32767.0*2.0, NIOS_QUEUE);
  WriteData(destAzMcAddr, axes_mode.az_dest * DEG2I, NIOS_QUEUE);
  WriteData(destElMcAddr, axes_mode.el_dest * DEG2I, NIOS_QUEUE);
  WriteData(velAzMcAddr, axes_mode.az_vel * 6000., NIOS_QUEUE);
  WriteData(velElMcAddr, axes_mode.el_vel * 6000., NIOS_QUEUE);

  /********** Sun Sensor Data **********/
  WriteData(phaseSsAddr, PointingData[i_point].ss_phase * DEG2I, NIOS_QUEUE);
  WriteData(snrSsAddr, PointingData[i_point].ss_snr * 1000, NIOS_QUEUE);
  WriteData(azRelSunSsAddr, PointingData[i_point].ss_az_rel_sun * DEG2I,
      NIOS_QUEUE);
  /********* PSS data *************/
  WriteData(azrawPss1Addr, PointingData[i_point].pss1_azraw * DEG2I, NIOS_QUEUE);
  WriteData(elrawPss1Addr, PointingData[i_point].pss1_elraw * DEG2I, NIOS_QUEUE);
  WriteData(snrPss1Addr, PointingData[i_point].pss1_snr * 1000., NIOS_QUEUE);
  WriteData(azPss1Addr, (PointingData[i_point].pss1_az +
                      CommandData.pss1_az_trim) * DEG2I, NIOS_QUEUE);
  WriteData(azrawPss2Addr, PointingData[i_point].pss2_azraw * DEG2I, NIOS_QUEUE);
  WriteData(elrawPss2Addr, PointingData[i_point].pss2_elraw * DEG2I, NIOS_QUEUE);
  WriteData(snrPss2Addr, PointingData[i_point].pss2_snr * 1000., NIOS_QUEUE);
  WriteData(azPss2Addr, (PointingData[i_point].pss2_az +
                      CommandData.pss2_az_trim) * DEG2I, NIOS_QUEUE);
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
  WriteData(azRawMagAddr,
      (unsigned int)((PointingData[i_point].mag_az_raw) * DEG2I), NIOS_QUEUE);
  WriteData(pitchMagAddr,
      (unsigned int)(ACSData.mag_pitch * DEG2I), NIOS_QUEUE);
  WriteData(declinationMagAddr,
      (unsigned int)(PointingData[i_point].mag_model * DEG2I), NIOS_QUEUE);
  WriteData(sigmaMagAddr,
      (unsigned int)(PointingData[i_point].mag_sigma * DEG2I), NIOS_QUEUE);
  WriteData(trimMagAddr, CommandData.mag_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(trimPss1Addr, CommandData.pss1_az_trim * DEG2I, NIOS_QUEUE);
  WriteData(trimPss2Addr, CommandData.pss2_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(dgpsAzAddr,
      (unsigned int)((PointingData[i_point].dgps_az  +
                      CommandData.dgps_az_trim) * DEG2I), NIOS_QUEUE);
  WriteData(dgpsSigmaAddr,
  (((unsigned int)(PointingData[i_point].dgps_sigma * DEG2I))>65535)?65535:((unsigned int)(PointingData[i_point].dgps_sigma * DEG2I)), NIOS_QUEUE);
  WriteData(dgpsTrimAddr, CommandData.dgps_az_trim * DEG2I, NIOS_QUEUE);
  WriteData(dgpsCovLimAddr, CommandData.dgps_cov_limit*32768.0/100.0, NIOS_QUEUE);
  WriteData(dgpsAntsLimAddr, CommandData.dgps_ants_limit*32768.0/100.0, NIOS_QUEUE);
  WriteData(azSsAddr, (unsigned int)((PointingData[i_point].ss_az +
          CommandData.ss_az_trim) * DEG2I),
      NIOS_QUEUE);
  WriteData(sigmaSsAddr,
      (unsigned int)(PointingData[i_point].ss_sigma * DEG2I), NIOS_QUEUE);
  WriteData(azSunAddr, (unsigned int)(PointingData[i_point].sun_az*DEG2I),
      NIOS_QUEUE);
  WriteData(elSunAddr, (int)(PointingData[i_point].sun_el*DEG2I), NIOS_QUEUE);
  WriteData(trimSsAddr, CommandData.ss_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(trimEncAddr, CommandData.enc_el_trim * DEG2I, NIOS_QUEUE);

  WriteData(elClinAddr,
      (unsigned int)((PointingData[i_point].clin_el_lut +
                      CommandData.clin_el_trim) * DEG2I), NIOS_QUEUE);
  WriteData(elLutClinAddr,
      (unsigned int)(PointingData[i_point].clin_el * DEG2I), NIOS_QUEUE);
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
  WriteData(dithStepPAddr, (int)(CommandData.pointing_mode.dith*10.0*32768.0), NIOS_QUEUE);
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

  sensor_veto = (!CommandData.use_sun) |
    ((!CommandData.use_elenc) << 2) |
    ((!CommandData.use_mag) << 3) |
    ((!CommandData.use_gps) << 4) |
    ((!CommandData.use_elclin) << 5) |
    ((CommandData.disable_el) << 10) |
    ((CommandData.disable_az) << 11) |
    ((CommandData.force_el) << 12) |
    ((!CommandData.use_pss1) << 13) |
    ((!CommandData.use_pss2) << 14);

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
  WriteData(dgpsAltAddr, DGPSPos[i_dgps].alt, NIOS_QUEUE);
  WriteData(dgpsSpeedAddr, DGPSPos[i_dgps].speed*100, NIOS_QUEUE);
  WriteData(dgpsDirAddr, (unsigned int)DGPSPos[i_dgps].direction*DEG2I, NIOS_QUEUE);
  WriteData(dgpsClimbAddr, DGPSPos[i_dgps].climb*100, NIOS_QUEUE);
  WriteData(dgpsNSatAddr, DGPSPos[i_dgps].n_sat, NIOS_QUEUE);

  WriteData(lstSchedAddr, sched_lst, NIOS_QUEUE);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);
  WriteData(dgpsAzRawAddr, DGPSAtt[i_dgps].az * DEG2I, NIOS_QUEUE);
  WriteData(dgpsPitchRawAddr, DGPSAtt[i_dgps].pitch * DEG2I, NIOS_QUEUE);
  WriteData(dgpsRollRawAddr, DGPSAtt[i_dgps].roll * DEG2I, NIOS_QUEUE);
  WriteData(dgpsAzCovAddr, DGPSAtt[i_dgps].az_cov * DEG2I, NIOS_QUEUE);
  WriteData(dgpsPitchCovAddr, DGPSAtt[i_dgps].pitch_cov * DEG2I, NIOS_QUEUE);
  WriteData(dgpsRollCovAddr, DGPSAtt[i_dgps].roll_cov * DEG2I, NIOS_QUEUE);
  WriteData(dgpsAntEAddr,(int)(DGPSAtt[i_dgps].ant_E*100), NIOS_QUEUE);
  WriteData(dgpsAntNAddr,(int)(DGPSAtt[i_dgps].ant_N*100), NIOS_QUEUE);
  WriteData(dgpsAntUAddr,(int)(DGPSAtt[i_dgps].ant_U*100), NIOS_QUEUE);
  WriteData(dgpsAttOkAddr, DGPSAtt[i_dgps].att_ok, NIOS_QUEUE);
//WriteData(tMCRWAddr,RWMotorData[i_rw_motors].temp,NIOS_QUEUE);
  WriteData(iSerRWAddr,((int)(RWMotorData[i_rw_motors].current/60.0*32768.0))
            ,NIOS_QUEUE); // need to check scaling
//WriteData(stat1RWAddr,(RWMotorData[i_rw_motors].status & 0xffff),
//          NIOS_QUEUE);
//WriteData(stat2RWAddr,((RWMotorData[i_rw_motors].status & 0xffff0000)>> 16),
//          NIOS_QUEUE);
//WriteData(faultRWAddr,RWMotorData[i_rw_motors].fault_reg,NIOS_QUEUE);
  WriteData(statDrRWAddr,(RWMotorData[i_rw_motors].db_stat & 0xff)
            +((RWMotorData[i_rw_motors].dp_stat & 0xff)<< 8),NIOS_QUEUE);
  WriteData(statS1RWAddr,RWMotorData[i_rw_motors].ds1_stat,NIOS_QUEUE);
  WriteData(infoRWAddr,RWMotorData[i_rw_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsRWAddr,RWMotorData[i_rw_motors].err_count,NIOS_QUEUE);
  //WriteData(tMCElAddr,ElevMotorData[i_elev_motors].temp,NIOS_QUEUE);
//  WriteData(iSerElAddr,((int)(ElevMotorData[i_elev_motors].current/30.0*32768.0)),NIOS_QUEUE);
  //WriteData(stat1ElAddr,(ElevMotorData[i_elev_motors].status & 0xffff),
//            NIOS_QUEUE);
  //WriteData(stat2ElAddr,((ElevMotorData[i_elev_motors].status & 0xffff0000)>> 16),NIOS_QUEUE);
  //WriteData(faultElAddr,ElevMotorData[i_elev_motors].fault_reg,NIOS_QUEUE);
  WriteData(iSerPivAddr,PivotMotorData[i_pivot_motors].current*32768.0/20.0,NIOS_QUEUE);
  WriteData(statDrPivAddr,(PivotMotorData[i_pivot_motors].db_stat & 0xff)
                 +((PivotMotorData[i_pivot_motors].dp_stat & 0xff)<< 8),NIOS_QUEUE);
  WriteData(statS1PivAddr,PivotMotorData[i_pivot_motors].ds1_stat,NIOS_QUEUE);
  WriteData(velSerPivAddr,PivotMotorData[i_pivot_motors].dps_piv,NIOS_QUEUE);
  WriteData(resPivAddr,
      PivotMotorData[i_pivot_motors].res_piv*DEG2I, NIOS_QUEUE);
//  WriteData(infoElAddr,ElevMotorData[i_elev_motors].drive_info,NIOS_QUEUE);
//  WriteData(driveErrCtsElAddr,ElevMotorData[i_elev_motors].err_count,NIOS_QUEUE);
  WriteData(infoPivAddr,PivotMotorData[i_pivot_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsPivAddr,PivotMotorData[i_pivot_motors].err_count,NIOS_QUEUE);
  WriteData(verboseRWAddr,CommandData.verbose_rw,NIOS_QUEUE);
  WriteData(verboseElAddr,CommandData.verbose_el,NIOS_QUEUE);
  WriteData(verbosePivAddr,CommandData.verbose_piv,NIOS_QUEUE);

}
#endif

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
void WriteCalData(struct NiosStruct* addr, double data, int flush_flag)
{
  long long cal = (data - addr->b)/addr->m;

  //check that rounding doesn't lead to over/underflow
  if (addr->wide) {   //wide
    if (addr->sign) {   //signed
      if (cal > INT_MAX) cal = INT_MAX;
      else if (cal < INT_MIN) cal = INT_MIN;
    } else {	      //unsigned
      if (cal > UINT_MAX) cal = UINT_MAX;
      else if (cal < 0) cal = 0;
    }
  } else {	      //narrow
    if (addr->sign) {   //signed
      if (cal > SHRT_MAX) cal = SHRT_MAX;
      else if (cal < SHRT_MIN) cal = SHRT_MIN;
    } else {	      //unsigned
      if (cal > USHRT_MAX) cal = USHRT_MAX;
      else if (cal < 0) cal = 0;
    }
  }
  WriteData(addr, cal, flush_flag);
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
    if (addr->nios->wide)
      return ((double)(int)ReadData(addr) * addr->nios->m + addr->nios->b);
    else return ((double)(short)ReadData(addr) * addr->nios->m + addr->nios->b);
  } else {		    //unsigned
    return ((double)ReadData(addr) * addr->nios->m + addr->nios->b);
  }
}

/* called from mcp, should call all nios writing functions */
void UpdateBBCFrame()
{
  static struct BiPhaseStruct* frameNumAddr;
  static int firsttime = 1;
  static int index = 0;

  /*** do fast Controls ***/
  if (firsttime) {
    firsttime = 0;
    frameNumAddr = GetBiPhaseAddr("framenum");
  }

#ifndef BOLOTEST
  if (!mcp_initial_controls)
    DoSched();
  UpdateAxesMode();
  StoreData(index);
  ControlGyroHeat();
  WriteMot(index);
  updateTableSpeed();
#endif
#ifdef USE_XY_THREAD
  StoreStageBus(index);
#endif
  WriteChatter(index);
  HouseKeeping(index);

  /*** do slow Controls ***/
  if (index == 0) {
    if (!mcp_initial_controls)
      SyncADC();
    WriteAux();
    //StoreActBus();
    //SecondaryMirror();
    //StoreHWPRBus();
#ifndef BOLOTEST
    SetGyroMask();
    ChargeController();
    ControlPower();
    VideoTx();
    cameraFields();
#endif
  }

  if (!mcp_initial_controls)
    index = (index + 1) % FAST_PER_SLOW;

  //make sure frame is flushed
  RawNiosWrite(-1,-1,NIOS_FLUSH);
}
