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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <limits.h>

#include "bbc_pci.h"

#include "channels.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"
#include "chrgctrl.h"
#include "sip.h"
#include "mceserv.h"
#include "hwpr.h"
#include "mpc_proto.h"

#include "flcdataswap.h"

#define NIOS_BUFFER_SIZE 100

extern short int BitsyIAm;

extern int StartupVeto;

short int InCharge = 0;

int EthernetSC[3] = {3,3,3};

extern struct AxesModeStruct axes_mode; /* motors.c */

extern unsigned int sched_lst; /* sched_lst */

extern int bbc_fp;

extern short int bsc_trigger; /* sc.cpp */

extern struct chat_buf chatter_buffer;  /* mcp.c */

double round(double x);

/* in hk.c */
void HouseKeeping();
void SFTValveMotors();
void VetoMCE();

/* in auxiliary.c */
void ChargeController(void);
void WriteSyncBox(void);
void ControlAuxMotors();
void ControlHeaters();
void CameraTrigger(int which);
void ControlPower(void);
void LockMotor();

/* in motors.c */
void UpdateAxesMode(void);
void WriteMot(int TxIndex);
double calcVSerRW(void);

/* in sc.cpp */
void cameraTriggers();
void cameraFields(int which);

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
  static struct NiosStruct* rateTdrssAddr;
  static struct NiosStruct* rateIridiumAddr;
  static struct NiosStruct* ratePilotAddr;
  static struct NiosStruct* channelsetOthAddr;
  static struct NiosStruct* tChipFlcAddr;
  static struct NiosStruct* tMbFlcAddr;
  static struct NiosStruct* statusMCCAddr;
  static struct BiPhaseStruct* statusMCCReadAddr;
  static struct NiosStruct* bi0FifoSizeAddr;
  static struct NiosStruct* bbcFifoSizeAddr;
  static struct NiosStruct* ploverAddr;
  
  static struct NiosStruct* statusEthAddr;
  
  static struct NiosStruct* partsSchedAddr;
  static struct NiosStruct* upslotSchedAddr;
  
  static struct NiosStruct* frameIntBbcAddr;
  static struct NiosStruct* rateExtBbcAddr;
  static struct NiosStruct* frameExtBbcAddr;
  static struct NiosStruct* rateFrameBbcAddr;
  static struct NiosStruct* rateSampAdcAddr;
  static struct NiosStruct* bbcSyncAutoAddr;
  static struct NiosStruct* bandAzAddr;
  static struct NiosStruct *bsetAddr;

  static struct NiosStruct* timeMeFlcAddr;
  static struct NiosStruct* dfMeFlcAddr;
  static struct NiosStruct* timeoutMeAddr;
  static struct NiosStruct* tCpuMeFlcAddr;
  static struct NiosStruct* lastMeCmdAddr;
  static struct NiosStruct* countMeCmdAddr;
  
  static struct NiosStruct* timeOtherFlcAddr;
  static struct NiosStruct* dfOtherFlcAddr;
  static struct NiosStruct* timeoutOtherAddr;
  static struct NiosStruct* tCpuOtherFlcAddr;
  static struct NiosStruct* lastOtherCmdAddr;
  static struct NiosStruct* countOtherCmdAddr;
  static struct NiosStruct* mceCmplexAddr;
  static struct NiosStruct* mceCindexAddr;
  
  
  struct flc_data otherData, *myData;
  
  static int incharge = -1;
  time_t t;
  int i_point;
  struct timeval tv;
  struct timezone tz;

  unsigned short mccstatus;
  unsigned short stat_read;

  static int firsttime = 1;
  if (firsttime) {
    char buf[128];
    firsttime = 0;
    statusMCCAddr = GetNiosAddr("status_flc");
    statusMCCReadAddr = ExtractBiPhaseAddr(statusMCCAddr);

    tChipFlcAddr = GetNiosAddr("t_chip_flc");
    tMbFlcAddr = GetNiosAddr("t_mb_flc");
    timeAddr = GetNiosAddr("time");
    timeUSecAddr = GetNiosAddr("time_usec");
    rateTdrssAddr = GetNiosAddr("rate_tdrss");
    rateIridiumAddr= GetNiosAddr("rate_iridium");
    ratePilotAddr= GetNiosAddr("rate_pilot");
    channelsetOthAddr = GetNiosAddr("channelset_oth");
    bi0FifoSizeAddr = GetNiosAddr("bi0_fifo_size");
    bbcFifoSizeAddr = GetNiosAddr("bbc_fifo_size");
    ploverAddr = GetNiosAddr("plover");
    statusEthAddr = GetNiosAddr("status_eth");
    partsSchedAddr = GetNiosAddr("parts_sched");
    upslotSchedAddr = GetNiosAddr("upslot_sched");
    
    frameIntBbcAddr = GetNiosAddr("frame_int_bbc");
    rateExtBbcAddr = GetNiosAddr("rate_ext_bbc");
    frameExtBbcAddr = GetNiosAddr("frame_ext_bbc");
    rateFrameBbcAddr = GetNiosAddr("rate_frame_bbc");
    rateSampAdcAddr = GetNiosAddr("rate_samp_adc");
    bbcSyncAutoAddr = GetNiosAddr("bbc_sync_auto");
    bandAzAddr = GetNiosAddr("band_az");
    bsetAddr = GetNiosAddr("bset");

    sprintf(buf, "time_%c_flc", (BitsyIAm) ? 'b' : 'i');
    timeMeFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "df_%c_flc", (BitsyIAm) ? 'b' : 'i');
    dfMeFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "timeout_%c", (BitsyIAm) ? 'b' : 'i');
    timeoutMeAddr = GetNiosAddr(buf);
    sprintf(buf, "t_cpu_%c_flc", (BitsyIAm) ? 'b' : 'i');
    tCpuMeFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "last_%c_cmd", (BitsyIAm) ? 'b' : 'i');
    lastMeCmdAddr = GetNiosAddr(buf);
    sprintf(buf, "count_%c_cmd", (BitsyIAm) ? 'b' : 'i');
    countMeCmdAddr = GetNiosAddr(buf);
    
    sprintf(buf, "time_%c_flc", (!BitsyIAm) ? 'b' : 'i');
    timeOtherFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "df_%c_flc", (!BitsyIAm) ? 'b' : 'i');
    dfOtherFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "timeout_%c", (!BitsyIAm) ? 'b' : 'i');
    timeoutOtherAddr = GetNiosAddr(buf);
    sprintf(buf, "t_cpu_%c_flc", (!BitsyIAm) ? 'b' : 'i');
    tCpuOtherFlcAddr = GetNiosAddr(buf);
    sprintf(buf, "last_%c_cmd", (!BitsyIAm) ? 'b' : 'i');
    lastOtherCmdAddr = GetNiosAddr(buf);
    sprintf(buf, "count_%c_cmd", (!BitsyIAm) ? 'b' : 'i');
    countOtherCmdAddr = GetNiosAddr(buf);
    
    mceCindexAddr = GetNiosAddr("mce_cindex");
    mceCmplexAddr = GetNiosAddr("mce_cmplex");
  }

  myData = get_flc_out_data();
  
  if (StartupVeto>0) {
    InCharge = 0;
  } else {
    stat_read = slow_data[statusMCCReadAddr->index][statusMCCReadAddr->channel];
    if (BitsyIAm) {
      if ( (stat_read & 0x3) == 0x1 ) InCharge = 1;
      else InCharge = 0;
    } else {
      if ( (stat_read & 0x3) == 0x2 ) InCharge = 1;
      else InCharge = 0;
    }
  }
  if (InCharge != incharge && InCharge) {
    bprintf(info, "I, %s, have gained control.\n", BitsyIAm ? "Bitsy"
        : "Itsy");
  } else if (InCharge != incharge) {
    bprintf(info, "I, %s, have lost control.\n", BitsyIAm ? "Bitsy"
        : "Itsy");
  }

  incharge = InCharge;

  gettimeofday(&tv, &tz);

  WriteData(timeAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  WriteData(timeUSecAddr, tv.tv_usec, NIOS_QUEUE);
  WriteData(timeMeFlcAddr, tv.tv_sec + TEMPORAL_OFFSET, NIOS_QUEUE);
  myData->time = tv.tv_sec + TEMPORAL_OFFSET;
  
  WriteData(tChipFlcAddr, CommandData.temp1, NIOS_QUEUE);
  WriteData(tMbFlcAddr, CommandData.temp3, NIOS_QUEUE);
  WriteData(tCpuMeFlcAddr, CommandData.temp2, NIOS_QUEUE);
  myData->t_cpu = CommandData.temp2;

  WriteData(dfMeFlcAddr, (CommandData.df > 65535) ? 65535 : CommandData.df, NIOS_QUEUE);
  myData->df = (CommandData.df > 65535) ? 65535 : CommandData.df;
  
  WriteData(partsSchedAddr, CommandData.parts_sched&0xffffff, NIOS_QUEUE);
  WriteData(upslotSchedAddr, CommandData.upslot_sched, NIOS_QUEUE);

  i_point = GETREADINDEX(point_index);

  t = PointingData[i_point].t;

  if (CommandData.pointing_mode.t > t) {
    WriteData(timeoutMeAddr, CommandData.pointing_mode.t - t, NIOS_QUEUE);
    myData->timeout = CommandData.pointing_mode.t - t;
  } else {
    WriteData(timeoutMeAddr, 0, NIOS_QUEUE);
    myData->timeout = 0;
  }

  WriteData(bi0FifoSizeAddr, CommandData.bi0FifoSize, NIOS_QUEUE);
  WriteData(bbcFifoSizeAddr, CommandData.bbcFifoSize, NIOS_QUEUE);
  WriteData(ploverAddr, CommandData.plover, NIOS_QUEUE);
  WriteData(rateTdrssAddr, CommandData.tdrss_bw, NIOS_QUEUE);
  WriteData(rateIridiumAddr, CommandData.iridium_bw, NIOS_QUEUE);
  WriteData(ratePilotAddr, CommandData.pilot_bw, NIOS_QUEUE);
  WriteData(channelsetOthAddr, CommandData.channelset_oth, NIOS_QUEUE);

  WriteData(statusEthAddr,
       (EthernetSC[0] & 0x3) +
       ((EthernetSC[1] & 0x3) << 2) +
       ((EthernetSC[2] & 0x3) << 4),
       NIOS_QUEUE);

  WriteCalData(frameIntBbcAddr, CommandData.bbcIntFrameRate, NIOS_QUEUE);
  if (CommandData.bbcExtFrameMeas > BBC_SYNC_TIMEOUT) {
    WriteCalData(rateExtBbcAddr, 0.0, NIOS_QUEUE);
  } else {
    WriteCalData(rateExtBbcAddr,32.e6/CommandData.bbcExtFrameMeas,NIOS_QUEUE);
  }
  WriteData(frameExtBbcAddr, CommandData.bbcExtFrameRate, NIOS_QUEUE);

  WriteCalData(rateFrameBbcAddr, ACSData.bbc_rate, NIOS_QUEUE);
  WriteCalData(rateSampAdcAddr, ACSData.adc_rate, NIOS_QUEUE);
  WriteCalData(bbcSyncAutoAddr, CommandData.bbcAutoExt, NIOS_QUEUE);
  WriteCalData(bandAzAddr, CommandData.pointing_mode.overshoot_band,
      NIOS_QUEUE);
  WriteData(bsetAddr, CommandData.bset_num, NIOS_QUEUE);

  mccstatus =
    (BitsyIAm ? 0x1 : 0x0) +                 //0x01
    (BitsyIAm ? 0x0 : 0x2) +                 //0x02
    (CommandData.at_float ? 0x4 : 0x0) +     //0x04
    (CommandData.bbcIsExt ? 0x8 : 0x0) +     //0x08
    (CommandData.uplink_sched ? 0x10 : 0x00) + //0x10
    (CommandData.sucks ? 0x20 : 0x00) +      //0x20
       //((CommandData.lat_range & 0x3) << 6) +   //0xc0
    ((CommandData.slot_sched & 0xFF) << 8);  //0xFF00

  if (CommandData.uplink_sched) {
    mccstatus |= 0xc0;
  } else {
    mccstatus |= ((CommandData.lat_range & 0x3) << 6);
  }

  WriteData(statusMCCAddr, mccstatus, NIOS_FLUSH);
  
  WriteData(lastMeCmdAddr, CommandData.last_command, NIOS_QUEUE);
  WriteData(countMeCmdAddr, CommandData.command_count, NIOS_QUEUE);
  myData->last_command = CommandData.last_command;
  myData->command_count = CommandData.command_count;
  
  swap_flc_data(&otherData);
  WriteData(timeOtherFlcAddr, otherData.time, NIOS_QUEUE);
  WriteData(dfOtherFlcAddr, otherData.df, NIOS_QUEUE);
  WriteData(timeoutOtherAddr, otherData.timeout, NIOS_QUEUE);
  WriteData(tCpuOtherFlcAddr, otherData.t_cpu, NIOS_QUEUE);
  WriteData(lastOtherCmdAddr, otherData.last_command, NIOS_QUEUE);
  WriteData(countOtherCmdAddr, otherData.command_count, NIOS_QUEUE);
  
  WriteData(mceCindexAddr, CommandData.mce_param_index, NIOS_QUEUE);
  WriteData(mceCmplexAddr, mce_param[CommandData.mce_param_index], NIOS_QUEUE);
}

static struct NiosStruct *GetMCCNiosAddr(char *field, int i_mce)
{
  char field_name[FIELD_LEN];
  sprintf(field_name, "%s%d", field, i_mce+1);
  return (GetNiosAddr(field_name));
}

/* TES fifo debugging */
extern int nrx_c[NUM_MCE];
extern int last_mce_no[NUM_MCE];

/* write slow MCE data */
static void WriteMCESlow(void)
{
  static int firsttime = 1;
  static struct NiosStruct *driveMapAddr[NUM_MCE];
  static struct NiosStruct *df0MccAddr[NUM_MCE];
  static struct NiosStruct *df1MccAddr[NUM_MCE];
  static struct NiosStruct *df2MccAddr[NUM_MCE];
  static struct NiosStruct *df3MccAddr[NUM_MCE];
  static struct NiosStruct *stateMpcAddr[NUM_MCE];
  static struct NiosStruct *dtgMpcAddr[NUM_MCE];
  static struct NiosStruct *taskMpcAddr[NUM_MCE];
  static struct NiosStruct *tMccAddr[NUM_MCE];
  static struct NiosStruct *tMceAddr[NUM_MCE];
  static struct NiosStruct *timeMccAddr[NUM_MCE];
  static struct NiosStruct *uptimeMpcAddr[NUM_MCE];
  static struct NiosStruct *rampCountAddr[NUM_MCE];
  static struct NiosStruct *clampCountAddr[NUM_MCE];
  static struct NiosStruct *lastTuneAddr[NUM_MCE];
  static struct NiosStruct *refTuneAddr[NUM_MCE];
  static struct NiosStruct *tuneStatAddr[NUM_MCE];
  static struct NiosStruct *usedTuneAddr[NUM_MCE];
  static struct NiosStruct *lastIVAddr[NUM_MCE];
  static struct NiosStruct *tileHeaterAddr[NUM_MCE];

  static struct NiosStruct *blobNumAddr;
  static struct NiosStruct *reportingMPCsAddr;
  static struct NiosStruct *aliveMPCsAddr;
  static struct NiosStruct *squidVetoAddr;
  static struct NiosStruct *thermVetoAddr;
  static struct NiosStruct *syncVetoAddr;
  static struct NiosStruct *dataModeAddr;
  static struct NiosStruct *dataModeBitsAddr;
  
  static struct NiosStruct *boloFiltFreqAddr;
  static struct NiosStruct *boloFiltBWAddr;
  static struct NiosStruct *boloFiltLenAddr;

  uint16_t sync_veto = 0;
  unsigned int i;
  int ind;

  if (firsttime) {
    int i;
    firsttime = 0;
    for (i = 0; i < NUM_MCE; i++) {
      driveMapAddr[i] = GetMCCNiosAddr("drive_map_mpc", i);
      df0MccAddr[i] = GetMCCNiosAddr("df_0_mcc", i);
      df1MccAddr[i] = GetMCCNiosAddr("df_1_mcc", i);
      df2MccAddr[i] = GetMCCNiosAddr("df_2_mcc", i);
      df3MccAddr[i] = GetMCCNiosAddr("df_3_mcc", i);
      stateMpcAddr[i] = GetMCCNiosAddr("state_mpc", i);
      dtgMpcAddr[i] = GetMCCNiosAddr("dtg_mpc", i);
      taskMpcAddr[i] = GetMCCNiosAddr("task_mpc", i);
      tMccAddr[i] = GetMCCNiosAddr("t_mcc", i);
      tMceAddr[i] = GetMCCNiosAddr("t_mce", i);
      timeMccAddr[i] = GetMCCNiosAddr("time_mcc", i);
      uptimeMpcAddr[i] = GetMCCNiosAddr("uptime_mpc", i);
      rampCountAddr[i] = GetMCCNiosAddr("ramp_count_mce", i);
      clampCountAddr[i] = GetMCCNiosAddr("clamp_count_mce", i);
      lastTuneAddr[i] = GetMCCNiosAddr("last_tune_mpc", i);
      tuneStatAddr[i] = GetMCCNiosAddr("tune_stat_mpc", i);
      refTuneAddr[i] = GetMCCNiosAddr("ref_tune_mpc", i);
      usedTuneAddr[i] = GetMCCNiosAddr("used_tune_mpc", i);
      lastIVAddr[i] = GetMCCNiosAddr("last_iv_mpc", i);
      tileHeaterAddr[i] = GetMCCNiosAddr("tile_heater_mce", i);
    }
    blobNumAddr = GetNiosAddr("blob_num_mpc");
    reportingMPCsAddr = GetNiosAddr("reporting_mpcs");
    aliveMPCsAddr = GetNiosAddr("alive_mpcs");
    squidVetoAddr = GetNiosAddr("squid_veto_mpc");
    thermVetoAddr = GetNiosAddr("therm_veto_mpc");
    syncVetoAddr = GetNiosAddr("sync_veto_mpc");
    dataModeAddr = GetNiosAddr("data_mode_mce");
    dataModeBitsAddr = GetNiosAddr("data_mode_bits");
    boloFiltFreqAddr = GetNiosAddr("bolo_filt_freq");
    boloFiltBWAddr = GetNiosAddr("bolo_filt_bw");
    boloFiltLenAddr = GetNiosAddr("bolo_filt_len");
  }

  for (i = 0; i < NUM_MCE; i++) {
    ind = GETREADINDEX(mce_slow_index[i]);

    WriteData(driveMapAddr[i], mce_slow_dat[i][ind].drive_map, NIOS_QUEUE);
    WriteData(timeMccAddr[i], mce_slow_dat[i][ind].time, NIOS_QUEUE);
    WriteData(uptimeMpcAddr[i], mce_slow_dat[i][ind].uptime, NIOS_QUEUE);
    WriteData(df0MccAddr[i], mce_slow_dat[i][ind].df[0], NIOS_QUEUE);
    WriteData(df1MccAddr[i], mce_slow_dat[i][ind].df[1], NIOS_QUEUE);
    WriteData(df2MccAddr[i], mce_slow_dat[i][ind].df[2], NIOS_QUEUE);
    WriteData(df3MccAddr[i], mce_slow_dat[i][ind].df[3], NIOS_QUEUE);
    WriteData(stateMpcAddr[i], mce_slow_dat[i][ind].state, NIOS_QUEUE);
    WriteData(dtgMpcAddr[i], ((mce_slow_dat[i][ind].goal & 0xFF) << 8)
        | (mce_slow_dat[i][ind].dtask & 0xFF), NIOS_QUEUE);
    WriteData(taskMpcAddr[i], mce_slow_dat[i][ind].task, NIOS_QUEUE);
    WriteData(tMccAddr[i], mce_slow_dat[i][ind].t_mcc, NIOS_QUEUE);
    WriteData(tMceAddr[i], mce_slow_dat[i][ind].t_mce, NIOS_QUEUE);
    WriteData(rampCountAddr[i], mce_slow_dat[i][ind].ramp_count,
        NIOS_QUEUE);
    WriteData(clampCountAddr[i], mce_slow_dat[i][ind].clamp_count,
        NIOS_QUEUE);
    WriteData(refTuneAddr[i], mce_slow_dat[i][ind].ref_tune, NIOS_QUEUE);
    WriteData(tuneStatAddr[i], mce_slow_dat[i][ind].tune_stat, NIOS_QUEUE);
    WriteData(lastTuneAddr[i], mce_slow_dat[i][ind].last_tune, NIOS_QUEUE);
    WriteData(usedTuneAddr[i], mce_slow_dat[i][ind].used_tune, NIOS_QUEUE);
    WriteData(lastIVAddr[i], mce_slow_dat[i][ind].last_iv, NIOS_QUEUE);
    WriteData(tileHeaterAddr[i], mce_slow_dat[i][ind].tile_heater,
        NIOS_QUEUE);
  }

  for (i = 0; i < NUM_MCE; ++i) {
    ind = GETREADINDEX(mce_slow_index[i]);
    if (mce_slow_dat[i][ind].sync_veto)
      sync_veto |= (1U << i);
  }

  WriteData(blobNumAddr, CommandData.mce_blob_num, NIOS_QUEUE);
  WriteData(aliveMPCsAddr, mccs_alive, NIOS_QUEUE);
  WriteData(squidVetoAddr, CommandData.squidveto, NIOS_QUEUE);
  WriteData(thermVetoAddr, CommandData.thermveto, NIOS_QUEUE);
  WriteData(syncVetoAddr, sync_veto, NIOS_QUEUE);
  /* Don't crash */
  if (CommandData.data_mode >= 0 && CommandData.data_mode <= 12)
    WriteData(dataModeBitsAddr,
        ((CommandData.data_mode_bits[CommandData.data_mode][0][0] & 0x1F) << 10)
        | ((CommandData.data_mode_bits[CommandData.data_mode][0][1] & 0x1F) <<5)
        | (CommandData.data_mode_bits[CommandData.data_mode][1][0] & 0x1F),
        NIOS_QUEUE);
  WriteData(dataModeAddr, CommandData.data_mode, NIOS_QUEUE);
  
  WriteCalData(boloFiltFreqAddr, CommandData.bolo_filt_freq, NIOS_QUEUE);
  WriteCalData(boloFiltBWAddr, CommandData.bolo_filt_bw, NIOS_QUEUE);
  WriteData(boloFiltLenAddr, CommandData.bolo_filt_len, NIOS_QUEUE);
  
  /* this field is active low */
  WriteData(reportingMPCsAddr, (~mccs_reporting) & 0x3F, NIOS_QUEUE);
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

  chat += (unsigned int)(chatter_buffer.msg[chatter_buffer.reading][index * 2] &
      0x7F);
  chat += (unsigned int)((chatter_buffer.msg[chatter_buffer.reading][(index * 2)
        + 1]) & 0x7F) << 8;

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
#define NUM_SYNC 20
const unsigned short sync_nums[NUM_SYNC] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,\
  14,15,16,17,18,19};
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
    l = GET_NODE(syncAddr[m]->bbcAddr);
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

/************************************************************************/
/*                                                                      */
/*    Store derived acs and pointing data in frame                      */
/*                                                                      */
/************************************************************************/
static void StoreData(int write_slow)
{
  static int firsttime = 1;

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
  static struct NiosStruct* azDelayAddr;
  static struct NiosStruct* nScanPerStepAddr;
  static struct NiosStruct* sizeElStepAddr;
  static struct NiosStruct* nElStepsAddr;

  static struct NiosStruct* vetoSensorAddr;

  /** derived pointing data */
  static struct NiosStruct* OffsetIFelGYAddr;
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
  static struct NiosStruct* xMagAddr;
  static struct NiosStruct* yMagAddr;
  static struct NiosStruct* zMagAddr;
  static struct NiosStruct* azMagAddr;
  static struct NiosStruct* azRawMagAddr;
  static struct NiosStruct* pitchMagAddr;
  static struct NiosStruct* declinationMagAddr;
  static struct NiosStruct* sigmaMagAddr;
  
  static struct NiosStruct* calXMaxMagAddr;
  static struct NiosStruct* calXMinMagAddr;
  static struct NiosStruct* calYMaxMagAddr;
  static struct NiosStruct* calYMinMagAddr;
  static struct NiosStruct* calOffPss1Addr;
  static struct NiosStruct* calOffPss2Addr;
  static struct NiosStruct* calOffPss3Addr;
  static struct NiosStruct* calOffPss4Addr;
  static struct NiosStruct* calOffPss5Addr;
  static struct NiosStruct* calOffPss6Addr;
  static struct NiosStruct* calDPss1Addr;
  static struct NiosStruct* calDPss2Addr;
  static struct NiosStruct* calDPss3Addr;
  static struct NiosStruct* calDPss4Addr;
  static struct NiosStruct* calDPss5Addr;
  static struct NiosStruct* calDPss6Addr;
  static struct NiosStruct* calIMinPssAddr;

  static struct NiosStruct* dgpsAzAddr;
  static struct NiosStruct* dgpsSigmaAddr;
  static struct NiosStruct* azrawPssAddr;
  static struct NiosStruct* azrawPss1Addr;
  static struct NiosStruct* azrawPss2Addr;
  static struct NiosStruct* azrawPss3Addr;
  static struct NiosStruct* azrawPss4Addr;
  static struct NiosStruct* azrawPss5Addr;
  static struct NiosStruct* azrawPss6Addr;
  static struct NiosStruct* elrawPssAddr;
  static struct NiosStruct* elrawPss1Addr;
  static struct NiosStruct* elrawPss2Addr;
  static struct NiosStruct* elrawPss3Addr;
  static struct NiosStruct* elrawPss4Addr;
  static struct NiosStruct* elrawPss5Addr;
  static struct NiosStruct* elrawPss6Addr;
  static struct NiosStruct* snrPss1Addr;
  static struct NiosStruct* snrPss2Addr;
  static struct NiosStruct* snrPss3Addr;
  static struct NiosStruct* snrPss4Addr;
  static struct NiosStruct* snrPss5Addr;
  static struct NiosStruct* snrPss6Addr;
  static struct NiosStruct* azPssAddr;
  static struct NiosStruct* sigmaPssAddr;
  static struct NiosStruct* azSunAddr;
  static struct NiosStruct* elSunAddr;

  /** dgps fields **/
  static struct NiosStruct* dgpsTimeAddr;
  static struct NiosStruct* dgpsLatAddr;
  static struct NiosStruct* dgpsLonAddr;
  static struct NiosStruct* dgpsAltAddr;
  static struct NiosStruct* dgpsSpeedAddr;
  static struct NiosStruct* dgpsDirAddr;
  static struct NiosStruct* dgpsClimbAddr;
  static struct NiosStruct* attOkAddr;
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
  static struct NiosStruct *scPyramidAddr;
  static struct NiosStruct* tableVelAddr;
  static struct NiosStruct* tablePosAddr;
  static struct NiosStruct* tableModeAddr;
  static struct NiosStruct* bscTrigAddr;

  /* trim fields */
  static struct NiosStruct *trimNullAddr;
  static struct NiosStruct *trimMagAddr;
  static struct NiosStruct *trimPssAddr;
  static struct NiosStruct *dgpsTrimAddr;

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
  static struct NiosStruct *resRWAddr;
  static struct NiosStruct *iSerRWAddr;
  static struct NiosStruct *infoRWAddr;
  static struct NiosStruct *statDrRWAddr;
  static struct NiosStruct *statS1RWAddr;
  static struct NiosStruct *driveErrCtsRWAddr;
  static struct NiosStruct *resPivAddr;
  static struct NiosStruct *iSerPivAddr;
  static struct NiosStruct *statDrPivAddr;
  static struct NiosStruct *statS1PivAddr;
  static struct NiosStruct *velSerPivAddr;
  static struct NiosStruct *infoPivAddr;
  static struct NiosStruct *driveErrCtsPivAddr;
  static struct NiosStruct *verboseRWAddr;
  static struct NiosStruct *verbosePivAddr;

  int i_rw_motors;
  //int i_elev_motors;
  int i_pivot_motors;
  int i_point;
  int i_dgps;
  int sensor_veto;
  int att_ok;

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    azAddr = GetNiosAddr("az");
    elAddr = GetNiosAddr("el");

    latSipAddr = GetNiosAddr("lat_sip");
    lonSipAddr = GetNiosAddr("lon_sip");
    altSipAddr = GetNiosAddr("alt_sip");
    timeSipAddr = GetNiosAddr("time_sip");

    mksLoSipAddr = GetNiosAddr("mks_lo_sip");
    mksMedSipAddr = GetNiosAddr("mks_med_sip");
    mksHiSipAddr = GetNiosAddr("mks_hi_sip");

    OffsetIFelGYAddr = GetNiosAddr("offset_ofpch_gy");
    OffsetIFrollGYAddr = GetNiosAddr("offset_ofroll_gy");
    OffsetIFyawGYAddr = GetNiosAddr("offset_ofyaw_gy");
    raAddr = GetNiosAddr("ra");
    decAddr = GetNiosAddr("dec");
    latAddr = GetNiosAddr("lat");
    altAddr = GetNiosAddr("alt");
    lonAddr = GetNiosAddr("lon");
    lstAddr = GetNiosAddr("lst");
    xMagAddr = GetNiosAddr("x_mag");
    yMagAddr = GetNiosAddr("y_mag");
    zMagAddr = GetNiosAddr("z_mag");
    azMagAddr = GetNiosAddr("az_mag");
    azRawMagAddr = GetNiosAddr("az_raw_mag");
    pitchMagAddr = GetNiosAddr("pitch_mag");
    declinationMagAddr = GetNiosAddr("declination_mag");
    sigmaMagAddr = GetNiosAddr("sigma_mag");
    
    calXMaxMagAddr = GetNiosAddr("cal_xmax_mag");
    calXMinMagAddr = GetNiosAddr("cal_xmin_mag");
    calYMaxMagAddr = GetNiosAddr("cal_ymax_mag");
    calYMinMagAddr = GetNiosAddr("cal_ymin_mag");
    calOffPss1Addr = GetNiosAddr("cal_off_pss1");
    calOffPss2Addr = GetNiosAddr("cal_off_pss2");
    calOffPss3Addr = GetNiosAddr("cal_off_pss3");
    calOffPss4Addr = GetNiosAddr("cal_off_pss4");
    calOffPss5Addr = GetNiosAddr("cal_off_pss5");
    calOffPss6Addr = GetNiosAddr("cal_off_pss6");
    calDPss1Addr = GetNiosAddr("cal_d_pss1");
    calDPss2Addr = GetNiosAddr("cal_d_pss2");
    calDPss3Addr = GetNiosAddr("cal_d_pss3");
    calDPss4Addr = GetNiosAddr("cal_d_pss4");
    calDPss5Addr = GetNiosAddr("cal_d_pss5");
    calDPss6Addr = GetNiosAddr("cal_d_pss6");
    calIMinPssAddr = GetNiosAddr("cal_imin_pss");

    dgpsAzAddr = GetNiosAddr("az_dgps");
    dgpsSigmaAddr = GetNiosAddr("sigma_dgps");
    azSunAddr = GetNiosAddr("az_sun");
    elSunAddr = GetNiosAddr("el_sun");
    azrawPssAddr = GetNiosAddr("azraw_pss");
    azrawPss1Addr = GetNiosAddr("az_raw_pss1");
    azrawPss2Addr = GetNiosAddr("az_raw_pss2");
    azrawPss3Addr = GetNiosAddr("az_raw_pss3");
    azrawPss4Addr = GetNiosAddr("az_raw_pss4");
    azrawPss5Addr = GetNiosAddr("az_raw_pss5");
    azrawPss6Addr = GetNiosAddr("az_raw_pss6");
    elrawPssAddr = GetNiosAddr("elraw_pss");
    elrawPss1Addr = GetNiosAddr("el_raw_pss1");
    elrawPss2Addr = GetNiosAddr("el_raw_pss2");
    elrawPss3Addr = GetNiosAddr("el_raw_pss3");
    elrawPss4Addr = GetNiosAddr("el_raw_pss4");
    elrawPss5Addr = GetNiosAddr("el_raw_pss5");
    elrawPss6Addr = GetNiosAddr("el_raw_pss6");
    snrPss1Addr = GetNiosAddr("snr_pss1");
    snrPss2Addr = GetNiosAddr("snr_pss2");
    snrPss3Addr = GetNiosAddr("snr_pss3");
    snrPss4Addr = GetNiosAddr("snr_pss4");
    snrPss5Addr = GetNiosAddr("snr_pss5");
    snrPss6Addr = GetNiosAddr("snr_pss6");
    azPssAddr = GetNiosAddr("az_pss");  // evolved az
    sigmaPssAddr = GetNiosAddr("sigma_pss");

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
    azDelayAddr = GetNiosAddr("delay_az");
    nScanPerStepAddr = GetNiosAddr("n_scan_per_step");
    sizeElStepAddr = GetNiosAddr("size_el_step");
    nElStepsAddr = GetNiosAddr("n_el_steps");

    vetoSensorAddr = GetNiosAddr("veto_sensor");

    dgpsTimeAddr = GetNiosAddr("time_dgps");
    dgpsLatAddr = GetNiosAddr("lat_dgps");
    dgpsLonAddr = GetNiosAddr("lon_dgps");
    dgpsAltAddr = GetNiosAddr("alt_dgps");
    dgpsSpeedAddr = GetNiosAddr("speed_dgps");
    dgpsDirAddr = GetNiosAddr("dir_dgps");
    dgpsClimbAddr = GetNiosAddr("climb_dgps");
    dgpsNSatAddr = GetNiosAddr("n_sat_dgps");
    attOkAddr = GetNiosAddr("att_ok");
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

    trimNullAddr = GetNiosAddr("trim_null");
    trimMagAddr = GetNiosAddr("trim_mag");
    trimPssAddr = GetNiosAddr("trim_pss");
    dgpsTrimAddr = GetNiosAddr("trim_dgps");
    dgpsCovLimAddr = GetNiosAddr("cov_lim_dgps");
    dgpsAntsLimAddr = GetNiosAddr("ants_lim_dgps");
    scPyramidAddr = GetNiosAddr("pyramid");
    tableVelAddr = GetNiosAddr("vel_table");
    tablePosAddr = GetNiosAddr("pos_table");
    tableModeAddr = GetNiosAddr("mode_table");
    bscTrigAddr = GetNiosAddr("bsc_trigger");

    modeAzMcAddr = GetNiosAddr("mode_az_mc");
    modeElMcAddr = GetNiosAddr("mode_el_mc");
    destAzMcAddr = GetNiosAddr("dest_az_mc");
    destElMcAddr = GetNiosAddr("dest_el_mc");
    velAzMcAddr = GetNiosAddr("vel_az_mc");
    velElMcAddr = GetNiosAddr("vel_el_mc");
    dirAzMcAddr = GetNiosAddr("dir_az_mc");
    dirElMcAddr = GetNiosAddr("dir_el_mc");
    dithElAddr = GetNiosAddr("dith_el");
    velSerRWAddr = GetNiosAddr("vel_ser_rw");
    resRWAddr = GetNiosAddr("res_rw");
    iSerRWAddr = GetNiosAddr("i_ser_rw");
    statDrRWAddr = GetNiosAddr("stat_dr_rw");
    statS1RWAddr = GetNiosAddr("stat_s1_rw");
    infoRWAddr = GetNiosAddr("drive_info_rw");
    driveErrCtsRWAddr = GetNiosAddr("drive_err_cts_rw");
    resPivAddr = GetNiosAddr("res_piv");
    iSerPivAddr = GetNiosAddr("i_ser_piv");
    statDrPivAddr = GetNiosAddr("stat_dr_piv");
    statS1PivAddr = GetNiosAddr("stat_s1_piv");
    velSerPivAddr = GetNiosAddr("vel_ser_piv");
    infoPivAddr = GetNiosAddr("drive_info_piv");
    driveErrCtsPivAddr = GetNiosAddr("drive_err_cts_piv");
    verboseRWAddr = GetNiosAddr("verbose_rw");
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

  WriteCalData(velSerRWAddr, calcVSerRW(), NIOS_QUEUE);

  /*************************************************
   *             Slow Controls                     *
   ************************************************/
  if (!write_slow) return;

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

  /********* PSS data *************/
  WriteData(azrawPssAddr, PointingData[i_point].pss_azraw * DEG2I, NIOS_QUEUE);
  WriteData(azrawPss1Addr, PointingData[i_point].pss1_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(azrawPss2Addr, PointingData[i_point].pss2_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(azrawPss3Addr, PointingData[i_point].pss3_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(azrawPss4Addr, PointingData[i_point].pss4_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(azrawPss5Addr, PointingData[i_point].pss5_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(azrawPss6Addr, PointingData[i_point].pss6_azraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPssAddr, PointingData[i_point].pss_elraw * DEG2I, NIOS_QUEUE);
  WriteData(elrawPss1Addr, PointingData[i_point].pss1_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPss2Addr, PointingData[i_point].pss2_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPss3Addr, PointingData[i_point].pss3_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPss4Addr, PointingData[i_point].pss4_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPss5Addr, PointingData[i_point].pss5_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(elrawPss6Addr, PointingData[i_point].pss6_elraw * DEG2I,
      NIOS_QUEUE);
  WriteData(snrPss1Addr, PointingData[i_point].pss1_snr * 1000., NIOS_QUEUE);
  WriteData(snrPss2Addr, PointingData[i_point].pss2_snr * 1000., NIOS_QUEUE);
  WriteData(snrPss3Addr, PointingData[i_point].pss3_snr * 1000., NIOS_QUEUE);
  WriteData(snrPss4Addr, PointingData[i_point].pss4_snr * 1000., NIOS_QUEUE);
  WriteData(snrPss5Addr, PointingData[i_point].pss5_snr * 1000., NIOS_QUEUE);
  WriteData(snrPss6Addr, PointingData[i_point].pss6_snr * 1000., NIOS_QUEUE);
  WriteData(azPssAddr, (PointingData[i_point].pss_az +
                      CommandData.pss_az_trim) * DEG2I, NIOS_QUEUE);
  WriteData(sigmaPssAddr,
      (unsigned int)(PointingData[i_point].pss_sigma * DEG2I), NIOS_QUEUE);
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
  WriteData(resRWAddr, RWMotorData[i_rw_motors].res_rw*DEG2I, NIOS_QUEUE);
  WriteData(raAddr, (unsigned int)(PointingData[i_point].ra * H2LI),
      NIOS_QUEUE);
  WriteData(decAddr, (unsigned int)(PointingData[i_point].dec * DEG2LI),
      NIOS_QUEUE);

  WriteData(OffsetIFelGYAddr,
      (signed int)(PointingData[i_point].offset_ofpch_gy * 32768.), NIOS_QUEUE);
  WriteData(OffsetIFrollGYAddr,
      (signed int)(PointingData[i_point].offset_ofroll_gy * 32768.),
      NIOS_QUEUE);
  WriteData(OffsetIFyawGYAddr,
      (signed int)(PointingData[i_point].offset_ofyaw_gy * 32768.), NIOS_QUEUE);
  WriteData(latAddr, (unsigned int)(PointingData[i_point].lat * DEG2LI),
      NIOS_QUEUE);
  WriteData(lonAddr, (unsigned int)(PointingData[i_point].lon * DEG2LI),
      NIOS_QUEUE);
  WriteData(altAddr, (unsigned int)(PointingData[i_point].alt), NIOS_QUEUE);

  WriteData(lstAddr, PointingData[i_point].lst, NIOS_QUEUE);

  WriteData(xMagAddr, ACSData.mag_x, NIOS_QUEUE);
  WriteData(yMagAddr, ACSData.mag_y, NIOS_QUEUE);
  WriteData(zMagAddr, ACSData.mag_z, NIOS_QUEUE);
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
  
  WriteData(calXMaxMagAddr,
      (unsigned int)(CommandData.cal_xmax_mag), NIOS_QUEUE);
  WriteData(calXMinMagAddr,
      (unsigned int)(CommandData.cal_xmin_mag), NIOS_QUEUE);
  WriteData(calYMaxMagAddr,
      (unsigned int)(CommandData.cal_ymax_mag), NIOS_QUEUE);
  WriteData(calYMinMagAddr,
      (unsigned int)(CommandData.cal_ymin_mag), NIOS_QUEUE);

  WriteData(calOffPss1Addr, (unsigned int)(CommandData.cal_off_pss1*65536.0/40.0), NIOS_QUEUE);
  WriteData(calOffPss2Addr, (unsigned int)(CommandData.cal_off_pss2*65536.0/40.0), NIOS_QUEUE);
  WriteData(calOffPss3Addr, (unsigned int)(CommandData.cal_off_pss3*65536.0/40.0), NIOS_QUEUE);
  WriteData(calOffPss4Addr, (unsigned int)(CommandData.cal_off_pss4*65536.0/40.0), NIOS_QUEUE);
  WriteData(calOffPss5Addr, (unsigned int)(CommandData.cal_off_pss5*65536.0/40.0), NIOS_QUEUE);
  WriteData(calOffPss6Addr, (unsigned int)(CommandData.cal_off_pss6*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss1Addr, (unsigned int)(CommandData.cal_d_pss1*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss2Addr, (unsigned int)(CommandData.cal_d_pss2*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss3Addr, (unsigned int)(CommandData.cal_d_pss3*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss4Addr, (unsigned int)(CommandData.cal_d_pss4*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss5Addr, (unsigned int)(CommandData.cal_d_pss5*65536.0/40.0), NIOS_QUEUE);
  WriteData(calDPss6Addr, (unsigned int)(CommandData.cal_d_pss6*65536.0/40.0), NIOS_QUEUE);
  WriteData(calIMinPssAddr, (unsigned int)(CommandData.cal_imin_pss*65536.0/40.0), NIOS_QUEUE);

  WriteData(trimPssAddr, CommandData.pss_az_trim * DEG2I, NIOS_QUEUE);

  WriteData(dgpsAzAddr,
      (unsigned int)((PointingData[i_point].dgps_az  +
                      CommandData.dgps_az_trim) * DEG2I), NIOS_QUEUE);
  WriteData(dgpsSigmaAddr,
      (((unsigned int)(PointingData[i_point].dgps_sigma * DEG2I)) > 65535)
      ? 65535 : ((unsigned int)(PointingData[i_point].dgps_sigma * DEG2I)),
      NIOS_QUEUE);
  WriteData(dgpsTrimAddr, CommandData.dgps_az_trim * DEG2I, NIOS_QUEUE);
  WriteData(dgpsCovLimAddr, CommandData.dgps_cov_limit*32768.0/100.0,
      NIOS_QUEUE);
  WriteData(dgpsAntsLimAddr, CommandData.dgps_ants_limit*32768.0/100.0,
      NIOS_QUEUE);
  WriteData(azSunAddr, (unsigned int)(PointingData[i_point].sun_az*DEG2I),
      NIOS_QUEUE);
  WriteData(elSunAddr, (int)(PointingData[i_point].sun_el*DEG2I), NIOS_QUEUE);
  WriteData(scPyramidAddr, CommandData.pyramid, NIOS_QUEUE);
  WriteData(tableVelAddr, (int)(CommandData.table.vel*1000.0), NIOS_QUEUE);
  WriteData(tablePosAddr, (int)(CommandData.table.pos*1000.0), NIOS_QUEUE);
  WriteData(tableModeAddr, CommandData.table.mode, NIOS_QUEUE);
  WriteData(bscTrigAddr, bsc_trigger, NIOS_QUEUE);

  WriteData(trimNullAddr, CommandData.null_az_trim * DEG2I, NIOS_QUEUE);

  /************* Pointing mode fields *************/
  WriteCalData(slewVetoAddr, (CommandData.pointing_mode.nw)
      / (ACSData.bbc_rate), NIOS_QUEUE);
  WriteCalData(svetoLenAddr, (CommandData.slew_veto) / (ACSData.bbc_rate),
      NIOS_QUEUE);
  WriteData(modePAddr, (int)(CommandData.pointing_mode.mode), NIOS_QUEUE);
  if ((CommandData.pointing_mode.mode == P_AZEL_GOTO) ||
      (CommandData.pointing_mode.mode == P_AZ_SCAN))
    WriteData(xPAddr, (int)(CommandData.pointing_mode.X * DEG2I),
        NIOS_QUEUE);
  else
    WriteData(xPAddr, (int)(CommandData.pointing_mode.X * H2I), NIOS_QUEUE);

  WriteData(yPAddr, (int)(CommandData.pointing_mode.Y * DEG2I), NIOS_QUEUE);
  WriteData(velAzPAddr, (int)(CommandData.pointing_mode.vaz*VEL2I), NIOS_QUEUE);
  WriteCalData(delPAddr, CommandData.pointing_mode.del, NIOS_QUEUE);
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
  WriteCalData(azDelayAddr, CommandData.pointing_mode.az_delay, NIOS_QUEUE);
  WriteCalData(nScanPerStepAddr,CommandData.pointing_mode.Nscans,NIOS_QUEUE);
  WriteCalData(sizeElStepAddr, CommandData.pointing_mode.el_step, NIOS_QUEUE);
  WriteCalData(nElStepsAddr, CommandData.pointing_mode.Nsteps, NIOS_QUEUE);


  sensor_veto = (!CommandData.use_elenc1) |
    /* bit for << 1 unused */
    ((!CommandData.use_mag) << 2) |
    ((!CommandData.use_gps) << 3) |
    ((!CommandData.use_elenc2) << 4) |
    /* bit for << 5 unused */
    /* bit for << 6 unused */
    /* bit for is_sched (below) << 7 */
    ((CommandData.az_autogyro) << 8) |
    ((CommandData.el_autogyro) << 9) |
    ((CommandData.disable_el) << 10) |
    ((CommandData.disable_az) << 11) |
    ((CommandData.force_el) << 12) |
    ((!CommandData.use_pss) << 13);

  if (PointingData[i_point].t >= CommandData.pointing_mode.t)
    sensor_veto |= (1 << 7);

  WriteData(vetoSensorAddr, sensor_veto, NIOS_QUEUE);

  /************* dgps fields *************/
  WriteData(dgpsTimeAddr, DGPSTime, NIOS_QUEUE);

  /** Pos fields **/
  i_dgps = GETREADINDEX(dgpspos_index);
  WriteData(dgpsLatAddr, (int)(DGPSPos[i_dgps].lat * DEG2I), NIOS_QUEUE);
  WriteData(dgpsLonAddr, (int)(DGPSPos[i_dgps].lon * DEG2I), NIOS_QUEUE);
  WriteData(dgpsAltAddr, DGPSPos[i_dgps].alt, NIOS_QUEUE);
  WriteData(dgpsSpeedAddr, DGPSPos[i_dgps].speed*100, NIOS_QUEUE);
  WriteData(dgpsDirAddr, (unsigned int)DGPSPos[i_dgps].direction*DEG2I,
      NIOS_QUEUE);
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
  WriteData(iSerRWAddr,((int)(RWMotorData[i_rw_motors].current/60.0*32768.0))
            ,NIOS_QUEUE); // need to check scaling
  WriteData(statDrRWAddr,(RWMotorData[i_rw_motors].db_stat & 0xff)
            +((RWMotorData[i_rw_motors].dp_stat & 0xff)<< 8),NIOS_QUEUE);
  WriteData(statS1RWAddr,RWMotorData[i_rw_motors].ds1_stat,NIOS_QUEUE);
  WriteData(infoRWAddr,RWMotorData[i_rw_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsRWAddr,RWMotorData[i_rw_motors].err_count,NIOS_QUEUE);
  WriteData(iSerPivAddr,PivotMotorData[i_pivot_motors].current*32768.0/20.0,
      NIOS_QUEUE);
  WriteData(statDrPivAddr,(PivotMotorData[i_pivot_motors].db_stat & 0xff)
      +((PivotMotorData[i_pivot_motors].dp_stat & 0xff)<< 8), NIOS_QUEUE);
  WriteData(statS1PivAddr,PivotMotorData[i_pivot_motors].ds1_stat,NIOS_QUEUE);
  WriteCalData(velSerPivAddr, PivotMotorData[i_pivot_motors].dps_piv,
               NIOS_QUEUE);
  WriteData(resPivAddr,
      PivotMotorData[i_pivot_motors].res_piv*DEG2I, NIOS_QUEUE);
  WriteData(infoPivAddr,PivotMotorData[i_pivot_motors].drive_info,NIOS_QUEUE);
  WriteData(driveErrCtsPivAddr,PivotMotorData[i_pivot_motors].err_count,
      NIOS_QUEUE);
  WriteData(verboseRWAddr,CommandData.verbose_rw,NIOS_QUEUE);
  WriteData(verbosePivAddr,CommandData.verbose_piv,NIOS_QUEUE);

  att_ok = (PointingData[i_point].pss_ok << 2) | (PointingData[i_point].mag_ok << 1) | DGPSAtt[i_dgps].att_ok;
  WriteData(attOkAddr, att_ok, NIOS_QUEUE);
  
}

void InitTxFrame()
{
  int bus, m, i, j, niosAddr, m0addr;

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
  mcp_initial_controls = 1;
  UpdateBBCFrame();
  mcp_initial_controls = 0;

  /* write the framesync to address 0 to get things going... */
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
  long long cal = round( (data - addr->b)/addr->m );

  //check that rounding doesn't lead to over/underflow
  if (addr->wide) {   //wide
    if (addr->sign) {   //signed
      if (cal > INT_MAX) cal = INT_MAX;
      else if (cal < INT_MIN) cal = INT_MIN;
    } else {      //unsigned
      if (cal > UINT_MAX) cal = UINT_MAX;
      else if (cal < 0) cal = 0;
    }
  } else {      //narrow
    if (addr->sign) {   //signed
      if (cal > SHRT_MAX) cal = SHRT_MAX;
      else if (cal < SHRT_MIN) cal = SHRT_MIN;
    } else {     //unsigned
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
  } else {    //unsigned
    return ((double)ReadData(addr) * addr->nios->m + addr->nios->b);
  }
}

static void WatchMCC()
{
  int i;
  int timeout;
  static int reboottimer[6] = {0, 0, 0, 0, 0, 0};

  timeout = (int)(20.0*(ACSData.bbc_rate/FAST_PER_SLOW));
  timeout = (timeout > 0) ? timeout : 25;

  for (i=0; i<6; i++) {
    /* ignore MCCs which have been turned off */
    if (CommandData.mccs_off & (1U << i)) {
      mccs_alive &= (1U << i);
      continue;
    }

    mccSlowCount[i]++;
    if (reboottimer[i] > 0) {
      reboottimer[i]--;
    }
    if (mccSlowCount[i] >= timeout) {
      mccs_alive &= ~(1U << i);
      if ((CommandData.mcc_wdog) && (reboottimer[i] == 0) ) {
        switch (i) {
          case 0:
            CommandData.power.mcc1.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc1.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          case 1:
            CommandData.power.mcc2.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc2.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          case 2:
            CommandData.power.mcc3.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc3.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          case 3:
            CommandData.power.mcc4.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc4.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          case 4:
            CommandData.power.mcc5.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc5.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          case 5:
            CommandData.power.mcc6.set_count = PCYCLE_HOLD_LEN 
              + LATCH_PULSE_LEN;
            CommandData.power.mcc6.rst_count = LATCH_PULSE_LEN;
            reboottimer[i] = (int) 300*(ACSData.bbc_rate/FAST_PER_SLOW);
            break;
          default:
            break;
        }
        mccSlowCount[i] = 0;
      }
    }
  }
}

/* called from mcp, should call all nios writing functions */
void UpdateBBCFrame()
{
  static int index = 0;

  /*** do fast Controls ***/
  if (!mcp_initial_controls)
    DoSched();
  UpdateAxesMode();
  StoreData(index==15);
  WriteMot(index==14);
  updateTableSpeed();
  WriteChatter(index);
  countHWPEncoder(index);
  cameraTriggers();
  HouseKeeping(index==9);

  WriteAux();
  
  switch (index) {
    case 0:
      //FIXME WriteAux should be slow. What didn't work?
      //WriteAux();
      break;
    case 1:
      ChargeController();
      break;
    case 2:
      WriteSyncBox();
      break;
    case 3:
      ControlPower();
      break;
    case 4:
      LockMotor();
      break;
    case 5:
      ControlHeaters();
      break;
    case 6:
      SetGyroMask();
      break;
    case 7:
      StoreHWPBus();
      break;
    case 8:
      WriteMCESlow();
      break;
    case 9:
      //HouseKeeping() slows
      break;
    case 10:
      cameraFields(0);
      break;
    case 11:
      cameraFields(1);
      break;
    case 12:
      cameraFields(2);
      break;
    case 13:
      if (!mcp_initial_controls) SyncADC();
      break;
    case 14:
      //WriteMot() slows
      break;
    case 15:
      //StoreData() slows
      break;
    case 16:
      SFTValveMotors();
      break;
    case 17:
      WatchMCC();
      break;
    default:
      break;
  }
      
  if (!mcp_initial_controls)
    index = (index + 1) % FAST_PER_SLOW;

  //make sure frame is flushed
  RawNiosWrite(-1,-1,NIOS_FLUSH);
}
