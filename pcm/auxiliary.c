/* mcp: the Spider master control program
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

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>

#include "mcp.h"
#include "channels.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "chrgctrl.h"
#include "sync_comms.h"
#include "mceserv.h"
#include "lut.h"

#ifndef LUT_DIR
#define LUT_DIR "/data/etc/spider/"
#endif


/* Define to 1 to send synchronous star camera triggers based on ISC
 * handshaking */
#define SYNCHRONOUS_CAMERAS 0

/* This is the length of time to wait for an ACK from the star camera before
 * giving up */
#define ISC_ACK_TIMEOUT    100   /* in 100Hz frames */

/* This is the length of time to wait for a pointing solution from the star
 * camera before giving up */
#define ISC_DATA_TIMEOUT   900   /* in 100Hz frames */

/* ISC_ACK_TIMEOUT + ISC_DATA_TIMEOUT sets the maximum length of the cycle.
 * ISC_ACK_TIMEOUT is purely responsible for the link-ok check and should be
 * made small, since any excess time effectively ends up as data timeout time
 * anyways. */

/* This is the length of time to wait after receiving the ACK, bufore sending
 * the trigger */
#define ISC_TRIGGER_DELAY   ((which == 1) ? 30 : 30)  /* in 100Hz frames */

/* If mcp has decided the handshaking isn't working, this is the period of the
 * handshake-less triggers */
#define ISC_DEFAULT_PERIOD 150   /* in 100Hz frames */

/* limits for thermometers.  If the reading is outside this range,
 * we don't regulate the temperature at all, since it means the thermometer is probably
 * broken */
#define MIN_TEMP -53.0
#define MAX_TEMP 60.0

/* Gybox heater stuff */
#define GY_HEAT_MAX 40 /* percent */
#define GY_HEAT_MIN 5  /* percent */
#define GY_TEMP_MAX 50 /* setpoint maximum -- deg C */
#define GY_TEMP_MIN 0  /* setpoint minimum -- deg C */
#define GY_TEMP_STEP 1 /* setpoint step - deg C */
#define GY_HEAT_TC 30000 /* integral/age characteristic time in 100Hz Frames */

extern short int InCharge; /* tx.c */

#define PUMP_MAX 26214      /*  3.97*2.0V   */
#define PUMP_MIN  3277      /*  3.97*0.25V   */

#define PUMP_ZERO 32773

/* in commands.c */
double LockPosition(double elevation);

#define PGR_NSUM 4.0
#define PGR_OFF PGR_NSUM/2.0
// #define PGR_NORM sqrt(PGR_NSUM / 12.0)
#define PGR_NORM 1.7320508 
double pseudoGaussianRand() {
  double x;
  
  x = ((double)rand() 
    + (double)rand() 
    + (double)rand() 
    + (double)rand()
  ) * (1.0/(double)RAND_MAX);
  x-= PGR_OFF;
  x *= PGR_NORM;
  
  return x;
}


/************************************************************************/
/*    ControlHeaters:  Controls gyro box temp                          */
/************************************************************************/
void ControlHeaters()
{
  static struct BiPhaseStruct* tAddrs[N_HEATERS];
  static struct NiosStruct *tSetAddrs[N_HEATERS];
  static struct NiosStruct *heatIFAddr;
  static struct NiosStruct *heatGyAddr; 
  
  const char *tNames[] = {"vt_gy", "vt_piv", "vt_elport_mot", 
    "vt_elstar_mot", "vt_mt_tavco", "vt_sftv",""
  };
  const char *setNames[] = {"t_set_gybox", "t_set_pivot", "t_set_elmot_p",
    "t_set_elmot_s", "t_set_mttavco", "t_set_motvalve", ""
  };
    
  static struct LutType tLut = {.filename = LUT_DIR "thermistor.lut"};

  static int firsttime = 1;
  
  double set_point;
  double T;
  int i;
  
  unsigned short outbits = 0, gy_on = 0;
  
  if (firsttime) {
    firsttime = 0;
    for (i=0; i<N_HEATERS; i++) {
      tAddrs[i] =  GetBiPhaseAddr(tNames[i]);
      tSetAddrs[i] = GetNiosAddr(setNames[i]);
    }
    heatGyAddr = GetNiosAddr("heat_gy");
    heatIFAddr = GetNiosAddr("heat_if");

    LutInit(&tLut);
  }
  
  for (i=0; i<N_HEATERS; i++) {
    WriteCalData(tSetAddrs[i], CommandData.t_set[i], NIOS_QUEUE);
  
    T = ReadCalData(tAddrs[i]);
    T = LutCal(&tLut, T);
 
    if (T<-55) T = 59; // so we can heaters in an emergency
    set_point = CommandData.t_set[i] + 0.1 * pseudoGaussianRand();
    /* Only run these controls if we think the thermometer isn't broken */
    if (T < MAX_TEMP && T > MIN_TEMP && T<set_point) {
      if (i>0) {
        outbits |= 0x01 << (i+7);
      } else { 
        gy_on = 1;
      }
    }
  }
  // turn on the SFT pump
  if (CommandData.sft_pump) {
    outbits |= (0x01 << (N_HEATERS+7));
  }
  
  if (gy_on) {
    WriteData(heatGyAddr, 0x01, NIOS_FLUSH);
  } else {
    WriteData(heatGyAddr, 0x00, NIOS_FLUSH);
  }
  WriteData(heatIFAddr, outbits, NIOS_FLUSH);
}

#if 0
/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temp                          */
/************************************************************************/
void ControlGyroHeat()
{
  static struct BiPhaseStruct* tGyAddr;
  static struct NiosStruct *heatGyAddr, *tSetGyAddr, *gPHeatGyAddr;
  static struct NiosStruct *gIHeatGyAddr;
  static struct NiosStruct *gDHeatGyAddr;
  static int firsttime = 1;

  static int p_on = 0;
  static int p_off = -1;

  float error = 0, set_point;
  double t_gy;
  static float integral = 0;
  static float deriv = 0;
  static float error_last = 0;
  float P, I, D;
  static struct LutType tGyLut = {.filename = LUT_DIR "thermistor.lut"};

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    tGyAddr = GetBiPhaseAddr("vt_gy");
    heatGyAddr = GetNiosAddr("heat_gy");
    tSetGyAddr = GetNiosAddr("t_set_gy");

    gPHeatGyAddr = GetNiosAddr("g_p_heat_gy");
    gIHeatGyAddr = GetNiosAddr("g_i_heat_gy");
    gDHeatGyAddr = GetNiosAddr("g_d_heat_gy");
    LutInit(&tGyLut);
  }

  /* send down the setpoints and gains values */
  WriteData(tSetGyAddr, CommandData.gyheat.setpoint * 327.68, NIOS_QUEUE);
  WriteData(gPHeatGyAddr, CommandData.gyheat.gain.P, NIOS_QUEUE);
  WriteData(gIHeatGyAddr, CommandData.gyheat.gain.I, NIOS_QUEUE);
  WriteData(gDHeatGyAddr, CommandData.gyheat.gain.D, NIOS_QUEUE);

  t_gy = ReadCalData(tGyAddr);
  t_gy = LutCal(&tGyLut, t_gy);

  set_point = CommandData.gyheat.setpoint;
  /* Only run these controls if we think the thermometer isn't broken */
  if (t_gy < MAX_GYBOX_TEMP && t_gy > MIN_GYBOX_TEMP) {
    P = CommandData.gyheat.gain.P * (1);
    I = CommandData.gyheat.gain.I * (1.0 / 11.0);
    D = CommandData.gyheat.gain.D * ( 10.0 );
    
    /********* if end of pulse, calculate next pulse *********/
    if (p_off <= 0 && p_on <= 0) {
      error = set_point - t_gy;

      integral = integral * 0.999 + 0.001 * error;
      if (integral * I > 10)
        integral = 10.0 / I;

      if (integral * I < 0)
        integral = 0;

      deriv = error - error_last;
      error_last = error;

      p_on = P * error + (deriv / 10.0) * D + integral * I;

      if (p_on > 10)
        p_on = 10;
      else if (p_on < 0)
        p_on = 0;

      p_off = 10 - p_on;

    }

    /******** do the pulse *****/
    if (p_on > 0) {
      WriteData(heatGyAddr, 0x1, NIOS_FLUSH);
      //WriteData(heatGyAddr, 0x0, NIOS_FLUSH);
      p_on--;
    } else if (p_off > 0) {
      WriteData(heatGyAddr, 0x0, NIOS_FLUSH);
      p_off--;
    }
  } else {
    /* Turn off heater if thermometer appears broken */
    WriteData(heatGyAddr, 0x0, NIOS_FLUSH);  
  }
}
#endif

void ChargeController(void)
{

  static struct NiosStruct *VBattOfCCAddr;
  static struct NiosStruct *VBattIfCCAddr;
  static struct NiosStruct *VArrOfCCAddr;
  static struct NiosStruct *VArrIfCCAddr;
  static struct NiosStruct *IBattOfCCAddr;
  static struct NiosStruct *IBattIfCCAddr;
  static struct NiosStruct *IArrOfCCAddr;
  static struct NiosStruct *IArrIfCCAddr;
  static struct NiosStruct *VTargOfCCAddr;
  static struct NiosStruct *VTargIfCCAddr;
  static struct NiosStruct *ThsOfCCAddr;
  static struct NiosStruct *ThsIfCCAddr;
  static struct NiosStruct *FaultOfCCAddr;
  static struct NiosStruct *FaultIfCCAddr;
  static struct NiosStruct *AlarmHiOfCCAddr;
  static struct NiosStruct *AlarmHiIfCCAddr;
  static struct NiosStruct *AlarmLoOfCCAddr;
  static struct NiosStruct *AlarmLoIfCCAddr;
  static struct NiosStruct *ChargeOfCCAddr;
  static struct NiosStruct *ChargeIfCCAddr;
  static struct NiosStruct *LEDOfCCAddr;
  static struct NiosStruct *LEDIfCCAddr;

  static int firsttime = 1;

  if (firsttime) {

    firsttime = 0;

    VBattOfCCAddr = GetNiosAddr("v_batt_of_cc");
    VArrOfCCAddr = GetNiosAddr("v_arr_of_cc");
    IBattOfCCAddr = GetNiosAddr("i_batt_of_cc");
    IArrOfCCAddr  = GetNiosAddr("i_arr_of_cc");
    VTargOfCCAddr = GetNiosAddr("v_targ_of_cc");
    ThsOfCCAddr = GetNiosAddr("t_hs_of_cc");
    FaultOfCCAddr = GetNiosAddr("fault_of_cc");
    AlarmHiOfCCAddr = GetNiosAddr("alarm_hi_of_cc");
    AlarmLoOfCCAddr = GetNiosAddr("alarm_lo_of_cc");
    ChargeOfCCAddr = GetNiosAddr("state_of_cc");
    LEDOfCCAddr = GetNiosAddr("led_of_cc");

    VBattIfCCAddr = GetNiosAddr("v_batt_if_cc");
    VArrIfCCAddr = GetNiosAddr("v_arr_if_cc");
    IBattIfCCAddr = GetNiosAddr("i_batt_if_cc");
    IArrIfCCAddr  = GetNiosAddr("i_arr_if_cc");
    VTargIfCCAddr = GetNiosAddr("v_targ_if_cc");
    ThsIfCCAddr = GetNiosAddr("t_hs_if_cc");
    FaultIfCCAddr = GetNiosAddr("fault_if_cc");
    AlarmHiIfCCAddr = GetNiosAddr("alarm_hi_if_cc");
    AlarmLoIfCCAddr = GetNiosAddr("alarm_lo_if_cc");
    ChargeIfCCAddr = GetNiosAddr("state_if_cc");
    LEDIfCCAddr = GetNiosAddr("led_if_cc");


  }

  WriteData(VBattOfCCAddr, 180.0*ChrgCtrlData[0].V_batt + 32400.0, NIOS_QUEUE);
  WriteData(VArrOfCCAddr, 180.0*ChrgCtrlData[0].V_arr + 32400.0, NIOS_QUEUE);
  WriteData(IBattOfCCAddr, 400.0*ChrgCtrlData[0].I_batt + 32000.0, NIOS_QUEUE);
  WriteData(IArrOfCCAddr,  400.0*ChrgCtrlData[0].I_arr + 32000.0, NIOS_QUEUE);
  WriteData(VTargOfCCAddr, 180.0*ChrgCtrlData[0].V_targ + 32400.0, NIOS_QUEUE);
  WriteData(ThsOfCCAddr, ChrgCtrlData[0].T_hs, NIOS_QUEUE);
  WriteData(FaultOfCCAddr, ChrgCtrlData[0].fault_field, NIOS_QUEUE);
  WriteData(AlarmHiOfCCAddr, ChrgCtrlData[0].alarm_field_hi, NIOS_QUEUE);
  WriteData(AlarmLoOfCCAddr, ChrgCtrlData[0].alarm_field_lo, NIOS_QUEUE);
  WriteData(ChargeOfCCAddr, ChrgCtrlData[0].charge_state, NIOS_QUEUE);
  WriteData(LEDOfCCAddr, ChrgCtrlData[0].led_state, NIOS_QUEUE);

  WriteData(VBattIfCCAddr, 180.0*ChrgCtrlData[1].V_batt + 32400.0, NIOS_QUEUE);
  WriteData(VArrIfCCAddr, 180.0*ChrgCtrlData[1].V_arr + 32400.0, NIOS_QUEUE);
  WriteData(IBattIfCCAddr, 400.0*ChrgCtrlData[1].I_batt + 32000.0, NIOS_QUEUE);
  WriteData(IArrIfCCAddr,  400.0*ChrgCtrlData[1].I_arr + 32000.0, NIOS_QUEUE);
  WriteData(VTargIfCCAddr, 180.0*ChrgCtrlData[1].V_targ + 32400.0, NIOS_QUEUE);
  WriteData(ThsIfCCAddr, ChrgCtrlData[1].T_hs, NIOS_QUEUE);
  WriteData(FaultIfCCAddr, ChrgCtrlData[1].fault_field, NIOS_QUEUE);
  WriteData(AlarmHiIfCCAddr, ChrgCtrlData[1].alarm_field_hi, NIOS_QUEUE);
  WriteData(AlarmLoIfCCAddr, ChrgCtrlData[1].alarm_field_lo, NIOS_QUEUE);
  WriteData(ChargeIfCCAddr, ChrgCtrlData[1].charge_state, NIOS_QUEUE);
  WriteData(LEDIfCCAddr, ChrgCtrlData[1].led_state, NIOS_QUEUE);

}

void WriteSyncBox(void)
{
  static struct NiosStruct* rowLenSyncAddr;
  static struct NiosStruct* numRowsSyncAddr;
  static struct NiosStruct* freeRunSyncAddr;

  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    rowLenSyncAddr = GetNiosAddr("row_len_sync");
    numRowsSyncAddr = GetNiosAddr("num_rows_sync");
    freeRunSyncAddr = GetNiosAddr("data_rate_sync");
  }

  WriteData(rowLenSyncAddr, SyncBoxData.row_len, NIOS_QUEUE);
  WriteData(numRowsSyncAddr, SyncBoxData.num_rows, NIOS_QUEUE);
  WriteData(freeRunSyncAddr, SyncBoxData.free_run, NIOS_QUEUE);
}

/* convert mce_pow_op command into a latch_pulse, and have MCEServ veto
 * appropriately */
static void do_mce_power_op(void)
{
  static int pwr_timer[3] = {-1, -1, -1};

  int i;

  for (i = 0; i < 3; ++i) {
    if (CommandData.ifpower.mce_op[i] == mce_pow_off ||
        CommandData.ifpower.mce_op[i] == mce_pow_cyc)
    { /* turn off */
      if (pwr_timer[i] == -1) {
        /* veto the MCEs -- will cause MPC to debias SQUIDs */
        CommandData.mce_power |= (1U << i);

        /* wait for debias */
        pwr_timer[i] = (int)(5.0*(ACSData.bbc_rate/FAST_PER_SLOW)); /* sec */
        bprintf(info, "veto MCEbank %i", i);
      } else if (pwr_timer[i] == 0) {
        CommandData.ifpower.mce[i].set_count = 0;
        CommandData.ifpower.mce[i].rst_count = LATCH_PULSE_LEN;
        if (CommandData.ifpower.mce_op[i] == mce_pow_off) {
          CommandData.ifpower.mce_op[i] = mce_pow_nop;
          pwr_timer[i] = -1;
        } else {
          /* wait for power cycle */
          CommandData.ifpower.mce_op[i] = mce_pow_wait;
          pwr_timer[i] = (int)(5.0*(ACSData.bbc_rate/FAST_PER_SLOW)); /* sec */
        }
        bprintf(info, "MCEbank off %i", i);
      } else
        pwr_timer[i]--;
    } else if (CommandData.ifpower.mce_op[i] == mce_pow_on) { /* turn on */
      if (pwr_timer[i] == -1) {
        CommandData.ifpower.mce[i].rst_count = 0;
        CommandData.ifpower.mce[i].set_count = LATCH_PULSE_LEN;
        bprintf(info, "MCEbank on %i", i);

        /* wait for power on */
        pwr_timer[i] = (int)(5.0*(ACSData.bbc_rate/FAST_PER_SLOW)); /* sec */
      } else if (pwr_timer[i] == 0) {
        /* unveto the MCEs */
        CommandData.mce_power &= ~(1U << i);
        CommandData.ifpower.mce_op[i] = mce_pow_nop;
        pwr_timer[i] = -1;
        bprintf(info, "unveto MCEbank %i", i);
      } else
        pwr_timer[i]--;
    } else if (CommandData.ifpower.mce_op[i] == mce_pow_wait) {
      /* power cycle wait */
      if (pwr_timer[i] <= 0) {
        /* turn on */
        CommandData.ifpower.mce_op[i] = mce_pow_on;
        pwr_timer[i] = -1;
      } else
        pwr_timer[i]--;
    }

    if (CommandData.ifpower.mce_mpcveto[i])
      CommandData.ifpower.mce_mpcveto[i]--;
  }
}

/* create latching relay pulses, and update enable/disbale levels */
void ControlPower(void) {
  static int firsttime = 1;
  static struct NiosStruct* latchingAddr[2];
  static struct NiosStruct* switchGyAddr;
  static struct NiosStruct* switchMiscAddr;
  static struct NiosStruct* switchPvAddr;
  static struct NiosStruct* ifPwrAddr;
  static struct NiosStruct* mcePowerAddr;
  /* grp2 = MCC and charge controller relays */
  static struct NiosStruct* switchGrp2Addr;
  int latch0 = 0, latch1 = 0, gybox = 0, misc = 0, ifpwr = 0, grp2 = 0;
  int pv = 0;
  int i;

  if (firsttime) {
    firsttime = 0;
    latchingAddr[0] = GetNiosAddr("latch0");
    latchingAddr[1] = GetNiosAddr("latch1");
    ifPwrAddr = GetNiosAddr("ifpwr");
    mcePowerAddr = GetNiosAddr("mce_power");
    switchGyAddr = GetNiosAddr("switch_gy");
    switchMiscAddr = GetNiosAddr("switch_misc");
    switchPvAddr = GetNiosAddr("switch_pv");
    switchGrp2Addr = GetNiosAddr("switch_grp2");
  }

  /* PV1 */
  if (CommandData.power.pv_data3_136_off) {
    if (CommandData.power.pv_data3_136_off > 0)
      CommandData.power.pv_data3_136_off--;
    misc |= 0x02;
  }

  /* PV2 */
  if (CommandData.power.pv_data2_236_off) {
    if (CommandData.power.pv_data2_236_off > 0)
      CommandData.power.pv_data2_236_off--;
    misc |= 0x01;
  }

  /* PV3 */
  if (CommandData.power.pv_data2_145_off) {
    if (CommandData.power.pv_data2_145_off > 0)
      CommandData.power.pv_data2_145_off--;
    pv = 1; /* this is bit 0x10 on the MISC group */
  }

  /* PV4 */
  if (CommandData.power.pv_data3_245_off) {
    if (CommandData.power.pv_data3_245_off > 0)
      CommandData.power.pv_data3_245_off--;
    misc |= 0x04;
  }

  if (CommandData.power.hub232_off) {
    if (CommandData.power.hub232_off > 0) CommandData.power.hub232_off--;
    misc |= 0x08;
  }

  //NB: the bit for lock power is asserted with the motors are ON
  if (CommandData.power.lock_off) {
    if (CommandData.power.lock_off > 0) CommandData.power.lock_off--;
    misc &= ~0x20;
  } else
    misc |= 0x20;

  if (CommandData.power.sync.set_count > 0) {
    CommandData.power.sync.set_count--;
    if (CommandData.power.sync.set_count < LATCH_PULSE_LEN) misc |= 0x40;
  }
  if (CommandData.power.sync.rst_count > 0) {
    CommandData.power.sync.rst_count--;
    if (CommandData.power.sync.rst_count < LATCH_PULSE_LEN) misc |= 0x80;
  }

  if (CommandData.power.charge.set_count > 0) {
    CommandData.power.charge.set_count--;
    if (CommandData.power.charge.set_count < LATCH_PULSE_LEN) grp2 |= 0x0001;
  }
  if (CommandData.power.charge.rst_count > 0) {
    CommandData.power.charge.rst_count--;
    if (CommandData.power.charge.rst_count < LATCH_PULSE_LEN) grp2 |= 0x0002;
  }
  if (CommandData.power.ifcharge.set_count > 0) {
    CommandData.power.ifcharge.set_count--;
    if (CommandData.power.ifcharge.set_count < LATCH_PULSE_LEN) grp2 |= 0x0004;
  }
  if (CommandData.power.ifcharge.rst_count > 0) {
    CommandData.power.ifcharge.rst_count--;
    if (CommandData.power.ifcharge.rst_count < LATCH_PULSE_LEN) grp2 |= 0x0008;
  }
  for (i=0; i<6; i++) {
    if (CommandData.power.gyro_off[i] || CommandData.power.gyro_off_auto[i]) {
      if (CommandData.power.gyro_off[i] > 0)
        CommandData.power.gyro_off[i]--;
      if (CommandData.power.gyro_off_auto[i] > 0)
        CommandData.power.gyro_off_auto[i]--;
      gybox |= 0x01 << i;
    }
  }

  if (CommandData.power.gybox_off) {
    if (CommandData.power.gybox_off > 0) CommandData.power.gybox_off--;
    gybox |= 0x80;
  }

  if (CommandData.power.sc_tx.set_count > 0) {
    CommandData.power.sc_tx.set_count--;
    if (CommandData.power.sc_tx.set_count < LATCH_PULSE_LEN) latch0 |= 0x0001;
  }
  if (CommandData.power.sc_tx.rst_count > 0) {
    CommandData.power.sc_tx.rst_count--;
    if (CommandData.power.sc_tx.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0002;
  }
  if (CommandData.power.das.set_count > 0) {
    CommandData.power.das.set_count--;
    if (CommandData.power.das.set_count < LATCH_PULSE_LEN) latch0 |= 0x0004;
  }
  if (CommandData.power.das.rst_count > 0) {
    CommandData.power.das.rst_count--;
    if (CommandData.power.das.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0008;
  }
  if (CommandData.power.rsc.set_count > 0) {
    CommandData.power.rsc.set_count--;
    if (CommandData.power.rsc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0010;
  }
  if (CommandData.power.rsc.rst_count > 0) {
    CommandData.power.rsc.rst_count--;
    if (CommandData.power.rsc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0020;
  }
  if (CommandData.power.bsc.set_count > 0) {
    CommandData.power.bsc.set_count--;
    if (CommandData.power.bsc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0040;
  }
  if (CommandData.power.bsc.rst_count > 0) {
    CommandData.power.bsc.rst_count--;
    if (CommandData.power.bsc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0080;
  }
  if (CommandData.power.gps.set_count > 0) {
    CommandData.power.gps.set_count--;
    if (CommandData.power.gps.set_count < LATCH_PULSE_LEN) latch0 |= 0x0100;
  }
  if (CommandData.power.gps.rst_count > 0) {
    CommandData.power.gps.rst_count--;
    if (CommandData.power.gps.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0200;
  }
  if (CommandData.power.rw.set_count > 0) {
    CommandData.power.rw.set_count--;
    if (CommandData.power.rw.set_count < LATCH_PULSE_LEN) latch0 |= 0x0400;
  }
  if (CommandData.power.rw.rst_count > 0) {
    CommandData.power.rw.rst_count--;
    if (CommandData.power.rw.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0800;
  }
  if (CommandData.power.piv.set_count > 0) {
    CommandData.power.piv.set_count--;
    if (CommandData.power.piv.set_count < LATCH_PULSE_LEN) latch0 |= 0x1000;
  }
  if (CommandData.power.piv.rst_count > 0) {
    CommandData.power.piv.rst_count--;
    if (CommandData.power.piv.rst_count < LATCH_PULSE_LEN) latch0 |= 0x2000;
  }
  if (CommandData.power.elmot.set_count > 0) {
    CommandData.power.elmot.set_count--;
    if (CommandData.power.elmot.set_count < LATCH_PULSE_LEN) latch0 |= 0x4000;
  }
  if (CommandData.power.elmot.rst_count > 0) {
    CommandData.power.elmot.rst_count--;
    if (CommandData.power.elmot.rst_count < LATCH_PULSE_LEN) latch0 |= 0x8000;
  }
  if (CommandData.power.bi0.set_count > 0) {
    CommandData.power.bi0.set_count--;
    if (CommandData.power.bi0.set_count < LATCH_PULSE_LEN) latch1 |= 0x0001;
  }
  if (CommandData.power.bi0.rst_count > 0) {
    CommandData.power.bi0.rst_count--;
    if (CommandData.power.bi0.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0002;
  }
  if (CommandData.power.table.set_count > 0) {
    CommandData.power.table.set_count--;
    if (CommandData.power.table.set_count < LATCH_PULSE_LEN) latch1 |= 0x0040;
  }
  if (CommandData.power.table.rst_count > 0) {
    CommandData.power.table.rst_count--;
    if (CommandData.power.table.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0080;
  }

  /* MCC power switching now uses ACS1 D1 grp 6 = latch1 high byte
   * and grp 2 = switch_grp2 low byte
   * and bits 6 & 7 of grp SIP/Spare = switch_misc */

  if (CommandData.power.mcc1.set_count > 0) {
    CommandData.power.mcc1.set_count--;
    if (CommandData.power.mcc1.set_count < LATCH_PULSE_LEN) latch1 |= 0x0100;
  }

  if (CommandData.power.mcc1.rst_count > 0) {
    CommandData.power.mcc1.rst_count--;
    if (CommandData.power.mcc1.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0200;
  }

  if (CommandData.power.mcc2.set_count > 0) {
    CommandData.power.mcc2.set_count--;
    if (CommandData.power.mcc2.set_count < LATCH_PULSE_LEN) latch1 |= 0x0400;
  }

  if (CommandData.power.mcc2.rst_count > 0) {
    CommandData.power.mcc2.rst_count--;
    if (CommandData.power.mcc2.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0800;
  }

  if (CommandData.power.mcc3.set_count > 0) {
    CommandData.power.mcc3.set_count--;
    if (CommandData.power.mcc3.set_count < LATCH_PULSE_LEN) latch1 |= 0x1000;
  }

  if (CommandData.power.mcc3.rst_count > 0) {
    CommandData.power.mcc3.rst_count--;
    if (CommandData.power.mcc3.rst_count < LATCH_PULSE_LEN) latch1 |= 0x2000;
  }

  if (CommandData.power.mcc4.set_count > 0) {
    CommandData.power.mcc4.set_count--;
    if (CommandData.power.mcc4.set_count < LATCH_PULSE_LEN) latch1 |= 0x4000;
  }

  if (CommandData.power.mcc4.rst_count > 0) {
    CommandData.power.mcc4.rst_count--;
    if (CommandData.power.mcc4.rst_count < LATCH_PULSE_LEN) latch1 |= 0x8000;
  }

  if (CommandData.power.mcc5.set_count > 0) {
    CommandData.power.mcc5.set_count--;
    if (CommandData.power.mcc5.set_count < LATCH_PULSE_LEN) grp2 |= 0x0010;
  }

  if (CommandData.power.mcc5.rst_count > 0) {
    CommandData.power.mcc5.rst_count--;
    if (CommandData.power.mcc5.rst_count < LATCH_PULSE_LEN) grp2 |= 0x0020;
  }

  if (CommandData.power.mcc6.set_count > 0) {
    CommandData.power.mcc6.set_count--;
    if (CommandData.power.mcc6.set_count < LATCH_PULSE_LEN) grp2 |= 0x0040;
  }

  if (CommandData.power.mcc6.rst_count > 0) {
    CommandData.power.mcc6.rst_count--;
    if (CommandData.power.mcc6.rst_count < LATCH_PULSE_LEN) grp2 |= 0x0080;
  }

  do_mce_power_op();

  if (CommandData.ifpower.mce[0].set_count > 0) {
    CommandData.ifpower.mce[0].set_count--;
    if (CommandData.ifpower.mce[0].set_count < LATCH_PULSE_LEN) ifpwr |= 0x0002;
  }
  if (CommandData.ifpower.mce[0].rst_count > 0) {
    CommandData.ifpower.mce[0].rst_count--;
    if (CommandData.ifpower.mce[0].rst_count < LATCH_PULSE_LEN) ifpwr |= 0x0001;
  }
  if (CommandData.ifpower.mce[1].set_count > 0) {
    CommandData.ifpower.mce[1].set_count--;
    if (CommandData.ifpower.mce[1].set_count < LATCH_PULSE_LEN) ifpwr |= 0x0008;
  }
  if (CommandData.ifpower.mce[1].rst_count > 0) {
    CommandData.ifpower.mce[1].rst_count--;
    if (CommandData.ifpower.mce[1].rst_count < LATCH_PULSE_LEN) ifpwr |= 0x0004;
  }
  if (CommandData.ifpower.mce[2].set_count > 0) {
    CommandData.ifpower.mce[2].set_count--;
    if (CommandData.ifpower.mce[2].set_count < LATCH_PULSE_LEN) ifpwr |= 0x0020;
  }
  if (CommandData.ifpower.mce[2].rst_count > 0) {
    CommandData.ifpower.mce[2].rst_count--;
    if (CommandData.ifpower.mce[2].rst_count < LATCH_PULSE_LEN) ifpwr |= 0x0010;
  }
  if (CommandData.ifpower.hwp.set_count > 0) {
    CommandData.ifpower.hwp.set_count--;
    if (CommandData.ifpower.hwp.set_count < LATCH_PULSE_LEN) ifpwr |= 0x0040;
  }
  if (CommandData.ifpower.hwp.rst_count > 0) {
    CommandData.ifpower.hwp.rst_count--;
    if (CommandData.ifpower.hwp.rst_count < LATCH_PULSE_LEN) ifpwr |= 0x0080;
  }
  if (CommandData.ifpower.sftv.set_count > 0) {
    CommandData.ifpower.sftv.set_count--;
    if (CommandData.ifpower.sftv.set_count < LATCH_PULSE_LEN) ifpwr |= 0x0100;
  }
  if (CommandData.ifpower.sftv.rst_count > 0) {
    CommandData.ifpower.sftv.rst_count--;
    if (CommandData.ifpower.sftv.rst_count < LATCH_PULSE_LEN) ifpwr |= 0x0200;
  }
  if (CommandData.ifpower.hk_preamp_off) {
    if (CommandData.ifpower.hk_preamp_off > 0)
      CommandData.ifpower.hk_preamp_off--;
    ifpwr &= ~0x4000;
  } else
    ifpwr |= 0x4000;

  WriteData(latchingAddr[0], latch0, NIOS_QUEUE);
  WriteData(latchingAddr[1], latch1, NIOS_QUEUE);
  WriteData(ifPwrAddr, ifpwr, NIOS_QUEUE);
  WriteData(mcePowerAddr, CommandData.mce_power, NIOS_QUEUE);
  WriteData(switchGyAddr, gybox, NIOS_QUEUE);
  WriteData(switchMiscAddr, misc, NIOS_QUEUE);
  WriteData(switchPvAddr, pv, NIOS_QUEUE);
  WriteData(switchGrp2Addr, grp2, NIOS_QUEUE);
}

//limit switch positions. NB: active low
#define LIM_CL_P  0x1
#define LIM_OP_P  0x2
#define LIM_CL_S  0x4
#define LIM_OP_S  0x8

//command bits to insert or retract locks (always to both at once)
#define LOCK_INSERT   0x5
#define LOCK_RETRACT  0xa

void LockMotor()
{
  static struct NiosStruct* controlLockAddr;
  static struct NiosStruct* stateLockAddr;
  static struct BiPhaseStruct* limitLockAddr;
  static int firsttime =1;
  unsigned int limits;
  int lock_bits = 0;

  if (firsttime) {
    firsttime = 0;
    controlLockAddr = GetNiosAddr("control_lock");
    stateLockAddr = GetNiosAddr("state_lock");
    limitLockAddr = GetBiPhaseAddr("limit_lock");
  }

  //set the control bits as per mode/goal
  switch (CommandData.lock.goal) {
    case lock_insert:
      lock_bits = LOCK_INSERT;
      break;
    case lock_retract:
      lock_bits = LOCK_RETRACT;
      break;
    case lock_el_wait_insert:
      if (fabs(ACSData.enc_mean_el - LOCK_POSITION) <= 0.05)
        lock_bits = LOCK_INSERT;
      else lock_bits = 0x0;
      break;
    case lock_do_nothing:
    default:
      lock_bits = 0x0;
      break;
  }

  WriteData(controlLockAddr, lock_bits, NIOS_QUEUE);

  //examine limit switches. Do nothing when lock is off (switches unpowered)
  //also ignore case when both switches asserted (disconnect/aphysical)
  //NB: limit switches are asserted low
  if (!CommandData.power.lock_off) {
    limits = ReadData(limitLockAddr);
    if ( !(limits & LIM_CL_P) && (limits & LIM_OP_P) )
      CommandData.lock.state_p = lock_closed;
    else if ( (limits & LIM_CL_P) && !(limits & LIM_OP_P) )
      CommandData.lock.state_p = lock_open;
    else if ( (limits & LIM_CL_P) && (limits & LIM_OP_P) ) {
      if (lock_bits == LOCK_INSERT)
        CommandData.lock.state_p = lock_closing;
      else if (lock_bits == LOCK_RETRACT)
        CommandData.lock.state_p = lock_opening;
    }
    if ( !(limits & LIM_CL_S) && (limits & LIM_OP_S) )
      CommandData.lock.state_s = lock_closed;
    else if ( (limits & LIM_CL_S) && !(limits & LIM_OP_S) )
      CommandData.lock.state_s = lock_open;
    else if ( (limits & LIM_CL_S) && (limits & LIM_OP_S) ) {
      if (lock_bits == LOCK_INSERT)
        CommandData.lock.state_s = lock_closing;
      else if (lock_bits == LOCK_RETRACT)
        CommandData.lock.state_s = lock_opening;
    }
  }

  //compress state to a single bit (pin_is_in) for use elsewhere
  //if either side is closed or closing, consider closed
  //otherwise consider open, unless state is unknown (then don't change)
  if (CommandData.lock.state_p == lock_closed
      || CommandData.lock.state_p == lock_closing
      || CommandData.lock.state_s == lock_closed
      || CommandData.lock.state_s == lock_closing)
    CommandData.lock.pin_is_in = 1;
  else if (CommandData.lock.state_p != lock_unknown
      && CommandData.lock.state_s != lock_unknown)
    CommandData.lock.pin_is_in = 0;

  WriteData(stateLockAddr, (CommandData.lock.pin_is_in & 0x0001)
      | ((CommandData.lock.goal & 0x000f) << 1)
      | ((CommandData.lock.state_p & 0x000f) << 5)
      | ((CommandData.lock.state_s & 0x000f) << 9)
      , NIOS_QUEUE);
}

