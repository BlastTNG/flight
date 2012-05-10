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
#define MIN_GYBOX_TEMP ((223.15 / M_16T) - B_16T)   /* -50 C */
#define MAX_GYBOX_TEMP ((333.15 / M_16T) - B_16T)   /* +60 C */
#define MIN_BSC_TEMP  ((223.15 / M_16T) - B_16T)   /* -50 C */
 
/* Gybox heater stuff */
#define GY_HEAT_MAX 40 /* percent */
#define GY_HEAT_MIN 5  /* percent */
#define GY_TEMP_MAX 50 /* setpoint maximum -- deg C */
#define GY_TEMP_MIN 0  /* setpoint minimum -- deg C */
#define GY_TEMP_STEP 1 /* setpoint step - deg C */
#define GY_HEAT_TC 30000 /* integral/age characteristic time in 100Hz Frames */

extern short int InCharge; /* tx.c */

/* ACS2 digital signals */
#define BSC_HEAT    0x01  /* ACS1_D Spare-0 */

#define PUMP_MAX 26214      /*  3.97*2.0V   */
#define PUMP_MIN  3277      /*  3.97*0.25V   */

#define PUMP_ZERO 32773

/* in commands.c */
double LockPosition(double elevation);


/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temp                          */
/************************************************************************/
void ControlGyroHeat()
{
  static struct BiPhaseStruct* tGyAddr;
  static struct NiosStruct *heatGyAddr, *tSetGyAddr, *gPHeatGyAddr;
  static struct NiosStruct *gIHeatGyAddr;
  static struct NiosStruct *gDHeatGyAddr, *hHistGyAddr, *hAgeGyAddr;
  static int firsttime = 1;
  static double history = 0;

  static int p_on = 0;
  static int p_off = -1;

  float error = 0, set_point;
  unsigned int temp;
  static float integral = 0;
  static float deriv = 0;
  static float error_last = 0;
  float P, I, D;

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    tGyAddr = GetBiPhaseAddr("t_gy");
    heatGyAddr = GetNiosAddr("heat_gy");
    hHistGyAddr = GetNiosAddr("h_hist_gy");
    hAgeGyAddr = GetNiosAddr("h_age_gy");
    tSetGyAddr = GetNiosAddr("t_set_gy");

    gPHeatGyAddr = GetNiosAddr("g_p_heat_gy");
    gIHeatGyAddr = GetNiosAddr("g_i_heat_gy");
    gDHeatGyAddr = GetNiosAddr("g_d_heat_gy");
  }

  /* send down the setpoints and gains values */
  WriteData(tSetGyAddr, CommandData.gyheat.setpoint * 327.68, NIOS_QUEUE);
  WriteData(gPHeatGyAddr, CommandData.gyheat.gain.P, NIOS_QUEUE);
  WriteData(gIHeatGyAddr, CommandData.gyheat.gain.I, NIOS_QUEUE);
  WriteData(gDHeatGyAddr, CommandData.gyheat.gain.D, NIOS_QUEUE);

  temp = slow_data[tGyAddr->index][tGyAddr->channel];
  
  /* Only run these controls if we think the thermometer isn't broken */
  if (temp < MAX_GYBOX_TEMP && temp > MIN_GYBOX_TEMP) {
    set_point = ((CommandData.gyheat.setpoint + 273.15) / M_16T) - B_16T;
    P = CommandData.gyheat.gain.P * (1.0 / 10.0);
    I = CommandData.gyheat.gain.I * (1.0 / 110000.0);
    D = CommandData.gyheat.gain.D * ( 1.0 / 1000.0);
    //P = CommandData.gyheat.gain.P * (-1.0 / 1000000.0);
    //I = CommandData.gyheat.gain.I * (-1.0 / 110000.0);
    //D = CommandData.gyheat.gain.D * ( 1.0 / 1000.0);

    /********* if end of pulse, calculate next pulse *********/
    if (p_off <= 0 && p_on <= 0) {
      error = set_point - temp;

      integral = integral * 0.999 + 0.001 * error;
      if (integral * I > 60)
        integral = 60.0 / I;

      if (integral * I < 0)
        integral = 0;

      deriv = error_last - error;
      error_last = error;

      p_on = P * error + (deriv / 60.0) * D + integral * I;

      if (p_on > 60)
        p_on = 60;
      else if (p_on < 0)
        p_on = 0;

      p_off = 60 - p_on;

      history = p_on * 100. / GY_HEAT_TC + (1. - 60. / GY_HEAT_TC) * history;
    }

    if (CommandData.gyheat.age <= GY_HEAT_TC * 2)
      ++CommandData.gyheat.age;

    /******** do the pulse *****/
    if (p_on > 0) {
      WriteData(heatGyAddr, 0x1, NIOS_FLUSH);
      p_on--;
    } else if (p_off > 0) {
      WriteData(heatGyAddr, 0x0, NIOS_FLUSH);
      p_off--;
    }
  } else
    /* Turn off heater if thermometer appears broken */
    WriteData(heatGyAddr, 0x0, NIOS_FLUSH);

  WriteData(hAgeGyAddr, CommandData.gyheat.age, NIOS_QUEUE);
  WriteData(hHistGyAddr, (history * 32768. / 100.), NIOS_QUEUE);
}

/************************************************************************/
/*    ControlBSCHeat:  Controls BSC temp		                */
/************************************************************************/
/*
static int ControlBSCHeat()
{

  static struct BiPhaseStruct *tBSCAddr;
  static int firsttime = 1;

  unsigned int temp;

  if (firsttime) {
    firsttime = 0;
    tBSCAddr = GetBiPhaseAddr("t_bsc");
  }

  temp = slow_data[tBSCAddr->index][tBSCAddr->channel]; 

  if (temp > MIN_BSC_TEMP) {
      if (temp < CommandData.t_set_bsc) {
	return 0x1;	
      } else {
	return 0x0;
      }
  } else {
    // Turn off heater if thermometer appears broken 
    return 0x0;
  }
}
*/
void ChargeController(void)
{

  static struct NiosStruct *VBattCCAddr;
  static struct NiosStruct *VArrCCAddr;
  static struct NiosStruct *IBattCCAddr;
  static struct NiosStruct *IArrCCAddr;
  static struct NiosStruct *VTargCCAddr;
  static struct NiosStruct *ThsCCAddr;
  static struct NiosStruct *FaultCCAddr;
  static struct NiosStruct *AlarmHiCCAddr;
  static struct NiosStruct *AlarmLoCCAddr;
  static struct NiosStruct *ChargeCCAddr;
  static struct NiosStruct *LEDCCAddr;

  static int firsttime = 1;
  
  if (firsttime) {

    firsttime = 0;

    VBattCCAddr = GetNiosAddr("v_batt_cc");
    VArrCCAddr = GetNiosAddr("v_arr_cc");
    IBattCCAddr = GetNiosAddr("i_batt_cc");
    IArrCCAddr  = GetNiosAddr("i_arr_cc");
    VTargCCAddr = GetNiosAddr("v_targ_cc");
    ThsCCAddr = GetNiosAddr("t_hs_cc");
    FaultCCAddr = GetNiosAddr("fault_cc");
    AlarmHiCCAddr = GetNiosAddr("alarm_hi_cc");
    AlarmLoCCAddr = GetNiosAddr("alarm_lo_cc");
    ChargeCCAddr = GetNiosAddr("state_cc");
    LEDCCAddr = GetNiosAddr("led_cc");
  }

  WriteData(VBattCCAddr, 180.0*ChrgCtrlData.V_batt + 32400.0, NIOS_QUEUE); 
  WriteData(VArrCCAddr, 180.0*ChrgCtrlData.V_arr + 32400.0, NIOS_QUEUE);
  WriteData(IBattCCAddr, 400.0*ChrgCtrlData.I_batt + 32000.0, NIOS_QUEUE);
  WriteData(IArrCCAddr,  400.0*ChrgCtrlData.I_arr + 32000.0, NIOS_QUEUE);
  WriteData(VTargCCAddr, 180.0*ChrgCtrlData.V_targ + 32400.0, NIOS_QUEUE);
  WriteData(ThsCCAddr, ChrgCtrlData.T_hs, NIOS_QUEUE);
  WriteData(FaultCCAddr, ChrgCtrlData.fault_field, NIOS_QUEUE);
  WriteData(AlarmHiCCAddr, ChrgCtrlData.alarm_field_hi, NIOS_QUEUE);
  WriteData(AlarmLoCCAddr, ChrgCtrlData.alarm_field_lo, NIOS_QUEUE);
  WriteData(ChargeCCAddr, ChrgCtrlData.charge_state, NIOS_QUEUE);
  WriteData(LEDCCAddr, ChrgCtrlData.led_state, NIOS_QUEUE);

}

/* create latching relay pulses, and update enable/disbale levels */
/* actbus/steppers enable is handled separately in StoreActBus() */
void ControlPower(void) {

  static int firsttime = 1;
  static struct NiosStruct* latchingAddr[2];
  static struct NiosStruct* switchGyAddr;
  static struct NiosStruct* switchMiscAddr;
  int latch0 = 0, latch1 = 0, gybox = 0, misc = 0;
  int i;

  if (firsttime) {
    firsttime = 0;
    latchingAddr[0] = GetNiosAddr("latch0");
    latchingAddr[1] = GetNiosAddr("latch1");
    switchGyAddr = GetNiosAddr("switch_gy");
    switchMiscAddr = GetNiosAddr("switch_misc");
  }

  if (CommandData.power.theugly_cam_off) {
    if (CommandData.power.theugly_cam_off > 0) CommandData.power.theugly_cam_off--;
    misc |= 0x02;
  }

  if (CommandData.power.theugly_cpu_off) {
    if (CommandData.power.theugly_cpu_off > 0) CommandData.power.theugly_cpu_off--;
    misc |= 0x04;
  }

  //TODO make the HWP bus switchable. used to be handled in actuators.c

  if (CommandData.power.hub232_off) {
    if (CommandData.power.hub232_off > 0) CommandData.power.hub232_off--;
    misc |= 0x08;
  }

  //NB: the bit for lock power is asserted with the motors are ON
  if (CommandData.power.lock_off) {
    if (CommandData.power.lock_off > 0) CommandData.power.lock_off--;
    misc &= ~0x20;
  } else misc |= 0x20;

  if (CommandData.power.charge.set_count > 0) {
    CommandData.power.charge.set_count--;
    if (CommandData.power.charge.set_count < LATCH_PULSE_LEN) misc |= 0x0040;
  }
  if (CommandData.power.charge.rst_count > 0) {
    CommandData.power.charge.rst_count--;
    if (CommandData.power.charge.rst_count < LATCH_PULSE_LEN) misc |= 0x0080;
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
  if (CommandData.power.isc.set_count > 0) {
    CommandData.power.isc.set_count--;
    if (CommandData.power.isc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0010;
  }
  if (CommandData.power.isc.rst_count > 0) {
    CommandData.power.isc.rst_count--;
    if (CommandData.power.isc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0020;
  }
  if (CommandData.power.osc.set_count > 0) {
    CommandData.power.osc.set_count--;
    if (CommandData.power.osc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0040;
  }
  if (CommandData.power.osc.rst_count > 0) {
    CommandData.power.osc.rst_count--;
    if (CommandData.power.osc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0080;
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
  if (CommandData.power.rx_main.set_count > 0) {
    CommandData.power.rx_main.set_count--;
    if (CommandData.power.rx_main.set_count < LATCH_PULSE_LEN) latch1 |= 0x0004;
  }
  if (CommandData.power.rx_main.rst_count > 0) {
    CommandData.power.rx_main.rst_count--;
    if (CommandData.power.rx_main.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0008;
  }
  if (CommandData.power.rx_hk.set_count > 0) {
    CommandData.power.rx_hk.set_count--;
    if (CommandData.power.rx_hk.set_count < LATCH_PULSE_LEN) latch1 |= 0x5050;
  }
  if (CommandData.power.rx_hk.rst_count > 0) {
    CommandData.power.rx_hk.rst_count--;
    if (CommandData.power.rx_hk.rst_count < LATCH_PULSE_LEN) latch1 |= 0xa0a0;
  }
  if (CommandData.power.rx_amps.set_count > 0) {
    CommandData.power.rx_amps.set_count--;
    if (CommandData.power.rx_amps.set_count < LATCH_PULSE_LEN) latch1 |= 0x0500;
  }
  if (CommandData.power.rx_amps.rst_count > 0) {
    CommandData.power.rx_amps.rst_count--;
    if (CommandData.power.rx_amps.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0a00;
  }

//  misc |= ControlBSCHeat();

  WriteData(latchingAddr[0], latch0, NIOS_QUEUE);
  WriteData(latchingAddr[1], latch1, NIOS_QUEUE);
  WriteData(switchGyAddr, gybox, NIOS_QUEUE);
  WriteData(switchMiscAddr, misc, NIOS_QUEUE);
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
      if (fabs(ACSData.enc_mean_el - LOCK_POSITION) <= 0.5)
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

