/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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

#include <math.h>
#include <time.h>
#include <stdio.h>

#include "mcp.h"
#include "channels.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define ISC_ACK_TIMEOUT 5000  /* in 100Hz frames */
#define ISC_TRIG_PERIOD 100   /* in 100Hz frames */
#define MAX_ISC_SLOW_PULSE_SPEED 0.015

struct ISCPulseType isc_pulses[2] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};

int pin_is_in = 1;

/* Semaphores for hanshaking with the ISC/OSC threads (isc.c) */
extern short int write_ISC_pointing[2];
extern short int write_ISC_trigger[2];

/* ACS0 digital signals (G1 and G3 output, G2 input) */
#define BAL1_ON      0x04  /* ACS3 Group 1 Bit 3 - ifpmBits */
#define BAL1_REV     0x08  /* ACS3 Group 1 Bit 4 */
#define IF_COOL1_OFF 0x10  /* ACS3 Group 1 Bit 5 */
#define IF_COOL1_ON  0x20  /* ACS3 Group 1 Bit 6 */
#define BAL2_ON      0x40  /* ACS3 Group 1 Bit 7 */
#define BAL2_REV     0x80  /* ACS3 Group 1 Bit 8 */

#define LOKMOT_ISIN  0x40  /* ACS3 Group 2 Bit 7 - lockBits */
#define LOKMOT_ISOUT 0x80  /* ACS3 Group 2 Bit 8 */

#define OF_COOL2_ON  0x01  /* ACS3 Group 3 Bit 1 - ofpmBits */
#define OF_COOL2_OFF 0x02  /* ACS3 Group 3 Bit 2 */
#define OF_COOL1_ON  0x04  /* ACS3 Group 3 Bit 3 */
#define OF_COOL1_OFF 0x08  /* ACS3 Group 3 Bit 4 */
#define LOKMOT_ON    0x10  /* ACS3 Group 3 Bit 5 */
#define LOKMOT_OFF   0x20  /* ACS3 Group 3 Bit 6 */
#define LOKMOT_OUT   0x40  /* ACS3 Group 3 Bit 7 */
#define LOKMOT_IN    0x80  /* ACS3 Group 3 Bit 8 */

#define BAL_OFF_VETO  1000            /* # of frames to veto balance system
                                         after turning off pump */

/* in commands.c */
double LockPosition(double elevation); 

/****************************************************************/
/*                                                              */
/* Read the state of the lock motor pin (or guess it, whatever) */
/*                                                              */
/****************************************************************/
int pinIsIn(void) {
  return(pin_is_in);
}

/************************************************************************/
/*                                                                      */
/*    ControlGyroHeat:  Controls gyro box temp by turning heater bit in */
/*    ACS1 on and off.  Also calculates gyro offsets.                   */
/*                                                                      */
/************************************************************************/
void ControlGyroHeat(unsigned short *RxFrame) {
  static struct BiPhaseStruct* tGyboxAddr;
  static struct NiosStruct *gyHeatAddr, *tGySetAddr, *pGyheatAddr, *iGyheatAddr;
  static struct NiosStruct *dGyheatAddr;
  static int firsttime = 1;

  int on = 0x40, off = 0x00;
  static int p_on = 0;
  static int p_off = -1;

  float error = 0, set_point;
  static float integral = 0;
  static float deriv = 0;
  static float error_last = 0;
  float P, I, D;

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    tGyboxAddr = GetBiPhaseAddr("t_gybox");
    gyHeatAddr = GetNiosAddr("gy_heat");

    tGySetAddr = GetNiosAddr("t_gy_set");
    pGyheatAddr = GetNiosAddr("g_p_gyheat");
    iGyheatAddr = GetNiosAddr("g_i_gyheat");
    dGyheatAddr = GetNiosAddr("g_d_gyheat");
  }

  /* send down the setpoints and gains values */
  WriteData(tGySetAddr,
      (unsigned short)(CommandData.t_gybox_setpoint * 32768.0 / 100.0), 0);

  WriteData(pGyheatAddr, CommandData.gy_heat_gain.P, NIOS_QUEUE);
  WriteData(iGyheatAddr, CommandData.gy_heat_gain.I, NIOS_QUEUE);
  WriteData(dGyheatAddr, CommandData.gy_heat_gain.D, NIOS_QUEUE);

  /* control the heat */
  set_point = (CommandData.t_gybox_setpoint - 136.45) / (-9.5367431641e-08);
  P = CommandData.gy_heat_gain.P * (-1.0 / 1000000.0);
  I = CommandData.gy_heat_gain.I * (-1.0 / 110000.0);
  D = CommandData.gy_heat_gain.D * ( 1.0 / 1000.0);

  /********* if end of pulse, calculate next pulse *********/
  if (p_off < 0) {
    error = set_point -
      ((unsigned int)(RxFrame[tGyboxAddr->channel + 1] << 16
                      | RxFrame[tGyboxAddr->channel]));

    integral = integral * 0.9975 + 0.0025 * error;
    if (integral * I > 60){
      integral = 60.0 / I;
    }
    if (integral * I < 0){
      integral = 0;
    }

    deriv = error_last - error;
    error_last = error;

    p_on = P * error + (deriv / 60.0) * D + integral * I;

    if (p_on > 60)
      p_on = 60;
    if (p_on < 0)
      p_on = 0;
    p_off = 60 - p_on;

  }

  /******** do the pulse *****/
  if (p_on > 0) {
    WriteData(gyHeatAddr, on, NIOS_FLUSH);
    p_on--;
  } else {
    WriteData(gyHeatAddr, off, NIOS_FLUSH);
    p_off--;
  }
}

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
int Balance(int ifpmBits) {
  static struct BiPhaseStruct *iElAddr;
  static struct NiosStruct *balPwm1Addr;
  static int pumpon = 0;
  int pumppwm;
  int error;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    iElAddr = GetBiPhaseAddr("i_el");
    balPwm1Addr = GetNiosAddr("balpump_lev");
  }

  /* don't turn on pump if we're reading very small numbers */
  if (slow_data[iElAddr->index][iElAddr->channel] < 8000)
    error = 0;
  else
    error = slow_data[iElAddr->index][iElAddr->channel]
      - 32758 - CommandData.pumps.bal_target;

  if (error > 0) {
    ifpmBits |= BAL1_REV;  /* set reverse bit */
  } else {
    ifpmBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
    error = -error;
  }

  pumppwm = CommandData.pumps.bal_min - error * CommandData.pumps.bal_gain;

  if (pumppwm < CommandData.pumps.bal_max) {
    pumppwm = CommandData.pumps.bal_max;
  } else if (pumppwm > 2047) {
    pumppwm = 2047;
  }

  if (error > CommandData.pumps.bal_on) {
    pumpon = 1;
  } else if (error < CommandData.pumps.bal_off) {
    pumpon = 0;
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_OFF_VETO;
  }

  if (pumpon) {
    ifpmBits |= BAL1_ON; /* turn on pump */
  } else {
    ifpmBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

  WriteData(balPwm1Addr, pumppwm, NIOS_QUEUE);

  return ifpmBits;
}

/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
#define PULSE_LENGTH 400   /* 4 seconds */
#define SEARCH_COUNTS 500 /* 5 seconds */
int GetLockBits(unsigned short lockBits) {
  static int is_closing = 0;
  static int is_opening = 0;
  static int is_searching = 0;

  /* check for commands from CommandData */
  if (CommandData.pumps.lock_in) {
    CommandData.pumps.lock_in = 0;
    is_opening = 0;
    is_closing = PULSE_LENGTH;
    is_searching = 0;
  } else if (CommandData.pumps.lock_out) {
    CommandData.pumps.lock_out = 0;
    is_opening = PULSE_LENGTH;
    is_closing = 0;
    is_searching = 0;
  } else if (CommandData.pumps.lock_point) {
    CommandData.pumps.lock_point = 0;
    is_searching = SEARCH_COUNTS;
    is_opening = 0;
    is_closing = 0;
  } else if (CommandData.pumps.lock_off) {
    CommandData.pumps.lock_off = 0;
    is_opening = 0;
    is_closing = 0;
    is_searching = 0;
  }

  /* elevation searching stuff */
  if (is_searching > 1) {
    if (fabs(ACSData.enc_elev - LockPosition(ACSData.enc_elev)) > 0.2)
      is_searching = SEARCH_COUNTS;
    else
      is_searching--;
  } else if (is_searching == 1) {
    is_searching = 0;
    is_closing = PULSE_LENGTH;
  }

  /* check limit switches -- if both bits are set, the motor is running
   * -- if we have reached a limit we can stop going in the direction
   * of the limitswitch that is active */
  if ((lockBits & LOKMOT_ISIN) && (~lockBits & LOKMOT_ISOUT)) {
    pin_is_in = 1;
    is_closing = 0;
  } else if ((lockBits & LOKMOT_ISOUT) && (~lockBits & LOKMOT_ISIN)) {
    pin_is_in = 0;
    is_opening = 0;
  }
  
  /* set lock bits */
  if (is_closing) {
    is_closing--;
    return(LOKMOT_IN | LOKMOT_ON);
  } else if (is_opening) {
    is_opening--;
    return(LOKMOT_OUT | LOKMOT_ON);
  }


  return 0;
}

/*********************/
/* ISC Pulsing stuff */
void CameraTrigger(int which)
{
  static int firsttime = 1;
  static struct NiosStruct* TriggerAddr[2];
  int iscPulse = 0;

  if (firsttime) {
    firsttime = 0;
    TriggerAddr[0] = GetNiosAddr("isc_trigger");
    TriggerAddr[1] = GetNiosAddr("osc_trigger");
  }

  if (isc_pulses[which].age >= 0)
    isc_pulses[which].age++;
  
  if (isc_pulses[which].ctr < ISC_TRIG_PERIOD) {
    isc_pulses[which].ctr++;
  } else {
    write_ISC_pointing[which] = 1;	/* no blastbus latency any more */
    isc_pulses[which].pulse_index = (isc_pulses[which].pulse_index + 1) % 4;
    isc_pulses[which].ctr = 0;

    if (isc_pulses[which].is_fast) {
      iscPulse = (isc_pulses[which].pulse_index << 14)
        | CommandData.ISCControl[which].fast_pulse_width;
      CommandData.ISCState[which].exposure
        = CommandData.ISCControl[which].fast_pulse_width / 10416.6666666667;
    } else if (fabs(axes_mode.az_vel) < MAX_ISC_SLOW_PULSE_SPEED) {
      iscPulse = (isc_pulses[which].pulse_index << 14)
        | CommandData.ISCControl[which].pulse_width;

      /* Trigger automatic image write-to-disk */
      if (isc_pulses[which].last_save
          >= CommandData.ISCControl[which].save_period &&
          CommandData.ISCControl[which].save_period > 0) {
        CommandData.ISCControl[which].auto_save = 1;
        isc_pulses[which].last_save = 0;
      }
      CommandData.ISCState[which].exposure
        = CommandData.ISCControl[which].pulse_width / 10416.6666666667;
    }

    if (isc_pulses[which].age < 0)
      isc_pulses[which].age = 0;

    WriteData(TriggerAddr[which], iscPulse, NIOS_FLUSH);
  }

  isc_pulses[which].last_save++;
}

/*****************************************************************/
/*                                                               */
/*   Control the pumps and the lock                              */
/*                                                               */
/*****************************************************************/
void ControlAuxMotors(unsigned short *RxFrame) {
  static struct NiosStruct* ofpmBitsAddr;
  static struct NiosStruct* balpumpLevAddr;
  static struct NiosStruct* sprpumpLevAddr;
  static struct NiosStruct* inpumpLevAddr;
  static struct NiosStruct* outpumpLevAddr;
  static struct NiosStruct* balOnAddr, *balOffAddr;
  static struct NiosStruct* balTargetAddr, *balVetoAddr;
  static struct NiosStruct* balGainAddr, *balMinAddr, *balMaxAddr;
  static struct NiosStruct* ifpmBitsAddr;
  static struct NiosStruct* lokmotPinAddr;

  static struct BiPhaseStruct* lockBitsAddr;

  int ifpmBits = 0;
  int ofpmBits = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    lockBitsAddr = GetBiPhaseAddr("lock_bits");

    ifpmBitsAddr = GetNiosAddr("ifpm_bits");
    ofpmBitsAddr = GetNiosAddr("ofpm_bits");
    balpumpLevAddr = GetNiosAddr("balpump_lev");
    sprpumpLevAddr = GetNiosAddr("sprpump_lev");
    inpumpLevAddr = GetNiosAddr("inpump_lev");
    outpumpLevAddr = GetNiosAddr("outpump_lev");
    balOnAddr = GetNiosAddr("bal_on");
    balOffAddr = GetNiosAddr("bal_off");
    balTargetAddr = GetNiosAddr("bal_target");
    balGainAddr = GetNiosAddr("bal_gain");
    balMinAddr = GetNiosAddr("bal_min");
    balMaxAddr = GetNiosAddr("bal_max");
    balVetoAddr = GetNiosAddr("bal_veto");
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
  }

  /* inner frame box */
  /* two latching pumps 3/4 */
  /* two non latching: on/off, fwd/rev */
  if (CommandData.pumps.bal_veto) {
    if (CommandData.pumps.bal1_on)
      ifpmBits |= BAL1_ON;
    if (CommandData.pumps.bal1_reverse)
      ifpmBits |= BAL1_REV;
    if (CommandData.pumps.bal2_on)
      ifpmBits |= BAL2_ON;
    if (CommandData.pumps.bal2_reverse)
      ifpmBits |= BAL2_REV;
  }

  /* two latching pumps: */
  if (CommandData.pumps.inframe_cool1_on > 0) {
    ifpmBits |= IF_COOL1_ON;
    CommandData.pumps.inframe_cool1_on--;
  } else if (CommandData.pumps.inframe_cool1_off > 0) {
    ifpmBits |= IF_COOL1_OFF;
    CommandData.pumps.inframe_cool1_off--;
  }

  /* outer frame box */
  /* three on, off motors (pulses) */
  if (CommandData.pumps.outframe_cool1_on > 0) {
    ofpmBits |= OF_COOL1_ON;
    CommandData.pumps.outframe_cool1_on--;
  } else if (CommandData.pumps.outframe_cool1_off > 0) {
    ofpmBits |= OF_COOL1_OFF;
    CommandData.pumps.outframe_cool1_off--;
  }
  if (CommandData.pumps.outframe_cool2_on > 0) {
    ofpmBits |= OF_COOL2_ON;
    CommandData.pumps.outframe_cool2_on--;
  } else if (CommandData.pumps.outframe_cool2_off > 0) {
    ofpmBits |= OF_COOL2_OFF;
    CommandData.pumps.outframe_cool2_off--;
  }

  ofpmBits |=
    GetLockBits(slow_data[lockBitsAddr->index][lockBitsAddr->channel]);

  if (CommandData.pumps.bal_veto) {
    /* if we're in timeout mode, decrement the timer */
    if (CommandData.pumps.bal_veto != -1)
      CommandData.pumps.bal_veto--;

    WriteData(balpumpLevAddr, CommandData.pumps.pwm1 & 0x7ff, NIOS_QUEUE);
  } else {
    ifpmBits = Balance(ifpmBits);
  }

  WriteData(lokmotPinAddr, pin_is_in, NIOS_QUEUE);
  WriteData(ofpmBitsAddr, ofpmBits, NIOS_QUEUE);
  WriteData(sprpumpLevAddr, CommandData.pumps.pwm2 & 0x7ff, NIOS_QUEUE);
  WriteData(inpumpLevAddr, CommandData.pumps.pwm3 & 0x7ff, NIOS_QUEUE);
  WriteData(outpumpLevAddr, CommandData.pumps.pwm4 & 0x7ff, NIOS_QUEUE);
  WriteData(balOnAddr, (int)CommandData.pumps.bal_on, NIOS_QUEUE);
  WriteData(balOffAddr, (int)CommandData.pumps.bal_off, NIOS_QUEUE);
  WriteData(balVetoAddr, (int)CommandData.pumps.bal_veto, NIOS_QUEUE);
  WriteData(balTargetAddr, (int)(CommandData.pumps.bal_target + 1648. * 5.),
      NIOS_QUEUE);
  WriteData(balGainAddr, (int)(CommandData.pumps.bal_gain * 1000.), NIOS_QUEUE);
  WriteData(balMinAddr, (int)CommandData.pumps.bal_min, NIOS_QUEUE);
  WriteData(balMaxAddr,(int)CommandData.pumps.bal_max, NIOS_QUEUE);
  WriteData(ifpmBitsAddr, ifpmBits, NIOS_FLUSH);
}

/* SensorResets: Power cycle ISC, OSC, GPS and GYBOX2 */
void SensorResets(void)
{
  static int firsttime = 1;
  static struct NiosStruct* sensorResetAddr;
  int sensor_resets = 0;
  
  if (firsttime) {
    firsttime = 0;
    sensorResetAddr = GetNiosAddr("sensor_reset");
  }

  WriteData(sensorResetAddr, sensor_resets, NIOS_QUEUE);
}
