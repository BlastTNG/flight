/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2005 University of Toronto
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
#define MAX_ISC_SLOW_PULSE_SPEED 0.015

/* limits for the gyrobox thermometer.  If the reading is outside this range,
 * we don't regulate the box at all, since it means the thermometer is probably
 * broken */
#define MIN_GYBOX_TEMP ((-50 - TGYBOX_B) / TGYBOX_M)  /* -50 C */
#define MAX_GYBOX_TEMP ((60 - TGYBOX_B) / TGYBOX_M)   /* +60 C */

struct ISCPulseType isc_pulses[2] = {
  {-1, 0, 0, 0, 0, 0, 0, 0}, {-1, 0, 0, 0, 0, 0, 0, 0}
};

/* Semaphores for handshaking with the ISC/OSC threads (isc.c) */
extern short int write_ISC_pointing[2];
extern short int write_ISC_trigger[2];
extern short int ISC_link_ok[2];
extern short int start_ISC_cycle[2];

extern short int InCharge; /* tx.c */

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
/* Read the state of the lock motor pin (or guess it, whatever) */
/****************************************************************/
int pinIsIn(void)
{
  return(CommandData.pin_is_in);
}

int SetGyHeatSetpoint(double history, int age)
{
  double setpoint = CommandData.gyheat.setpoint;

  if (age < CommandData.gyheat.tc * 2)
    return age;

  if (history < CommandData.gyheat.min_heat)
    setpoint += CommandData.gyheat.step;
  else if (history > CommandData.gyheat.max_heat)
    setpoint -= CommandData.gyheat.step;

  if (setpoint < CommandData.gyheat.min_set)
    setpoint = CommandData.gyheat.min_set;
  else if (setpoint > CommandData.gyheat.max_set)
    setpoint = CommandData.gyheat.max_set;

  if (setpoint != CommandData.gyheat.setpoint) {
    age = 0;
    CommandData.gyheat.setpoint = setpoint;
  }

  return age;
}

/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temp by turning heater bit in */
/*    ACS1 on and off.  Also calculates gyro offsets.                   */
/************************************************************************/
void ControlGyroHeat(unsigned short *RxFrame)
{
  static struct BiPhaseStruct* tGyboxAddr;
  static struct NiosStruct *gyHeatAddr, *tGySetAddr, *pGyheatAddr, *iGyheatAddr;
  static struct NiosStruct *dGyheatAddr, *tGyMinAddr, *tGyMaxAddr, *gyHHistAddr;
  static struct NiosStruct *gyHAgeAddr, *gyHMinAddr, *gyHMaxAddr, *gyHTcAddr;
  static struct NiosStruct *tGyStepAddr;
  static int firsttime = 1;
  static double history = 0;

  int on = 0x40, off = 0x00;
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
    tGyboxAddr = GetBiPhaseAddr("t_gybox");

    gyHeatAddr = GetNiosAddr("gy_heat");
    gyHMinAddr = GetNiosAddr("gy_h_min");
    gyHMaxAddr = GetNiosAddr("gy_h_max");
    gyHTcAddr = GetNiosAddr("gy_h_tc");
    gyHHistAddr = GetNiosAddr("gy_h_hist");
    gyHAgeAddr = GetNiosAddr("gy_h_age");
    tGyStepAddr = GetNiosAddr("t_gy_step");
    tGySetAddr = GetNiosAddr("t_gy_set");
    tGyMinAddr = GetNiosAddr("t_gy_min");
    tGyMaxAddr = GetNiosAddr("t_gy_max");

    pGyheatAddr = GetNiosAddr("g_p_gyheat");
    iGyheatAddr = GetNiosAddr("g_i_gyheat");
    dGyheatAddr = GetNiosAddr("g_d_gyheat");
  }

  /* send down the setpoints and gains values */
  WriteData(tGySetAddr, CommandData.gyheat.setpoint * 327.68, NIOS_QUEUE);
  WriteData(tGyMinAddr, CommandData.gyheat.min_set * 327.68, NIOS_QUEUE);
  WriteData(tGyMaxAddr, CommandData.gyheat.max_set * 327.68, NIOS_QUEUE);
  WriteData(tGyStepAddr, CommandData.gyheat.step * 3276.8, NIOS_QUEUE);

  WriteData(gyHMinAddr, CommandData.gyheat.min_heat * 327.68, NIOS_QUEUE);
  WriteData(gyHMaxAddr, CommandData.gyheat.max_heat * 327.68, NIOS_QUEUE);
  WriteData(gyHTcAddr, CommandData.gyheat.tc, NIOS_QUEUE);

  WriteData(pGyheatAddr, CommandData.gyheat.gain.P, NIOS_QUEUE);
  WriteData(iGyheatAddr, CommandData.gyheat.gain.I, NIOS_QUEUE);
  WriteData(dGyheatAddr, CommandData.gyheat.gain.D, NIOS_QUEUE);

  temp = (RxFrame[tGyboxAddr->channel + 1] << 16 |
      RxFrame[tGyboxAddr->channel]);

  /* Only run these controls if we think the thermometer isn't broken */
  if (temp < MAX_GYBOX_TEMP && temp > MIN_GYBOX_TEMP) {
    /* control the heat */
    CommandData.gyheat.age = SetGyHeatSetpoint(history, CommandData.gyheat.age);

    set_point = (CommandData.gyheat.setpoint - TGYBOX_B) / TGYBOX_M;
    P = CommandData.gyheat.gain.P * (-1.0 / 1000000.0);
    I = CommandData.gyheat.gain.I * (-1.0 / 110000.0);
    D = CommandData.gyheat.gain.D * ( 1.0 / 1000.0);

    /********* if end of pulse, calculate next pulse *********/
    if (p_off <= 0 && p_on <= 0) {
      error = set_point - temp;

      integral = integral * 0.999 + 0.001 * error;
      if (integral * I > 60) {
        integral = 60.0 / I;
      }
      if (integral * I < 0) {
        integral = 0;
      }

      deriv = error_last - error;
      error_last = error;

      p_on = P * error + (deriv / 60.0) * D + integral * I;

      if (p_on > 60)
        p_on = 60;
      else if (p_on < 0)
        p_on = 0;

      p_off = 60 - p_on;

      history = p_on * 100. / CommandData.gyheat.tc + (1. - 60. /
          CommandData.gyheat.tc) * history;
    }

    if (CommandData.gyheat.age <= CommandData.gyheat.tc * 2)
      ++CommandData.gyheat.age;

    /******** do the pulse *****/
    if (p_on > 0) {
      WriteData(gyHeatAddr, on, NIOS_FLUSH);
      p_on--;
    } else if (p_off > 0) {
      WriteData(gyHeatAddr, off, NIOS_FLUSH);
      p_off--;
    }
  } else 
    /* Turn off heater if thermometer appears broken */
    WriteData(gyHeatAddr, off, NIOS_FLUSH);

  WriteData(gyHAgeAddr, CommandData.gyheat.age, NIOS_QUEUE);
  WriteData(gyHHistAddr, (history * 32768. / 100.), NIOS_QUEUE);
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
    ifpmBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
  } else {
    ifpmBits |= BAL1_REV;  /* set reverse bit */
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
    CommandData.pin_is_in = 1;
    is_closing = 0;
  } else if ((lockBits & LOKMOT_ISOUT) && (~lockBits & LOKMOT_ISIN)) {
    CommandData.pin_is_in = 0;
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
  static int delay[2] = {0, 0};

  if (firsttime) {
    firsttime = 0;
    TriggerAddr[0] = GetNiosAddr("isc_trigger");
    TriggerAddr[1] = GetNiosAddr("osc_trigger");
  }

  /* age is needed by pointing.c to tell it how old the solution is */
  if (isc_pulses[which].age >= 0)
    isc_pulses[which].age++;

  isc_pulses[which].last_save++;

  if (isc_pulses[which].start_wait == 0) { /* start of new pulse */
    if (!isc_pulses[which].ack_wait) { /* not waiting for ack: send new data */
      if (WHICH && delay == 0)
        bprintf(info, "%iSC (t): Start new pulse (%i)\n", which,
            isc_pulses[which].pulse_index);

      start_ISC_cycle[which] = 0;
      if (WHICH && delay == 0)
        bprintf(info, "%iSC (t): Lowering start_ISC_cycle\n", which);

      if (isc_pulses[which].is_fast) {  /* fast pulse */
        /* use fast (short) pulse length */
        isc_pulses[which].pulse_req =
          CommandData.ISCControl[which].fast_pulse_width;
      } else {  /* slow pulse */
        /* use slow (long) pulse length */
        isc_pulses[which].pulse_req = CommandData.ISCControl[which].pulse_width;

        /* autosave next image -- we only do this on long pulses */
        if (isc_pulses[which].last_save >=
            CommandData.ISCControl[which].save_period &&
            CommandData.ISCControl[which].save_period > 0) {

          CommandData.ISCControl[which].auto_save = 1;
          isc_pulses[which].last_save = 0;
        }
      }

      /* Inform the Star camera of the exposure time */
      /* pulse request is in 100Hz frames, so multiply by 10k to get usecs */
      CommandData.ISCState[which].exposure =
        isc_pulses[which].pulse_req * 10000;

      /* Add pulse serial number */
      isc_pulses[which].pulse_req += isc_pulses[which].pulse_index << 14;

      delay[which] = 0;

      /* If force_sync is high, we've detected an out-of-sync condition.
       * We get back in phase by skipping the ackwait stage.  We do this by
       * simply not resetting the semaphores before going into the ackwait.
       * Since write_ISC_trigger is already high, the ackwait will end 
       * immediately. */
      if (!isc_pulses[which].force_sync) {
        /* Signal isc thread to send new pointing data */
        if (write_ISC_trigger[which])
          if (WHICH)
            bprintf(info,
                "%iSC (t): Unexpectedly lowered write_ISC_trigger Semaphore\n",
                which);

        write_ISC_trigger[which] = 0;
        write_ISC_pointing[which] = 1;

        if (WHICH)
          bprintf(info, "%iSC (t): Raised write_ISC_pointing Semaphore\n",
              which);
      } else {
        isc_pulses[which].force_sync = 0;
        bprintf(warning,
            "%s: out-of-sync condition detected, attempting to resync",
            (which) ? "Osc" : "Isc");

        if (WHICH)
          bprintf(info,
              "%iSC (t): Lowered force_sync Semaphore ++++++++++++++++++\n",
              which);
      }

      /* Start waiting for ACK from star camera */
      isc_pulses[which].ack_wait = 1;
      isc_pulses[which].ack_timeout =
        (ISC_link_ok[which]) ? ISC_ACK_TIMEOUT : 10;
      if (WHICH) 
        bprintf(info, "%iSC (t): ackwait starts with timeout = %i\n", which,
            isc_pulses[which].ack_timeout);
    } else { /* ACK wait state */
      if (isc_pulses[which].ack_wait > isc_pulses[which].ack_timeout
          || write_ISC_trigger[which]) {

        if (delay[which] == 0) {
          /* if we exceeded the time-out, flag the link as bad */
          if (ISC_link_ok[which] && isc_pulses[which].ack_wait
              > isc_pulses[which].ack_timeout) {
            if (InCharge) {
              bprintf(warning,
                  "%s: timeout on ACK, flagging link as bad\n",
                  (which) ? "Osc" : "Isc");
              ISC_link_ok[which] = 0;
            } else if (WHICH)
              bprintf(warning, "%iSC (t): timeout on ACK while NiC\n", which);
          }

          delay[which] = ISC_TRIGGER_DELAY;
          if (WHICH)
            bprintf(info, "%iSC (t): Trigger Delay Starts: %i\n", which,
                delay[which]);
        } else if (delay[which] > 1)
          delay[which]--;
        else {
          delay[which] = 0;
          /* write the pulse */
          if (WHICH)
            bprintf(info, "%iSC (t): Writing trigger (%04x)\n", which,
                isc_pulses[which].pulse_req);
          WriteData(TriggerAddr[which], isc_pulses[which].pulse_req,
              NIOS_FLUSH);

          if (WHICH)
            bprintf(info, "%iSC (t): Lowering write_ISC_trigger\n", which);
          write_ISC_trigger[which] = 0;
          isc_pulses[which].ack_wait = 0;

          /* increment pulse serial number */
          isc_pulses[which].pulse_index
            = (isc_pulses[which].pulse_index + 1) % 4;

          /* start the start cycle timer */
          isc_pulses[which].start_wait = isc_pulses[which].ack_wait + 1;

          /* the -2 is due to processing time in the loop */
          isc_pulses[which].start_timeout = (ISC_link_ok[which]) ?
            ISC_DATA_TIMEOUT : ISC_DEFAULT_PERIOD - ISC_TRIGGER_DELAY - 2;

          if (WHICH)
            bprintf(info, "%iSC (t): startwait starts with timeout = %i\n",
                which, isc_pulses[which].start_timeout);

          /* re-zero age, if needed */
          if (isc_pulses[which].age < 0)
            isc_pulses[which].age = 0;
        }
      } else
            isc_pulses[which].ack_wait++;
    }
  } else { /* startwait state */
    if ((++isc_pulses[which].start_wait > isc_pulses[which].start_timeout)
        || (start_ISC_cycle[which])) {

      if (ISC_link_ok[which] && isc_pulses[which].start_wait >
          isc_pulses[which].start_timeout) {
        if (InCharge) {
        bprintf(warning, "%s: Timeout while waiting for solution.\n",
            (which) ? "Osc" : "Isc");
        } else if (WHICH)
          bprintf(warning,
              "%iSC (t): Timeout while waiting for solution and NiC.\n",
              which);
      }

      isc_pulses[which].start_wait = 0;
    }

    /* If the write_ISC_trigger semaphore goes high during startwait,
     * it means that the ISC thread just got a handshake packet from
     * the star camera -- ie. we're out of phase due to missing a packet
     * earlier (most likely, this is because mcp has just started up)
     * We need to resynchronise -- we do this by skipping the ackwait phase
     * during the next cycle */
    if (InCharge && write_ISC_trigger[which]) {
      isc_pulses[which].force_sync = 1;
      isc_pulses[which].start_wait = 0;

      if (WHICH)
        bprintf(info,
            "%iSC (t): Raised force_sync Semaphore +++++++++++++++++++\n",
            which);
    }
  }
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

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);
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

#define SENS_RST_GPS  0x08;
#define SENS_RST_ISC  0x10;
#define SENS_RST_GYRO 0x20;
#define SENS_RST_SUNS 0x40;
#define SENS_RST_OSC  0x80;

/* SensorResets: Power veto ISC, OSC, GPS and GYBOX2 */
void SensorResets(void)
{
  static int firsttime = 1;
  static struct NiosStruct* sensorResetAddr;
  int sensor_resets = 0;

  if (firsttime) {
    firsttime = 0;
    sensorResetAddr = GetNiosAddr("sensor_reset");
  }

  if (CommandData.sensors_off.gps)
    sensor_resets |= SENS_RST_GPS;
  if (CommandData.sensors_off.gyro)
    sensor_resets |= SENS_RST_GYRO;
  if (CommandData.sensors_off.isc)
    sensor_resets |= SENS_RST_ISC;
  if (CommandData.sensors_off.osc)
    sensor_resets |= SENS_RST_OSC;
  if (CommandData.sensors_off.ss)
    sensor_resets |= SENS_RST_SUNS;

  WriteData(sensorResetAddr, sensor_resets, NIOS_QUEUE);
}
