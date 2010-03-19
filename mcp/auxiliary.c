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

/* limits for the gyrobox thermometer.  If the reading is outside this range,
 * we don't regulate the box at all, since it means the thermometer is probably
 * broken */
#define MIN_GYBOX_TEMP ((223.15 / M_16T) - B_16T)   /* -50 C */
#define MAX_GYBOX_TEMP ((333.15 / M_16T) - B_16T)   /* +60 C */

/* Gybox heater stuff */
#define GY_HEAT_MAX 40 /* percent */
#define GY_HEAT_MIN 5  /* percent */
#define GY_TEMP_MAX 50 /* setpoint maximum -- deg C */
#define GY_TEMP_MIN 0  /* setpoint minimum -- deg C */
#define GY_TEMP_STEP 1 /* setpoint step - deg C */
#define GY_HEAT_TC 30000 /* integral/age characteristic time in 100Hz Frames */

struct ISCPulseType isc_pulses[2] = {
  {-1, 0, 0, 0, 0, 0, 0, 0}, {-1, 0, 0, 0, 0, 0, 0, 0}
};

/* Semaphores for handshaking with the ISC/OSC threads (isc.c) */
extern short int write_ISC_pointing[2];
extern short int write_ISC_trigger[2];
extern short int ISC_link_ok[2];
extern short int start_ISC_cycle[2];

extern short int InCharge; /* tx.c */

/* ACS2 digital signals */
#define BAL_DIRE     0x01  /* ACS2 Group 2 Bit 1 */
#define BAL_VALV     0x02  /* ACS2 Group 2 Bit 2 */
#define BAL_HEAT     0x04  /* ACS2 Group 2 Bit 3 - DAC */

#define PUMP_MAX 32714       /*  2.50V   */
#define PUMP_MIN 13107       /*  1.00V   */

#define PUMP_ZERO 32820

/* in commands.c */
double LockPosition(double elevation);

static int SetGyHeatSetpoint(double history, int age)
{
  double setpoint = CommandData.gyheat.setpoint;

  if (age < GY_HEAT_TC * 2)
    return age;

  if (history < GY_HEAT_MIN)
    setpoint += GY_TEMP_STEP;
  else if (history > GY_HEAT_MAX)
    setpoint -= GY_TEMP_STEP;

  if (setpoint < GY_TEMP_MIN)
    setpoint = GY_TEMP_MIN;
  else if (setpoint > GY_TEMP_MAX)
    setpoint = GY_TEMP_MAX;

  if (setpoint != CommandData.gyheat.setpoint) {
    bprintf(info, "Gyrobox Heat: Stepped setpoint to %.2f deg C\n", setpoint);
    age = 0;
    CommandData.gyheat.setpoint = setpoint;
  }

  return age;
}

/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temp                          */
/************************************************************************/
void ControlGyroHeat(unsigned short *RxFrame)
{
  static struct BiPhaseStruct* tGyboxAddr;
  static struct NiosStruct *gyHeatAddr, *tGySetAddr, *pGyheatAddr;
  static struct NiosStruct *iGyheatAddr;
  static struct NiosStruct *dGyheatAddr, *gyHHistAddr, *gyHAgeAddr;
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
    tGyboxAddr = GetBiPhaseAddr("t_gybox");
    gyHeatAddr = GetNiosAddr("gy_heat");
    gyHHistAddr = GetNiosAddr("gy_h_hist");
    gyHAgeAddr = GetNiosAddr("gy_h_age");
    tGySetAddr = GetNiosAddr("t_gy_set");

    pGyheatAddr = GetNiosAddr("g_p_gyheat");
    iGyheatAddr = GetNiosAddr("g_i_gyheat");
    dGyheatAddr = GetNiosAddr("g_d_gyheat");
  }

  /* send down the setpoints and gains values */
  WriteData(tGySetAddr, CommandData.gyheat.setpoint * 327.68, NIOS_QUEUE);
  WriteData(pGyheatAddr, CommandData.gyheat.gain.P, NIOS_QUEUE);
  WriteData(iGyheatAddr, CommandData.gyheat.gain.I, NIOS_QUEUE);
  WriteData(dGyheatAddr, CommandData.gyheat.gain.D, NIOS_QUEUE);

  temp = slow_data[tGyboxAddr->index][tGyboxAddr->channel];
  
  /* Only run these controls if we think the thermometer isn't broken */
  if (temp < MAX_GYBOX_TEMP && temp > MIN_GYBOX_TEMP) {
    /* control the heat */
    CommandData.gyheat.age = SetGyHeatSetpoint(history,
        CommandData.gyheat.age);

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
      WriteData(gyHeatAddr, 0x1, NIOS_FLUSH);
      p_on--;
    } else if (p_off > 0) {
      WriteData(gyHeatAddr, 0x0, NIOS_FLUSH);
      p_off--;
    }
  } else
    /* Turn off heater if thermometer appears broken */
    WriteData(gyHeatAddr, 0x0, NIOS_FLUSH);

  WriteData(gyHAgeAddr, CommandData.gyheat.age, NIOS_QUEUE);
  WriteData(gyHHistAddr, (history * 32768. / 100.), NIOS_QUEUE);
}

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
static int Balance(int bits_bal)
{
  static struct BiPhaseStruct *elDacAddr;
  static struct NiosStruct *vPumpBalAddr;
  static int pumpon = 0;
  int level;
  int error;
  static int pump_is_on = -2;
  static double smoothed_i = I_EL_ZERO;
  
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elDacAddr = GetBiPhaseAddr("el_dac");
    vPumpBalAddr = GetNiosAddr("v_pump_bal");
  }
 
  // if vetoed {
  if (CommandData.pumps.bal_veto > 0) {
    CommandData.pumps.bal_veto--;
  }

  if ((CommandData.pumps.mode == bal_rest) || (CommandData.pumps.bal_veto > 0)) {
 
    // set direction
    bits_bal &= (0xFF - BAL_DIRE); /* Clear reverse bit */ 

    // close valve
    bits_bal &= (0xFF - BAL_VALV); /* Close the valve */ 

    level = 0;

  } else if (CommandData.pumps.mode == bal_manual) {

    //   set direction and valve bits
    //   set speed

    bits_bal |= BAL_VALV; /* Open valve */

    if (CommandData.pumps.level > 0) {
      bits_bal &= (0xFF - BAL_DIRE); /* clear reverse bit */
      level = CommandData.pumps.level * PUMP_MAX;
    } else if (CommandData.pumps.level < 0) {
      bits_bal |= BAL_DIRE; /* set reverse bit */
      level = -CommandData.pumps.level * PUMP_MAX;
    } else {
      bits_bal &= (0xFF - BAL_VALV); /* Close valve */
      level = 0;
    }
 
  } else {

    //   calculate speed and direction
    smoothed_i = slow_data[elDacAddr->index][elDacAddr->channel] / 500. +
          smoothed_i * (499. / 500.);
    error = smoothed_i - I_EL_ZERO - CommandData.pumps.bal_target;

    //   set direction and valve bits
    if (error > 0) {
      bits_bal &= (0xFF - BAL_DIRE);  /* clear reverse bit */
    } else {
      bits_bal |= BAL_DIRE;  /* set reverse bit */
      error = -error;
    }

    //   set gain
    // level = error * CommandData.pumps.bal_gain;
    level = PUMP_MAX*CommandData.pumps.bal_gain;

    //   compare to preset values

    if (level < PUMP_MIN) {
       level = PUMP_MIN;
    } else if (level > PUMP_MAX) {
       level = PUMP_MAX;
    }

    if (error > CommandData.pumps.bal_on) {
      pumpon = 1;
      CommandData.pumps.bal_veto = 0;
    } else if (error < CommandData.pumps.bal_off) {
      pumpon = 0;
      if(CommandData.pumps.bal_veto > 1) {
	CommandData.pumps.bal_veto = BAL_VETO_MAX;
      }
    }

    if (pumpon) {
      if (pump_is_on != 1) {
	bprintf(info, "Balance System: Pump On\n");
	pump_is_on = 1;
      }
      bits_bal |= BAL_VALV; /* open valve pump */
    } else {
      if (pump_is_on != 0) {
	bprintf(info, "Balance System: Pump Off\n");
	pump_is_on = 0;
      }
      bits_bal &= (0xFF - BAL_VALV); /* close valve pump */
      level = 0;
    }

  }

  // write direction and valve bits
  WriteData(vPumpBalAddr, (int) PUMP_ZERO + level, NIOS_QUEUE);
  return bits_bal;

}

/************************************************************************/
/*    ControlPumpHeat:  Controls balance system pump temp               */
/************************************************************************/

static int ControlPumpHeat(int bits_bal)
{

  static struct BiPhaseStruct *t1BalAddr, *t2BalAddr;
  static int firsttime = 1;

  unsigned int temp1, temp2;

  if (firsttime) {
    firsttime = 0;
    t1BalAddr = GetBiPhaseAddr("t_1bal");
    t2BalAddr = GetBiPhaseAddr("t_2bal");  
  }

  temp1 = slow_data[t1BalAddr->index][t1BalAddr->channel];
  temp2 = slow_data[t2BalAddr->index][t2BalAddr->channel];

//  if (CommandData.pumps.heat_on) {
//      temp1 > CommandData.pumps.heat_tset
    if (temp1 < MAX_GYBOX_TEMP && temp1 > MIN_GYBOX_TEMP) {
      bits_bal |= BAL_DIRE;  /* set heat bit */
    } else {
      bits_bal &= (0xFF - BAL_DIRE); /* clear heat bit */
    }
//  }

 // CommandData.pumps.heat_tset
 // CommandData.pump.heat_on

  return bits_bal;

}

void ChargeController(void)
{
  static struct NiosStruct *apcuRegAddr;
  static struct NiosStruct *apcuTrimAddr;
  static struct NiosStruct *apcuAutoAddr;
  static struct NiosStruct *dpcuRegAddr;
  static struct NiosStruct *dpcuTrimAddr;
  static struct NiosStruct *dpcuAutoAddr;
  static struct BiPhaseStruct *tDasBatAddr, *tAcsBatAddr;
  double apcu_control, T, V;
  double dpcu_control;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    apcuRegAddr = GetNiosAddr("apcu_reg");
    apcuTrimAddr = GetNiosAddr("apcu_trim");
    apcuAutoAddr = GetNiosAddr("apcu_auto");
    dpcuRegAddr = GetNiosAddr("dpcu_reg");
    dpcuTrimAddr = GetNiosAddr("dpcu_trim");
    dpcuAutoAddr = GetNiosAddr("dpcu_auto");
    tDasBatAddr = GetBiPhaseAddr("t_batt_das");
    tAcsBatAddr = GetBiPhaseAddr("t_batt_acs");
  }

  if (CommandData.apcu_auto) {
    T = M_16T*( slow_data[tAcsBatAddr->index][tAcsBatAddr->channel] 
	+ B_16T ) - 273.15;
    if (T<-60) T = 50; // if disconnected, assume hot.
    V = 30.18 - 0.0436*T - exp((T-29.0)*0.25) + CommandData.apcu_trim;
    //apcu_control = (V - 28.0209)/0.02402664;
    apcu_control = (V - 27.25)/0.0382;
  } else {
    //apcu_control = (CommandData.apcu_reg - 28.0209)/0.02402664;
    apcu_control = (CommandData.apcu_reg - 27.25)/0.0382;
  }

  if (CommandData.dpcu_auto) {
    T = M_16T*( slow_data[tDasBatAddr->index][tDasBatAddr->channel] 
	+ B_16T ) - 273.15;
    if (T<-60) T = 50; // if disconnected, assume hot.
    V = 30.18 - 0.0436*T - exp((T-29.0)*0.25) + CommandData.dpcu_trim;
    //dpcu_control = (V - 28.0209)/0.02402664;
    dpcu_control = (V - 27.25)/0.0382;
  } else {
    //dpcu_control = (CommandData.dpcu_reg - 28.0209)/0.02402664;
    dpcu_control = (CommandData.dpcu_reg - 27.25)/0.0382;
  }

  if (apcu_control>100) apcu_control = 100;
  if (apcu_control<0) apcu_control = 0;
  if (dpcu_control>100) dpcu_control = 100;
  if (dpcu_control<0) dpcu_control = 0;

  WriteData(apcuRegAddr, (int)apcu_control, NIOS_QUEUE);
  WriteData(apcuTrimAddr, CommandData.apcu_trim*100.0, NIOS_QUEUE);
  WriteData(apcuAutoAddr, CommandData.apcu_auto, NIOS_QUEUE);
  WriteData(dpcuRegAddr, (int)dpcu_control, NIOS_QUEUE);
  WriteData(dpcuTrimAddr, CommandData.dpcu_trim*100.0, NIOS_QUEUE);
  WriteData(dpcuAutoAddr, CommandData.dpcu_auto, NIOS_FLUSH);
}

/*********************/
/* ISC Pulsing stuff */
void CameraTrigger(int which)
{
  static int firsttime = 1;
  static struct NiosStruct* TriggerAddr[2];
  static int delay[2] = {0, 0};
  static int waiting[2] = {0, 0};
#if SYNCHRONOUS_CAMERAS
  static int cameras_ready = 0;
#endif

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
      if (WHICH && delay[which] == 0)
        bprintf(info, "%iSC (t): Start new pulse (%i/%i)\n", which,
            isc_pulses[which].pulse_index, isc_pulses[which].is_fast);

      start_ISC_cycle[which] = 0;
      if (WHICH && delay[which] == 0)
        bprintf(info, "%iSC (t): Lowering start_ISC_cycle\n", which);

      if (!isc_pulses[which].is_fast) {  /* slow pulse */
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

      delay[which] = 0;

      /* If force_sync is high, we've detected an out-of-sync condition.
       * We get back in phase by skipping the ackwait stage.  We do this by
       * simply not resetting the semaphores before going into the ackwait.
       * Since write_ISC_trigger is already high, the ackwait will end
       * immediately. */
      if (!isc_pulses[which].force_sync) {
        /* Signal isc thread to send new pointing data */
        if (WHICH && write_ISC_trigger[which])
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

        /* don't bother doing the trigger wait if we're already in velocity
         * wait state */
        if (waiting[which])
          delay[which] = 1;

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

          if (!isc_pulses[which].is_fast) {
            /* wait until we're below the slow speed */
            if (fabs(axes_mode.az_vel) >= MAX_ISC_SLOW_PULSE_SPEED) {
              if (!waiting[which] && WHICH)
                bprintf(info,
                    "%iSC (t): Velocity wait starts (%.3f %.3f) <----- v\n",
                    which, fabs(axes_mode.az_vel), MAX_ISC_SLOW_PULSE_SPEED);
              waiting[which] = 1;
              return;
            }

            if (WHICH)
              bprintf(info, "%iSC (t): Velocity wait ends. -------> v\n",
                  which);

            /* use slow (long) pulse length */
            isc_pulses[which].pulse_req
              = CommandData.ISCControl[which].pulse_width;

            waiting[which] = 0;
          } else {
            if (waiting[which])
              bprintf(warning, "%s: Velocity wait stated aborted.\n",
                  (which) ? "Osc" : "Isc");
            waiting[which] = 0;

            /* use fast (short) pulse length */
            isc_pulses[which].pulse_req =
              CommandData.ISCControl[which].fast_pulse_width;
          }

          /* Add pulse serial number */
          isc_pulses[which].pulse_req += isc_pulses[which].pulse_index << 14;

          /* write the pulse */
          if (WHICH)
            bprintf(info, "%iSC (t): Writing trigger (%04x)\n", which,
                isc_pulses[which].pulse_req);
#if SYNCHRONOUS_CAMERAS
          if (which)
            cameras_ready |= 2;
          else
            cameras_ready |= 1;

          if (cameras_ready == 3) {
            WriteData(TriggerAddr[0], isc_pulses[0].pulse_req, NIOS_FLUSH);
            WriteData(TriggerAddr[1], isc_pulses[1].pulse_req, NIOS_FLUSH);
            cameras_ready = 0;
          }
#else
          WriteData(TriggerAddr[which], isc_pulses[which].pulse_req,
              NIOS_FLUSH);
#endif

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
void ControlAuxMotors(unsigned short *RxFrame)
{
  static struct NiosStruct* levPumpBalAddr;
  static struct NiosStruct* balOnAddr, *balOffAddr;
  static struct NiosStruct* balTargetAddr, *balVetoAddr;
  static struct NiosStruct* balGainAddr;
  static struct NiosStruct* bitsBalAddr;

  int bits_bal = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    bitsBalAddr = GetNiosAddr("bits_bal");
    levPumpBalAddr = GetNiosAddr("lev_pump_bal");
    balOnAddr = GetNiosAddr("bal_on");
    balOffAddr = GetNiosAddr("bal_off");
    balTargetAddr = GetNiosAddr("bal_target");
    balGainAddr = GetNiosAddr("bal_gain");
    balVetoAddr = GetNiosAddr("bal_veto");
  }

  /* inner frame box */
  /* two latching pumps 3/4 */
  /* two non latching: on/off, fwd/rev */

  /* Run Balance System, maybe */
  bits_bal = Balance(bits_bal);

  /* Run Heating card, maybe */
  bits_bal = ControlPumpHeat(bits_bal);

  //bprintf(info, "MotorControl: (%i)\n", ifpmBits);

  if (CommandData.pumps.bal_veto) {
    /* if we're in timeout mode, decrement the timer */
    if (CommandData.pumps.bal_veto > 1)
      CommandData.pumps.bal_veto--;
    //FIX THIS
    //WriteData(levPumpBalAddr, CommandData.pumps.pwm1 & 0x7ff, NIOS_QUEUE);
  }
  
  //WriteData(sprpumpLevAddr, CommandData.pumps.pwm2 & 0x7ff, NIOS_QUEUE);
  WriteData(balOnAddr, (int)CommandData.pumps.bal_on, NIOS_QUEUE);
  WriteData(balOffAddr, (int)CommandData.pumps.bal_off, NIOS_QUEUE);
  WriteData(balVetoAddr, (int)CommandData.pumps.bal_veto, NIOS_QUEUE);
  WriteData(balTargetAddr, (int)(CommandData.pumps.bal_target + 1990.13 * 5.),
      NIOS_QUEUE);
  WriteData(balGainAddr, (int)(CommandData.pumps.bal_gain * 1000.), NIOS_QUEUE);
  WriteData(bitsBalAddr, bits_bal, NIOS_FLUSH);
}

/* create latching relay pulses, and update enable/disbale levels */
/* actbus/steppers enable is handled separately in StoreActBus() */
void ControlPower(void) {

  static int firsttime = 1;
  static struct NiosStruct* latchingAddr[2];
  static struct NiosStruct* gyboxSwitchAddr;
  static struct NiosStruct* miscSwitchAddr;
  int latch0 = 0, latch1 = 0, gybox = 0, misc = 0;
  int i;

  if (firsttime) {
    firsttime = 0;
    latchingAddr[0] = GetNiosAddr("latch0");
    latchingAddr[1] = GetNiosAddr("latch1");
    gyboxSwitchAddr = GetNiosAddr("gybox_switch");
    miscSwitchAddr = GetNiosAddr("misc_switch");
  }

  if (CommandData.power.hub232_off) {
    if (CommandData.power.hub232_off > 0) CommandData.power.hub232_off--;
    misc |= 0x08;
  }

  if (CommandData.power.ss_off) {
    if (CommandData.power.ss_off > 0) CommandData.power.ss_off--;
    misc |= 0x20;
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
  if (CommandData.power.reac.set_count > 0) {
    CommandData.power.reac.set_count--;
    if (CommandData.power.reac.set_count < LATCH_PULSE_LEN) latch0 |= 0x0400;
  }
  if (CommandData.power.reac.rst_count > 0) {
    CommandData.power.reac.rst_count--;
    if (CommandData.power.reac.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0800;
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
  if (CommandData.power.preamp.set_count > 0) {
    CommandData.power.preamp.set_count--;
    if (CommandData.power.preamp.set_count < LATCH_PULSE_LEN) latch1 |= 0x0004;
  }
  if (CommandData.power.preamp.rst_count > 0) {
    CommandData.power.preamp.rst_count--;
    if (CommandData.power.preamp.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0008;
  }
  if (CommandData.power.bias.set_count > 0) {
    CommandData.power.bias.set_count--;
    if (CommandData.power.bias.set_count < LATCH_PULSE_LEN) latch1 |= 0x0010;
  }
  if (CommandData.power.bias.rst_count > 0) {
    CommandData.power.bias.rst_count--;
    if (CommandData.power.bias.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0020;
  }
  if (CommandData.power.hk.set_count > 0) {
    CommandData.power.hk.set_count--;
    if (CommandData.power.hk.set_count < LATCH_PULSE_LEN) latch1 |= 0x0040;
  }
  if (CommandData.power.hk.rst_count > 0) {
    CommandData.power.hk.rst_count--;
    if (CommandData.power.hk.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0080;
  }
  if (CommandData.power.um250.set_count > 0) {
    CommandData.power.um250.set_count--;
    if (CommandData.power.um250.set_count < LATCH_PULSE_LEN) latch1 |= 0x0100;
  }
  if (CommandData.power.um250.rst_count > 0) {
    CommandData.power.um250.rst_count--;
    if (CommandData.power.um250.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0200;
  }
  if (CommandData.power.um350.set_count > 0) {
    CommandData.power.um350.set_count--;
    if (CommandData.power.um350.set_count < LATCH_PULSE_LEN) latch1 |= 0x0400;
  }
  if (CommandData.power.um350.rst_count > 0) {
    CommandData.power.um350.rst_count--;
    if (CommandData.power.um350.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0800;
  }
  if (CommandData.power.um500.set_count > 0) {
    CommandData.power.um500.set_count--;
    if (CommandData.power.um500.set_count < LATCH_PULSE_LEN) latch1 |= 0x1000;
  }
  if (CommandData.power.um500.rst_count > 0) {
    CommandData.power.um500.rst_count--;
    if (CommandData.power.um500.rst_count < LATCH_PULSE_LEN) latch1 |= 0x2000;
  }
  if (CommandData.power.heat.set_count > 0) {
    CommandData.power.heat.set_count--;
    if (CommandData.power.heat.set_count < LATCH_PULSE_LEN) latch1 |= 0x4000;
  }
  if (CommandData.power.heat.rst_count > 0) {
    CommandData.power.heat.rst_count--;
    if (CommandData.power.heat.rst_count < LATCH_PULSE_LEN) latch1 |= 0x8000;
  }

  WriteData(latchingAddr[0], latch0, NIOS_QUEUE);
  WriteData(latchingAddr[1], latch1, NIOS_QUEUE);
  WriteData(gyboxSwitchAddr, gybox, NIOS_QUEUE);
  WriteData(miscSwitchAddr, misc, NIOS_QUEUE);
}
