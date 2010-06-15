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
#define BAL_DIR      0x01  /* ACS2 Group 2 Bit 1 */
#define BAL_VALV     0x02  /* ACS2 Group 2 Bit 2 */
#define BAL_HEAT     0x04  /* ACS2 Group 2 Bit 3 - DAC */

#define PUMP_MAX 26214      /*  3.97*2.0V   */
#define PUMP_MIN  3277      /*  3.97*0.25V   */

#define PUMP_ZERO 32773

/* in commands.c */
double LockPosition(double elevation);


/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temp                          */
/************************************************************************/
void ControlGyroHeat(unsigned short *RxFrame)
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

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
static int Balance(int bits_bal)
{
  static struct BiPhaseStruct *dacElAddr;
  static struct NiosStruct *vPumpBalAddr;
  static struct NiosStruct *modeBalAddr;
  static int pumpon = 0;
  int level;
  int error;
  static int pump_is_on = -2;
  static double smoothed_i = I_EL_ZERO;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    dacElAddr = GetBiPhaseAddr("dac_el");
    vPumpBalAddr = GetNiosAddr("v_pump_bal");
    modeBalAddr = GetNiosAddr("mode_bal");
  }
  
  // enable slew mode
  if (CommandData.pumps.veto_bal == 0){
     CommandData.pumps.veto_bal = CommandData.pointing_mode.nw;
  }

  // if vetoed 
  if (CommandData.pumps.veto_bal > 0) {
    CommandData.pumps.veto_bal--;
  } 

  if ((CommandData.pumps.mode == bal_rest) || (CommandData.pumps.veto_bal > 0)) {
 
    // set direction
    bits_bal &= (0xFF - BAL_DIR); /* Clear reverse bit */ 

    // close valve
    bits_bal &= (0xFF - BAL_VALV); /* Close the valve */ 

    level = 0;

  } else if (CommandData.pumps.mode == bal_manual) {

    //   set direction and valve bits
    //   set speed

    bits_bal |= BAL_VALV; /* Open valve */

    if (CommandData.pumps.level > 0.) {
      bits_bal &= (0xFF - BAL_DIR); /* clear reverse bit */
      level = CommandData.pumps.level * PUMP_MAX;
    } else if (CommandData.pumps.level < 0.) {
      bits_bal |= BAL_DIR; /* set reverse bit */
      level = -CommandData.pumps.level * PUMP_MAX;
    } else {
      bits_bal &= (0xFF - BAL_VALV); /* Close valve */
      level = 0;
    }

  } else {

    //   calculate speed and direction
    smoothed_i = slow_data[dacElAddr->index][dacElAddr->channel] / 500. +
          smoothed_i * (499. / 500.);
    error = smoothed_i - I_EL_ZERO - CommandData.pumps.level_target_bal;

    //   set direction and valve bits
    if (error > 0) {
      bits_bal &= (0xFF - BAL_DIR);  /* clear reverse bit */
    } else {
      bits_bal |= BAL_DIR;  /* set reverse bit */
      error = -error;
    }

    //   set gain
    // level = error * CommandData.pumps.gain_bal;
    level = PUMP_MAX*CommandData.pumps.gain_bal;

    //   compare to preset values

    if (level < PUMP_MIN) {
       level = PUMP_MIN;
    } else if (level > PUMP_MAX) {
       level = PUMP_MAX;
    }

    if (error > CommandData.pumps.level_on_bal) {
      pumpon = 1;
      CommandData.pumps.veto_bal = 0;
    } else if (error < CommandData.pumps.level_off_bal) {
      pumpon = 0;
      if(CommandData.pumps.veto_bal > 1) {
	CommandData.pumps.veto_bal = BAL_VETO_MAX;
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
  WriteData(modeBalAddr, CommandData.pumps.mode, NIOS_FLUSH);
  return bits_bal;

}

/************************************************************************/
/*    ControlPumpHeat:  Controls balance system pump temp               */
/************************************************************************/

static int ControlPumpHeat(int bits_bal)
{

  static struct BiPhaseStruct *tBoxBalAddr, *tPumpBalAddr;
  static int firsttime = 1;

  unsigned int temp1, temp2;

  if (firsttime) {
    firsttime = 0;
    tBoxBalAddr = GetBiPhaseAddr("t_box_bal");
    tPumpBalAddr = GetBiPhaseAddr("t_pump_bal");  
  }

  temp1 = slow_data[tBoxBalAddr->index][tBoxBalAddr->channel];
  temp2 = slow_data[tPumpBalAddr->index][tPumpBalAddr->channel];

  if (CommandData.pumps.heat_on) {
    if (temp1 < CommandData.pumps.heat_tset) {
      bits_bal |= BAL_HEAT;  /* set heat bit */
    } else {
      bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
    }
  } else {
      bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
  }

  return bits_bal;

}

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

  }

  WriteData(VBattCCAddr, 1000.0*ChrgCtrlData.V_batt, NIOS_QUEUE);
  WriteData(VArrCCAddr, 1000.0*ChrgCtrlData.V_arr, NIOS_QUEUE);
  WriteData(IBattCCAddr, 1000.0*ChrgCtrlData.I_batt, NIOS_QUEUE);
  WriteData(IArrCCAddr, 1000.0*ChrgCtrlData.I_arr, NIOS_QUEUE);
  WriteData(VTargCCAddr, 1000.0*ChrgCtrlData.V_targ, NIOS_QUEUE);
  WriteData(ThsCCAddr, ChrgCtrlData.T_hs, NIOS_QUEUE);
  WriteData(FaultCCAddr, ChrgCtrlData.fault_field, NIOS_QUEUE);
  WriteData(AlarmHiCCAddr, ChrgCtrlData.alarm_field_hi, NIOS_QUEUE);
  WriteData(AlarmLoCCAddr, ChrgCtrlData.alarm_field_lo, NIOS_QUEUE);
  WriteData(ChargeCCAddr, ChrgCtrlData.charge_state, NIOS_QUEUE);

  /* original code for BLAST06 MEER charge controller follows */

  /*  static struct NiosStruct *apcuRegAddr;
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
  WriteData(dpcuAutoAddr, CommandData.dpcu_auto, NIOS_FLUSH);*/

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
  char swhich[4] = "ISC";
  if (which) swhich[0] = 'O';

  if (firsttime) {
    firsttime = 0;
    //NB before renaming, 0 was osc, 1 was isc. I think that was wrong
    TriggerAddr[0] = GetNiosAddr("trigger_isc");
    TriggerAddr[1] = GetNiosAddr("trigger_osc");
  }

  /* age is needed by pointing.c to tell it how old the solution is */
  if (isc_pulses[which].age >= 0)
    isc_pulses[which].age++;

  isc_pulses[which].last_save++;

  if (isc_pulses[which].start_wait == 0) { /* start of new pulse */
    if (!isc_pulses[which].ack_wait) { /* not waiting for ack: send new data */
      if (WHICH && delay[which] == 0)
        bprintf(info, "%s (t): Start new pulse (%i/%i)\n", swhich,
            isc_pulses[which].pulse_index, isc_pulses[which].is_fast);

      start_ISC_cycle[which] = 0;
      if (WHICH && delay[which] == 0)
        bprintf(info, "%s (t): Lowering start_ISC_cycle\n", swhich);

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
              "%s (t): Unexpectedly lowered write_ISC_trigger Semaphore\n",
              swhich);

        write_ISC_trigger[which] = 0;
        write_ISC_pointing[which] = 1;

        if (WHICH)
          bprintf(info, "%s (t): Raised write_ISC_pointing Semaphore\n",
              swhich);
      } else {
        isc_pulses[which].force_sync = 0;
        bprintf(warning,
            "%s: out-of-sync condition detected, attempting to resync",
            swhich);

        if (WHICH)
          bprintf(info,
              "%s (t): Lowered force_sync Semaphore ++++++++++++++++++\n",
              swhich);
      }

      /* Start waiting for ACK from star camera */
      isc_pulses[which].ack_wait = 1;
      isc_pulses[which].ack_timeout =
        (ISC_link_ok[which]) ? ISC_ACK_TIMEOUT : 10;
      if (WHICH)
        bprintf(info, "%s (t): ackwait starts with timeout = %i\n", 
	    swhich, isc_pulses[which].ack_timeout);
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
              bprintf(warning, "%s: timeout on ACK, flagging link as bad\n",
                  swhich);
              ISC_link_ok[which] = 0;
            } else if (WHICH)
              bprintf(warning, "%s (t): timeout on ACK while NiC\n", 
		  swhich);
          }

          delay[which] = ISC_TRIGGER_DELAY;
          if (WHICH)
            bprintf(info, "%s (t): Trigger Delay Starts: %i\n", 
		swhich, delay[which]);
        } else if (delay[which] > 1)
          delay[which]--;
        else {
          delay[which] = 0;

          if (!isc_pulses[which].is_fast) {
            /* wait until we're below the slow speed */
            if (fabs(axes_mode.az_vel) >= MAX_ISC_SLOW_PULSE_SPEED) {
              if (!waiting[which] && WHICH)
                bprintf(info,
                    "%s (t): Velocity wait starts (%.3f %.3f) <----- v\n",
                    swhich, 
		    fabs(axes_mode.az_vel), MAX_ISC_SLOW_PULSE_SPEED);
              waiting[which] = 1;
              return;
            }

            if (WHICH)
              bprintf(info, "%s (t): Velocity wait ends. -------> v\n",
                  swhich);

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
            bprintf(info, "%s (t): Writing trigger (%04x)\n", swhich,
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
            bprintf(info, "%s (t): Lowering write_ISC_trigger\n", swhich);
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
            bprintf(info, "%s (t): startwait starts with timeout = %i\n",
                swhich, isc_pulses[which].start_timeout);

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
              swhich);
        } else if (WHICH)
          bprintf(warning,
              "%s (t): Timeout while waiting for solution and NiC.\n",
              swhich);
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
            "%s (t): Raised force_sync Semaphore +++++++++++++++++++\n",
            swhich);
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
  static struct NiosStruct* vPumpBalAddr;
  static struct NiosStruct* levelOnBalAddr, *levelOffBalAddr;
  static struct NiosStruct* levelTargetBalAddr, *vetoBalAddr;
  static struct NiosStruct* gainBalAddr;
  static struct NiosStruct* bitsBalAddr;

  int bits_bal = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    bitsBalAddr = GetNiosAddr("bits_bal");
    vPumpBalAddr = GetNiosAddr("v_pump_bal");
    levelOnBalAddr = GetNiosAddr("level_on_bal");
    levelOffBalAddr = GetNiosAddr("level_off_bal");
    levelTargetBalAddr = GetNiosAddr("level_target_bal");
    gainBalAddr = GetNiosAddr("gain_bal");
    vetoBalAddr = GetNiosAddr("veto_bal");
  }

  /* inner frame box */
  /* two latching pumps 3/4 */
  /* two non latching: on/off, fwd/rev */

  /* Run Balance System, maybe */
  bits_bal = Balance(bits_bal);

  /* Run Heating card, maybe */
  bits_bal = ControlPumpHeat(bits_bal);
  
  WriteData(levelOnBalAddr, CommandData.pumps.level_on_bal, NIOS_QUEUE);
  WriteData(levelOffBalAddr, CommandData.pumps.level_off_bal, NIOS_QUEUE);
  WriteData(vetoBalAddr, (int)CommandData.pumps.veto_bal/4.0, NIOS_QUEUE);
  WriteData(levelTargetBalAddr, (CommandData.pumps.level_target_bal + 1990.13*5.),
      NIOS_QUEUE);
  WriteData(gainBalAddr, (int)(CommandData.pumps.gain_bal * 1000.), NIOS_QUEUE);
  WriteData(bitsBalAddr, bits_bal, NIOS_FLUSH);

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
  WriteData(switchGyAddr, gybox, NIOS_QUEUE);
  WriteData(switchMiscAddr, misc, NIOS_QUEUE);
}
