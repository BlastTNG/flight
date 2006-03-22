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
#define MAX_ISC_SLOW_PULSE_SPEED 0.015

/* limits for the gyrobox thermometer.  If the reading is outside this range,
 * we don't regulate the box at all, since it means the thermometer is probably
 * broken */
#define MIN_GYBOX_TEMP ((-50 - TGYBOX_B) / TGYBOX_M)  /* -50 C */
#define MAX_GYBOX_TEMP ((60 - TGYBOX_B) / TGYBOX_M)   /* +60 C */

/* limits for the inner cooling stuff */
#define MIN_TEMP ((-50 - I2T_B) / I2T_M)  /* -50 C */
#define MAX_TEMP ((60 - I2T_B) / I2T_M)   /* +60 C */
#define BAL_PUMP_MAX 1228 /* 60% */
#define PUMP_MAX 819 /* 40% */
#define PUMP_MIN 307  /* 15% */

/* inner cool */
#define IF_COOL_GOAL 30   /* in deg C */
#define IF_COOL_DELTA 10
#define IF_COOL_RANGE 20   /* At a temp of IF_COOL_GOAL + IF_COOL_RANGE,
                              pump on full */
/* outer cool */
#define OF_COOL_GOAL 25   /* in deg C */
#define OF_COOL_DELTA 10
#define OF_COOL_RANGE 20   /* At a temp of OF_COOL_GOAL + OF_COOL_RANGE,
                              pump on full */

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

/* ACS0 digital signals (G1 and G3 output, G2 input) */
#define BAL1_ON      0x04  /* ACS3 Group 1 Bit 3 - ifpmBits */
#define BAL1_REV     0x08  /* ACS3 Group 1 Bit 4 */
#define IF_COOL1_OFF 0x10  /* ACS3 Group 1 Bit 5 */
#define IF_COOL1_ON  0x20  /* ACS3 Group 1 Bit 6 */
#define BAL2_ON      0x40  /* ACS3 Group 1 Bit 7 */
#define BAL2_REV     0x80  /* ACS3 Group 1 Bit 8 */

#define OF_COOL2_ON  0x01  /* ACS3 Group 3 Bit 1 - ofpmBits */
#define OF_COOL2_OFF 0x02  /* ACS3 Group 3 Bit 2 */
#define OF_COOL1_ON  0x04  /* ACS3 Group 3 Bit 3 */
#define OF_COOL1_OFF 0x08  /* ACS3 Group 3 Bit 4 */
#define LOKMOT_ON    0x10  /* ACS3 Group 3 Bit 5 */
#define LOKMOT_OFF   0x20  /* ACS3 Group 3 Bit 6 */
#define LOKMOT_OUT   0x40  /* ACS3 Group 3 Bit 7 */
#define LOKMOT_IN    0x80  /* ACS3 Group 3 Bit 8 */

/* in commands.c */
double LockPosition(double elevation);

static short int incool_state = 0;
static short int outcool_state = 0;

static int SetGyHeatSetpoint(double history, int age, int box)
{
  double setpoint = CommandData.gyheat[box].setpoint;

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

  if (setpoint != CommandData.gyheat[box].setpoint) {
    bprintf(info, "Gyrobox Heat: Stepped gybox %i setpoint to %.2f degrees C\n",
        box + 1, setpoint);
    age = 0;
    CommandData.gyheat[box].setpoint = setpoint;
  }

  return age;
}

static int ControlInnerCool(void)
{
  static struct BiPhaseStruct *tDasAddr, *tRecAddr;
  static int firsttime = 1;
  unsigned int das, rec;
  int dg = 1, rg = 1;
  double temp = 0;
  double error;
  short int pwm;

  if (firsttime) {
    firsttime = 0;
    tDasAddr = GetBiPhaseAddr("t_das");
    tRecAddr = GetBiPhaseAddr("t_rec");
  }

  if (CommandData.pumps.inframe_auto == 0) {
    if (incool_state > 0) {
      bprintf(info, "Inner Frame Cooling: Vetoed\n");
      incool_state = 0;
    }
    return CommandData.pumps.pwm3;
  }

  das = slow_data[tDasAddr->index][tDasAddr->channel];
  rec = slow_data[tRecAddr->index][tRecAddr->channel];

  /* NB: these tests are backwards due to a sign flip in the calibration */
  if (das < MAX_TEMP || das > MIN_TEMP)
    dg = 0;
  if (rec < MAX_TEMP || rec > MIN_TEMP)
    rg = 0;

  if (dg && rg)
    temp = I2T_M * 0.5 * (das + rec) + I2T_B;
  else if (dg)
    temp = I2T_M * das + I2T_B;
  else if (rg)
    temp = I2T_M * rec + I2T_B;
  else {
    if (incool_state != 3) {
      bprintf(info, "Inner Frame Cooling: Auto-Vetoed\n");
      CommandData.pumps.inframe_cool_on = 40; /* turn on pump */
      CommandData.pumps.inframe_cool_off = 0;
      incool_state = 3;
    }
    return CommandData.pumps.pwm3; /* both temps bad --
                                      revert to manual settings */
  }

  if (temp < IF_COOL_GOAL - IF_COOL_DELTA ||
      (temp < IF_COOL_GOAL && incool_state == 1)) {
    if (incool_state != 1) {
      bprintf(info, "Inner Frame Cooling: Pump Off\n");
      CommandData.pumps.inframe_cool_off = 40;
      CommandData.pumps.inframe_cool_on = 0;
      incool_state = 1;
    }
    return 2047; /* temperature below goal, nothing to do, turn off pump */
  }

  if (incool_state != 2) {
    bprintf(info, "Inner Frame Cooling: Pump On\n");
    CommandData.pumps.inframe_cool_on = 40; /* turn on pump */
    CommandData.pumps.inframe_cool_off = 0;
    incool_state = 2;
  }

  error = temp - IF_COOL_GOAL;

  pwm = 2047. * error / IF_COOL_RANGE;

  if (pwm > PUMP_MAX)
    pwm = PUMP_MAX;
  else if (pwm < PUMP_MIN)
    pwm = PUMP_MIN;

  return 2047 - pwm;
}

static int ControlOuterCool(void)
{
  static struct BiPhaseStruct *tSunSensorAddr;
  static int firsttime = 1;
  double temp;
  double error;
  short int pwm;

  if (firsttime) {
    firsttime = 0;
    tSunSensorAddr = GetBiPhaseAddr("t_sun_sensor");
  }

  if (CommandData.pumps.outframe_auto == 0) {
    if (outcool_state > 0) {
      bprintf(info, "Outer Frame Cooling: Vetoed\n");
      outcool_state = 0;
    }
    return CommandData.pumps.pwm4;
  }

  temp = slow_data[tSunSensorAddr->index][tSunSensorAddr->channel];

  /* NB: these tests are backwards due to a sign flip in the calibration */
  if (temp < MAX_TEMP || temp > MIN_TEMP) {
    if (outcool_state != 3) {
      bprintf(info, "Outer Frame Cooling: Auto-Vetoed\n");
      CommandData.pumps.outframe_cool1_on = 40; /* turn on pump */
      CommandData.pumps.outframe_cool1_off = 0;
      outcool_state = 3;
    }
    return CommandData.pumps.pwm3; /* temp bad -- revert to manual settings */
  }

  temp = I2T_M * temp + I2T_B;

  if (temp < OF_COOL_GOAL - OF_COOL_DELTA ||
      (temp < OF_COOL_GOAL && outcool_state == 1)) {
    if (outcool_state != 1) {
      bprintf(info, "Outer Frame Cooling: Pump Off\n");
      CommandData.pumps.outframe_cool1_off = 40;
      CommandData.pumps.outframe_cool1_on = 0;
      outcool_state = 1;
    }
    return 2047; /* temperature below goal, nothing to do, turn off pump */
  }

  if (outcool_state != 2) {
    bprintf(info, "Outer Frame Cooling: Pump On\n");
    CommandData.pumps.outframe_cool1_on = 40; /* turn on pump */
    CommandData.pumps.outframe_cool1_off = 0;
    outcool_state = 2;
  }

  error = temp - OF_COOL_GOAL;

  pwm = 2047. * error / OF_COOL_RANGE;

  if (pwm > PUMP_MAX)
    pwm = PUMP_MAX;
  else if (pwm < PUMP_MIN)
    pwm = PUMP_MIN;

  return 2047 - pwm;
}

/************************************************************************/
/*    ControlGyroHeat:  Controls gyro box temps                         */
/************************************************************************/
void ControlGyroHeat(unsigned short *RxFrame, int box)
{
  static struct BiPhaseStruct* tGyboxAddr[2];
  static struct NiosStruct *gyHeatAddr[2], *tGySetAddr[2], *pGyheatAddr[2];
  static struct NiosStruct *iGyheatAddr[2];
  static struct NiosStruct *dGyheatAddr[2], *gyHHistAddr[2], *gyHAgeAddr[2];
  static int firsttime = 1;
  static double history[2] = {0, 0};

  /* the 0x5's in here for gybox 2 are the enable bits for the star camera
   * heaters */
  int on[2] = {0x40, 0x15}, off[2] = {0x00, 0x05};
  static int p_on[2] = {0, 0};
  static int p_off[2] = {-1, -1};

  float error = 0, set_point;
  unsigned int temp;
  static float integral[2] = {0, 0};
  static float deriv[2] = {0, 0};
  static float error_last[2] = {0, 0};
  float P, I, D;

  /******** Obtain correct indexes the first time here ***********/
  if (firsttime) {
    firsttime = 0;
    tGyboxAddr[0] = GetBiPhaseAddr("t_gybox1");
    tGyboxAddr[1] = GetBiPhaseAddr("t_gybox2");

    gyHeatAddr[0] = GetNiosAddr("gy1_heat");
    gyHeatAddr[1] = GetNiosAddr("gy2_heat");

    gyHHistAddr[0] = GetNiosAddr("gy1_h_hist");
    gyHHistAddr[1] = GetNiosAddr("gy2_h_hist");

    gyHAgeAddr[0] = GetNiosAddr("gy1_h_age");
    gyHAgeAddr[1] = GetNiosAddr("gy2_h_age");

    tGySetAddr[0] = GetNiosAddr("t_gy1_set");
    tGySetAddr[1] = GetNiosAddr("t_gy2_set");

    pGyheatAddr[0] = GetNiosAddr("g_p_gyheat1");
    iGyheatAddr[0] = GetNiosAddr("g_i_gyheat1");
    dGyheatAddr[0] = GetNiosAddr("g_d_gyheat1");

    pGyheatAddr[1] = GetNiosAddr("g_p_gyheat2");
    iGyheatAddr[1] = GetNiosAddr("g_i_gyheat2");
    dGyheatAddr[1] = GetNiosAddr("g_d_gyheat2");
  }

  /* send down the setpoints and gains values */
  WriteData(tGySetAddr[box], CommandData.gyheat[box].setpoint * 327.68,
      NIOS_QUEUE);

  WriteData(pGyheatAddr[box], CommandData.gyheat[box].gain.P, NIOS_QUEUE);
  WriteData(iGyheatAddr[box], CommandData.gyheat[box].gain.I, NIOS_QUEUE);
  WriteData(dGyheatAddr[box], CommandData.gyheat[box].gain.D, NIOS_QUEUE);

  temp = (RxFrame[tGyboxAddr[box]->channel + 1] << 16 |
      RxFrame[tGyboxAddr[box]->channel]);

  /* Only run these controls if we think the thermometer isn't broken */
  /* NB: these tests are backwards due to a sign flip in the calibration */
  if (temp > MAX_GYBOX_TEMP && temp < MIN_GYBOX_TEMP) {
    /* control the heat */
    CommandData.gyheat[box].age = SetGyHeatSetpoint(history[box],
        CommandData.gyheat[box].age, box);

    set_point = (CommandData.gyheat[box].setpoint - TGYBOX_B) / TGYBOX_M;
    P = CommandData.gyheat[box].gain.P * (-1.0 / 1000000.0);
    I = CommandData.gyheat[box].gain.I * (-1.0 / 110000.0);
    D = CommandData.gyheat[box].gain.D * ( 1.0 / 1000.0);

    /********* if end of pulse, calculate next pulse *********/
    if (p_off[box] <= 0 && p_on[box] <= 0) {
      error = set_point - temp;

      integral[box] = integral[box] * 0.999 + 0.001 * error;
      if (integral[box] * I > 60)
        integral[box] = 60.0 / I;

      if (integral[box] * I < 0)
        integral[box] = 0;

      deriv[box] = error_last[box] - error;
      error_last[box] = error;

      p_on[box] = P * error + (deriv[box] / 60.0) * D + integral[box] * I;

      if (p_on[box] > 60)
        p_on[box] = 60;
      else if (p_on[box] < 0)
        p_on[box] = 0;

      p_off[box] = 60 - p_on[box];

      history[box] = p_on[box] * 100. / GY_HEAT_TC + (1. - 60. / GY_HEAT_TC)
        * history[box];
    }

    if (CommandData.gyheat[box].age <= GY_HEAT_TC * 2)
      ++CommandData.gyheat[box].age;

    /******** do the pulse *****/
    if (p_on[box] > 0) {
      WriteData(gyHeatAddr[box], on[box], NIOS_FLUSH);
      p_on[box]--;
    } else if (p_off[box] > 0) {
      WriteData(gyHeatAddr[box], off[box], NIOS_FLUSH);
      p_off[box]--;
    }
  } else
    /* Turn off heater if thermometer appears broken */
    WriteData(gyHeatAddr[box], off[box], NIOS_FLUSH);

  WriteData(gyHAgeAddr[box], CommandData.gyheat[box].age, NIOS_QUEUE);
  WriteData(gyHHistAddr[box], (history[box] * 32768. / 100.), NIOS_QUEUE);
}

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
static int Balance(int ifpmBits)
{
  static struct BiPhaseStruct *iElAddr;
  static struct NiosStruct *balPwm1Addr;
  static int pumpon = 0;
  int pumppwm;
  int error;
  static int pump_is_on = -2;
  static double smoothed_i = I_EL_ZERO;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    iElAddr = GetBiPhaseAddr("i_el");
    balPwm1Addr = GetNiosAddr("balpump_lev");
  }

  /* don't turn on pump if we're reading very small numbers */
  if (slow_data[iElAddr->index][iElAddr->channel] < 8000)
    error = 0;
  else {
    smoothed_i = slow_data[iElAddr->index][iElAddr->channel] / 500. +
      smoothed_i * (499. / 500.);
    error = smoothed_i - I_EL_ZERO - CommandData.pumps.bal_target;
  }

  /* Don't do anything else if we're vetoed */
  if (CommandData.pumps.bal_veto == -1) {
    if (pump_is_on != -1) {
      bprintf(info, "Balance System: Vetoed\n");
      pump_is_on = -1;
    }
    return ifpmBits;
  } else if (CommandData.pumps.bal_veto > 1)
    return ifpmBits;

  if (error > 0)
    ifpmBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
  else {
    ifpmBits |= BAL1_REV;  /* set reverse bit */
    error = -error;
  }

  pumppwm = error * CommandData.pumps.bal_gain;

  if (pumppwm < PUMP_MIN)
    pumppwm = PUMP_MIN;
  else if (pumppwm > BAL_PUMP_MAX)
    pumppwm = BAL_PUMP_MAX;

  if (error > CommandData.pumps.bal_on) {
    pumpon = 1;
    CommandData.pumps.bal_veto = 0;
  } else if (error < CommandData.pumps.bal_off) {
    pumpon = 0;
    if (CommandData.pumps.bal_veto > 1)
      CommandData.pumps.bal_veto = BAL_VETO_MAX;
  }

  if (pumpon) {
    if (pump_is_on != 1) {
      bprintf(info, "Balance System: Pump On\n");
      pump_is_on = 1;
    }
    ifpmBits |= BAL1_ON; /* turn on pump */
  } else {
    if (pump_is_on != 0) {
      bprintf(info, "Balance System: Pump Off\n");
      pump_is_on = 0;
    }
    ifpmBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

  WriteData(balPwm1Addr, 2047 - pumppwm, NIOS_QUEUE);

  return ifpmBits;
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
    T = I2T_M*slow_data[tAcsBatAddr->index][tAcsBatAddr->channel] + I2T_B;
    if (T<-60) T = 50; // if disconnected, assume hot.
    V = 30.18 - 0.0436*T - exp((T-29.0)*0.25) + CommandData.apcu_trim;
    //apcu_control = (V - 28.0209)/0.02402664;
    apcu_control = (V - 27.25)/0.0382;
  } else {
    //apcu_control = (CommandData.apcu_reg - 28.0209)/0.02402664;
    apcu_control = (CommandData.apcu_reg - 27.25)/0.0382;
  }

  if (CommandData.dpcu_auto) {
    T = I2T_M*slow_data[tDasBatAddr->index][tDasBatAddr->channel] + I2T_B;
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
  WriteData(apcuTrimAddr, CommandData.apcu_trim, NIOS_QUEUE);
  WriteData(apcuAutoAddr, CommandData.apcu_auto, NIOS_QUEUE);
  WriteData(dpcuRegAddr, (int)dpcu_control, NIOS_QUEUE);
  WriteData(dpcuTrimAddr, CommandData.dpcu_trim, NIOS_QUEUE);
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
  static struct NiosStruct* ofpmBitsAddr;
  static struct NiosStruct* balpumpLevAddr;
  static struct NiosStruct* sprpumpLevAddr;
  static struct NiosStruct* inpumpLevAddr, *incoolStateAddr;
  static struct NiosStruct* outpumpLevAddr, *outcoolStateAddr;
  static struct NiosStruct* balOnAddr, *balOffAddr;
  static struct NiosStruct* balTargetAddr, *balVetoAddr;
  static struct NiosStruct* balGainAddr;
  static struct NiosStruct* ifpmBitsAddr;

  int ifpmBits = 0;
  int ofpmBits = 0;

  int inframe_pwm = ControlInnerCool();
  int outframe_pwm = ControlOuterCool();

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    ifpmBitsAddr = GetNiosAddr("ifpm_bits");
    ofpmBitsAddr = GetNiosAddr("ofpm_bits");
    balpumpLevAddr = GetNiosAddr("balpump_lev");
    sprpumpLevAddr = GetNiosAddr("sprpump_lev");
    incoolStateAddr = GetNiosAddr("incool_state");
    outcoolStateAddr = GetNiosAddr("outcool_state");
    inpumpLevAddr = GetNiosAddr("inpump_lev");
    outpumpLevAddr = GetNiosAddr("outpump_lev");
    balOnAddr = GetNiosAddr("bal_on");
    balOffAddr = GetNiosAddr("bal_off");
    balTargetAddr = GetNiosAddr("bal_target");
    balGainAddr = GetNiosAddr("bal_gain");
    balVetoAddr = GetNiosAddr("bal_veto");
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
  if (CommandData.pumps.inframe_cool_on > 0) {
    ifpmBits |= IF_COOL1_ON;
    CommandData.pumps.inframe_cool_on--;
    if (CommandData.pumps.inframe_auto == 0 && incool_state != -2) {
      bprintf(info, "Inner Frame Cooling: Pump On\n");
      incool_state = -2;
    }
  } else if (CommandData.pumps.inframe_cool_off > 0) {
    ifpmBits |= IF_COOL1_OFF;
    CommandData.pumps.inframe_cool_off--;
    if (CommandData.pumps.inframe_auto == 0 && incool_state != -1) {
      bprintf(info, "Inner Frame Cooling: Pump Off\n");
      incool_state = -1;
    }
  }

  /* outer frame box */
  /* three on, off motors (pulses) */
  if (CommandData.pumps.outframe_cool1_on > 0) {
    ofpmBits |= OF_COOL1_ON;
    CommandData.pumps.outframe_cool1_on--;
    if (CommandData.pumps.outframe_auto == 0 && outcool_state != -2) {
      bprintf(info, "Outer Frame Cooling: Pump On\n");
      outcool_state = -2;
    }
  } else if (CommandData.pumps.outframe_cool1_off > 0) {
    ofpmBits |= OF_COOL1_OFF;
    CommandData.pumps.outframe_cool1_off--;
    if (CommandData.pumps.outframe_auto == 0 && outcool_state != -1) {
      bprintf(info, "Outer Frame Cooling: Pump Off\n");
      outcool_state = -1;
    }
  }

  /* this pump isn't used */
  if (CommandData.pumps.outframe_cool2_on > 0) {
    ofpmBits |= OF_COOL2_ON;
    CommandData.pumps.outframe_cool2_on--;
  } else if (CommandData.pumps.outframe_cool2_off > 0) {
    ofpmBits |= OF_COOL2_OFF;
    CommandData.pumps.outframe_cool2_off--;
  }

  /* Run Balance System, Maybe */
  ifpmBits = Balance(ifpmBits);

  if (CommandData.pumps.bal_veto) {
    /* if we're in timeout mode, decrement the timer */
    if (CommandData.pumps.bal_veto > 1)
      CommandData.pumps.bal_veto--;

    WriteData(balpumpLevAddr, CommandData.pumps.pwm1 & 0x7ff, NIOS_QUEUE);
  }

  WriteData(incoolStateAddr, incool_state, NIOS_QUEUE);
  WriteData(outcoolStateAddr, outcool_state, NIOS_QUEUE);
  WriteData(ofpmBitsAddr, ofpmBits, NIOS_QUEUE);
  WriteData(sprpumpLevAddr, CommandData.pumps.pwm2 & 0x7ff, NIOS_QUEUE);
  WriteData(inpumpLevAddr, inframe_pwm & 0x7ff, NIOS_QUEUE);
  WriteData(outpumpLevAddr, outframe_pwm & 0x7ff, NIOS_QUEUE);
  WriteData(balOnAddr, (int)CommandData.pumps.bal_on, NIOS_QUEUE);
  WriteData(balOffAddr, (int)CommandData.pumps.bal_off, NIOS_QUEUE);
  WriteData(balVetoAddr, (int)CommandData.pumps.bal_veto, NIOS_QUEUE);
  WriteData(balTargetAddr, (int)(CommandData.pumps.bal_target + 1648. * 5.),
      NIOS_QUEUE);
  WriteData(balGainAddr, (int)(CommandData.pumps.bal_gain * 1000.), NIOS_QUEUE);
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
