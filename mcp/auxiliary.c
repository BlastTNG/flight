#include <math.h>
#include <time.h>
#include <stdio.h>

#include "tx_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define ISC_TRIG_PERIOD 100
#define MAX_ISC_SLOW_PULSE_SPEED 0.015

struct ISCPulseType isc_pulses = {0,0,4,0};

extern unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

int pin_is_in = 1;

/* ACS0 digital signals (G1 and G3 output, G2 input) */
#define ISC_NOHEAT   0x00  /* N0G1 - iscBits */
#define ISC_HEAT     0x01  /* N0G1 */
#define ISC_TRIGGER  0x02  /* N0G1 */
#define BAL1_ON      0x04  /* N0G1 */
#define BAL1_REV     0x08  /* N0G1 */
#define IF_COOL1_OFF 0x10  /* N0G1 */
#define IF_COOL1_ON  0x20  /* N0G1 */
#define BAL2_ON      0x40  /* N0G1 */
#define BAL2_REV     0x80  /* N0G1 */

#define ISC_SYNC     0x01  /* N0G2  - acs0bits */
#define LOKMOT_ISIN  0x40  /* N0G2 */
#define LOKMOT_ISOUT 0x80  /* N0G2 */

#define OF_COOL2_ON  0x01  /* N0G3 - pumpBits */
#define OF_COOL2_OFF 0x02  /* N0G3 */
#define OF_COOL1_ON  0x04  /* N0G3 */
#define OF_COOL1_OFF 0x08  /* N0G3 */
#define LOKMOT_ON    0x10  /* N0G3 */
#define LOKMOT_OFF   0x20  /* N0G3 */
#define LOKMOT_OUT   0x40  /* N0G3 */
#define LOKMOT_IN    0x80  /* N0G3 */

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
void ControlGyroHeat(unsigned int *Txframe,  unsigned short *Rxframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_T_GYBOX = -1;
  static int i_GY_HEAT = -1;

  static int i_T_GY_SET = -1, j_T_GY_SET = -1;
  static int i_G_PGYH = -1, j_G_PGYH = -1;
  static int i_G_IGYH = -1, j_G_IGYH = -1;
  static int i_G_DGYH = -1, j_G_DGYH = -1;

  int on = 0x40, off = 0x00;
  static int p_on = 0;
  static int p_off = -1;

  float error = 0, set_point;
  static float integral = 0;
  static float deriv = 0;
  static float error_last = 0;
  float P, I, D;

  /******** Obtain correct indexes the first time here ***********/
  if (i_T_GYBOX == -1) {
    FastChIndex("t_gybox", &i_T_GYBOX);
    FastChIndex("gy_heat", &i_GY_HEAT);

    SlowChIndex("t_gy_set", &i_T_GY_SET, &j_T_GY_SET);
    SlowChIndex("g_p_gyheat", &i_G_PGYH, &j_G_PGYH);
    SlowChIndex("g_i_gyheat", &i_G_IGYH, &j_G_IGYH);
    SlowChIndex("g_d_gyheat", &i_G_DGYH, &j_G_DGYH);
  }

  /* send down the setpoints and gains values */
  WriteSlow(i_T_GY_SET, j_T_GY_SET,
      (unsigned short)(CommandData.t_gybox_setpoint * 32768.0 / 100.0));

  WriteSlow(i_G_PGYH, j_G_PGYH, CommandData.gy_heat_gain.P);
  WriteSlow(i_G_IGYH, j_G_IGYH, CommandData.gy_heat_gain.I);
  WriteSlow(i_G_DGYH, j_G_DGYH, CommandData.gy_heat_gain.D);

  /* control the heat */
  set_point = (CommandData.t_gybox_setpoint - 136.45) / (-9.5367431641e-08);
  P = CommandData.gy_heat_gain.P * (-1.0 / 1000000.0);
  I = CommandData.gy_heat_gain.I * (-1.0 / 110000.0);
  D = CommandData.gy_heat_gain.D * ( 1.0 / 1000.0);

  /********* if end of pulse, calculate next pulse *********/
  if (p_off < 0) {

    error = set_point -
      ((unsigned int)(Rxframe[i_T_GYBOX + 1]<< 16 | Rxframe[i_T_GYBOX]));

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

    if (p_on > 60) p_on = 60;
    if (p_on < 0) p_on = 0;
    p_off = 60 - p_on;

  }

  /******** do the pulse *****/
  if (p_on > 0) {
    WriteFast(i_GY_HEAT, on);
    p_on--;
  } else {
    WriteFast(i_GY_HEAT, off);
    p_off--;
  }
}

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
int Balance(int iscBits, unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int iElCh = -1, iElInd;
  static int balPwm1Ch, balPwm1Ind;
  static int pumpon = 0;
  int pumppwm;
  int error;

  if (iElCh == -1) {
    SlowChIndex("i_el", &iElCh, &iElInd);
    SlowChIndex("balpump_lev", &balPwm1Ch, &balPwm1Ind);
  }

  if (slow_data[iElCh][iElInd] < 8000)  /* don't turn on pump if we're */
    error = 0;                          /* reading very small numbers */
  else
    error = slow_data[iElCh][iElInd] - 32758 - CommandData.pumps.bal_target;

  if (error > 0) {
    iscBits |= BAL1_REV;  /* set reverse bit */
  } else {
    iscBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
    error = -error;
  }

  printf("Balance: error = %i gain = %lf bal_min = %i bal_max = %i\n",
      error, CommandData.pumps.bal_gain, CommandData.pumps.bal_min,
      CommandData.pumps.bal_max);

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
    iscBits |= BAL1_ON; /* turn on pump */
  } else {
    iscBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

  WriteSlow(balPwm1Ch, balPwm1Ind, pumppwm);

  return iscBits;
}

/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
#define MOVE_COUNTS 200
#define SEARCH_COUNTS 500
int GetLockBits(int acs0bits) {
  static int closing = 0;
  static int opening = 0;
  static int searching = 0;

  /* Override limit switches */
  if (CommandData.lock_override != 0) {
    pin_is_in = (CommandData.lock_override == 1);
    return 0;
  }

  /* check for commands from CommandData */
  if (CommandData.pumps.lock_in) {
    CommandData.pumps.lock_in = 0;
    closing = MOVE_COUNTS;
    opening = 0;
    searching = 0;
  } else if (CommandData.pumps.lock_out) {
    CommandData.pumps.lock_out = 0;
    opening = MOVE_COUNTS;
    searching = 0;
  } else if (CommandData.pumps.lock_point) {
    CommandData.pumps.lock_point = 0;
    searching = SEARCH_COUNTS;
  }

  if (searching>1) {
    if (fabs(ACSData.enc_elev -
          LockPosition(ACSData.enc_elev)) > 0.2) {
      searching = SEARCH_COUNTS;
    } else {
      searching--;
    }
  }

  if (closing > 0) {
    closing--;
    return(LOKMOT_IN | LOKMOT_ON);
  } else if (opening > 0) {
    opening--;
    return(LOKMOT_OUT | LOKMOT_ON);
  } else if (searching == 1) {
    searching = 0;
    closing = MOVE_COUNTS;
    opening = 0;
    return(LOKMOT_IN | LOKMOT_ON);
  } else if ((acs0bits & 0xc0) == 0) { /* if motor is on, these bits are 0 */
    return (LOKMOT_OFF);
  } else { /* motor is off - we can read position */
    if (acs0bits & 64)
      pin_is_in = 1;
    else
      pin_is_in = 0;
    return 0;
  }
}

/*****************************************************************/
/*                                                               */
/*   Control the pumps and the lock and the ISC pulse            */
/*                                                               */
/*****************************************************************/
void ControlAuxMotors(unsigned int *Txframe,  unsigned short *Rxframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int pumpBitsCh, pumpBitsInd = -1;
  static int pumpPwm1Ch, pumpPwm1Ind;
  static int pumpPwm2Ch, pumpPwm2Ind;
  static int pumpPwm3Ch, pumpPwm3Ind;
  static int pumpPwm4Ch, pumpPwm4Ind;
  static int acs0bitsCh;
  static int balOnCh, balOnInd, balOffCh, balOffInd;
  static int balTargetCh, balTargetInd, balVetoCh, balVetoInd;
  static int balGainCh, balGainInd, balMinCh, balMinInd, balMaxCh, balMaxInd;
  static int iscBitsCh;
  static int i_lockpin, j_lockpin;
  static int pinOverrideCh, pinOverrideInd;

  static int since_last_save = 0;

  int iscBits = 0;
  int pumpBits = 0;
  int pin_override;

  if (pumpBitsInd == -1) {
    FastChIndex("isc_bits", &iscBitsCh);
    FastChIndex("acs0bits", &acs0bitsCh);
    SlowChIndex("pump_bits", &pumpBitsCh, &pumpBitsInd);
    SlowChIndex("balpump_lev", &pumpPwm1Ch, &pumpPwm1Ind);
    SlowChIndex("sprpump_lev", &pumpPwm2Ch, &pumpPwm2Ind);
    SlowChIndex("inpump_lev", &pumpPwm3Ch, &pumpPwm3Ind);
    SlowChIndex("outpump_lev", &pumpPwm4Ch, &pumpPwm4Ind);
    SlowChIndex("bal_on", &balOnCh, &balOnInd);
    SlowChIndex("bal_off", &balOffCh, &balOffInd);
    SlowChIndex("bal_target", &balTargetCh, &balTargetInd);
    SlowChIndex("bal_gain", &balGainCh, &balGainInd);
    SlowChIndex("bal_min", &balMinCh, &balMinInd);
    SlowChIndex("bal_max", &balMaxCh, &balMaxInd);
    SlowChIndex("bal_veto", &balVetoCh, &balVetoInd);
    SlowChIndex("lokmot_pin", &i_lockpin, &j_lockpin);
    SlowChIndex("l_override", &pinOverrideCh, &pinOverrideInd);
  }

  /* inner frame box */
  /* two latching pumps 3/4 */
  /* two non latching: on/off, fwd/rev */
  if (CommandData.pumps.bal_veto) {
    if (CommandData.pumps.bal1_on) iscBits |= BAL1_ON;
    if (CommandData.pumps.bal1_reverse) iscBits |= BAL1_REV;
    if (CommandData.pumps.bal2_on) iscBits |= BAL2_ON;
    if (CommandData.pumps.bal2_reverse) iscBits |= BAL2_REV;
  }

  /* two latching pumps: */
  if (CommandData.pumps.inframe_cool1_on > 0) {
    iscBits |= IF_COOL1_ON;
    CommandData.pumps.inframe_cool1_on--;
  } else if (CommandData.pumps.inframe_cool1_off > 0) {
    iscBits |= IF_COOL1_OFF;
    CommandData.pumps.inframe_cool1_off--;
  }

  /* outer frame box */
  /* three on, off motors (pulses) */
  if (CommandData.pumps.outframe_cool1_on > 0) {
    pumpBits |= OF_COOL1_ON;
    CommandData.pumps.outframe_cool1_on--;
  } else if (CommandData.pumps.outframe_cool1_off > 0) {
    pumpBits |= OF_COOL1_OFF;
    CommandData.pumps.outframe_cool1_off--;
  }

  if (CommandData.pumps.outframe_cool2_on > 0) {
    pumpBits |= OF_COOL2_ON;
    CommandData.pumps.outframe_cool2_on--;
  } else if (CommandData.pumps.outframe_cool2_off > 0) {
    pumpBits |= OF_COOL2_OFF;
    CommandData.pumps.outframe_cool2_off--;
  }

  pumpBits |= GetLockBits(Rxframe[acs0bitsCh]);

  if (CommandData.pumps.bal_veto) {
    /* if we're in timeout mode, decrement the timer */
    if (CommandData.pumps.bal_veto != -1)
      CommandData.pumps.bal_veto--;

    WriteSlow(pumpPwm1Ch, pumpPwm1Ind, CommandData.pumps.pwm1 & 0x7ff);
  } else {
    iscBits = Balance(iscBits, slowTxFields);
  }

  /*********************/
  /* ISC Pulsing stuff */
  if (isc_pulses.ctr<isc_pulses.pulse_width) {
    iscBits |= ISC_TRIGGER;
  }
  
  /* We want to trigger sending the frame slightly after the pulse is sent
   * to offset the 300 ms latency in the BLASTbus */
  if (isc_pulses.ctr == 20)
    write_ISC_pointing = 1;	

  if (isc_pulses.age>=0) isc_pulses.age++;
  
  if (isc_pulses.ctr < ISC_TRIG_PERIOD) {
    isc_pulses.ctr++;
  } else {
    if (isc_pulses.is_fast) {
      isc_pulses.pulse_width = CommandData.ISC_fast_pulse_width;
      isc_pulses.ctr = 0;
      if (isc_pulses.age < 0)
        isc_pulses.age = 0;
    } else if (fabs(axes_mode.az_vel) < MAX_ISC_SLOW_PULSE_SPEED) {
      isc_pulses.pulse_width = CommandData.ISC_pulse_width;
      isc_pulses.ctr = 0;

      /* Trigger automatic image write-to-disk */
      if (since_last_save >= CommandData.ISC_save_period &&
          CommandData.ISC_save_period > 0) {
        CommandData.ISC_auto_save = 1;
        since_last_save = 0;
      }
      
      if (isc_pulses.age < 0)
        isc_pulses.age = 0;
    }
  }

  since_last_save++;
  /*********************/

  /* Lock motor override writeback */
  pin_override = (CommandData.lock_override) ? 1 : 0;

  WriteSlow(pinOverrideCh, pinOverrideInd, pin_override);
  WriteSlow(i_lockpin, j_lockpin, pin_is_in);
  WriteSlow(pumpBitsCh, pumpBitsInd, pumpBits);
  WriteSlow(pumpPwm2Ch, pumpPwm2Ind, CommandData.pumps.pwm2 & 0x7ff);
  WriteSlow(pumpPwm3Ch, pumpPwm3Ind, CommandData.pumps.pwm3 & 0x7ff);
  WriteSlow(pumpPwm4Ch, pumpPwm4Ind, CommandData.pumps.pwm4 & 0x7ff);
  WriteSlow(balOnCh, balOnInd, (int)CommandData.pumps.bal_on);
  WriteSlow(balOffCh, balOffInd, (int)CommandData.pumps.bal_off);
  WriteSlow(balVetoCh, balVetoInd, (int)CommandData.pumps.bal_veto);
  WriteSlow(balTargetCh, balTargetInd, (int)CommandData.pumps.bal_target);
  WriteSlow(balGainCh, balGainInd, (int)(CommandData.pumps.bal_gain * 1000.));
  WriteSlow(balMinCh, balMinInd, (int)CommandData.pumps.bal_min);
  WriteSlow(balMaxCh, balMaxInd, (int)CommandData.pumps.bal_max);
  WriteFast(iscBitsCh, iscBits);

}
