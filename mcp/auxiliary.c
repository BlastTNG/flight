#include <math.h>
#include <time.h>
#include <stdio.h>

#include "mcp.h"
#include "tx_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"

#define ISC_TRIG_PERIOD 100
#define MAX_ISC_SLOW_PULSE_SPEED 0.015

struct ISCPulseType isc_pulses[2] = {{0, 0, 4, 0, 0}, {0, 0, 4, 0, 0}};

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
      (unsigned short)(CommandData.t_gybox_setpoint * 32768.0 / 100.0));

  WriteData(pGyheatAddr, CommandData.gy_heat_gain.P);
  WriteData(iGyheatAddr, CommandData.gy_heat_gain.I);
  WriteData(dGyheatAddr, CommandData.gy_heat_gain.D);

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
    WriteData(gyHeatAddr, on);
    p_on--;
  } else {
    WriteData(gyHeatAddr, off);
    p_off--;
  }
}

/******************************************************************/
/*                                                                */
/* Balance: control balance system                                */
/*                                                                */
/******************************************************************/
int Balance(int iscBits) {
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
    iscBits |= BAL1_REV;  /* set reverse bit */
  } else {
    iscBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
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
    iscBits |= BAL1_ON; /* turn on pump */
  } else {
    iscBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

  WriteData(balPwm1Addr, pumppwm);

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

  if (searching > 1) {
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

/*********************/
/* ISC Pulsing stuff */
int CameraTrigger(int which)
{
  int iscBits = 0;

  if (isc_pulses[which].ctr < isc_pulses[which].pulse_width) {
    iscBits |= ISC_TRIGGER;
  }
  
  /* We want to trigger sending the frame slightly after the pulse is sent
   * to offset the 300 ms latency in the BLASTbus */
  if (isc_pulses[which].ctr == 20)
    write_ISC_pointing[which] = 1;	

  if (isc_pulses[which].age >= 0)
    isc_pulses[which].age++;
  
  if (isc_pulses[which].ctr < ISC_TRIG_PERIOD) {
    isc_pulses[which].ctr++;
  } else {
    if (isc_pulses[which].is_fast) {
      isc_pulses[which].pulse_width = CommandData.ISCControl[which].fast_pulse_width;
      isc_pulses[which].ctr = 0;
      if (isc_pulses[which].age < 0)
        isc_pulses[which].age = 0;
    } else if (fabs(axes_mode.az_vel) < MAX_ISC_SLOW_PULSE_SPEED) {
      isc_pulses[which].pulse_width = CommandData.ISCControl[which].pulse_width;
      isc_pulses[which].ctr = 0;

      /* Trigger automatic image write-to-disk */
      if (isc_pulses[which].last_save
          >= CommandData.ISCControl[which].save_period &&
          CommandData.ISCControl[which].save_period > 0) {
        CommandData.ISCControl[which].auto_save = 1;
        isc_pulses[which].last_save = 0;
      }
      
      if (isc_pulses[which].age < 0)
        isc_pulses[which].age = 0;
    }
  }

  isc_pulses[which].last_save++;
  /*********************/

  return iscBits;
}

/*****************************************************************/
/*                                                               */
/*   Control the pumps and the lock and the ISC pulse            */
/*                                                               */
/*****************************************************************/
void ControlAuxMotors(unsigned short *RxFrame) {
  static struct NiosStruct* pumpBitsAddr;
  static struct NiosStruct* balpumpLevAddr;
  static struct NiosStruct* sprpumpLevAddr;
  static struct NiosStruct* inpumpLevAddr;
  static struct NiosStruct* outpumpLevAddr;
  static struct NiosStruct* acs0BitsAddr;
  static struct NiosStruct* balOnAddr, *balOffAddr;
  static struct NiosStruct* balTargetAddr, *balVetoAddr;
  static struct NiosStruct* balGainAddr, *balMinAddr, *balMaxAddr;
  static struct NiosStruct* iscBitsAddr;
  static struct NiosStruct* lokmotPinAddr;
  static struct NiosStruct* lOverrideAddr;

  int iscBits = 0;
  int pumpBits = 0;
  int pin_override;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    iscBitsAddr = GetNiosAddr("isc_bits");
    acs0BitsAddr = GetNiosAddr("acs0bits");
    pumpBitsAddr = GetNiosAddr("pump_bits");
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
    lOverrideAddr = GetNiosAddr("l_override");
  }

  /* inner frame box */
  /* two latching pumps 3/4 */
  /* two non latching: on/off, fwd/rev */
  if (CommandData.pumps.bal_veto) {
    if (CommandData.pumps.bal1_on)
      iscBits |= BAL1_ON;
    if (CommandData.pumps.bal1_reverse)
      iscBits |= BAL1_REV;
    if (CommandData.pumps.bal2_on)
      iscBits |= BAL2_ON;
    if (CommandData.pumps.bal2_reverse)
      iscBits |= BAL2_REV;
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

  pumpBits |= GetLockBits(RxFrame[(ExtractBiPhaseAddr(acs0BitsAddr))->channel]);

  if (CommandData.pumps.bal_veto) {
    /* if we're in timeout mode, decrement the timer */
    if (CommandData.pumps.bal_veto != -1)
      CommandData.pumps.bal_veto--;

    WriteData(balpumpLevAddr, CommandData.pumps.pwm1 & 0x7ff);
  } else {
    iscBits = Balance(iscBits);
  }

  CameraTrigger(0); /* isc */
  CameraTrigger(1); /* osc */

  /* Lock motor override writeback */
  pin_override = (CommandData.lock_override) ? 1 : 0;

  WriteData(lOverrideAddr, pin_override);
  WriteData(lokmotPinAddr, pin_is_in);
  WriteData(pumpBitsAddr, pumpBits);
  WriteData(sprpumpLevAddr, CommandData.pumps.pwm2 & 0x7ff);
  WriteData(inpumpLevAddr, CommandData.pumps.pwm3 & 0x7ff);
  WriteData(outpumpLevAddr, CommandData.pumps.pwm4 & 0x7ff);
  WriteData(balOnAddr, (int)CommandData.pumps.bal_on);
  WriteData(balOffAddr, (int)CommandData.pumps.bal_off);
  WriteData(balVetoAddr, (int)CommandData.pumps.bal_veto);
  WriteData(balTargetAddr, (int)(CommandData.pumps.bal_target + 1648. * 5.));
  WriteData(balGainAddr, (int)(CommandData.pumps.bal_gain * 1000.));
  WriteData(balMinAddr, (int)CommandData.pumps.bal_min);
  WriteData(balMaxAddr,(int)CommandData.pumps.bal_max);
  WriteData(iscBitsAddr, iscBits);

}
