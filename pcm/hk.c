/* pcm: the Spider master control program
 *
 * hk.c: handle housekeeping commands and logic
 *
 * This software is copyright (C) 2011 University of Toronto
 *
 * This file is part of pcm.
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
#include <stdio.h>
#include <string.h>

#include "mcp.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"

static struct {
  double ssa;
  double fp;
  double pump;
  double cp;
  double still;
  double hsw;
} insert_temp[6];

static struct {
  double mt_bot_lo;
  double sft_bottom;
  double capillary;
} theo_temp;

/************************************************************************/
/*                                                                      */
/* PhaseStep: sweep through phase shifts for the bias channels          */
/*                                                                      */
/************************************************************************/
static void PhaseStep(struct RTDStruct *rtd)
{
  // do nothing if not enough time has passed
  if (mcp_systime(NULL) - rtd->phase_time < rtd->phase_dt) return;
  
  // increment phase if we're within limits
  if ( (rtd->phase_step > 0) ? 
       (rtd->phase < rtd->phase_end && rtd->phase >= rtd->phase_start) :
       (rtd->phase > rtd->phase_end && rtd->phase <= rtd->phase_start) ) {
    rtd->phase += rtd->phase_step;
    rtd->phase_time = mcp_systime(NULL);
  }
  
  // turn off phase step mode if we've reached the end
  if ( (rtd->phase_step > 0) ? (rtd->phase >= rtd->phase_end) : 
      (rtd->phase <= rtd->phase_end) ) rtd->do_phase_step = 0;
  
  // check if we've reached limits
  if (rtd->phase < 0) {
    rtd->phase = 0.0;
    rtd->do_phase_step = 0;
  } else if (rtd->phase > 360) {
    rtd->phase = 360.0;
    rtd->do_phase_step = 0;
  }
}

/************************************************************************/
/*                                                                      */
/* PhaseControl: set phase shifts for the bias channels                 */
/*                                                                      */
/************************************************************************/
static void PhaseControl()
{
  static struct NiosStruct* phaseCnxAddr[6];
  static struct NiosStruct* phaseNtdAddr[6];
  char field[20];
  int i;

  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    for(i = 0; i < 6; i++) {
      sprintf(field, "ph_cnx_x%1d_hk", i+1);
      phaseCnxAddr[i] = GetNiosAddr(field);
      sprintf(field, "ph_ntd_x%1d_hk", i+1);
      phaseNtdAddr[i] = GetNiosAddr(field);
    }
  }	

  for(i = 0; i < 6; i++) {
    if (CommandData.hk[i].cernox.do_phase_step)
      PhaseStep(&CommandData.hk[i].cernox);
    if (CommandData.hk[i].ntd.do_phase_step)
      PhaseStep(&CommandData.hk[i].ntd);
    
    WriteCalData(phaseCnxAddr[i], CommandData.hk[i].cernox.phase, NIOS_QUEUE);
    WriteCalData(phaseNtdAddr[i], CommandData.hk[i].ntd.phase, NIOS_QUEUE);
  }
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Amplitude of HK DAC signals (bias and heat)           */
/*                                                                      */
/************************************************************************/

// FP temperature (K) above which to automatically bias cernoxes to full
#define T_FP_FULL_BIAS  2.2
// FP temperature (K) below which to use commnded cernox bias
#define T_FP_COMM_BIAS  2.0
// timeout on full bias scale changes (in slow frames)
#define FULL_BIAS_TIMEOUT 100

#define VETO_MCE_TIMEOUT 30
static int veto_mce_veto[6] = {VETO_MCE_TIMEOUT, VETO_MCE_TIMEOUT,
  VETO_MCE_TIMEOUT, VETO_MCE_TIMEOUT, VETO_MCE_TIMEOUT, VETO_MCE_TIMEOUT};

static void BiasControl()
{
  static struct NiosStruct* vCnxAddr[6];
  static struct NiosStruct* vNtdAddr[6];
  static struct NiosStruct* fBiasCmdHkAddr;
  static struct NiosStruct* fBiasHkAddr;
  char field[20];
  int i;
  unsigned short period;

  double t_fp;

  static int full_bias_timeout[6];

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (i=0; i<6; i++) {
      sprintf(field, "v_cnx_x%1d_hk", i+1);
      vCnxAddr[i] = GetNiosAddr(field);
      sprintf(field, "v_ntd_x%1d_hk", i+1);
      vNtdAddr[i] = GetNiosAddr(field);
      full_bias_timeout[i] = FULL_BIAS_TIMEOUT;
    }
    fBiasCmdHkAddr = GetNiosAddr("f_bias_cmd_hk");
    fBiasHkAddr = GetNiosAddr("f_bias_hk");
  }

  for (i=0; i<6; i++) {
    if (full_bias_timeout[i] <= 0) {
      t_fp = insert_temp[i].fp;
      if (i== 4) t_fp = 0;    //FIXME X5 autobias disabled
      if ( t_fp > T_FP_FULL_BIAS ) {
        if (!CommandData.hk[i].cernox_full_bias) {
//          bprintf(info, "Biasing X%i cernoxes full (FP:%.1f)\n", i+ 1, t_fp);
          CommandData.hk[i].cernox_full_bias = 1;
          veto_mce_veto[i] = VETO_MCE_TIMEOUT;
          full_bias_timeout[i] = FULL_BIAS_TIMEOUT;
        }
      }

      if ( t_fp < T_FP_COMM_BIAS ) {
        if (CommandData.hk[i].cernox_full_bias) {
//          bprintf(info, "Restoring X%i cernox bias (FP:%.1f)\n", i+ 1, t_fp);
          CommandData.hk[i].cernox_full_bias = 0;
          veto_mce_veto[i] = VETO_MCE_TIMEOUT;
          full_bias_timeout[i] = FULL_BIAS_TIMEOUT;
        }
      }
    } else full_bias_timeout[i]--;

    if (CommandData.hk[i].cernox_full_bias) {
      WriteCalData(vCnxAddr[i], 5.0, NIOS_QUEUE);
    } else {
      WriteCalData(vCnxAddr[i], CommandData.hk[i].cernox.ampl, NIOS_QUEUE);
    }
    WriteCalData(vNtdAddr[i], CommandData.hk[i].ntd.ampl, NIOS_QUEUE);
  }

  WriteData(fBiasCmdHkAddr, CommandData.hk_bias_freq, NIOS_QUEUE);
  //cast as short important for reporting correct period
  period = (ACSData.adc_rate)/CommandData.hk_bias_freq;
  WriteCalData(fBiasHkAddr, (ACSData.adc_rate)/period, NIOS_QUEUE);
}

/************************************************************************/
/*                                                                      */
/*   FridgeCycle: auto-cycle the He3 fridge. Overrides pump/hs controls */
/*                                                                      */
/************************************************************************/

/* bit positions of hk pwm heaters */
#define HK_PWM_PUMP   0x01
#define HK_PWM_HSW    0x02
#define HK_PWM_HTR2   0x04
#define HK_PWM_HTR1   0x08
#define HK_PWM_SSA    0x10
#define HK_PWM_FPHI   0x20
#define HK_PWM_HTR3   0x40
#define HK_SERVO_PUMP 0x80

/* wait this many slow frames before starting to run fridge cycle */
#define FRIDGE_CYCLE_START_WAIT 50

/* state machine states */
#define CYCLE_COLD        0x0001
#define CYCLE_SFT_BOIL    0x0002
#define CYCLE_FP_BAKE     0x0004
#define CYCLE_BAKE_SETTLE 0x0008
#define CYCLE_HSW_OFF     0x0010
#define CYCLE_PUMP_HEAT   0x0020
#define CYCLE_SETTLE      0x0040
#define CYCLE_COOL        0x0080
#define CYCLE_OUT         0x0100

/* extra state bits used for servoing */
#define CYCLE_PUMP_ON     0x1000
#define CYCLE_FP_ON       0x2000
#define CYCLE_SFT_ON      0x4000
#define CYCLE_CAP_ON      0x8000
#define CYCLE_STATE_MASK  0x0fff

/* temperature limits for the auto-cycle */
//TODO is this 4K dry limit reasonable in Theo?
#define T_4K_MAX      10.000      /* above this, LHe gone. Do nothing */
//#define T_STILL_MAX   2.500     /* stop heating pump if still gets too hot */
// TODO do I want to have T_STILL_MAX again? commandable?

/* parameters for capillary/SFT servo loop */
#define SFT_SERVO_DT 30    /* seconds between power updates */
#define SFT_SERVO_DP 0.05  /* power (W) to add or subtract */

//NB: insert numbered from 0
static unsigned short FridgeCycle(int insert, int reset)
{
  static int firsttime[6] = {1, 1, 1, 1, 1, 1};
  static struct NiosStruct*    stimeCycleWAddr[6];
  static struct BiPhaseStruct* stimeCycleRAddr[6];
  static struct NiosStruct*    stateCycleWAddr[6];
  static struct BiPhaseStruct* stateCycleRAddr[6];

  double t_4k, t_pump, t_fp, t_sft, t_capillary;
  double duty_cycle;

  time_t state_elapsed;
  unsigned short cycle_state, next_state;
  unsigned short heat_pump, heat_hsw, heat_fp, servo_pump;
  unsigned short retval = 0;

  if (firsttime[insert]) {
    char field[64];
    firsttime[insert] = 0; 
    sprintf(field, "state_x%1d_cycle", insert+1);
    stateCycleWAddr[insert] = GetNiosAddr(field);
    stateCycleRAddr[insert] = ExtractBiPhaseAddr(stateCycleWAddr[insert]);
    WriteData(stateCycleWAddr[insert], CYCLE_OUT, NIOS_QUEUE);
    sprintf(field, "time_state_x%1d_cycle", insert+1);
    stimeCycleWAddr[insert] = GetNiosAddr(field);
    stimeCycleRAddr[insert] = ExtractBiPhaseAddr(stimeCycleWAddr[insert]);
  }

  if (reset) {
    WriteData(stateCycleWAddr[insert], CYCLE_OUT, NIOS_QUEUE);
    return 0;
  }

  /* get current cycle state */
  state_elapsed = mcp_systime(NULL) - ReadData(stimeCycleRAddr[insert]);
  cycle_state = ReadData(stateCycleRAddr[insert]);
  next_state = cycle_state;   // stay the same unless told otherwise

  t_4k = theo_temp.mt_bot_lo;
  t_sft = theo_temp.sft_bottom;
  t_capillary = theo_temp.capillary;
  t_pump = insert_temp[insert].pump;
  t_fp = insert_temp[insert].fp;

  /****************** figure out next_state of finite state machine */

  /* do nothing if out of liquid helium -> OUT */
  if (t_4k > T_4K_MAX) {
    next_state = CYCLE_OUT;
    if (cycle_state != CYCLE_OUT)
      if (insert == 0) {
        bprintf(info, "Auto Cycle: LHe is DRY!\n");
      }
  }

  /* COLD: start a cycle when FP too hot (or forced by command)
   * -> SFT_BOIL, with FP_ON to start */
  else if (cycle_state & CYCLE_COLD) {
    /* TODO for now, only allow forced cycle starts. Use X1's force flag
    if ((t_fp > CommandData.hk[insert].cycle.t_fp_warm)
      || CommandData.hk[insert].cycle.force) {
      */
    if (CommandData.hk[insert].cycle.force) {
      CommandData.hk[insert].cycle.force = 0;
      next_state = CYCLE_SFT_BOIL | CYCLE_FP_ON;
      if (insert == 0) {
        bprintf(info, "Auto Cycle: Starting SFT boil-off.");
      }
    }
  }

  /* SFT_BOIL: heat SFT and capillaries until sft empty
   * -> SFT_BAKE, with FP_ON in existing state
   * Also servos focal planes */
  else if (cycle_state & CYCLE_SFT_BOIL) {
    if (t_sft > CommandData.burp_cycle.t_empty_sft
       || state_elapsed > CommandData.burp_cycle.boil_timeout) {
      next_state = CYCLE_FP_BAKE | (next_state & CYCLE_FP_ON)
        | CYCLE_SFT_ON | CYCLE_CAP_ON;
      if (insert == 0) {
        bprintf(info, "Auto Cycle: SFT boiled. Continuing with FP bake.");
      }
    }
  }

  /* FP_BAKE: servo the FP, SFT, and capillary temperatures until MT cold
   * -> BAKE_SETTLE */
  else if (cycle_state & CYCLE_FP_BAKE) {
    if (t_4k < CommandData.burp_cycle.t_mt_cool
       || state_elapsed > CommandData.burp_cycle.bake_timeout) {
      next_state = CYCLE_BAKE_SETTLE;
      if (insert == 0) {
        bprintf(info, "Auto Cycle: Bake completed. Settling");
      }
    }
  }

  /* BAKE_SETTLE: wait for things to settle after baking
   * -> HSW_OFF */
  else if (cycle_state & CYCLE_BAKE_SETTLE) {
    if (state_elapsed > CommandData.burp_cycle.settle_time) {
      next_state = CYCLE_HSW_OFF;
      bprintf(info, "Auto Cycle X%1d: Turning HSw off", insert+1);
    }
  }

  /* HSW_OFF: wait for heat switch to cool
   * -> PUMP_HEAT, starting with PUMP_ON */
  else if (cycle_state & CYCLE_HSW_OFF) {
    if (state_elapsed > CommandData.hk[insert].cycle.hsw_timeout) {
      next_state = CYCLE_PUMP_HEAT | CYCLE_PUMP_ON;
      bprintf(info, "Auto Cycle X%1d: Starting pump servo.", insert+1);
    }
  }

  /* PUMP_HEAT: heat pump (servo) until timeout
   * -> SETTLE */
  else if (cycle_state & CYCLE_PUMP_HEAT) {
    if (state_elapsed > CommandData.hk[insert].cycle.pump_timeout
        || t_fp < CommandData.hk[insert].cycle.t_fp_cool) {
      next_state = CYCLE_SETTLE;
      bprintf(info, "Auto Cycle X%1d: Stopping pump servo.", insert+1);
    }
  }

  /* SETTLE: let hot pump settle until timeout
   * -> COOL */
  else if (cycle_state & CYCLE_SETTLE) {
    if (state_elapsed > CommandData.hk[insert].cycle.settle_time) {
      next_state = CYCLE_COOL;
      bprintf(info, "Auto Cycle X%1d: Turning heat switch on.", insert+1);
    }
  }
  /* COOL: wait until fridge is cold
   * -> COLD */
  else if (cycle_state & CYCLE_COOL) {
    if (t_fp < CommandData.hk[insert].cycle.t_fp_cold
        || state_elapsed > CommandData.hk[insert].cycle.cool_timeout) {
      CommandData.hk[insert].cycle.force = 0; //clear pending cycles
      next_state = CYCLE_COLD;
      bprintf(info, "Auto Cycle X%1d: Complete.", insert+1);
    }
  }
  /* OUT: do nothing if out of LHe. Unless not, then be cold -> COLD */
  else if (cycle_state & CYCLE_OUT) {
    if (t_4k < T_4K_MAX) {
      next_state = CYCLE_COLD;
      bprintf(info, "Auto Cycle X%1d: Activated.", insert+1);
    }
  }
  else {
    bprintf(err, "Auto Cycle X%1d: cycle_state: %i unknown!",
        insert+1, cycle_state);
    next_state = CYCLE_COLD;
  }


  /****************** set outputs of state machine */

  /* servo the FP temperature  in SFT_BOIL and FP_BAKE states*/
  if (next_state & CYCLE_SFT_BOIL || next_state & CYCLE_FP_BAKE) {
    if (next_state & CYCLE_FP_ON) {
      if (t_fp > CommandData.burp_cycle.t_fp_hi) {
        next_state &= ~CYCLE_FP_ON;
        bprintf(info, "Auto Cycle X%1d: Turning off FP heat", insert+1);
      }
    } else {
      if (t_fp < CommandData.burp_cycle.t_fp_lo) {
        next_state |= CYCLE_FP_ON;
        bprintf(info, "Auto Cycle X%1d: Turning on FP heat", insert+1);
      }
    }
  }

  /* servo the Pump temperature in PUMP_HEAT state */
  if (next_state & CYCLE_PUMP_HEAT) {
    if (next_state & CYCLE_PUMP_ON) {
      if (t_pump > CommandData.hk[insert].cycle.t_pump_hi) {
        next_state &= ~CYCLE_PUMP_ON;
        bprintf(info, "Auto Cycle X%1d: Turning off pump heat", insert+1);
      }
    } else {
      if (t_pump < CommandData.hk[insert].cycle.t_pump_lo) {
        next_state |= CYCLE_PUMP_ON;
        bprintf(info, "Auto Cycle X%1d: Turning on pump heat", insert+1);
      }
    }
  }

  if (insert == 0) {
    /* control the SFT and capillary heaters, during boil and bake */

    /* SFT and capillary should default to off */
    /* hk_capillary_heat_off */
    CommandData.hk_theo_heat[2].state = 0;
    CommandData.hk_theo_heat[2].duration = 0;
    CommandData.hk_theo_heat[2].start_time = 0;
    /* hk_sft_bottom_heat_off */
    CommandData.hk_theo_heat[6].state = 0;
    CommandData.hk_theo_heat[6].duration = 0;
    CommandData.hk_theo_heat[6].start_time = 0;

    if (next_state & CYCLE_SFT_BOIL) {
      /* during boil, use PWM to set constant power */
      /* hk_capillary_pulse */
      duty_cycle = CommandData.burp_cycle.p_cap_boil/HK_CAPILLARY_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      if (duty_cycle<0) duty_cycle = 0.0;
      CommandData.hk_theo_heat[2].duty_target = ((int)(duty_cycle*256) << 8);
      CommandData.hk_theo_heat[2].duration = -1;
      if (!(cycle_state & CYCLE_SFT_BOIL)) {
        /* initialize on start of new state */
        CommandData.hk_theo_heat[2].duty_avg =
          CommandData.hk_theo_heat[2].duty_target;
      }

      /* hk_sft_bottom_pulse */
      duty_cycle = CommandData.burp_cycle.p_sft_boil/HK_SFT_BOTTOM_PMAX;
      if (duty_cycle>=1) duty_cycle = 0.999;
      if (duty_cycle<0) duty_cycle = 0.0;
      CommandData.hk_theo_heat[6].duty_target = ((int)(duty_cycle*256) << 8);
      CommandData.hk_theo_heat[6].duration = -1;
      if (!(cycle_state & CYCLE_SFT_BOIL)) {
        /* initialize on start of new state */
        CommandData.hk_theo_heat[6].duty_avg =
          CommandData.hk_theo_heat[6].duty_target;
      }

    } else if (next_state & CYCLE_FP_BAKE) {
      /* during bake, servo the temperature */
      if (next_state & CYCLE_SFT_ON) {
        if (t_sft > CommandData.burp_cycle.t_sft_bake_hi) {
          next_state &= ~CYCLE_SFT_ON;
          bprintf(info, "Auto Cycle: Turning off SFT heat");
        }
      } else {
        if (t_sft < CommandData.burp_cycle.t_sft_bake_lo) {
          next_state |= CYCLE_SFT_ON;
          bprintf(info, "Auto Cycle: Turning on SFT heat");
        }
      }

      if (next_state & CYCLE_CAP_ON) {
        if (t_capillary > CommandData.burp_cycle.t_cap_bake_hi) {
          next_state &= ~CYCLE_CAP_ON;
          bprintf(info, "Auto Cycle: Turning off capillary heat");
        }
      } else {
        if (t_capillary < CommandData.burp_cycle.t_cap_bake_lo) {
          next_state |= CYCLE_CAP_ON;
          bprintf(info, "Auto Cycle: Turning on capillary heat");
        }
      }

      if (next_state & CYCLE_SFT_ON) CommandData.hk_theo_heat[6].state = 1;
      if (next_state & CYCLE_CAP_ON) CommandData.hk_theo_heat[2].state = 1;

    }
  }

  /* set per-insert outputs of finite state machine */
  if (next_state & CYCLE_COLD || next_state & CYCLE_OUT) {
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  } else if (next_state & CYCLE_SFT_BOIL) {
    heat_fp = (next_state & CYCLE_FP_ON) ? 1 : 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  } else if (next_state & CYCLE_FP_BAKE) {
    heat_fp = (next_state & CYCLE_FP_ON) ? 1 : 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  } else if (next_state & CYCLE_BAKE_SETTLE) {
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  } else if (next_state & CYCLE_HSW_OFF) {
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 0;
    servo_pump = 0;
  } else if (next_state & CYCLE_PUMP_HEAT) {
    heat_fp = 0;
    heat_pump = (next_state & CYCLE_PUMP_ON) ? 1 : 0;
    heat_hsw = 0;
    servo_pump = 1;
  } else if (next_state & CYCLE_SETTLE) {
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 0;
    servo_pump = 0;
  } else if (next_state & CYCLE_COOL) {
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  } else {
    // catch bad state
    heat_fp = 0;
    heat_pump = 0;
    heat_hsw = 1;
    servo_pump = 0;
  }

  /* set the heater control bits in the output, as needed */
  if (heat_fp)   retval |= HK_PWM_FPHI;
  if (heat_pump) retval |= HK_PWM_PUMP;
  if (servo_pump) retval |= HK_SERVO_PUMP;
  if (!heat_hsw) retval |= HK_PWM_HSW;  /* inverted logic because normally on */

  /* update state time when state changes */
  if ((next_state & CYCLE_STATE_MASK) != (cycle_state & CYCLE_STATE_MASK)) {
    WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
  }

  /* store next state in bus */
  WriteData(stateCycleWAddr[insert], next_state, NIOS_QUEUE);

  return retval;
}

/************************************************************************/
/*                                                                      */
/*   PulseHeater: pulse PWM heater. Overrides heater controls           */
/*                                                                      */
/************************************************************************/
static void PulseHeater(struct PWMStruct *heater) {
  
  time_t elapsed;
  if (heater->duration >= 0) elapsed = mcp_systime(NULL) - heater->start_time;
  else elapsed = 0;

  // turn off pulse mode and turn off heater if pulse mode ended
  if (elapsed >= heater->duration && heater->duration > 0) {
    heater->state = 0;
    heater->duration = 0;
    heater->start_time = 0;
    return;
  }
  
  // do nothing if total time has elapsed or pulse mode is off
  if (elapsed >= heater->duration && heater->duration >= 0) return;

  // update state and increment running average
  // average duty cycle is exact on periods of 256 frames
  if (heater->duty_avg < heater->duty_target) {
    heater->state = 1;
    heater->duty_avg += -(heater->duty_avg >> 8) + 0x00ff;
  } else {
    heater->state = 0;
    heater->duty_avg -= (heater->duty_avg >> 8);
  }
}

/************************************************************************/
/*                                                                      */
/*   PumpServo: maintain pump temperature between set points.           */
/*   Overrides pump controls                                            */
/*                                                                      */
/************************************************************************/
static void PumpServo(int insert)
{
  static int servo_ctr[6] = {0, 0, 0, 0, 0, 0};
  double t_pump;
  
  if (servo_ctr[insert] <= 0) {
    t_pump = insert_temp[insert].pump;
    
    /* figure out next pump heater state */
    if (t_pump > CommandData.hk[insert].pump_servo_high) {
      if (CommandData.hk[insert].pump_heat == 1) servo_ctr[insert] = 3;
      CommandData.hk[insert].pump_heat = 0;
    } else if (t_pump < CommandData.hk[insert].pump_servo_low) {
      if (CommandData.hk[insert].pump_heat == 0) servo_ctr[insert] = 3;
      CommandData.hk[insert].pump_heat = 1;
    }
  } else servo_ctr[insert]--;
}


/************************************************************************/
/*                                                                      */
/*   HeatControl: Switching logic for the PWM (digital) heaters         */
/*                                                                      */
/************************************************************************/
static void HeatControl()
{
  static struct NiosStruct* heat13Addr;
  static struct NiosStruct* heat45Addr;
  static struct NiosStruct* heat26Addr;
  static struct NiosStruct* heatTAddr;
  static struct NiosStruct* heatRingAddr[6];
  static struct NiosStruct* heatFploAddr[6];

  static int fridge_start_wait = FRIDGE_CYCLE_START_WAIT;

  int i;
  unsigned short temp;
  unsigned short bits[6];
  char buf[16];

  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    heat13Addr = GetNiosAddr("heat_13_hk");
    heat45Addr = GetNiosAddr("heat_45_hk");
    heat26Addr = GetNiosAddr("heat_26_hk");
    heatTAddr = GetNiosAddr("heat_t_hk");
    for (i=0; i<6; i++) {
      sprintf(buf, "heat_ring_x%1d_hk", i+1);
      heatRingAddr[i] = GetNiosAddr(buf);
    }
    for (i=0; i<6; i++) {
      sprintf(buf, "heat_fplo_x%1d_hk", i+1);
      heatFploAddr[i] = GetNiosAddr(buf);
    }
  }	

  if (fridge_start_wait > 0) fridge_start_wait--;

  //PWM heaters
  for (i=0; i<6; i++) {
    bits[i] = 0;
    if (CommandData.hk[i].cycle.enabled && fridge_start_wait <= 0) {
      //using auto cycle, get PUMP, HSW and FPhi bits from fridge control
      bits[i] |= FridgeCycle(i, 0);
    } else {
      //not using auto cycle. Command PUMP and HSW manually
      FridgeCycle(i, 1);  //reset cycle state
      //servo pump temperature
      if (CommandData.hk[i].pump_servo_on) {
	PumpServo(i);
	bits[i] |= HK_SERVO_PUMP;
      }
      if (CommandData.hk[i].pump_heat) bits[i] |= HK_PWM_PUMP;
      //NB: heat switch is normally closed, so logic inverted
      if (!CommandData.hk[i].heat_switch) bits[i] |= HK_PWM_HSW;
      //pulse fphi heater
      if (CommandData.hk[i].fphi_heat) {
        if (CommandData.hk[i].fphi_heat > 0) CommandData.hk[i].fphi_heat--;
        bits[i] |= HK_PWM_FPHI;
      }
    }
    if (CommandData.hk[i].ssa_heat) bits[i] |= HK_PWM_SSA;
    if (CommandData.hk[i].htr1_heat) bits[i] |= HK_PWM_HTR1;
    if (CommandData.hk[i].htr2_heat) bits[i] |= HK_PWM_HTR2;
    if (CommandData.hk[i].htr3_heat) bits[i] |= HK_PWM_HTR3;
  }
  temp = ((bits[0] & 0xff) << 8) | (bits[2] & 0xff);
  WriteData(heat13Addr, temp, NIOS_QUEUE);
  temp = ((bits[3] & 0xff) << 8) | (bits[4] & 0xff);
  WriteData(heat45Addr, temp, NIOS_QUEUE);
  temp = ((bits[1] & 0xff) << 8) | (bits[5] & 0xff);
  WriteData(heat26Addr, temp, NIOS_QUEUE);

  //Theo PWM heaters
  temp = 0x0000;
  for (i=0; i<8; i++) {
    PulseHeater(&CommandData.hk_theo_heat[i]);
    if (CommandData.hk_theo_heat[i].state) temp |= (0x1 << i);
  }
  WriteData(heatTAddr, temp, NIOS_QUEUE);

  //DAC heaters
  for (i=0; i<6; i++) {
    WriteCalData(heatRingAddr[i], CommandData.hk[i].ring_heat, NIOS_QUEUE);
    WriteCalData(heatFploAddr[i], CommandData.hk[i].fplo_heat, NIOS_QUEUE);
  }
}

//limit switch positions. NB: active low
#define SFT_LIM_OP_ATM  0x0001
#define SFT_LIM_CL_ATM  0x0004
#define SFT_LIM_OP_PUMP 0x0010
#define SFT_LIM_CL_PUMP 0x0040

//command bits to open or close valves
#define SFT_CTL_OPEN_ATM    0x0100
#define SFT_CTL_CLOSE_ATM   0x0400
#define SFT_CTL_OPEN_PUMP   0x1000
#define SFT_CTL_CLOSE_PUMP  0x4000

void SFTValveMotors()
{
  //TODO handshake with DSP to prevent erroneous SFT open/close commands
  static struct NiosStruct* ctlSftvAddr;
  static struct BiPhaseStruct* limSftvAddr;
  static struct NiosStruct* stateSftvAddr;
  static int firsttime = 1;
  unsigned int limits;
  unsigned int sft_ctl_bits = 0;

  if (firsttime) {
    firsttime = 0;
    ctlSftvAddr = GetNiosAddr("ctl_sftv");
    limSftvAddr = GetBiPhaseAddr("lim_sftv");
    stateSftvAddr = GetNiosAddr("state_sftv");
  }

  //set the control bits as per mode/goal
  switch (CommandData.sftv.goal_atm) {
    case sft_do_open:
      sft_ctl_bits |= SFT_CTL_OPEN_ATM;
      break;
    case sft_do_close:
      sft_ctl_bits |= SFT_CTL_CLOSE_ATM;
      break;
    case sft_do_nothing:
    default:
      sft_ctl_bits &= ~(SFT_CTL_OPEN_ATM | SFT_CTL_CLOSE_ATM);
      break;
  }

  switch (CommandData.sftv.goal_pump) {
    case sft_do_open:
      sft_ctl_bits |= SFT_CTL_OPEN_PUMP;
      break;
    case sft_do_close:
      sft_ctl_bits |= SFT_CTL_CLOSE_PUMP;
      break;
    case sft_do_nothing:
    default:
      sft_ctl_bits &= ~(SFT_CTL_OPEN_PUMP | SFT_CTL_CLOSE_PUMP);
      break;
  }

  WriteData(ctlSftvAddr, sft_ctl_bits, NIOS_QUEUE);

  //examine limit switches. Do nothing when lock is off (switches unpowered)
  //also ignore case when both switches asserted (disconnect/aphysical)
  //NB: limit switches are asserted low
  limits = ReadData(limSftvAddr);

  if ( !(limits & SFT_LIM_CL_ATM) && (limits & SFT_LIM_OP_ATM) )
    CommandData.sftv.state_atm = sft_closed;
  else if ( (limits & SFT_LIM_CL_ATM) && !(limits & SFT_LIM_OP_ATM) )
    CommandData.sftv.state_atm = sft_open;
  else if ( (limits & SFT_LIM_CL_ATM) && (limits & SFT_LIM_OP_ATM) ) {
    if (sft_ctl_bits == SFT_CTL_CLOSE_ATM)
      CommandData.sftv.state_atm = sft_closing;
    else if (sft_ctl_bits == SFT_CTL_OPEN_ATM)
      CommandData.sftv.state_atm = sft_opening;
  }

  if ( !(limits & SFT_LIM_CL_PUMP) && (limits & SFT_LIM_OP_PUMP) )
    CommandData.sftv.state_pump = sft_closed;
  else if ( (limits & SFT_LIM_CL_PUMP) && !(limits & SFT_LIM_OP_PUMP) )
    CommandData.sftv.state_pump = sft_open;
  else if ( (limits & SFT_LIM_CL_PUMP) && (limits & SFT_LIM_OP_PUMP) ) {
    if (sft_ctl_bits == SFT_CTL_CLOSE_PUMP)
      CommandData.sftv.state_pump = sft_closing;
    else if (sft_ctl_bits == SFT_CTL_OPEN_PUMP)
      CommandData.sftv.state_pump = sft_opening;
  }

  WriteData(stateSftvAddr, (CommandData.sftv.goal_atm & 0x000f)
      | ((CommandData.sftv.state_atm & 0x000f) << 4)
      | ((CommandData.sftv.goal_pump & 0x000f) << 8)
      | ((CommandData.sftv.state_pump & 0x000f) << 12)
      , NIOS_QUEUE);
}

#ifndef LUT_DIR
#define LUT_DIR "/data/etc/spider/"
#endif

#define T_FP_FIR_LEN 100
#define T_STILL_FIR_LEN 100

void GetHKTemperatures(int do_slow, int do_init)
{
  static struct BiPhaseStruct* vCnxAddr[6];
  static struct BiPhaseStruct* tSsaAddr[6];
  static struct BiPhaseStruct* tFpAddr[6]; 
  static struct BiPhaseStruct* tPumpAddr[6];
  static struct BiPhaseStruct* tCpAddr[6];
  static struct BiPhaseStruct* tStillAddr[6];
  static struct BiPhaseStruct* tHswAddr[6];

  static struct BiPhaseStruct* tMtBotLoAddr;
  static struct BiPhaseStruct* tSftBottomAddr;
  static struct BiPhaseStruct* tCapillaryAddr;
  
  static struct LutType rFpLut = {.filename = LUT_DIR "r_cernox.lut"};

  static struct LutType tSsaLut[6] = {
    {.filename = LUT_DIR "D84461.lut"},
    {.filename = LUT_DIR "d_curve10.lut"},
    {.filename = LUT_DIR "d_curve10.lut"},
    {.filename = LUT_DIR "dt670.lut"},
    {.filename = LUT_DIR "d_curve10.lut"},
    {.filename = LUT_DIR "dt670.lut"}
  };

  static struct LutType tFpLut[6] = {
    {.filename = LUT_DIR "X80210.lut"},
    {.filename = LUT_DIR "X41767.lut"},
    {.filename = LUT_DIR "X58085.lut"},
    {.filename = LUT_DIR "X41468.lut"},
    {.filename = LUT_DIR "blank_fp_x5.lut"},
    {.filename = LUT_DIR "X41474.lut"}
  };

  static struct LutType tPumpLut[6] = {
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"}
  };
  static struct LutType tCpLut[6] = {
    {.filename = LUT_DIR "thelma3_cp.lut"},
    {.filename = LUT_DIR "thelma4_cp.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"}
  };
  static struct LutType tStillLut[6] = {
    {.filename = LUT_DIR "thelma3_still.lut"},
    {.filename = LUT_DIR "thelma4_still.lut"},
    {.filename = LUT_DIR "X42401.lut"},
    {.filename = LUT_DIR "thelma7_still.lut"},
    {.filename = LUT_DIR "thelma2_still.lut"},
    {.filename = LUT_DIR "thelma6_still.lut"}
  };
  static struct LutType tHswLut[6] = {
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"}
  };

  static struct LutType tMtBotLoLut = {.filename = LUT_DIR "D75551.lut"};
  static struct LutType tSftBottomLut = {.filename = LUT_DIR "D77239.lut"};
  static struct LutType tCapillaryLut = {.filename = LUT_DIR "D77232.lut"};

  /* FIR filter buffers for cernoxes. TODO remove once warm bias incresased? */
  static double t_fp_buf[6][T_FP_FIR_LEN];
  static int t_fp_ind[6] = {0, 0, 0, 0, 0, 0};
  static double t_still_buf[6][T_STILL_FIR_LEN];
  static int t_still_ind[6] = {0, 0, 0, 0, 0, 0};

  double v_cnx, t_ssa, t_fp, t_pump, t_cp, t_still, t_hsw;
  double t_mt_bot_lo, t_sft_bottom, t_capillary;
  
  int insert, i;

  static int firsttime = 1;   // for setting up FIR buffers, not lookups
  
  if (do_init) {
    char field[64];

    for (insert = 0; insert < 6; insert++) {
      sprintf(field, "v_cnx_x%1d_hk", insert+1);
      vCnxAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vd_ssa_x%1d_hk", insert+1);
      tSsaAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vr_fp_x%1d_hk", insert+1);
      tFpAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vd_pump_x%1d_hk", insert+1);
      tPumpAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vd_cp_x%1d_hk", insert+1);
      tCpAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vr_still_x%1d_hk", insert+1);
      tStillAddr[insert] = GetBiPhaseAddr(field);
      sprintf(field, "vd_hsw_x%1d_hk", insert+1);
      tHswAddr[insert] = GetBiPhaseAddr(field);

      LutInit(&tSsaLut[insert]);
      LutInit(&tFpLut[insert]);
      LutInit(&tPumpLut[insert]);
      LutInit(&tCpLut[insert]);
      LutInit(&tStillLut[insert]);
      LutInit(&tHswLut[insert]);
    }

    tMtBotLoAddr = GetBiPhaseAddr("vd_mt_botlo_t_hk");
    tSftBottomAddr = GetBiPhaseAddr("vd_sft_bottom_t_hk");
    tCapillaryAddr = GetBiPhaseAddr("vd_capillary_t_hk");

    LutInit(&rFpLut);
    LutInit(&tMtBotLoLut);
    LutInit(&tSftBottomLut);
    LutInit(&tCapillaryLut);

    return;
  }

  /******************** fast controls. sample filtered data */

  for (insert = 0; insert < 6; insert++) {
    /* read the FP themometer for filetering */
    t_fp_buf[insert][t_fp_ind[insert]] = ReadCalData(tFpAddr[insert]);
    t_still_buf[insert][t_still_ind[insert]] = ReadCalData(tStillAddr[insert]);

    /* initialise FIR */
    if (firsttime) {
      firsttime = 0; 

      for (i = 0; i < T_FP_FIR_LEN; ++i)
        t_fp_buf[insert][i] = t_fp_buf[insert][t_fp_ind[insert]];

      for (i = 0; i < T_STILL_FIR_LEN; ++i)
        t_still_buf[insert][i] = t_still_buf[insert][t_still_ind[insert]];
    }

    t_fp_ind[insert] = (t_fp_ind[insert] + 1) % T_FP_FIR_LEN;
    t_still_ind[insert] = (t_still_ind[insert] + 1) % T_STILL_FIR_LEN;
  }

  /* do everything else slowly */
  if (!do_slow) return;

  /************ slow controls. do averaging and calibration into real units */

  /* Per-insert temperatures */

  for (insert = 0; insert < 6; insert++) {
    /* compute the outer mean of filtered data */
    t_fp = 0;
    for (i = 0; i < T_FP_FIR_LEN; ++i)
      t_fp += t_fp_buf[insert][i];
    t_fp /= T_FP_FIR_LEN;

    t_still = 0;
    for (i = 0; i < T_STILL_FIR_LEN; ++i)
      t_still += t_still_buf[insert][i];
    t_still /= T_STILL_FIR_LEN;

    /* normalize cernoxes by bias voltage, and convert to resistance */
    v_cnx = ReadCalData(vCnxAddr[insert]);
    t_fp /= v_cnx;
    t_fp = LutCal(&rFpLut, t_fp);
    t_still /= v_cnx;
    t_still = LutCal(&rFpLut, t_still);

    /* Read the diode voltages */
    t_ssa = ReadCalData(tSsaAddr[insert]);
    t_pump = ReadCalData(tPumpAddr[insert]);
    t_cp = ReadCalData(tCpAddr[insert]);
    t_hsw = ReadCalData(tHswAddr[insert]);
    
    /* Look-up calibrated temperatures */
    t_ssa = LutCal(&tSsaLut[insert], t_ssa);
    t_fp = LutCal(&tFpLut[insert], t_fp);
    t_pump = LutCal(&tPumpLut[insert], t_pump);
    t_cp = LutCal(&tCpLut[insert], t_cp);
    t_still = LutCal(&tStillLut[insert], t_still);
    t_hsw = LutCal(&tHswLut[insert], t_hsw);

    /* store data in global struct */
    insert_temp[insert].ssa = t_ssa;
    insert_temp[insert].fp = t_fp;
    insert_temp[insert].pump = t_pump;
    insert_temp[insert].cp = t_cp;
    insert_temp[insert].still = t_still;
    insert_temp[insert].hsw = t_hsw;
  }

  /* Theo temperatures */

  /* Read the diode voltages */
  t_mt_bot_lo = ReadCalData(tMtBotLoAddr);
  t_sft_bottom = ReadCalData(tSftBottomAddr);
  t_capillary = ReadCalData(tCapillaryAddr);

  /* Look-up calibrated temperatures */
  t_mt_bot_lo = LutCal(&tMtBotLoLut, t_mt_bot_lo);
  t_sft_bottom = LutCal(&tSftBottomLut, t_sft_bottom);
  t_capillary = LutCal(&tCapillaryLut, t_capillary);

  /* store data in global struct */
  theo_temp.mt_bot_lo = t_mt_bot_lo;
  theo_temp.sft_bottom = t_sft_bottom;
  theo_temp.capillary = t_capillary;

}

/* veto SQUIDs if SSA or FPU temp gets too high */
void VetoMCE()
{
  int insert;
  static int timeout[6] = {0, 0, 0, 0, 0, 0};
  uint16_t bit;

  double t_fp, t_ssa;

  if (CommandData.thermveto_veto) {
    CommandData.thermveto = 0;
    return;
  }

  for (insert=0; insert<6; insert++) {
    /* wait for things to settle down, I guess...? */
    if (veto_mce_veto[insert]) {

      veto_mce_veto[insert]--;
      if (timeout[insert] > 0)
        timeout[insert]--;

      continue;
    }

    bit = 1U << insert;

    /* get temperatures from central resource */
    t_fp = insert_temp[insert].fp;
    t_ssa = insert_temp[insert].ssa;

    /* XXX X5 hack */
    if (insert == 4)
      t_fp = t_ssa = 0;

    if ( ((t_fp > 8.0) || (t_ssa > 8.0)) ) {
      if (~CommandData.thermveto & bit) {
        bprintf(info, "Vetoing X%i for thermal reasons (FP:%.1f SSA:%.1f)\n",
            insert + 1, t_fp, t_ssa);
        CommandData.thermveto |= bit;
        timeout[insert] = VETO_MCE_TIMEOUT;
      }
    }

    if (timeout[insert] == 0) {
      if ( (t_fp < 7.0) && (t_ssa < 7.0) ) {
        if (CommandData.thermveto & bit) {
          bprintf(info,
              "Unvetoing X%i for thermal reasons (FP:%.1f SSA:%.1f)\n",
              insert + 1, t_fp, t_ssa);
          CommandData.thermveto &= ~bit;
          timeout[insert] = VETO_MCE_TIMEOUT;
        }
      }
    } else
      timeout[insert]--;
  }
} 

void HouseKeeping(int do_slow)
{
  static struct NiosStruct* insertLastHkAddr;
  static struct NiosStruct* vHeatLastHkAddr;
  static int first_time = 1;

  // hack. number of fast frames to wait for good data
  static int startup_timeout = 300;

  if (first_time) {
    first_time = 0;
    insertLastHkAddr = GetNiosAddr("insert_last_hk");
    vHeatLastHkAddr = GetNiosAddr("v_heat_last_hk");
    GetHKTemperatures(0, 1);
  }

  if (startup_timeout-- > 0) return;

  GetHKTemperatures(do_slow, 0);

  // do everything else slowly
  if (!do_slow) return;

#if 0
  static int print_now = 0;
  int i;

  if (print_now-- <= 0) {
    print_now = 150;
    bprintf(info, "erase me: temperatures\n");
    for (i=0; i<6; i++) {
      bprintf(info, "X%1d: fp %.2f, pump %.2f, ssa %.2f\n", i+1,
          insert_temp[i].fp, insert_temp[i].pump, insert_temp[i].ssa);
    }
    bprintf(info, "Theo: mt %.2f sft %.2f capillary %.2f\n",
        theo_temp.mt_bot_lo, theo_temp.sft_bottom, theo_temp.capillary);
  }
#endif

  BiasControl();
  PhaseControl();
  HeatControl();
  VetoMCE();

  WriteData(insertLastHkAddr, CommandData.hk_last, NIOS_QUEUE);
  WriteCalData(vHeatLastHkAddr, CommandData.hk_vheat_last, NIOS_QUEUE);
}

