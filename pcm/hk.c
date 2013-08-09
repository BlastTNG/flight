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
static void BiasControl()
{
  static struct NiosStruct* vCnxAddr[6];
  static struct NiosStruct* vNtdAddr[6];
  static struct NiosStruct* fBiasCmdHkAddr;
  static struct NiosStruct* fBiasHkAddr;
  char field[20];
  int i;
  unsigned short period;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (i=0; i<6; i++) {
      sprintf(field, "v_cnx_x%1d_hk", i+1);
      vCnxAddr[i] = GetNiosAddr(field);
      sprintf(field, "v_ntd_x%1d_hk", i+1);
      vNtdAddr[i] = GetNiosAddr(field);
    }
    fBiasCmdHkAddr = GetNiosAddr("f_bias_cmd_hk");
    fBiasHkAddr = GetNiosAddr("f_bias_hk");
  }

  //otherwise need to change scaling in tx_struct
  for (i=0; i<6; i++) {
    WriteCalData(vCnxAddr[i], CommandData.hk[i].cernox.ampl, NIOS_QUEUE);
    WriteCalData(vNtdAddr[i], CommandData.hk[i].ntd.ampl, NIOS_QUEUE);
  }
  WriteData(fBiasCmdHkAddr, CommandData.hk_bias_freq, NIOS_QUEUE);
  period = (ACSData.adc_rate)/CommandData.hk_bias_freq; //cast as short important here!
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

/* wait this many slow frames before starting to run fridge cycle */
#define FRIDGE_CYCLE_START_WAIT 50
/* update fridge cycle state once per this many slow frames. At least 6 */
#define FRIDGE_CYCLE_WAIT       10

/* state machine states */
#define CRYO_CYCLE_OUT        0x0000
#define CRYO_CYCLE_COLD       0x0001
#define CRYO_CYCLE_HSW_OFF    0x0010
#define CRYO_CYCLE_ON_HEAT    0x0002
#define CRYO_CYCLE_ON_SETTLE  0x0004
#define CRYO_CYCLE_COOL       0x0008

/* auto-cycle timeouts (all in seconds from start of cycle) */
#define CRYO_CYCLE_HSW_TIMEOUT    (5*60)
#define CRYO_CYCLE_HEAT_TIMEOUT   (35*60)
#define CRYO_CYCLE_SET_TIMEOUT    (1.5*60*60)
#define CRYO_CYCLE_COOL_TIMEOUT   (2*60*60)

/* minimum times for each state (in seconds from start of state) */
#define CRYO_CYCLE_HEAT_MIN       (20*60) // time that pump is above minimum
#define CRYO_CYCLE_SET_MIN        (5*60) // time between pump off and hsw on

/* temperature limits/control points for the auto-cycle */
#define T_4K_MAX      5.000     /* above this, LHe gone. Do nothing */
#define T_STILL_HOT   0.600     /* above this, start a cycle */
#define T_HSW_COLD    10.000    /* don't turn on pump until hsw this cold */
#define T_STILL_MAX   2.500     /* stop heating pump if still gets too hot */
#define T_PUMP_MAX    38.000    /* stop heating pump when it gets too hot */
#define T_PUMP_MIN    30.000    /* above this, pump is hot enough */
#define T_CP_SET      1.800     /* turn hsw on after settling below this */
#define T_STILL_COLD  0.350     /* below this, cycle is complete */

#ifndef LUT_DIR
#define LUT_DIR "/data/etc/spider/"
#endif

//NB: insert numbered from 0
static unsigned short FridgeCycle(int insert, int reset)
{
  static int firsttime[6] = {1, 1, 1, 1, 1, 1};
  static int firsttime_4k = 1;
  static struct BiPhaseStruct* t4kAddr;
  static struct BiPhaseStruct* tCpAddr[6];
  static struct BiPhaseStruct* tPumpAddr[6];
  static struct BiPhaseStruct* tStillAddr[6];
  static struct BiPhaseStruct* tHswAddr[6];
  static struct BiPhaseStruct* vCnxAddr[6];
  static struct NiosStruct*    startCycleWAddr[6];
  static struct BiPhaseStruct* startCycleRAddr[6];
  static struct NiosStruct*    stimeCycleWAddr[6];
  static struct BiPhaseStruct* stimeCycleRAddr[6];
  static struct NiosStruct*    stateCycleWAddr[6];
  static struct BiPhaseStruct* stateCycleRAddr[6];

  static struct LutType t4kLut = {.filename = LUT_DIR "D75551.lut"};
  static struct LutType tCpLut[6] = {
    {.filename = LUT_DIR "thelma3_cp.lut"},
    {.filename = LUT_DIR "thelma4_cp.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"}
  };
  static struct LutType tPumpLut[6] = {
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
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

  static struct LutType rStillLut = 
  {.filename = LUT_DIR "r_cernox.lut"};
  

  double t_4k, t_cp, t_pump, t_still, t_hsw, v_cnx;

  time_t start_time, state_time;
  unsigned short cycle_state, next_state;
  unsigned short heat_pump, heat_hsw;
  unsigned short retval = 0;

  if (firsttime[insert]) {
    char field[64];
    firsttime[insert] = 0; 
    if (firsttime_4k) {
      firsttime_4k = 0;
      sprintf(field, "vd_mt_botlo_t_hk");
      t4kAddr = GetBiPhaseAddr(field);
      LutInit(&t4kLut);
    }
    sprintf(field, "vd_cp_x%1d_hk", insert+1);
    tCpAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "vd_pump_x%1d_hk", insert+1);
    tPumpAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "vr_still_x%1d_hk", insert+1);
    tStillAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "vd_hsw_x%1d_hk", insert+1);
    tHswAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "v_cnx_x%1d_hk", insert+1);
    vCnxAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "time_start_x%1d_cycle", insert+1);
    startCycleWAddr[insert] = GetNiosAddr(field);
    startCycleRAddr[insert] = ExtractBiPhaseAddr(startCycleWAddr[insert]);
    sprintf(field, "state_x%1d_cycle", insert+1);
    stateCycleWAddr[insert] = GetNiosAddr(field);
    stateCycleRAddr[insert] = ExtractBiPhaseAddr(stateCycleWAddr[insert]);
    WriteData(stateCycleWAddr[insert], CRYO_CYCLE_OUT, NIOS_QUEUE);
    sprintf(field, "time_state_x%1d_cycle", insert+1);
    stimeCycleWAddr[insert] = GetNiosAddr(field);
    stimeCycleRAddr[insert] = ExtractBiPhaseAddr(stimeCycleWAddr[insert]);

    LutInit(&tCpLut[insert]);
    LutInit(&tPumpLut[insert]);
    LutInit(&tStillLut[insert]);
    LutInit(&tHswLut[insert]);
    LutInit(&rStillLut);
  }

  if (reset) {
    WriteData(stateCycleWAddr[insert], CRYO_CYCLE_OUT, NIOS_QUEUE);
    return 0;
  }

  /* get current cycle state */
  start_time = ReadData(startCycleRAddr[insert]);
  state_time = ReadData(stimeCycleRAddr[insert]);
  cycle_state = ReadData(stateCycleRAddr[insert]);

  /* Read voltages for all the thermometers of interest */
  t_4k = ReadCalData(t4kAddr);
  t_cp = ReadCalData(tCpAddr[insert]);
  t_pump = ReadCalData(tPumpAddr[insert]);
  t_still = ReadCalData(tStillAddr[insert]);
  t_hsw = ReadCalData(tHswAddr[insert]);
  v_cnx = ReadCalData(vCnxAddr[insert]);

  /* normalize the cernox readings with bias level */
  t_still /= v_cnx;

  /* put in units of resistance */
  t_still = LutCal(&rStillLut, t_still);

  /* Look-up calibrated temperatures */
  t_4k = LutCal(&t4kLut, t_4k);
  t_cp = LutCal(&tCpLut[insert], t_cp);
  t_pump = LutCal(&tPumpLut[insert], t_pump);
  t_hsw = LutCal(&tHswLut[insert], t_hsw);
  t_still = LutCal(&tStillLut[insert], t_still);

  /* figure out next_state of finite state machine */

  /* do nothing if out of liquid helium -> OUT */
  if (t_4k > T_4K_MAX) {
    next_state = CRYO_CYCLE_OUT;
    if (cycle_state != CRYO_CYCLE_OUT)
      bprintf(info, "Auto Cycle X%1d: LHe is DRY!\n", insert+1);
  }
  /* COLD: start a cycle when FP too hot (or forced by command) -> HSW_OFF */
  else if (cycle_state == CRYO_CYCLE_COLD) {
    if ((t_still > T_STILL_HOT)
        || CommandData.hk[insert].force_cycle) {
      CommandData.hk[insert].force_cycle = 0;
      WriteData(startCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      next_state = CRYO_CYCLE_HSW_OFF;
      bprintf(info, "Auto Cycle X%1d: Turning heat switch off.", insert+1);
    } else next_state = CRYO_CYCLE_COLD;
  }
  /* HSW_OFF: wait for heat switch to cool -> ON_HEAT */
  else if (cycle_state == CRYO_CYCLE_HSW_OFF) {
    if (t_hsw < T_HSW_COLD
        || ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_HSW_TIMEOUT)) {
      WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      next_state = CRYO_CYCLE_ON_HEAT;
      bprintf(info, "Auto Cycle X%1d: Turning pump heat on.", insert+1);
    } else next_state = CRYO_CYCLE_HSW_OFF;
  }
  /* ON_HEAT: heat pump until timeout -> COOL
   *          or if pump or still too hot, let it settle -> ON_SETTLE */
  else if (cycle_state == CRYO_CYCLE_ON_HEAT) {
    if (((mcp_systime(NULL) - start_time) > CRYO_CYCLE_HEAT_TIMEOUT) || 
	t_pump > T_PUMP_MAX) {
      WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      next_state = CRYO_CYCLE_ON_SETTLE;
      bprintf(info, "Auto Cycle X%1d: Turning pump heat off.", insert+1);
    } else next_state = CRYO_CYCLE_ON_HEAT;
  }
  /* ON_SETTLE: let hot pump settle until timeout, or CP cold enough -> COOL */
  else if (cycle_state == CRYO_CYCLE_ON_SETTLE) {
    if (((mcp_systime(NULL) - state_time) > CRYO_CYCLE_SET_MIN) && 
	(t_cp < T_CP_SET ||
	 ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_SET_TIMEOUT))) {
      WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      next_state = CRYO_CYCLE_COOL;
      bprintf(info, "Auto Cycle X%1d: Turning heat switch on.", insert+1);
    } else next_state = CRYO_CYCLE_ON_SETTLE;
  }
  /* COOL: wait until fridge is cold -> COLD */
  else if (cycle_state == CRYO_CYCLE_COOL) {
    if ((t_still < T_STILL_COLD)
        || ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_COOL_TIMEOUT) ) {
      CommandData.hk[insert].force_cycle = 0; //clear pending cycles
      WriteData(stimeCycleWAddr[insert], mcp_systime(NULL), NIOS_QUEUE);
      next_state = CRYO_CYCLE_COLD;
      bprintf(info, "Auto Cycle X%1d: Fridge is now cold!.", insert+1);
    } else next_state = CRYO_CYCLE_COOL;
  }
  /* OUT: do nothing if out of LHe. Unless not, then be cold -> COLD */
  else if (cycle_state == CRYO_CYCLE_OUT) {
    if (t_4k < T_4K_MAX) {
      next_state = CRYO_CYCLE_COLD;
      bprintf(info, "Auto Cycle X%1d: Activated.", insert+1);
    } else next_state = CRYO_CYCLE_OUT;
  }
  else {
    bprintf(err, "Auto Cycle X%1d: cycle_state: %i unknown!",
        insert+1, cycle_state);
    next_state = CRYO_CYCLE_COLD;
  }

  WriteData(stateCycleWAddr[insert], next_state, NIOS_QUEUE);

  /* set outputs of finite state machine */
  if (next_state == CRYO_CYCLE_COLD) {
    heat_pump = 0;
    heat_hsw = 1;
  } else if (next_state == CRYO_CYCLE_HSW_OFF) {
    heat_pump = 0;
    heat_hsw = 0;
  } else if (next_state == CRYO_CYCLE_ON_HEAT) {
    heat_pump = 1;
    heat_hsw = 0;
  } else if (next_state == CRYO_CYCLE_ON_SETTLE) {
    heat_pump = 0;
    heat_hsw = 0;
  } else {    /* CRYO_CYCLE_COOL, CRYO_CYCLE_OUT, or bad state */
    heat_pump = 0;
    heat_hsw = 1;
  }

  /* set the heater control bits in the output, as needed */
  if (heat_pump) retval |= HK_PWM_PUMP;
  if (!heat_hsw) retval |= HK_PWM_HSW;  /* inverted logic because normally on */

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
  static int firsttime[6] = {1, 1, 1, 1, 1, 1};
  static int servo_ctr[6] = {0, 0, 0, 0, 0, 0};
  static struct BiPhaseStruct* tPumpAddr[6];
  
  //TODO update these calibrations
  static struct LutType tPumpLut[6] = {
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"},
    {.filename = LUT_DIR "d_simonchase.lut"}
  };
  
  double t_pump;
  
  if (firsttime[insert]) {
    char field[64];
    firsttime[insert] = 0;
    sprintf(field, "vd_pump_x%1d_hk", insert+1);
    tPumpAddr[insert] = GetBiPhaseAddr(field);
    LutInit(&tPumpLut[insert]);
  }
  
  if (servo_ctr[insert] <= 0) {
    /* Read pump thermometer voltage */
    t_pump = ReadCalData(tPumpAddr[insert]);
    
    /* Look-up calibrated temperature */
    t_pump = LutCal(&tPumpLut[insert], t_pump);
    
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
  static int fridge_wait = 0;

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
  fridge_wait = (fridge_wait + 1) % FRIDGE_CYCLE_WAIT;

  //PWM heaters
  for (i=0; i<6; i++) {
    bits[i] = 0;
    if (CommandData.hk[i].auto_cycle_on && fridge_start_wait <= 0) {
      //using auto cycle, get PUMP and HSW bits from fridge control
      if (fridge_wait == i) bits[i] |= FridgeCycle(i, 0);
    } else {
      //not using auto cycle. Command PUMP and HSW manually
      FridgeCycle(i, 1);  //reset cycle state
      //servo pump temperature
      if (CommandData.hk[i].pump_servo_on) PumpServo(i);
      if (CommandData.hk[i].pump_heat) bits[i] |= HK_PWM_PUMP;
      //NB: heat switch is normally closed, so logic inverted
      if (!CommandData.hk[i].heat_switch) bits[i] |= HK_PWM_HSW;
    }
    if (CommandData.hk[i].fphi_heat) {
      if (CommandData.hk[i].fphi_heat > 0) CommandData.hk[i].fphi_heat--;
      bits[i] |= HK_PWM_FPHI;
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

void HouseKeeping()
{
  static struct NiosStruct* insertLastHkAddr;
  static struct NiosStruct* vHeatLastHkAddr;
  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    insertLastHkAddr = GetNiosAddr("insert_last_hk");
    vHeatLastHkAddr = GetNiosAddr("v_heat_last_hk");
  }

  BiasControl();
  PhaseControl();
  HeatControl();

  WriteData(insertLastHkAddr, CommandData.hk_last, NIOS_QUEUE);
  WriteCalData(vHeatLastHkAddr, CommandData.hk_vheat_last, NIOS_QUEUE);
}
/* veto SQUIDs if SSA or FPU temp gets too high */
#define VETO_MCE_TIMEOUT 30
#define T_FP_FIR_LEN 60
void VetoMCE(int index)
{
  static int insert = 0;
  static int firsttime[6] = {300, 1, 1, 1, 1, 1};
  
  static struct BiPhaseStruct* tSsaAddr[6];
  static struct BiPhaseStruct* tFpAddr[6]; 
  static struct BiPhaseStruct* vCnxAddr[6];

  static double t_fp[6][T_FP_FIR_LEN];
  static int t_fp_ind[6] = {0, 0, 0, 0, 0, 0};

  double v_cnx, t_ssa, t_fp_mean;
  
  int i;

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

  static struct LutType rFpLut = 
  {.filename = LUT_DIR "r_cernox.lut"};

  static int timeout[6] = {0, 0, 0, 0, 0, 0};

  uint16_t bit = 1U << insert;

  if (firsttime[insert] > 1) {
    firsttime[insert]--;
    return;
  } else if (firsttime[insert]) {
    char field[64];

    sprintf(field, "vd_ssa_x%1d_hk", insert+1);
    tSsaAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "vr_fp_x%1d_hk", insert+1);
    tFpAddr[insert] = GetBiPhaseAddr(field);
    sprintf(field, "v_cnx_x%1d_hk", insert+1);
    vCnxAddr[insert] = GetBiPhaseAddr(field);

    LutInit(&tSsaLut[insert]);
    LutInit(&tFpLut[insert]);

    if (insert == 0)
      LutInit(&rFpLut);
  }

  /* read and calibrate the FP themometer */
  t_fp[insert][t_fp_ind[insert]] = ReadCalData(tFpAddr[insert]);
  v_cnx = ReadCalData(vCnxAddr[insert]);

  /* normalize the cernox readings with bias level */
  t_fp[insert][t_fp_ind[insert]] /= v_cnx;

  /* initialise FIR */
  if (firsttime[insert]) {
    firsttime[insert] = 0; 

    for (i = 0; i < T_FP_FIR_LEN; ++i)
      t_fp[insert][i] = t_fp[insert][t_fp_ind[insert]];
  }

  t_fp_ind[insert] = (t_fp_ind[insert] + 1) % T_FP_FIR_LEN;

  /* do the rest of this at the slow rate */
  if (index != 16)
    return;

  if (timeout[insert] == 0) {
    /* compute the outer mean */
    t_fp_mean = 0;
    for (i = 0; i < T_FP_FIR_LEN; ++i)
      t_fp_mean += t_fp[insert][i];
    t_fp_mean /= T_FP_FIR_LEN;

    /* put in units of resistance */
    t_fp_mean = LutCal(&rFpLut, t_fp_mean);

    /* Read the SSA thermometer */
    t_ssa = ReadCalData(tSsaAddr[insert]);

    /* Look-up calibrated temperatures */
    t_ssa = LutCal(&tSsaLut[insert], t_ssa);
    t_fp_mean = LutCal(&tFpLut[insert], t_fp_mean);

    /* XXX X5 hack */
    if (insert == 4)
      t_fp_mean = t_ssa = 0;

    if ( (t_fp_mean > 8.0) || (t_ssa > 8.0) ) {
      if (~CommandData.thermveto & bit) {
        bprintf(info, "Vetoing X%i for thermal reasons (FP:%.1f SSA:%.1f)\n",
            insert + 1, t_fp_mean, t_ssa);
        CommandData.thermveto |= bit;
        timeout[insert] = VETO_MCE_TIMEOUT;
      }
    }

    if ( (t_fp_mean < 7.0) && (t_ssa < 7.0) ) {
      if (CommandData.thermveto & bit) {
        bprintf(info, "Unvetoing X%i for thermal reasons (FP:%.1f SSA:%.1f)\n",
            insert + 1, t_fp_mean, t_ssa);
        CommandData.thermveto &= ~bit;
        timeout[insert] = VETO_MCE_TIMEOUT;
      }
    }
  } else
    timeout[insert]--;

  insert = (insert+1) % 6;
} 
