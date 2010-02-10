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
#include <stdio.h>

#include "mcp.h"
#include "channels.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"

/* Heater control bits (BIAS_D G4) */
#define HEAT_HELIUM_LEVEL    0x01
#define HEAT_CHARCOAL        0x02
#define HEAT_CHARCOAL_HS     0x04
#define HEAT_POT_HS          0x08
#define HEAT_JFET            0x10
#define HEAT_BDA             0x20
#define HEAT_CALIBRATOR      0x40
#define HEAT_HWPR_POS        0x80

/* Valve control bits (BIAS_D G3) */
//TODO update valve bit defines
#define VALVE_LHe_OFF     0x01
#define VALVE_L_OPEN      0x04
#define VALVE_L_CLOSE     0x08
#define VALVE_POT_OFF     0x10
#define VALVE_LN_OFF      0x20
#define VALVE_POT_OPEN    0x40
#define VALVE_POT_CLOSE   0x80

/* he3 cycle */
#define CRYO_CYCLE_OUT_OF_HELIUM 0x0000
#define CRYO_CYCLE_COLD          0x0001
#define CRYO_CYCLE_ON            0x0002
#define CRYO_CYCLE_COOL          0x0004

// times changed from 45 min to 35 min
// for charcoal, and from 2hrs to 3hrs 
// for timout, due to new (lower conduction)
// charcoal pump tin link.  cbn/md 12_10_06
#define CRYO_CYCLE_TIMEOUT       (35*60)
#define CRYO_CYCLE_COOL_TIMEOUT  (3*60*60)

#define T_HE3FRIDGE_TOO_HOT      3.733304    /* 0.391508    K */
#define T_HE3FRIDGE_COLD         4.34106     /* 0.326861    K */
#define T_HE4POT_SET             1.368951538 /* 2.312414576 K */
#define T_CHARCOAL_SET           6.693948    /* 25          K */
#define T_LHE_SET                9.39198     /* 4.4         K */

/* CryoState bitfield */
//#define CS_HELIUMLEVEL    0x0001
//#define CS_CHARCOAL       0x0002
//#define CS_HWPR_POS       0x0004
#define CS_POTVALVE_ON    0x0010
#define CS_POTVALVE_OPEN  0x0020
#define CS_LHeVALVE_ON    0x0040
#define CS_LVALVE_OPEN    0x0080
#define CS_LNVALVE_ON     0x0100
#define CS_AUTO_JFET      0x0200

void WritePrevStatus();

/************************************************************************/
/*                                                                      */
/* PhaseControl: set phase shifts for DAS cards                         */
/*                                                                      */
/************************************************************************/
void PhaseControl(void)
{
  static int first_time = 1;
  static struct NiosStruct* NiosAddr[DAS_CARDS + 1];
  char field[20];
  int i, j;

  if (first_time) {
    first_time = 0;
    j = 16;   //start of DAS node numbers
    for(i = 0; i < DAS_CARDS; i++) {
      if (j%4 == 0) j++;   //skip motherboard common nodes
      sprintf(field, "phase%02d", j);
      NiosAddr[i] = GetNiosAddr(field);
      j++;
    }
    sprintf(field, "phase13"); //Get nios address of cryo phase as well.
    NiosAddr[DAS_CARDS] = GetNiosAddr(field);
  }	

  for(i = 0; i < DAS_CARDS; i++)
    WriteData(NiosAddr[i], CommandData.Phase[i]<<1, NIOS_QUEUE);
  WriteData(NiosAddr[DAS_CARDS], CommandData.Phase[DAS_CARDS]<<1, NIOS_FLUSH);
}

/***********************************************************************/
/* CalLamp: Flash calibrator                                           */
/***********************************************************************/
static int CalLamp (int index)
{
  static struct NiosStruct* calPulseAddr;
  static int pulse_cnt = 0;             //count of pulse length
  static unsigned int elapsed = 0;  //count for wait between pulses
  static enum calmode last_mode = off;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    calPulseAddr = GetNiosAddr("cal_pulse");
  }

  /* Save the current pulse length */
  if (index == 0)
    WriteData(calPulseAddr, CommandData.Cryo.calib_pulse, NIOS_QUEUE);

  if (CommandData.Cryo.calibrator == on) {
    last_mode = on;
    return 1;
  } else if (CommandData.Cryo.calibrator == repeat) {
    if (last_mode != repeat || elapsed >= CommandData.Cryo.calib_period) {
      /* end of cycle -- send a new pulse */
      elapsed = 0;
      pulse_cnt = CommandData.Cryo.calib_pulse;
    } else if (index == 0) elapsed++;  //period measured in slow frames
    last_mode = repeat;
  } else if (CommandData.Cryo.calibrator == pulse) {
      if (last_mode != pulse) pulse_cnt = CommandData.Cryo.calib_pulse;
      last_mode = pulse;
      if (pulse_cnt <= 0) last_mode = CommandData.Cryo.calibrator = off;
  } else {    //off mode
    pulse_cnt = 0;
    last_mode = off;
  }

  if (pulse_cnt > 0) {
    pulse_cnt--;
    return 1;
  }
  return 0;
}

/* Automatic conrol of JFET heater */
static int JFETthermostat(void)
{
  double jfet_temp;
  static struct BiPhaseStruct* tJfetAddr;

  static struct LutType DiodeLut = {"/data/etc/dt600.txt", 0, NULL, NULL, 0};

  static int lasttime = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    tJfetAddr = GetBiPhaseAddr("td_jfet");
    LutInit(&DiodeLut);
  }

  jfet_temp = (double)slow_data[tJfetAddr->index][tJfetAddr->channel]
    * CRYO_D_M + CRYO_D_B;
  jfet_temp = LutCal(&DiodeLut, jfet_temp);

  if (jfet_temp < 0 || jfet_temp > 400)
    return CommandData.Cryo.JFETHeat;
  else if (jfet_temp > CommandData.Cryo.JFETSetOff)
  {
    lasttime = 0;
    return 0;
  }
  else if (jfet_temp < CommandData.Cryo.JFETSetOn)
  {
    lasttime = 1;
    return 1;
  }
  else //We're inbetween the setpoints, so keep doin' what we were doin'
    return lasttime;
}

static void FridgeCycle(int *heatctrl, int *cryostate, int  reset,
    unsigned short *force_cycle)
{
  static int firsttime = 1;
  static struct BiPhaseStruct* t_lhe_Addr;
  static struct BiPhaseStruct* t_he3fridge_Addr;
  static struct BiPhaseStruct* t_charcoal_Addr;
  static struct BiPhaseStruct* t_he4pot_Addr;
  static struct NiosStruct*    cycleStartWAddr;
  static struct BiPhaseStruct* cycleStartRAddr;
  static struct NiosStruct*    cycleStateWAddr;
  static struct BiPhaseStruct* cycleStateRAddr;

  double t_lhe, t_he3fridge, t_charcoal, t_he4pot;

  time_t start_time;
  unsigned short cycle_state;
  static unsigned short iterator = 1;

  if (firsttime) {
    firsttime = 0;
    t_lhe_Addr = GetBiPhaseAddr("td_lhe");
    t_he3fridge_Addr = GetBiPhaseAddr("td_charcoal");
    t_charcoal_Addr = GetBiPhaseAddr("td_charcoal");
    t_he4pot_Addr = GetBiPhaseAddr("td_charcoal");
    cycleStartWAddr = GetNiosAddr("cycle_start");
    cycleStartRAddr = ExtractBiPhaseAddr(cycleStartWAddr);
    cycleStateWAddr = GetNiosAddr("cycle_state");
    cycleStateRAddr = ExtractBiPhaseAddr(cycleStateWAddr);
    WriteData(cycleStateWAddr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
    return;
  }

  if (reset || force_cycle == NULL) {
   WriteData(cycleStateWAddr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
   iterator = 1;
   return;
  }

  if(iterator++ % 10)  /* Run this loop at 0.5 Hz */
    return;

  //TODO rewrite fridge cycle code
  bprintf(warning, "Auto Cycle not implemented."
      " You should be in manual mode.\n");
  return;

  start_time = slow_data[cycleStartRAddr->index][cycleStartRAddr->channel];
  start_time |=
    (unsigned long)slow_data[cycleStartRAddr->index][cycleStartRAddr->channel
    + 1] << 16;
  cycle_state = slow_data[cycleStateRAddr->index][cycleStateRAddr->channel];

  t_lhe = (double)slow_data[t_lhe_Addr->index][t_lhe_Addr->channel];
  t_charcoal
    = (double)slow_data[t_charcoal_Addr->index][t_charcoal_Addr->channel];
  t_he3fridge
    = (double)(slow_data[t_he3fridge_Addr->index][t_he3fridge_Addr->channel] +
		    ((unsigned long)slow_data[t_he3fridge_Addr->index][1 +
         t_he3fridge_Addr->channel] << 16));
  t_he4pot
    = (double)(slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel] +
        ((unsigned long)slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel
         + 1] << 16));	

  t_lhe       = CRYO_D_M*t_lhe + CRYO_D_B;
  t_charcoal  = CRYO_D_M*t_charcoal + CRYO_D_B;
  t_he3fridge = (ROX_C2V)*t_he3fridge + ROX_OFFSET;
  t_he4pot    = (ROX_C2V)*t_he4pot    + ROX_OFFSET;

  if (t_lhe < T_LHE_SET) {
    *heatctrl &= ~HEAT_CHARCOAL;
    WriteData(cycleStateWAddr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
    return;
  }

  if (cycle_state == CRYO_CYCLE_COLD) {
    if((t_he3fridge < T_HE3FRIDGE_TOO_HOT && t_he4pot > T_HE4POT_SET)
        || *force_cycle) {
      *force_cycle = 0;
      WriteData(cycleStateWAddr, CRYO_CYCLE_ON, NIOS_QUEUE);
      WriteData(cycleStartWAddr, mcp_systime(NULL), NIOS_QUEUE);
      *heatctrl |= HEAT_CHARCOAL;
      bprintf(info, "Auto Cycle: Turning charcoal heat on.");
      return;
    }
    *heatctrl &= ~HEAT_CHARCOAL;
  } else if (cycle_state == CRYO_CYCLE_ON) {
    if (((mcp_systime(NULL) - start_time) > CRYO_CYCLE_TIMEOUT) ||
        t_charcoal < T_CHARCOAL_SET ||
        t_he4pot < T_HE4POT_SET) {
      WriteData(cycleStateWAddr, CRYO_CYCLE_COOL, NIOS_QUEUE);
      *heatctrl &= ~HEAT_CHARCOAL;
      bprintf(info, "Auto Cycle: Turning charcoal heat off.");
      return;
    }
    *heatctrl |= HEAT_CHARCOAL;
  } else if ( cycle_state == CRYO_CYCLE_COOL) {
    if ((t_he3fridge > T_HE3FRIDGE_COLD)
        || ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_COOL_TIMEOUT) ) {
      WriteData(cycleStateWAddr, CRYO_CYCLE_COLD, NIOS_QUEUE);
      *heatctrl &= ~HEAT_CHARCOAL;
      *force_cycle = 0; // clear any pending cycle commands...
      bprintf(info, "Auto Cycle: Fridge is now cold!.");
      return;
    }
    *heatctrl &= ~HEAT_CHARCOAL;
  } else if (cycle_state == CRYO_CYCLE_OUT_OF_HELIUM) {
    WriteData(cycleStateWAddr, CRYO_CYCLE_COLD, NIOS_QUEUE);
    *heatctrl &= ~HEAT_CHARCOAL;
    bprintf(info, "Auto Cycle: Everything A-OK.");
  } else {
    bprintf(err, "Auto Cycle: cycle_state: %i unknown!", cycle_state);
    *heatctrl &= ~HEAT_CHARCOAL;
    return;
  }
  return;
}

/***********************************************************************/
/* CryoControl: Control heaters and calibrator (a slow control)        */
/***********************************************************************/
void CryoControl (int index)
{
  static struct NiosStruct* cryostateAddr;
  static struct NiosStruct* jfetSetOnAddr;
  static struct NiosStruct* jfetSetOffAddr;
  //static struct NiosStruct* dig21Addr;
  static struct NiosStruct* dig43Addr;
  //static struct NiosStruct* dig65Addr;

  static int cryostate = 0;
  int heatctrl = 0, valvectrl = 0;

  /************** Set indices first time around *************/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    cryostateAddr = GetNiosAddr("cryostate");
    jfetSetOnAddr = GetNiosAddr("jfet_set_on");
    jfetSetOffAddr = GetNiosAddr("jfet_set_off");
    //dig21Addr = GetNiosAddr("das_dig21");
    dig43Addr = GetNiosAddr("das_dig43");
    //dig65Addr = GetNiosAddr("das_dig65");
  }

#if 0
  // purely for testing, output a count to each digital output group
  static int count = 0;
  int nibbcnt = (count&0xf) << 4 | (count++&0xf);
  WriteData(dig21Addr, nibbcnt<<8 | nibbcnt, NIOS_QUEUE);
  WriteData(dig43Addr, nibbcnt<<8 | nibbcnt, NIOS_QUEUE);
  WriteData(dig65Addr, nibbcnt<<8 | nibbcnt, NIOS_QUEUE);
#endif

  /********** Set Output Bits **********/
  if (CommandData.Cryo.heliumLevel) {
    heatctrl |= HEAT_HELIUM_LEVEL;
    if (CommandData.Cryo.heliumLevel > 0)
      CommandData.Cryo.heliumLevel--;
  }

  if (CommandData.Cryo.hwprPos) {
    heatctrl |= HEAT_HWPR_POS;
    if (CommandData.Cryo.hwprPos > 0)
      CommandData.Cryo.hwprPos--;
  }

  if (CommandData.Cryo.autoJFETheat) {
    if (JFETthermostat()) heatctrl |= HEAT_JFET;
    cryostate |= CS_AUTO_JFET;
  } else {
    if (CommandData.Cryo.JFETHeat) heatctrl |= HEAT_JFET;
    cryostate &= 0xFFFF - CS_AUTO_JFET;
  }

  if(CommandData.Cryo.hsPot) heatctrl |= HEAT_POT_HS;

  if(CommandData.Cryo.BDAHeat) heatctrl |= HEAT_BDA;

  if (CommandData.Cryo.fridgeCycle)
    FridgeCycle(&heatctrl, &cryostate, 0, &CommandData.Cryo.force_cycle);
  else {
    FridgeCycle(&heatctrl, &cryostate, 1, NULL);
    if (CommandData.Cryo.charcoalHeater) heatctrl |= HEAT_CHARCOAL;
    if (CommandData.Cryo.hsCharcoal) heatctrl |= HEAT_CHARCOAL_HS;
  }

  if (CalLamp(index)) heatctrl |= HEAT_CALIBRATOR;

  /* Control motorised valves -- latching relays */
  valvectrl = VALVE_POT_OPEN | VALVE_POT_CLOSE |
    VALVE_L_OPEN | VALVE_L_CLOSE;

  if (CommandData.Cryo.potvalve_open > 0) {
    valvectrl &= ~VALVE_POT_OPEN;
    cryostate |= CS_POTVALVE_OPEN;
    CommandData.Cryo.potvalve_open--;
  } else if (CommandData.Cryo.potvalve_close > 0) {
    valvectrl &= ~VALVE_POT_CLOSE;
    cryostate &= 0xFFFF - CS_POTVALVE_OPEN;
    CommandData.Cryo.potvalve_close--;
  }
  if (CommandData.Cryo.potvalve_on)
    cryostate |= CS_POTVALVE_ON;
  else {
    valvectrl |= VALVE_POT_OFF;
    cryostate &= 0xFFFF - CS_POTVALVE_ON;
  }

  if (CommandData.Cryo.lvalve_open > 0) {
    valvectrl &= ~VALVE_L_OPEN;
    cryostate |= CS_LVALVE_OPEN;
    CommandData.Cryo.lvalve_open--;
  } else if (CommandData.Cryo.lvalve_close > 0) {
    valvectrl &= ~VALVE_L_CLOSE;
    cryostate &= 0xFFFF - CS_LVALVE_OPEN;
    CommandData.Cryo.lvalve_close--;
  }
  if (CommandData.Cryo.lhevalve_on)
    cryostate |= CS_LHeVALVE_ON;
  else {
    valvectrl |= VALVE_LHe_OFF;
    cryostate &= 0xFFFF - CS_LHeVALVE_ON;
  }
  if (CommandData.Cryo.lnvalve_on)
    cryostate |= CS_LNVALVE_ON;
  else {
    valvectrl |= VALVE_LN_OFF;
    cryostate &= 0xFFFF - CS_LNVALVE_ON;
  }

  if (index == 0) {
  WriteData(cryostateAddr, cryostate, NIOS_QUEUE);
  WriteData(jfetSetOnAddr, CommandData.Cryo.JFETSetOn * 100, NIOS_QUEUE);
  WriteData(jfetSetOffAddr, CommandData.Cryo.JFETSetOff * 100, NIOS_QUEUE);
  }
  WriteData(dig43Addr, (heatctrl<<8) | valvectrl, NIOS_FLUSH);
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Digital IO with the Bias Generator Card               */
/*                                                                      */
/************************************************************************/
void BiasControl (unsigned short* RxFrame)
{
  static struct NiosStruct* biasAmplAddr[5];
  static struct NiosStruct* rampEnaAddr;
  static struct BiPhaseStruct* rampAmplAddr;
  int i;
  int isBiasRamp;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    biasAmplAddr[0] = GetNiosAddr("bias_ampl_500");
    biasAmplAddr[1] = GetNiosAddr("bias_ampl_350");
    biasAmplAddr[2] = GetNiosAddr("bias_ampl_250");
    biasAmplAddr[3] = GetNiosAddr("bias_ampl_rox");
    biasAmplAddr[4] = GetNiosAddr("bias_ampl_x");
    rampEnaAddr = GetNiosAddr("bias_ramp_ena");
    rampAmplAddr = GetBiPhaseAddr("ramp_ampl");
  }

  if (CommandData.Bias.dont_do_anything) return;

  /********** set Bias (ramp)  *******/
  isBiasRamp = slow_data[rampAmplAddr->index][rampAmplAddr->channel];
  if ( (isBiasRamp && CommandData.Bias.biasRamp == 0) ||
	  (!isBiasRamp && CommandData.Bias.biasRamp == 1) ) {
    WriteData(rampEnaAddr, CommandData.Bias.biasRamp, NIOS_QUEUE);
  }

  /************* Set the Bias Levels *******/
  for (i=0; i<5; i++)
    if (CommandData.Bias.setLevel[i]) {
      WriteData(biasAmplAddr[i], CommandData.Bias.bias[i]<<1, NIOS_QUEUE);
      CommandData.Bias.setLevel[i] = 0;
    }
}

