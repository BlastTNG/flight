/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 * And copyright 2010 Matthew Truch
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

/* the HWPR pot fraction of full scale should not read higher than this */
#define HWPR_POT_MAX	0.99

/* Heater control bits (BIAS_D G4) */
#define HEAT_HELIUM_LEVEL    0x01
#define HEAT_CHARCOAL        0x02
#define HEAT_POT_HS          0x04
#define HEAT_CHARCOAL_HS     0x08
#define HEAT_JFET            0x10
#define HEAT_BDA             0x20
#define HEAT_CALIBRATOR      0x40
#define HEAT_HWPR_POS        0x80

/* Valve control bits (BIAS_D G3) */
#define VALVE_LHe_OFF     0x01
#define VALVE_L_OPEN      0x04
#define VALVE_L_CLOSE     0x02
#define VALVE_LN_OFF      0x08
#define VALVE_POT_OFF     0x90	  //more cowbell
#define VALVE_POT_OPEN    0x40
#define VALVE_POT_CLOSE   0x20

/* he3 cycle */
#define CRYO_CYCLE_OUT    0x0000    //out of LHe. Nothing to do
#define CRYO_CYCLE_COLD   0x0001    //Cold. All is well
#define CRYO_CYCLE_ON     0x0002    //Charcoal heating
#define CRYO_CYCLE_COOL   0x0004    //Cycle done, cooling
#define CRYO_CYCLE_HS_OFF 0x0008    //Start of cycle, open head switch
#define CRYO_CYCLE_SETTLE 0x0010    //Charcoal off, settle before connecting HS

/* he3 cycle timeouts (all in seconds) */
#define CRYO_CYCLE_COOL_TIMEOUT  (2*60*60)  //TODO should this be longer?
#define CRYO_CYCLE_HS_TIMEOUT    (2.75*60)

#define T_LHE_MAX         4.600
#define T_CHAR_HS_COLD   10.000

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
  static struct NiosStruct* phaseStepEnaAddr;
  static struct NiosStruct* phaseStepStartAddr;
  static struct NiosStruct* phaseStepEndAddr;
  static struct NiosStruct* phaseStepNstepsAddr;
  static struct NiosStruct* phaseStepTimeAddr;

  char field[20];
  int i, j;
  static int k=0;
  static int step_size=1;
  static int dk=1;
  static int end=0;
  static int start=0;
  int phase=0;

  if (first_time) {
    first_time = 0;
    j = 16;   //start of DAS node numbers
    for(i = 0; i < DAS_CARDS; i++) {
      if (j%4 == 0) j++;   //skip motherboard common nodes
      sprintf(field, "n%02d_phase", j);
      NiosAddr[i] = GetNiosAddr(field);
      j++;
    }
    sprintf(field, "n13_phase"); //Get nios address of cryo phase as well.
    NiosAddr[DAS_CARDS] = GetNiosAddr(field);
    phaseStepEnaAddr = GetNiosAddr("step_ena_phase");
    phaseStepStartAddr = GetNiosAddr("step_start_phase");
    phaseStepEndAddr = GetNiosAddr("step_end_phase");
    phaseStepNstepsAddr = GetNiosAddr("step_nsteps_phase");
    phaseStepTimeAddr = GetNiosAddr("step_time_phase");
   
  }	

  if(CommandData.phaseStep.do_step) {
    if (k==0) {
      start = CommandData.phaseStep.start;
      end = CommandData.phaseStep.end;
      step_size=(end-start)/CommandData.phaseStep.nsteps;

      if(step_size==0) { // minimum step size is 1
	if (end >= start) {
	  step_size=1;
	}
	if (end < start) {
	  step_size=-1;
	}
      }
      end +=step_size;
      dk = (unsigned int)(CommandData.phaseStep.dt*SR/1000/20);
      if (dk == 0) 
	dk =1;
    }

    phase = start+(k/dk)*step_size;

    if (step_size > 0) {
      if (phase >= end) CommandData.phaseStep.do_step=0;
      if (phase > 32767) { 
	phase = 32767;
	CommandData.phaseStep.do_step=0;
      }
    } else {
      if (phase <= end) {
	CommandData.phaseStep.do_step=0; 
      }
      if (phase < 1) { 
	phase = 1;
	CommandData.phaseStep.do_step=0;
      }
    }

    WriteData(phaseStepEnaAddr,CommandData.phaseStep.do_step, NIOS_QUEUE);
    WriteData(phaseStepStartAddr,CommandData.phaseStep.start<<1, NIOS_QUEUE);
    WriteData(phaseStepEndAddr,CommandData.phaseStep.end<<1, NIOS_QUEUE);
    WriteData(phaseStepNstepsAddr,CommandData.phaseStep.nsteps, NIOS_QUEUE);
    WriteData(phaseStepTimeAddr,CommandData.phaseStep.dt, NIOS_QUEUE);
    for(i = 0; i < DAS_CARDS; i++)
      WriteData(NiosAddr[i], phase<<1, NIOS_QUEUE);
    k++;
  } else {
    k=0;
    for(i = 0; i < DAS_CARDS; i++)
      WriteData(NiosAddr[i], CommandData.Phase[i]<<1, NIOS_QUEUE);
    WriteData(NiosAddr[DAS_CARDS], CommandData.Phase[DAS_CARDS]<<1, NIOS_FLUSH);
  }
}

/***********************************************************************/
/* CalLamp: Flash calibrator                                           */
/***********************************************************************/
static int CalLamp (int index)
{
  static struct NiosStruct* pulseCalAddr;
  static int pulse_cnt = 0;             //count of pulse length
  static unsigned int elapsed = 0;  //count for wait between pulses
  static enum calmode last_mode = off;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    pulseCalAddr = GetNiosAddr("pulse_cal");
  }

  /* Save the current pulse length */
  if (index == 0)
    WriteData(pulseCalAddr, CommandData.Cryo.calib_pulse, NIOS_QUEUE);

  if (CommandData.Cryo.calibrator == on) {
    last_mode = on;
    return 1;
  } else if (CommandData.Cryo.calibrator == repeat) {
    if (elapsed >= CommandData.Cryo.calib_period || last_mode != repeat) {
      if (CommandData.Cryo.calib_repeats < 0 
	  || --CommandData.Cryo.calib_repeats > 0) {
	/* end of cycle -- send a new pulse */
	elapsed = 0;
	pulse_cnt = CommandData.Cryo.calib_pulse;
      }
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

  static struct LutType DiodeLut =
     {"/data/etc/blast/dt600.txt", 0, NULL, NULL, 0};

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
  static int firsttime = 1000; 
           // Skip first 1000 frames (10 seconds) as startup veto.
  static struct BiPhaseStruct* t_lhe_Addr;
  static struct BiPhaseStruct* t_he3fridge_Addr;
  static struct BiPhaseStruct* t_charcoal_Addr;
  static struct BiPhaseStruct* t_he4pot_Addr;
  static struct BiPhaseStruct* t_char_hs_Addr;
  static struct NiosStruct*    startCycleWAddr;
  static struct BiPhaseStruct* startCycleRAddr;
  static struct NiosStruct*    stateCycleWAddr;
  static struct BiPhaseStruct* stateCycleRAddr;
  static struct NiosStruct*    startSetCycleWAddr;
  static struct BiPhaseStruct* startSetCycleRAddr;

  //for writing fridge cycle parameters back to the frame
  static struct NiosStruct* tStartCycleAddr;
  static struct NiosStruct* tPotMaxCycleAddr;
  static struct NiosStruct* tCharMaxCycleAddr;
  static struct NiosStruct* tCharSetCycleAddr;
  static struct NiosStruct* timeCharCycleAddr;
  static struct NiosStruct* timeSetCycleAddr;

  static struct LutType DiodeLut =
     {"/data/etc/blast/dt600.txt", 0, NULL, NULL, 0};
  static struct LutType HSLut =
      {"/data/etc/blast/dt-simonchase.txt", 0, NULL, NULL, 0};
  static struct LutType ROXLut =
      {"/data/etc/blast/rox-raw.txt", 0, NULL, NULL, 0};

  double t_lhe, t_he3fridge, t_charcoal, t_he4pot, t_char_hs;

  //derived command parameters
  double t_he3fridge_cold;
  time_t overall_settle_timeout;

  time_t start_time, settle_start_time;
  unsigned short cycle_state;
  static unsigned short iterator = 1;
  static unsigned short heat_char = 0;
  static unsigned short heat_hs = 0;

  if (firsttime > 1) {
    firsttime--;
    return;
  } else if (firsttime) {
    firsttime = 0;
    t_lhe_Addr = GetBiPhaseAddr("td_lhe");
    t_he3fridge_Addr = GetBiPhaseAddr("tr_300mk_strap"); //NOTE: tr_he3fridge is broken.
    t_charcoal_Addr = GetBiPhaseAddr("td_charcoal");
    t_he4pot_Addr = GetBiPhaseAddr("tr_m4"); //NOTE: tr_he4_pot is broken.
    t_char_hs_Addr = GetBiPhaseAddr("td_hs_charcoal");
    startCycleWAddr = GetNiosAddr("start_cycle");
    startCycleRAddr = ExtractBiPhaseAddr(startCycleWAddr);
    stateCycleWAddr = GetNiosAddr("state_cycle");
    stateCycleRAddr = ExtractBiPhaseAddr(stateCycleWAddr);
    startSetCycleWAddr = GetNiosAddr("start_set_cycle");
    startSetCycleRAddr = ExtractBiPhaseAddr(startSetCycleWAddr);
    WriteData(stateCycleWAddr, CRYO_CYCLE_OUT, NIOS_QUEUE);

    //for writing fridge cycle parameters back to the frame
    tStartCycleAddr = GetNiosAddr("t_start_cycle");
    tPotMaxCycleAddr = GetNiosAddr("t_pot_max_cycle");
    tCharMaxCycleAddr = GetNiosAddr("t_char_max_cycle");
    tCharSetCycleAddr = GetNiosAddr("t_char_set_cycle");
    timeCharCycleAddr = GetNiosAddr("time_char_cycle");
    timeSetCycleAddr = GetNiosAddr("time_set_cycle");

    LutInit(&DiodeLut);
    LutInit(&HSLut);
    LutInit(&ROXLut);
  }

  WriteData(tStartCycleAddr,
      CommandData.Cryo.cycle_start_temp*65536.0/4.0, NIOS_QUEUE);
  WriteData(tPotMaxCycleAddr,
      CommandData.Cryo.cycle_pot_max*65536.0/10.0, NIOS_QUEUE);
  WriteData(tCharMaxCycleAddr,
      CommandData.Cryo.cycle_charcoal_max*65536.0/70.0, NIOS_QUEUE);
  WriteData(tCharSetCycleAddr,
      CommandData.Cryo.cycle_charcoal_settle*65536.0/70.0, NIOS_QUEUE);
  WriteData(timeCharCycleAddr,
      CommandData.Cryo.cycle_charcoal_timeout*65536.0/120.0, NIOS_QUEUE);
  WriteData(timeSetCycleAddr,
      CommandData.Cryo.cycle_settle_timeout*65536.0/120.0, NIOS_QUEUE);

  overall_settle_timeout = CommandData.Cryo.cycle_charcoal_timeout +
    CommandData.Cryo.cycle_settle_timeout + 2;
  t_he3fridge_cold = CommandData.Cryo.cycle_start_temp - 0.025;

  if (reset || force_cycle == NULL) {
    WriteData(stateCycleWAddr, CRYO_CYCLE_OUT, NIOS_QUEUE);
    iterator = 1;
    heat_char = 0;
    heat_hs = 0;
    return;
  }

  if(!(iterator++ % 199))  /* Run this loop at ~0.5 Hz */
  {

    start_time = slow_data[startCycleRAddr->index][startCycleRAddr->channel];
    start_time |= (unsigned long)slow_data[startCycleRAddr->index][startCycleRAddr->channel + 1] << 16;
    cycle_state = slow_data[stateCycleRAddr->index][stateCycleRAddr->channel];
    settle_start_time = slow_data[startSetCycleRAddr->index][startSetCycleRAddr->channel];
    settle_start_time |= (unsigned long)slow_data[startSetCycleRAddr->index][startSetCycleRAddr->channel + 1] << 16;

    /* Extract and calculate all the temperatures of interest */
    t_lhe = (double)(slow_data[t_lhe_Addr->index][t_lhe_Addr->channel] +
        ((unsigned long)slow_data[t_lhe_Addr->index][t_lhe_Addr->channel + 1] << 16));
    t_charcoal = (double)(slow_data[t_charcoal_Addr->index][t_charcoal_Addr->channel] +
        ((unsigned long)slow_data[t_charcoal_Addr->index][t_charcoal_Addr->channel + 1] << 16));
    t_he3fridge = (double)(slow_data[t_he3fridge_Addr->index][t_he3fridge_Addr->channel] +
        ((unsigned long)slow_data[t_he3fridge_Addr->index][1 + t_he3fridge_Addr->channel] << 16));
    t_he4pot = (double)(slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel] +
        ((unsigned long)slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel + 1] << 16));	
    t_char_hs = (double)(slow_data[t_char_hs_Addr->index][t_char_hs_Addr->channel] +
        ((unsigned long)slow_data[t_char_hs_Addr->index][t_char_hs_Addr->channel + 1] << 16));

    t_lhe       = CRYO_D_M          * t_lhe       + CRYO_D_B;
    t_charcoal  = CRYO_D_M          * t_charcoal  + CRYO_D_B;
    t_char_hs   = CRYO_D_M          * t_char_hs   + CRYO_D_B;
    t_he3fridge = CRYO_HE3_FRIDGE_M * t_he3fridge + CRYO_HE3_FRIDGE_B;
    t_he4pot    = CRYO_HE4_POT_M    * t_he4pot    + CRYO_HE4_POT_B;
  
    t_lhe       = LutCal(&DiodeLut, t_lhe);
    t_charcoal  = LutCal(&DiodeLut, t_charcoal);
    t_char_hs   = LutCal(&HSLut,    t_char_hs);
    t_he3fridge = LutCal(&ROXLut,   t_he3fridge);
    t_he4pot    = LutCal(&ROXLut,   t_he4pot);
  
    if (t_lhe > T_LHE_MAX) {
      if (cycle_state != CRYO_CYCLE_OUT)
        bprintf(info, "Auto Cycle: LHe is DRY!\n");
      heat_char = 0;
      heat_hs = 1;
      WriteData(stateCycleWAddr, CRYO_CYCLE_OUT, NIOS_QUEUE);
    } else if (cycle_state == CRYO_CYCLE_COLD) {
      if ((t_he3fridge > CommandData.Cryo.cycle_start_temp &&
            t_he4pot < CommandData.Cryo.cycle_pot_max)
          || *force_cycle) {
        *force_cycle = 0;
        WriteData(stateCycleWAddr, CRYO_CYCLE_HS_OFF, NIOS_QUEUE);
        WriteData(startCycleWAddr, mcp_systime(NULL), NIOS_QUEUE);
        heat_char = heat_hs = 0;
        bprintf(info, "Auto Cycle: Turning charcoal heatswitch off.");
      } else {
        heat_char = 0;
        heat_hs = 1;
      }
    } else if (cycle_state == CRYO_CYCLE_HS_OFF) {
      if (((mcp_systime(NULL) - start_time) > CRYO_CYCLE_HS_TIMEOUT) ||
          t_char_hs < T_CHAR_HS_COLD) {
        WriteData(stateCycleWAddr, CRYO_CYCLE_ON, NIOS_QUEUE);
        heat_char = 1;
        heat_hs = 0;
        bprintf(info, "Auto Cycle: Turning charcoal heat on.");
      } else {
        heat_char = heat_hs = 0;
      }
    } else if (cycle_state == CRYO_CYCLE_ON) {
      if (((mcp_systime(NULL) - start_time) >
            CommandData.Cryo.cycle_charcoal_timeout*60) ||
          t_charcoal > CommandData.Cryo.cycle_charcoal_max) {
        WriteData(startSetCycleWAddr, mcp_systime(NULL), NIOS_QUEUE);
        WriteData(stateCycleWAddr, CRYO_CYCLE_SETTLE, NIOS_QUEUE);
        heat_char = 0;
        heat_hs = 0;
        bprintf(info, "Auto Cycle: Charcoal heat off");
      } else if (t_he4pot > CommandData.Cryo.cycle_pot_max) {
        WriteData(stateCycleWAddr, CRYO_CYCLE_COOL, NIOS_QUEUE);
        heat_char = 0;
        heat_hs = 1;
        bprintf(info, "Auto Cycle: Pot out. Charcoal heat off, heatswitch on.");
      } else {
        heat_char = 1;
        heat_hs = 0;
      }
    } else if ( cycle_state == CRYO_CYCLE_SETTLE) {
      if (((mcp_systime(NULL) - start_time) > overall_settle_timeout*60) ||
          ((mcp_systime(NULL) - settle_start_time) >
           CommandData.Cryo.cycle_settle_timeout*60) ||
          t_charcoal < CommandData.Cryo.cycle_charcoal_settle) {
        WriteData(stateCycleWAddr, CRYO_CYCLE_COOL, NIOS_QUEUE);
        heat_char = 0;
        heat_hs = 1;
        if ((mcp_systime(NULL) - start_time) > overall_settle_timeout*60)
          bprintf(warning, "Auto Cycle: aborting settle on overall timeout");
        bprintf(info, "Auto Cycle: Charcoal heatswitch on");
      } else {
        heat_char = heat_hs = 0;
      }
    } else if ( cycle_state == CRYO_CYCLE_COOL) {
      if ((t_he3fridge < t_he3fridge_cold)
          || ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_COOL_TIMEOUT) ) {
        WriteData(stateCycleWAddr, CRYO_CYCLE_COLD, NIOS_QUEUE);
        heat_char = 0;
        heat_hs = 1;
        *force_cycle = 0; // clear any pending cycle commands...
        bprintf(info, "Auto Cycle: Fridge is now cold!.");
      } else {
        heat_char = 0;
        heat_hs = 1;
      }
    } else if (cycle_state == CRYO_CYCLE_OUT) {
      if (t_lhe < T_LHE_MAX) {
        WriteData(stateCycleWAddr, CRYO_CYCLE_COLD, NIOS_QUEUE);
        heat_char = 0;
        heat_hs = 1;
        bprintf(info, "Auto Cycle: Activated.");
      }
    } else {
      bprintf(err, "Auto Cycle: cycle_state: %i unknown!", cycle_state);
      WriteData(stateCycleWAddr, CRYO_CYCLE_COLD, NIOS_QUEUE);
      heat_char = 0;
      heat_hs = 1;
    }
  }

  if (heat_char)
    *heatctrl |= HEAT_CHARCOAL;
  else
    *heatctrl &= ~HEAT_CHARCOAL;

  if (heat_hs)
    *heatctrl |= HEAT_CHARCOAL_HS;
  else
    *heatctrl &= ~HEAT_CHARCOAL_HS;

  return;
}

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
void CryoControl (int index)
{
  static struct NiosStruct* cryostateAddr;
  static struct NiosStruct* jfetSetOnAddr;
  static struct NiosStruct* jfetSetOffAddr;
  static struct NiosStruct* dig43DasAddr;
  static struct NiosStruct* potHwprAddr;
  static struct BiPhaseStruct* potRawHwprAddr;
  static struct BiPhaseStruct* potRefHwprAddr;

  double pot_hwpr_raw, pot_hwpr_ref, pot_hwpr;

  static int cryostate = 0;
  int heatctrl = 0, valvectrl = 0;

  /************** Set indices first time around *************/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    cryostateAddr = GetNiosAddr("cryostate");
    jfetSetOnAddr = GetNiosAddr("jfet_set_on");
    jfetSetOffAddr = GetNiosAddr("jfet_set_off");
    dig43DasAddr = GetNiosAddr("dig43_das");
    potHwprAddr = GetNiosAddr("pot_hwpr");
    potRawHwprAddr = GetBiPhaseAddr("pot_raw_hwpr");
    potRefHwprAddr = GetBiPhaseAddr("pot_ref_hwpr");
  }

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

  //HWPR potentiometer logic
  pot_hwpr_raw = (double)
      (slow_data[potRawHwprAddr->index][potRawHwprAddr->channel] +
      ((unsigned long)
      slow_data[potRawHwprAddr->index][potRawHwprAddr->channel + 1] << 16));
  pot_hwpr_raw -= 2147483648.0;	  //convert from 32-bit offset signed
  pot_hwpr_ref = (double)
      (slow_data[potRefHwprAddr->index][potRefHwprAddr->channel] +
      ((unsigned long)
      slow_data[potRefHwprAddr->index][potRefHwprAddr->channel + 1] << 16));
  pot_hwpr_ref -= 2147483648.0;	  //convert from 32-bit offset signed
  pot_hwpr = pot_hwpr_raw / pot_hwpr_ref;
  //don't update hwpr_pot when not pulsing the read, or when reading too high
  if (heatctrl & HEAT_HWPR_POS && pot_hwpr < HWPR_POT_MAX) {
    WriteData(potHwprAddr, (int)(pot_hwpr*65535.0), NIOS_QUEUE);
  }

  if (index == 0) {
    WriteData(cryostateAddr, cryostate, NIOS_QUEUE);
    WriteData(jfetSetOnAddr, CommandData.Cryo.JFETSetOn * 100, NIOS_QUEUE);
    WriteData(jfetSetOffAddr, CommandData.Cryo.JFETSetOff * 100, NIOS_QUEUE);
  }
  WriteData(dig43DasAddr, (heatctrl<<8) | valvectrl, NIOS_FLUSH);
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Digital IO with the Bias Generator Card               */
/*                                                                      */
/************************************************************************/
void BiasControl ()
{
  static struct NiosStruct* amplBiasAddr[5];
  static struct NiosStruct* rampEnaBiasAddr;
  static struct BiPhaseStruct* rampAmplBiasAddr;
  static struct NiosStruct* stepEnaBiasAddr;
  static struct NiosStruct* stepStartBiasAddr;
  static struct NiosStruct* stepEndBiasAddr;
  static struct NiosStruct* stepNBiasAddr;
  static struct NiosStruct* stepTimeBiasAddr;
  static struct NiosStruct* stepPulLenBiasAddr;
  static struct NiosStruct* stepArrayBiasAddr;
  static int i_arr=3;
  static int k=0;
  static int step_size=1;
  static int dk=1;
  static int end=0;
  static int start=0;
  static int kp=0;
  static int pulse_len_k=0;
  int bias=0;

  int i;
  int isBiasRamp;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    amplBiasAddr[0] = GetNiosAddr("ampl_500_bias");
    amplBiasAddr[1] = GetNiosAddr("ampl_350_bias");
    amplBiasAddr[2] = GetNiosAddr("ampl_250_bias");
    amplBiasAddr[3] = GetNiosAddr("ampl_rox_bias");
    amplBiasAddr[4] = GetNiosAddr("ampl_x_bias");
    rampEnaBiasAddr = GetNiosAddr("ramp_ena_bias");
    rampAmplBiasAddr = GetBiPhaseAddr("ramp_ampl_bias");
    stepEnaBiasAddr = GetNiosAddr("step_ena_bias");
    stepStartBiasAddr = GetNiosAddr("step_start_bias");
    stepEndBiasAddr = GetNiosAddr("step_end_bias");
    stepNBiasAddr = GetNiosAddr("step_n_bias");
    stepTimeBiasAddr = GetNiosAddr("step_time_bias");
    stepPulLenBiasAddr = GetNiosAddr("step_pul_len_bias");
    stepArrayBiasAddr = GetNiosAddr("step_array_bias");
  }

  /* Check to make sure that the user selected an array.  0 means step all arrays.*/
  if (CommandData.Bias.biasStep.do_step) {
    switch (CommandData.Bias.biasStep.arr_ind) {
    case 250:
      i_arr = 2;
      break;
    case 350:
      i_arr = 1;
      break;
    case 500:
      i_arr = 0;
      break;
    case 0:
      i_arr = 3; // i.e. all wavelengths
      break;
    default:
      CommandData.Bias.biasStep.do_step = 0; // don't step
      break;
    }
  } 


    /************* Set the Bias Levels *******/
  for (i=0; i<5; i++) {
    if (CommandData.Bias.setLevel[i]) {
      WriteData(amplBiasAddr[i], CommandData.Bias.bias[i]<<1, NIOS_QUEUE);
      CommandData.Bias.setLevel[i] = 0;
      CommandData.Bias.biasStep.do_step = 0;
    }
  }
  if (CommandData.Bias.biasStep.do_step) {
    if (k==0) {
      start = CommandData.Bias.biasStep.start;
      end = CommandData.Bias.biasStep.end;
      step_size=(end-start)/CommandData.Bias.biasStep.nsteps;

      if(step_size==0) { // minimum step size is 1
	if (end >= start) {
	  step_size=1;
	}
	if (end < start) {
	  step_size=-1;
	}
      }
      end +=step_size;
      dk = (unsigned int)(CommandData.Bias.biasStep.dt*SR/1000);
      if (dk == 0) 
	dk =1; 
      pulse_len_k = (unsigned int)(CommandData.Bias.biasStep.pulse_len*SR/1000);

      /* kp sets when in the step the cal pulse will be sent */
      /* NOTE: if pulse_len= 0, no pulse is sent. */
      kp=dk-4*pulse_len_k; 
      
      if (kp < 0) kp = 1;
    }

    bias = start+(k/dk)*step_size;

    if (step_size > 0) {
      if (bias >= end) CommandData.Bias.biasStep.do_step=0;
      if (bias > 32767) { 
	bias = 32767;
	CommandData.Bias.biasStep.do_step=0;
      }

    } else {
      if (bias <= end) {
	CommandData.Bias.biasStep.do_step=0; 
      }
      if (bias < 1) { 
	bias = 1;
	CommandData.Bias.biasStep.do_step=0;
      }
    }

    WriteData(stepEnaBiasAddr,CommandData.Bias.biasStep.do_step, NIOS_QUEUE);
    WriteData(stepStartBiasAddr,CommandData.Bias.biasStep.start<<1, NIOS_QUEUE);
    WriteData(stepEndBiasAddr,CommandData.Bias.biasStep.end<<1, NIOS_QUEUE);
    WriteData(stepNBiasAddr,CommandData.Bias.biasStep.nsteps, NIOS_QUEUE);
    WriteData(stepTimeBiasAddr,CommandData.Bias.biasStep.dt, NIOS_QUEUE);
    WriteData(stepPulLenBiasAddr,CommandData.Bias.biasStep.pulse_len, NIOS_QUEUE);
    WriteData(stepArrayBiasAddr,CommandData.Bias.biasStep.arr_ind, NIOS_QUEUE);
    if (i_arr >= 0 && i_arr < 3) {
      WriteData(amplBiasAddr[i_arr], bias<<1, NIOS_QUEUE);
    } else {
    for(i = 0; i <= 2; i++)
      WriteData(amplBiasAddr[i], bias<<1, NIOS_QUEUE);
    }

    /* Send a cal pulse at 4x the width of the cal pulse before the next step.*/
    /* pulse_len = 0 means don't send a cal pulse */
    if (k%dk==kp && pulse_len_k >0) {
      CommandData.Cryo.calibrator = pulse;
      CommandData.Cryo.calib_pulse = pulse_len_k;
    }
    k++;
  } else {
    k=0;
    /********** set Bias (ramp)  *******/
    isBiasRamp = slow_data[rampAmplBiasAddr->index][rampAmplBiasAddr->channel];
    if ( (isBiasRamp && CommandData.Bias.biasRamp == 0) ||
	 (!isBiasRamp && CommandData.Bias.biasRamp == 1) ) {
      WriteData(rampEnaBiasAddr, CommandData.Bias.biasRamp, NIOS_QUEUE);
    }
    
  }
}
