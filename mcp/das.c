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

#include <stdio.h>
#include <time.h>

#include "mcp.h"
#include "channels.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"

/* Cryostat digital signals (G2 and G3 outputs) */
#define CRYO_COLDPLATE_ON    0x10 /* N3G3 - cryoout3 */
#define CRYO_COLDPLATE_OFF   0x20 /* N3G3 */
#define CRYO_CALIBRATOR_ON   0x40 /* N3G3 */
#define CRYO_CALIBRATOR_OFF  0x80 /* N3G3 */
#define CRYO_HELIUMLEVEL_ON  0x01 /* N3G3 */
#define CRYO_HELIUMLEVEL_OFF 0x02 /* N3G3 */
#define CRYO_CHARCOAL_ON     0x04 /* N3G3 */
#define CRYO_CHARCOAL_OFF    0x08 /* N3G3 */

#define CRYO_POTVALVE_ON      0x10 /* N3G2 - cryoout2 */
#define CRYO_LNVALVE_ON       0x20 /* N3G2 */
#define CRYO_POTVALVE_OPEN    0x40 /* N3G2 Group two of the cryo card */
#define CRYO_POTVALVE_CLOSE   0x80 /* N3G2 appears to have its nybbles */
#define CRYO_LHeVALVE_ON      0x01 /* N3G2 backwards! */
#define CRYO_LVALVE_OPEN      0x04 /* N3G2 */
#define CRYO_LVALVE_CLOSE     0x08 /* N3G2 */

/* CryoControl bits */
#define CRYOCTRL_CALON     0x4000
#define CRYOCTRL_BDAHEATON 0x8000
#define CRYOCTRL_PULSE_INDEX(x) ((x & 0x3) << 12)
#define CRYOCTRL_PULSE_LEN(x) (x & 0x00FF)

/* he3 cycle */
#define CRYO_CYCLE_OUT_OF_HELIUM 0x0000
#define CRYO_CYCLE_COLD          0x0001
#define CRYO_CYCLE_ON            0x0002
#define CRYO_CYCLE_COOL          0x0004

#define CRYO_CYCLE_TIMEOUT       (45*60)
#define CRYO_CYCLE_COOL_TIMEOUT  (2*60*60)

#define T_HE3FRIDGE_TOO_HOT      3.733304    /* 0.391508    K */
#define T_HE3FRIDGE_COLD         4.34106     /* 0.326861    K */
#define T_HE4POT_SET             1.368951538 /* 2.312414576 K */
#define T_CHARCOAL_SET           6.693948    /* 25          K */
#define T_LHE_SET                9.39198     /* 4.4         K */

/* CryoState bitfield */
#define CS_HELIUMLEVEL    0x0001
#define CS_CHARCOAL       0x0002
#define CS_COLDPLATE      0x0004
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
  static struct NiosStruct* NiosAddr[DAS_CARDS];
  char field[20];
  int i;

  if (first_time) {
    first_time = 0;
    for(i = 0; i < DAS_CARDS; i++) {
      sprintf(field, "phase%d", i+5);
      NiosAddr[i] = GetNiosAddr(field);
    }
  }	

  for(i = 0; i < DAS_CARDS; i++)
    WriteData(NiosAddr[i], CommandData.Phase[i], NIOS_FLUSH);
}

/***********************************************************************/
/* CalLamp: Flash calibrator                                           */
/***********************************************************************/
int CalLamp (void)
{
  static struct NiosStruct* calPulseAddr;
  static int update_counter = 0;
  static unsigned int elapsed = 0;
  static enum calmode last_mode = off;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    calPulseAddr = GetNiosAddr("cal_pulse");
  }

  /* Save the current pulse length */
  WriteData(calPulseAddr, CommandData.Cryo.calib_pulse, NIOS_QUEUE);

  if (CommandData.Cryo.calibrator == on) {
    if (last_mode != on) {
      update_counter = (update_counter + 1) & 3;
      last_mode = on;
    }
    return (CRYOCTRL_PULSE_INDEX(update_counter) | CRYOCTRL_CALON
        | CRYOCTRL_PULSE_LEN(CommandData.Cryo.calib_pulse));
  } else if (CommandData.Cryo.calibrator == repeat) {
    if (last_mode != repeat || elapsed >= CommandData.Cryo.calib_period) {
      /* end of cycle -- send a new pulse */
      elapsed = 0;
      update_counter = (update_counter + 1) & 3;
    } else
      elapsed++;
    last_mode = repeat;
  } else { /* off or single pulse mode */
    if (CommandData.Cryo.calibrator == pulse)
      update_counter = (update_counter + 1) & 3;
    last_mode = CommandData.Cryo.calibrator = off;
  }

  return (CRYOCTRL_PULSE_INDEX(update_counter)
      | CRYOCTRL_PULSE_LEN(CommandData.Cryo.calib_pulse));
}

/* Automatic conrol of JFET heater */
int JFETthermostat(void)
{
  double jfet_temp;
  static struct BiPhaseStruct* tJfetAddr;

  static struct LutType DiodeLut = {"/data/etc/dt600.txt", 0, NULL, NULL, 0};

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    tJfetAddr = GetBiPhaseAddr("t_jfet");
    LutInit(&DiodeLut);
  }

  jfet_temp = (double)slow_data[tJfetAddr->index][tJfetAddr->channel]
    * T_JFET_M + T_JFET_B;
  jfet_temp = LutCal(&DiodeLut, jfet_temp);

  if (jfet_temp < 0 || jfet_temp > 400)
    return CommandData.Cryo.JFETHeat;
  else if (jfet_temp > CommandData.Cryo.JFETSetOff)
    return 0;
  else if (jfet_temp < CommandData.Cryo.JFETSetOn)
    return 2047;
  else
    return 2047 * (CommandData.Cryo.JFETSetOff - jfet_temp) /
      (CommandData.Cryo.JFETSetOff - CommandData.Cryo.JFETSetOn);
}

void FridgeCycle(int *cryoout, int *cryostate, int  reset)
{
  static int firsttime = 1;
  static struct BiPhaseStruct* t_lhe_Addr;
  static struct BiPhaseStruct* t_he3fridge_Addr;
  static struct BiPhaseStruct* t_charcoal_Addr;
  static struct BiPhaseStruct* t_he4pot_Addr;
  static struct NiosStruct*    cycle_start_W_Addr;
  static struct BiPhaseStruct* cycle_start_R_Addr;
  static struct NiosStruct*    cycle_state_W_Addr;
  static struct BiPhaseStruct* cycle_state_R_Addr;

  double t_lhe, t_he3fridge, t_charcoal, t_he4pot;

  time_t start_time;
  unsigned short cycle_state;
  static unsigned short iterator = 1;

  if (firsttime) {
    firsttime = 0;
    t_lhe_Addr = GetBiPhaseAddr("t_lhe");
    t_he3fridge_Addr = GetBiPhaseAddr("t_he3fridge");
    t_charcoal_Addr = GetBiPhaseAddr("t_charcoal");
    t_he4pot_Addr = GetBiPhaseAddr("t_he4pot");
    cycle_start_W_Addr = GetNiosAddr("cycle_start");
    cycle_start_R_Addr = ExtractBiPhaseAddr(cycle_start_W_Addr);
    cycle_state_W_Addr = GetNiosAddr("cycle_state");
    cycle_state_R_Addr = ExtractBiPhaseAddr(cycle_state_W_Addr);
    WriteData(cycle_state_W_Addr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
    return;
  }

  if (reset) {
   WriteData(cycle_state_W_Addr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
   iterator = 1;
   return;
  }

  if(iterator++ % 10)  /* Run this loop at 0.5 Hz */
    return;

  start_time
    = slow_data[cycle_start_R_Addr->index][cycle_start_R_Addr->channel];
  start_time |=
    slow_data[cycle_start_R_Addr->index][cycle_start_R_Addr->channel+1] << 16;
  cycle_state
    = slow_data[cycle_state_R_Addr->index][cycle_state_R_Addr->channel];

  t_lhe = (double)slow_data[t_lhe_Addr->index][t_lhe_Addr->channel];
  t_charcoal
    = (double)slow_data[t_charcoal_Addr->index][t_charcoal_Addr->channel];
  t_he3fridge
    = (double)(slow_data[t_he3fridge_Addr->index][t_he3fridge_Addr->channel] +
		    (slow_data[t_he3fridge_Addr->index][t_he3fridge_Addr->channel + 1] << 16) );
  t_he4pot
    = (double)(slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel] +
       		(slow_data[t_he4pot_Addr->index][t_he4pot_Addr->channel+1] << 16) );	

  t_lhe       = T_LHE_M*t_lhe + T_LHE_B;
  t_charcoal  = T_CHARCOAL_M*t_charcoal + T_CHARCOAL_B;
  t_he3fridge = (ROX_C2V)*t_he3fridge + ROX_OFFSET;
  t_he4pot    = (ROX_C2V)*t_he4pot    + ROX_OFFSET;
  
  if (t_lhe < T_LHE_SET) {
    *cryoout |= CRYO_CHARCOAL_OFF;
    *cryostate &= ~CS_CHARCOAL;
    WriteData(cycle_state_W_Addr, CRYO_CYCLE_OUT_OF_HELIUM, NIOS_QUEUE);
    return;
  } 

  if (cycle_state == CRYO_CYCLE_COLD) {
    if(t_he3fridge < T_HE3FRIDGE_TOO_HOT && t_he4pot > T_HE4POT_SET) {
      WriteData(cycle_state_W_Addr, CRYO_CYCLE_ON, NIOS_QUEUE);
      WriteData(cycle_start_W_Addr, time(NULL), NIOS_QUEUE);
      *cryoout |= CRYO_CHARCOAL_ON;
      *cryostate |= CS_CHARCOAL;
      bprintf(info, "Auto Cycle: Turning charcoal heat on.");
      return;
    }
    *cryoout |= CRYO_CHARCOAL_OFF;
    *cryostate &= ~CS_CHARCOAL;
  } else if (cycle_state == CRYO_CYCLE_ON) {
    if (((time(NULL) - start_time) > CRYO_CYCLE_TIMEOUT) ||
        t_charcoal < T_CHARCOAL_SET ||
        t_he4pot < T_HE4POT_SET) {
      WriteData(cycle_state_W_Addr, CRYO_CYCLE_COOL, NIOS_QUEUE);
      *cryoout |= CRYO_CHARCOAL_OFF;
      *cryostate &= ~CS_CHARCOAL;
      bprintf(info, "Auto Cycle: Turning charcoal heat off.");
      return;
    }
    *cryoout |= CRYO_CHARCOAL_ON;
    *cryostate |= CS_CHARCOAL;
  } else if ( cycle_state == CRYO_CYCLE_COOL) {
    if ((t_he3fridge > T_HE3FRIDGE_COLD)
        || ((time(NULL) - start_time) > CRYO_CYCLE_COOL_TIMEOUT) ) {
      WriteData(cycle_state_W_Addr, CRYO_CYCLE_COLD, NIOS_QUEUE);
      *cryoout |= CRYO_CHARCOAL_OFF;
      *cryostate &= ~CS_CHARCOAL;
      bprintf(info, "Auto Cycle: Fridge is now cold!.");
      return;
    }
    *cryoout |= CRYO_CHARCOAL_OFF;
    *cryostate &= ~CS_CHARCOAL;
  } else if (cycle_state == CRYO_CYCLE_OUT_OF_HELIUM) {
    WriteData(cycle_state_W_Addr, CRYO_CYCLE_COLD, NIOS_QUEUE);
    *cryoout |= CRYO_CHARCOAL_OFF;
    *cryostate &= ~CS_CHARCOAL;
    bprintf(info, "Auto Cycle: Everything A-OK.");
  } else {
    bprintf(err, "cycle_state: %i unknown!", cycle_state);
    *cryoout |= CRYO_CHARCOAL_OFF;
    *cryostate &= ~CS_CHARCOAL;
    return;
  }
  return;
}

/***********************************************************************/
/* CryoControl: Control heaters and calibrator (a slow control)        */
/***********************************************************************/
void CryoControl (void)
{
  static struct NiosStruct* cryoout2Addr;
  static struct NiosStruct* cryoout3Addr;
  static struct NiosStruct* cryostateAddr;
  static struct NiosStruct* cryopwmAddr;
  static struct NiosStruct* jfetpwmAddr;
  static struct NiosStruct* hspwmAddr;
  static struct NiosStruct* bdapwmAddr;
  static struct NiosStruct* cryoctrlAddr;
  static struct NiosStruct* gPBdaheatAddr;
  static struct NiosStruct* gIBdaheatAddr;
  static struct NiosStruct* gDBdaheatAddr;
  static struct NiosStruct* gFlBdaheatAddr;
  static struct NiosStruct* setBdaheatAddr;
  static struct NiosStruct* jfetSetOnAddr;
  static struct NiosStruct* jfetSetOffAddr;

  int cryoout3 = 0, cryoout2 = 0;
  static int cryostate = 0;
  int cryoctrl;
  int jfetHeat;

  /************** Set indices first time around *************/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    bdapwmAddr = GetNiosAddr("bdapwm");
    cryoctrlAddr = GetNiosAddr("cryoctrl");
    cryoout2Addr = GetNiosAddr("cryoout2");
    cryoout3Addr = GetNiosAddr("cryoout3");
    cryostateAddr = GetNiosAddr("cryostate");
    gPBdaheatAddr = GetNiosAddr("g_p_bdaheat");
    gIBdaheatAddr = GetNiosAddr("g_i_bdaheat");
    gDBdaheatAddr = GetNiosAddr("g_d_bdaheat");
    gFlBdaheatAddr = GetNiosAddr("g_fl_bdaheat");
    setBdaheatAddr = GetNiosAddr("set_bdaheat");
    cryopwmAddr = GetNiosAddr("cryopwm");
    hspwmAddr = GetNiosAddr("hspwm");
    jfetpwmAddr = GetNiosAddr("jfetpwm");
    jfetSetOnAddr = GetNiosAddr("jfet_set_on");
    jfetSetOffAddr = GetNiosAddr("jfet_set_off");
  }

  /********** Set Output Bits **********/
  if (CommandData.Cryo.heliumLevel == 0) {
    cryoout3 |= CRYO_HELIUMLEVEL_OFF;
    cryostate &= 0xFFFF - CS_HELIUMLEVEL;
  } else {
    cryoout3 |= CRYO_HELIUMLEVEL_ON;
    cryostate |= CS_HELIUMLEVEL;
  }

  if (CommandData.Cryo.fridgeCycle) {
    FridgeCycle(&cryoout3, &cryostate, 0); 
  } else {
    FridgeCycle(&cryoout3, &cryostate, 1);
    if (CommandData.Cryo.charcoalHeater == 0) {
      cryoout3 |= CRYO_CHARCOAL_OFF;
      cryostate &= 0xFFFF - CS_CHARCOAL;
    } else {
      cryoout3 |= CRYO_CHARCOAL_ON;
      cryostate |= CS_CHARCOAL;
    }
  }
  if (CommandData.Cryo.coldPlate == 0) {
    cryoout3 |= CRYO_COLDPLATE_OFF;
    cryostate &= 0xFFFF - CS_COLDPLATE;
  } else {
    cryoout3 |= CRYO_COLDPLATE_ON;
    cryostate |= CS_COLDPLATE;
  }

  cryoctrl = CalLamp();
  if (CommandData.Cryo.autoBDAHeat)
    cryoctrl |= CRYOCTRL_BDAHEATON;

  cryoout2 = CRYO_POTVALVE_OPEN | CRYO_POTVALVE_CLOSE | 
    CRYO_LVALVE_OPEN | CRYO_LVALVE_CLOSE;

  /* Control motorised valves -- latching relays */
  if (CommandData.Cryo.potvalve_open > 0) {
    cryoout2 &= ~CRYO_POTVALVE_OPEN;
    cryostate |= CS_POTVALVE_OPEN;
    CommandData.Cryo.potvalve_open--;
  } else if (CommandData.Cryo.potvalve_close > 0) {
    cryoout2 &= ~CRYO_POTVALVE_CLOSE;
    cryostate &= 0xFFFF - CS_POTVALVE_OPEN;
    CommandData.Cryo.potvalve_close--;
  }
  if (CommandData.Cryo.potvalve_on) {
    cryoout2 |= CRYO_POTVALVE_ON;
    cryostate |= CS_POTVALVE_ON;
  } else
    cryostate &= 0xFFFF - CS_POTVALVE_ON;

  if (CommandData.Cryo.lvalve_open > 0) {
    cryoout2 &= ~CRYO_LVALVE_OPEN;
    cryostate |= CS_LVALVE_OPEN;
    CommandData.Cryo.lvalve_open--;
  } else if (CommandData.Cryo.lvalve_close > 0) {
    cryoout2 &= ~CRYO_LVALVE_CLOSE;
    cryostate &= 0xFFFF - CS_LVALVE_OPEN;
    CommandData.Cryo.lvalve_close--;
  }
  if (CommandData.Cryo.lhevalve_on) {
    cryoout2 |= CRYO_LHeVALVE_ON;
    cryostate |= CS_LHeVALVE_ON;
  } else
    cryostate &= 0xFFFF - CS_LHeVALVE_ON;
  if (CommandData.Cryo.lnvalve_on) {
    cryoout2 |= CRYO_LNVALVE_ON;
    cryostate |= CS_LNVALVE_ON;
  } else
    cryostate &= 0xFFFF - CS_LNVALVE_ON;

  if (CommandData.Cryo.autoJFETheat) {
    jfetHeat = JFETthermostat();
    cryostate |= CS_AUTO_JFET;
  } else {
    jfetHeat = CommandData.Cryo.JFETHeat;
    cryostate &= 0xFFFF - CS_AUTO_JFET;
  }

  WriteData(cryoout3Addr, cryoout3, NIOS_QUEUE);
  WriteData(cryoout2Addr, cryoout2, NIOS_QUEUE);
  WriteData(cryostateAddr, cryostate, NIOS_QUEUE);
  WriteData(cryopwmAddr, CommandData.Cryo.CryoSparePWM, NIOS_QUEUE);
  WriteData(hspwmAddr, CommandData.Cryo.heatSwitch, NIOS_QUEUE);
  WriteData(jfetpwmAddr, jfetHeat, NIOS_QUEUE);
  WriteData(setBdaheatAddr, CommandData.Cryo.BDAGain.SP, NIOS_QUEUE);
  WriteData(gPBdaheatAddr, CommandData.Cryo.BDAGain.P, NIOS_QUEUE);
  WriteData(gIBdaheatAddr, CommandData.Cryo.BDAGain.I, NIOS_QUEUE);
  WriteData(gDBdaheatAddr, CommandData.Cryo.BDAGain.D, NIOS_QUEUE);
  WriteData(gFlBdaheatAddr, CommandData.Cryo.BDAFiltLen, NIOS_QUEUE);
  WriteData(bdapwmAddr, CommandData.Cryo.BDAHeat, NIOS_QUEUE);
  WriteData(jfetSetOnAddr, CommandData.Cryo.JFETSetOn * 100, NIOS_QUEUE);
  WriteData(jfetSetOffAddr, CommandData.Cryo.JFETSetOff * 100, NIOS_QUEUE);
  WriteData(cryoctrlAddr, cryoctrl, NIOS_FLUSH);
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Digital IO with the Bias Generator Card               */
/*                                                                      */
/************************************************************************/
void BiasControl (unsigned short* RxFrame) {
  static struct BiPhaseStruct* biasinAddr;
  static struct NiosStruct* biasout1Addr;
  static struct NiosStruct* biasout2Addr;
  static struct NiosStruct* biasLev1Addr;
  static struct NiosStruct* biasLev2Addr;
  static struct NiosStruct* biasLev3Addr;
  unsigned short bias_status, biasout1 = 0;
  static unsigned short biasout2 = 0x70,
                        biaslsbs1=0; /* biaslsbs1 holds 2 lsbs for bias */
  int isBiasAC, isBiasRamp, isBiasClockInternal;
  static int hold = 0, ch = 0, rb_hold = 0;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    biasinAddr = GetBiPhaseAddr("biasin");
    biasout1Addr = GetNiosAddr("biasout1");
    biasout2Addr = GetNiosAddr("biasout2");
    biasLev1Addr = GetNiosAddr("bias_lev1");
    biasLev2Addr = GetNiosAddr("bias_lev2");
    biasLev3Addr = GetNiosAddr("bias_lev3");
  }

  bias_status = slow_data[biasinAddr->index][biasinAddr->channel];

  isBiasAC =            !(bias_status & 0x02);
  isBiasRamp =          !(bias_status & 0x08);
  isBiasClockInternal = ((bias_status & 0x04) >> 2);

  biasout1 = 0;
  /********** set Bias AC/DC *******/
  if (isBiasAC) { /*  Bias is currently AC */
    if (CommandData.Bias.biasAC == 0) { /* it should be DC */
      biasout1 |= 0x01;
      /*      bprintf(info, "to DC\n"); */
    }
  } else { /* Bias is currently DC */
    if (CommandData.Bias.biasAC == 1) { /* it should be AC */
      biasout1 |= 0x02;
      /*      bprintf(info, "to AC\n"); */
    }
  }

  /********** set Bias Internal/External (ramp)  *******/
  if (isBiasRamp) { /* Bias is currently external Ramp */
    if (CommandData.Bias.biasRamp == 0) { /* it should be internal/fixed */
      biasout1 |= 0x40;
      /*      bprintf(info, "to fixed\n"); */
    }
  } else { /* Bias is currently internal (fixed) */
    if (CommandData.Bias.biasRamp == 1) { /* it should be external/Ramp */
      biasout1 |= 0x80;
      /*      bprintf(info, "to ramp\n"); */
    }
  }

  /********** set Clock Internal/External *******/
  if (isBiasClockInternal) { /* Bias is currently internal */
    if (CommandData.Bias.clockInternal == 0) { /* it should be external */
      biasout1 |= 0x10;
      /*      bprintf(info, "to external\n"); */
    }
  } else { /* Bias clock is currenly external */
    if (CommandData.Bias.clockInternal == 1) { /* it should be internal */
      biasout1 |= 0x20;
      /*      bprintf(info, "to internal\n"); */
    }
  }

  if (CommandData.Bias.SetLevel1 || CommandData.Bias.SetLevel2 ||
      CommandData.Bias.SetLevel3) {
    rb_hold = 400;
  }

  /************* Set the Bias Levels *******/
  if (hold > FAST_PER_SLOW + 2) { /* hold data with write low */
    hold--;
  } else if (hold > 0) {  /* hold data with write high */
    biasout2 |= 0x07;
    hold--;
  } else if (ch == 0) {
    if (CommandData.Bias.SetLevel1) {
      biasout2 = ((CommandData.Bias.bias1 << 1) & 0xf8) | 0x03;
      biaslsbs1 = (~(CommandData.Bias.bias1 << 2) & 0x0c);
      hold = 2 * FAST_PER_SLOW + 4;
      rb_hold = 400;
      CommandData.Bias.SetLevel1 = 0;
    }
    ch++;
  } else if (ch == 1) {
    if (CommandData.Bias.SetLevel2) {
      biasout2 = ((CommandData.Bias.bias2 << 1) & 0xf8) | 0x05;
      biaslsbs1 = (~(CommandData.Bias.bias2 << 2) & 0x0c);
      hold = 2 * FAST_PER_SLOW + 4;
      rb_hold = 400;
      CommandData.Bias.SetLevel2 = 0;
    }
    ch++;
  } else if (ch == 2) {
    if (CommandData.Bias.SetLevel3) {
      biasout2 = ((CommandData.Bias.bias3 << 1) & 0xf8) | 0x06;
      biaslsbs1 = (~(CommandData.Bias.bias3 << 2) & 0x0c);
      hold = 2 * FAST_PER_SLOW + 4;
      rb_hold = 400;
      CommandData.Bias.SetLevel3 = 0;
    }
    ch = 0;
  } else {
    bprintf(err, "ch is an impossible value in bias control (%i)\n", ch);
    ch = 0;
  }

  /* Bias readback -- we wait a few seconds after finishing the write */
  if (rb_hold > 0) {
    rb_hold--;
  } else if (!CommandData.Bias.SetLevel3 && !CommandData.Bias.SetLevel3 &&
      !CommandData.Bias.SetLevel3) {
    /*     CommandData.Bias.bias1 = slow_data[Bias_lev1Ch][Bias_lev1Ind]; */
    /*     CommandData.Bias.bias2 = slow_data[Bias_lev2Ch][Bias_lev2Ind]; */
    /*     CommandData.Bias.bias3 = slow_data[Bias_lev3Ch][Bias_lev3Ind]; */
  }
  /*  Add the two lsbs of the bias to biasout1  */
  biasout1 |= biaslsbs1;

  /******************** set the outputs *********************/
  WriteData(biasout1Addr, biasout1 & 0xffff, NIOS_QUEUE);
  WriteData(biasout2Addr, (~biasout2) & 0xff, NIOS_QUEUE);
  WriteData(biasLev1Addr, CommandData.Bias.bias1, NIOS_QUEUE);
  WriteData(biasLev2Addr, CommandData.Bias.bias2, NIOS_QUEUE);
  WriteData(biasLev3Addr, CommandData.Bias.bias3, NIOS_FLUSH);
}
