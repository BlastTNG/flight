/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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
#include "tx_struct.h"
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
#define CRYO_POTVALVE_OPEN    0x40 /* N3G2 Group two of the cryo card */
#define CRYO_POTVALVE_CLOSE   0x80 /* N3G2 appears to have its nybbles */
#define CRYO_LHeVALVE_ON      0x01 /* N3G2 backwards */
#define CRYO_LHeVALVE_OPEN    0x04 /* N3G2 */
#define CRYO_LHeVALVE_CLOSE   0x08 /* N3G2 */

/* CryoControl bits */
#define CRYOCTRL_CALON     0x4000
#define CRYOCTRL_BDAHEATON 0x8000
#define CRYOCTRL_PULSE_INDEX(x) ((x & 0x3) << 12)
#define CRYOCTRL_PULSE_LEN(x) (x & 0x00FF)

/* CryoState bitfield */
#define CS_HELIUMLEVEL    0x0001
#define CS_CHARCOAL       0x0002
#define CS_COLDPLATE      0x0004
#define CS_POTVALVE_ON    0x0010
#define CS_POTVALVE_OPEN  0x0020
#define CS_LHeVALVE_ON    0x0040
#define CS_LHeVALVE_OPEN  0x0080

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

/***********************************************************************/
/* CryoControl: Control heaters and calibrator (a slow control)        */
/***********************************************************************/
void CryoControl (void)
{
  static struct NiosStruct* cryoout2Addr;
  static struct NiosStruct* cryoout3Addr;
  static struct NiosStruct* cryostateAddr;
  static struct NiosStruct* he3pwmAddr;
  static struct NiosStruct* jfetpwmAddr;
  static struct NiosStruct* hspwmAddr;
  static struct NiosStruct* bdapwmAddr;
  static struct NiosStruct* cryoctrlAddr;
  static struct NiosStruct* gPBdaheatAddr;
  static struct NiosStruct* gIBdaheatAddr;
  static struct NiosStruct* gDBdaheatAddr;
  static struct NiosStruct* gFlBdaheatAddr;
  static struct NiosStruct* setBdaheatAddr;

  int cryoout3 = 0, cryoout2 = 0;
  static int cryostate = 0;
  int cryoctrl;

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
    he3pwmAddr = GetNiosAddr("he3pwm");
    hspwmAddr = GetNiosAddr("hspwm");
    jfetpwmAddr = GetNiosAddr("jfetpwm");
  }

  /********** Set Output Bits **********/
  if (CommandData.Cryo.heliumLevel == 0) {
    cryoout3 |= CRYO_HELIUMLEVEL_OFF;
    cryostate &= 0xFFFF - CS_HELIUMLEVEL;
  } else {
    cryoout3 |= CRYO_HELIUMLEVEL_ON;
    cryostate |= CS_HELIUMLEVEL;
  }
  if (CommandData.Cryo.charcoalHeater == 0) {
    cryoout3 |= CRYO_CHARCOAL_OFF;
    cryostate &= 0xFFFF - CS_CHARCOAL;
  } else {
    cryoout3 |= CRYO_CHARCOAL_ON;
    cryostate |= CS_CHARCOAL;
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
    CRYO_LHeVALVE_OPEN | CRYO_LHeVALVE_CLOSE;

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

  if (CommandData.Cryo.lhevalve_open > 0) {
    cryoout2 &= ~CRYO_LHeVALVE_OPEN;
    cryostate |= CS_LHeVALVE_OPEN;
    CommandData.Cryo.lhevalve_open--;
  } else if (CommandData.Cryo.lhevalve_close > 0) {
    cryoout2 &= ~CRYO_LHeVALVE_CLOSE;
    cryostate &= 0xFFFF - CS_LHeVALVE_OPEN;
    CommandData.Cryo.lhevalve_close--;
  }
  if (CommandData.Cryo.lhevalve_on) {
    cryoout2 |= CRYO_LHeVALVE_ON;
    cryostate |= CS_LHeVALVE_ON;
  } else
    cryostate &= 0xFFFF - CS_LHeVALVE_ON;

  WriteData(cryoout3Addr, cryoout3, NIOS_QUEUE);
  WriteData(cryoout2Addr, cryoout2, NIOS_QUEUE);
  WriteData(cryostateAddr, cryostate, NIOS_QUEUE);
  WriteData(he3pwmAddr, CommandData.Cryo.heliumThree, NIOS_QUEUE);
  WriteData(hspwmAddr, CommandData.Cryo.heatSwitch, NIOS_QUEUE);
  WriteData(jfetpwmAddr, CommandData.Cryo.JFETHeat, NIOS_QUEUE);
  WriteData(setBdaheatAddr, CommandData.Cryo.BDAGain.SP, NIOS_QUEUE);
  WriteData(gPBdaheatAddr, CommandData.Cryo.BDAGain.P, NIOS_QUEUE);
  WriteData(gIBdaheatAddr, CommandData.Cryo.BDAGain.I, NIOS_QUEUE);
  WriteData(gDBdaheatAddr, CommandData.Cryo.BDAGain.D, NIOS_QUEUE);
  WriteData(gFlBdaheatAddr, CommandData.Cryo.BDAFiltLen, NIOS_QUEUE);
  WriteData(bdapwmAddr, CommandData.Cryo.BDAHeat, NIOS_QUEUE);
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
  static unsigned short biasout2 = 0x70, biaslsbs1=0; /* biaslsbs1 holds 2 lsbs for bias */
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

  bias_status = RxFrame[biasinAddr->channel];

  isBiasAC =            !(bias_status & 0x02);
  isBiasRamp =          !(bias_status & 0x08);
  isBiasClockInternal = ((bias_status & 0x04) >> 2);

  biasout1 = 0;
  /********** set Bias AC/DC *******/
  if (isBiasAC) { /*  Bias is currently AC */
    if (CommandData.Bias.biasAC == 0) { /* it should be DC */
      biasout1 |= 0x01;
      /*      fprintf(stderr, "to DC\n"); */
    }
  } else { /* Bias is currently DC */
    if (CommandData.Bias.biasAC == 1) { /* it should be AC */
      biasout1 |= 0x02;
      /*      fprintf(stderr, "to AC\n"); */
    }
  }

  /********** set Bias Internal/External (ramp)  *******/
  if (isBiasRamp) { /* Bias is currently external Ramp */
    if (CommandData.Bias.biasRamp == 0) { /* it should be internal/fixed */
      biasout1 |= 0x40;
      /*      fprintf(stderr, "to fixed\n"); */
    }
  } else { /* Bias is currently internal (fixed) */
    if (CommandData.Bias.biasRamp == 1) { /* it should be external/Ramp */
      biasout1 |= 0x80;
      /*      fprintf(stderr, "to ramp\n"); */
    }
  }

  /********** set Clock Internal/External *******/
  if (isBiasClockInternal) { /* Bias is currently internal */
    if (CommandData.Bias.clockInternal == 0) { /* it should be external */
      biasout1 |= 0x10;
      /*      fprintf(stderr, "to external\n"); */
    }
  } else { /* Bias clock is currenly external */
    if (CommandData.Bias.clockInternal == 1) { /* it should be internal */
      biasout1 |= 0x20;
      /*      fprintf(stderr, "to internal\n"); */
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
    ch = 0;
    mprintf(MCP_ERROR, "ch is an impossible value in bias control\n");
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
