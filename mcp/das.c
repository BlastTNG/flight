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
#define CRYO_LHeVALVE_ON     0x01 /* N3G2 backwards */
#define CRYO_LHeVALVE_OPEN   0x04 /* N3G2 */
#define CRYO_LHeVALVE_CLOSE  0x08 /* N3G2 */

/* CryoState bitfield */
#define CS_HELIUMLEVEL   0x0001
#define CS_CHARCOAL      0x0002
#define CS_COLDPLATE     0x0004
#define CS_CALIBRATOR    0x0008
#define CS_POTVALVE_ON    0x0010
#define CS_POTVALVE_OPEN  0x0020
#define CS_LHeVALVE_ON   0x0040
#define CS_LHeVALVE_OPEN 0x0080

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
int CalLamp (int* cryostate)
{
  int calLamp;
  static struct NiosStruct* calPulseAddr;
  static int pulse_left = 0;
  static int repeat_left = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    calPulseAddr = GetNiosAddr("cal_pulse");
  }

  if (CommandData.Cryo.calibrator == 1) {
    calLamp = CRYO_CALIBRATOR_ON;
    *cryostate |= CS_CALIBRATOR;
  } else {
    if (CommandData.Cryo.calib_repeat == -1) {  /* single pulse request */
      pulse_left = CommandData.Cryo.calib_pulse + 1;
      CommandData.Cryo.calib_repeat = 0;
      repeat_left = 0;
    }
    
    if (repeat_left == 0) {
      if (CommandData.Cryo.calib_repeat > 0 && /* we're repeating and */
          CommandData.Cryo.calib_pulse > 0) {  /* just got to the end */
        repeat_left = CommandData.Cryo.calib_repeat * 100 - 1;
        pulse_left = CommandData.Cryo.calib_pulse + 1;
      }
    } else {  /* we're repeat pulsing, and counting down to next pulse */
      repeat_left--;  
    }

    if (pulse_left > 0) {  /* pulser on, turn on lamp */
      calLamp = CRYO_CALIBRATOR_ON;
      *cryostate |= CS_CALIBRATOR;
      pulse_left--; 
      WriteData(calPulseAddr, 1, NIOS_FLUSH);
    } else {               /* pulser off, or we're not pulsing, turn off lamp */
      calLamp = CRYO_CALIBRATOR_OFF;
      *cryostate &= 0xFFFF - CS_CALIBRATOR;
      WriteData(calPulseAddr, 0, NIOS_FLUSH);
    }
  }

  return calLamp;
}

/***********************************************************************/
/* CryoControl: Set heaters to values contained within the CommandData */
/***********************************************************************/
void CryoControl (void)
{
  static struct NiosStruct* cryoout2Addr;
  static struct NiosStruct* cryoout3Addr;
  static struct NiosStruct* cryostateAddr;
  static struct NiosStruct* he3pwmAddr;
  static struct NiosStruct* jfetpwmAddr;
  static struct NiosStruct* hspwmAddr;
  static struct NiosStruct* cryopwmAddr;
  static struct NiosStruct* calRepeatAddr;
  static struct NiosStruct* pulseLenAddr;

  int cryoout3 = 0, cryoout2 = 0;
  static int cryostate = 0;

  /************** Set indices first time around *************/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    cryoout3Addr = GetNiosAddr("cryoout3");

    cryoout2Addr = GetNiosAddr("cryoout2");
    cryostateAddr = GetNiosAddr("cryostate");
    he3pwmAddr = GetNiosAddr("he3pwm");
    jfetpwmAddr = GetNiosAddr("jfetpwm");
    hspwmAddr = GetNiosAddr("hspwm");
    cryopwmAddr = GetNiosAddr("cryopwm");

    pulseLenAddr = GetNiosAddr("pulse_len");
    calRepeatAddr = GetNiosAddr("cal_repeat");
  }

  /* We want to save these here since CalLamp might destroy calib_repeat */
  WriteData(pulseLenAddr, CommandData.Cryo.calib_pulse, NIOS_QUEUE);
  WriteData(calRepeatAddr, CommandData.Cryo.calib_repeat, NIOS_QUEUE);

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
  cryoout3 |= CalLamp(&cryostate);

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
  WriteData(cryopwmAddr, CommandData.Cryo.sparePwm, NIOS_FLUSH);
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
  static unsigned short biasout2 = 0x70;
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
    biasout2 |= 0x0f;
    hold--;
  } else if (ch == 0) {
    if (CommandData.Bias.SetLevel1) {
      biasout2 = ((CommandData.Bias.bias1 << 4) & 0xf0) | 0x03;
      hold = 2 * FAST_PER_SLOW + 4;
      rb_hold = 400;
      CommandData.Bias.SetLevel1 = 0;
    }
    ch++;
  } else if (ch == 1) {
    if (CommandData.Bias.SetLevel2) {
      biasout2 = ((CommandData.Bias.bias2 << 4) & 0xf0) | 0x05;
      hold = 2 * FAST_PER_SLOW + 4;
      rb_hold = 400;
      CommandData.Bias.SetLevel2 = 0;
    }
    ch++;
  } else if (ch == 2) {
    if (CommandData.Bias.SetLevel3) {
      biasout2 = ((CommandData.Bias.bias3 << 4) & 0xf0) | 0x06;
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

  /******************** set the outputs *********************/
  WriteData(biasout1Addr, biasout1 & 0xffff, NIOS_QUEUE);
  WriteData(biasout1Addr, (~biasout2) & 0xff, NIOS_QUEUE);
  WriteData(biasLev1Addr, CommandData.Bias.bias1, NIOS_QUEUE);
  WriteData(biasLev2Addr, CommandData.Bias.bias2, NIOS_QUEUE);
  WriteData(biasLev3Addr, CommandData.Bias.bias3, NIOS_FLUSH);
}
