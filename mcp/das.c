#include <stdio.h>
#include <time.h>

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

extern unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

/************************************************************************/
/*                                                                      */
/* PhaseControl: set phase shifts for DAS cards                         */
/*                                                                      */
/************************************************************************/
void PhaseControl(unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW])
{
  static int first_time = 1;
  static int i_c[DAS_CARDS];
  static int j_c[DAS_CARDS];
  char field[20];
  int i;

  if(first_time) {
    first_time = 0;
    for(i = 0; i < DAS_CARDS; i++) {
      sprintf(field, "phase%d", i+5);
      SlowChIndex(field, &i_c[i], &j_c[i]);
    }
  }	

  for(i = 0; i < DAS_CARDS; i++) {
    WriteSlow(i_c[i], j_c[i], CommandData.Phase[i]);
  }
}

/***********************************************************************/
/* CalLamp: Flash calibrator                                           */
/***********************************************************************/
int CalLamp (int* cryostate, unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW])
{
  int calLamp;
  static int calPulseCh = -1, calPulseInd;
  static int pulse_left = 0;
  static int repeat_left = 0;

  if (calPulseCh == -1)
    SlowChIndex("cal_pulse", &calPulseCh, &calPulseInd);

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
      WriteSlow(calPulseCh, calPulseInd, 1);
    } else {               /* pulser off, or we're not pulsing, turn off lamp */
      calLamp = CRYO_CALIBRATOR_OFF;
      *cryostate &= 0xFFFF - CS_CALIBRATOR;
      WriteSlow(calPulseCh, calPulseInd, 0);
    }
  }

  return calLamp;
}

/***********************************************************************/
/* CryoControl: Set heaters to values contained within the CommandData */
/***********************************************************************/
void CryoControl (unsigned int* Txframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW])
{
  static int i_cryoin = -1, j_cryoin = -1;
  static int i_cryoout2 = -1, j_cryoout2 = -1;
  static int cryoOut3Ch;
  static int cryostateCh, cryostateInd;
  static int he3pwmCh, he3pwmInd;
  static int jfetpwmCh, jfetpwmInd;
  static int hspwmCh, hspwmInd;
  static int cryopwmCh, cryopwmInd;
  static int calRepeatCh, calRepeatInd;
  static int pulseLenCh, pulseLenInd;

  int cryoout3 = 0, cryoout2 = 0;
  static int cryostate = 0;

  /************** Set indices first time around *************/
  if (i_cryoin == -1) {
    FastChIndex("cryoout3", &cryoOut3Ch);

    SlowChIndex("cryoin", &i_cryoin, &j_cryoin);
    SlowChIndex("cryoout2", &i_cryoout2, &j_cryoout2);
    SlowChIndex("cryostate", &cryostateCh, &cryostateInd);
    SlowChIndex("he3pwm", &he3pwmCh, &he3pwmInd);
    SlowChIndex("jfetpwm", &jfetpwmCh, &jfetpwmInd);
    SlowChIndex("hspwm", &hspwmCh, &hspwmInd);
    SlowChIndex("cryopwm", &cryopwmCh, &cryopwmInd);

    SlowChIndex("pulse_len", &pulseLenCh, &pulseLenInd);
    SlowChIndex("cal_repeat", &calRepeatCh, &calRepeatInd);
  }

  /* We want to save these here since CalLamp might destroy calib_repeat */
  WriteSlow(pulseLenCh, pulseLenInd, CommandData.Cryo.calib_pulse);
  WriteSlow(calRepeatCh, calRepeatInd, CommandData.Cryo.calib_repeat);

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
  cryoout3 |= CalLamp(&cryostate, slowTxFields);

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

  WriteFast(cryoOut3Ch, cryoout3);

  WriteSlow(i_cryoout2, j_cryoout2, cryoout2);
  WriteSlow(cryostateCh, cryostateInd, cryostate);
  WriteSlow(he3pwmCh, he3pwmInd, CommandData.Cryo.heliumThree);
  WriteSlow(hspwmCh, hspwmInd, CommandData.Cryo.heatSwitch);
  WriteSlow(jfetpwmCh, jfetpwmInd, CommandData.Cryo.JFETHeat);
  WriteSlow(cryopwmCh, cryopwmInd, CommandData.Cryo.sparePwm);
}

/***************************************************************************/
/*                                                                         */
/* PulseCalibrator: Set heaters to values contained within the CommandData */
/*                                                                         */
/***************************************************************************/
void PulseCalibrator (unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW])
{
  static int calpulsCh = -1, calpulsInd = -1;
  static int pulsed = 0, waitfor = 0;
  int cal_pulse = 0;

  if (calpulsCh == -1) {
    SlowChIndex("cal_puls", &calpulsCh, &calpulsInd);
  }

  if (CommandData.Cryo.calib_pulse > 0) {
    if (waitfor > 0) {
      waitfor--;
    } else {
      if (pulsed < CommandData.Cryo.calib_pulse) {
        if (CommandData.Cryo.calib_pulse - pulsed <= 200) {
          cal_pulse = CommandData.Cryo.calib_pulse - pulsed;
        } else {
          cal_pulse = 300;
        }
        pulsed += 200;
      } else {
        if (CommandData.Cryo.calib_repeat > 0) {
          waitfor = CommandData.Cryo.calib_repeat * 5;
          pulsed = 0;
        } else
          pulsed = waitfor = CommandData.Cryo.calib_pulse = 0;
      }
    }
  } else {
    cal_pulse = 0;
  }

  WriteSlow(calpulsCh, calpulsInd, (int)(cal_pulse * 10.41666667));
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Digital IO with the Bias Generator Card               */
/*                                                                      */
/************************************************************************/
void BiasControl (unsigned int* Txframe,  unsigned short* Rxframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_BIASIN = -1;
  static int i_BIASOUT1 = -1, j_BIASOUT1 = -1;
  static int i_BIASOUT2 = -1, j_BIASOUT2 = -1;
  static int Bias_lev1Ch, Bias_lev1Ind;
  static int Bias_lev2Ch, Bias_lev2Ind;
  static int Bias_lev3Ch, Bias_lev3Ind;
  unsigned short bias_status, biasout1 = 0;
  static unsigned short biasout2 = 0x70;
  int isBiasAC, isBiasRamp, isBiasClockInternal;
  static int hold = 0, ch = 0, rb_hold = 0;

  /******** Obtain correct indexes the first time here ***********/
  if (i_BIASIN == -1) {
    FastChIndex("biasin", &i_BIASIN);

    SlowChIndex("biasout1", &i_BIASOUT1, &j_BIASOUT1);
    SlowChIndex("biasout2", &i_BIASOUT2, &j_BIASOUT2);
    SlowChIndex("bias_lev1", &Bias_lev1Ch, &Bias_lev1Ind);
    SlowChIndex("bias_lev2", &Bias_lev2Ch, &Bias_lev2Ind);
    SlowChIndex("bias_lev3", &Bias_lev3Ch, &Bias_lev3Ind);
  }

  bias_status = Rxframe[i_BIASIN];

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
    printf("Warning: ch an impossible value in bias control\n");
  }

  /* Bias readback -- we wait a few seconds after finishing the write */
  if (rb_hold > 0) {
    rb_hold--;
  } else if (!CommandData.Bias.SetLevel3 && !CommandData.Bias.SetLevel3 &&
      !CommandData.Bias.SetLevel3) {
    CommandData.Bias.bias1 = slow_data[Bias_lev1Ch][Bias_lev1Ind];
    CommandData.Bias.bias2 = slow_data[Bias_lev2Ch][Bias_lev2Ind];
    CommandData.Bias.bias3 = slow_data[Bias_lev3Ch][Bias_lev3Ind];
  }

  /******************** set the outputs *********************/
  WriteSlow(i_BIASOUT1, j_BIASOUT1, biasout1 & 0xffff);
  WriteSlow(i_BIASOUT2, j_BIASOUT2, (~biasout2) & 0xff);
  WriteSlow(Bias_lev1Ch, Bias_lev1Ind, CommandData.Bias.bias1);
  WriteSlow(Bias_lev2Ch, Bias_lev2Ind, CommandData.Bias.bias2);
  WriteSlow(Bias_lev3Ch, Bias_lev3Ind, CommandData.Bias.bias3);
  //printf("%d %d %d\n", CommandData.Bias.bias1, CommandData.Bias.bias2, CommandData.Bias.bias3);
}

void SetReadBits(unsigned int* Txframe) {
  static int i_readd3 = -1;
  static unsigned int bit = 0;
  int i_card;

  if (i_readd3 < 0) {
    FastChIndex("readd3", &i_readd3);
  }

  bit++;

  for(i_card = 0; i_card < DAS_CARDS + 2; i_card++) {
    WriteFast(i_readd3 + i_card, bit);
  }

}
