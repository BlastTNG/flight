#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include "bbc.h"

#include "tx_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"

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

#define DPS2GYU (66.7 * 65536.0/4000.0)

extern struct CommandDataStruct CommandData;
extern short int SamIAm;

extern unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

double round(double x);
double LockPosition(double elevation);

extern struct VSCDataStruct VSCData[3];
extern int vsc_index;

extern struct PointingDataStruct PointingData[3];
extern int point_index;

extern struct ACSDataStruct ACSData;

extern struct SunSensorDataStruct SunSensorData[3];
extern int ss_index;

int frame_num;
int pin_is_in = 1;

/****************************************************************
 *                                                              *
 * Read the state of the lock motor pin (or guess it, whatever) *
 *                                                              *
 ****************************************************************/
int pinIsIn(void) {
  return(pin_is_in);
}

/************************************************************************
 *                                                                      *
 *   GetVElev: get the current elevation velocity, given current        *
 *   pointing mode, etc..                                               *
 *                                                                      *
 ************************************************************************/
double GetVElev() {
  double vel=0;
  static double last_vel=0;
  double dvel;
  double max_dv = 20;

  if (CommandData.point_mode.el_mode == POINT_VEL) {
    vel = CommandData.point_mode.el_vel;
  } else if (CommandData.point_mode.el_mode == POINT_POSITION) {
    vel = (PointingData[point_index].el - CommandData.point_mode.el_dest) * 0.36;
  } else if (CommandData.point_mode.el_mode == POINT_LOCK) {
    /* for the lock, only use the elevation encoder */
    vel = (ACSData.enc_elev - CommandData.point_mode.el_dest) * 0.64;
  }

  //fprintf(stderr, "%10f %10f %10f  ", PointingData[point_index].el, CommandData.point_mode.el_dest, vel);

  vel += PointingData[point_index].gy1_offset;

  vel *= DPS2GYU; // convert to Gyro Units
  //fprintf(stderr, "%10f %10f\n", PointingData[point_index].gy1_offset, vel);

  /* Limit Maximim speed */
  if (vel > 2000.0) vel = 2000.0;
  if (vel < -2000.0) vel = -2000.0;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv) vel = last_vel + max_dv;
  if (dvel < -max_dv) vel = last_vel - max_dv;
  last_vel = vel;

  return (vel);
}

/************************************************************************
 *                                                                      *
 *   GetVAz: get the current az velocity, given current                 *
 *   pointing mode, etc..                                               *
 *                                                                      *
 ************************************************************************/
int GetVAz() {
  double vel=0;
  static int last_vel=0;
  int dvel;
  int max_dv = 20;

  if (CommandData.point_mode.az_mode == POINT_VEL) {
    vel = CommandData.point_mode.az_vel;
  } else if (CommandData.point_mode.az_mode == POINT_POSITION) {
    vel = -(PointingData[point_index].az - CommandData.point_mode.az_dest) * 400.0;
  }

  vel *= DPS2GYU; // convert to gyro units

  /* Limit Maximim speed */
  if (vel > 2000) vel = 2000;
  if (vel < -2000) vel = -2000;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv) vel = last_vel + max_dv;
  if (dvel < -max_dv) vel = last_vel - max_dv;
  last_vel = vel;

  return (vel);
}


/************************************************************************
 *                                                                      *
 *    WriteMot: motors, and, for convenience, the inner frame lock      *
 *                                                                      *
 ************************************************************************/
void WriteMot(int TxIndex, unsigned int *Txframe, unsigned short *Rxframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_elVreq = -1;
  static int i_azVreq = -1;
  static int i_cos_el = -1;
  static int i_sin_el = -1;

  static int i_g_Pel = -1, j_g_Pel = -1;
  static int i_g_Iel = -1, j_g_Iel = -1;
  static int i_g_Proll = -1, j_g_Proll = -1;
  static int i_g_Paz = -1, j_g_Paz = -1;
  static int i_g_Iaz = -1, j_g_Iaz = -1;
  static int i_g_pivot = -1, j_g_pivot = -1;
  static int i_set_reac = -1, j_set_reac = -1;

  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, elGainP, elGainI, rollGainP;

  /******** Obtain correct indexes the first time here ***********/
  if (i_g_Pel == -1) {
    FastChIndex("el_vreq", &i_elVreq);
    FastChIndex("az_vreq", &i_azVreq);
    FastChIndex("cos_el", &i_cos_el);
    FastChIndex("sin_el", &i_sin_el);

    SlowChIndex("g_p_el", &i_g_Pel, &j_g_Pel);
    SlowChIndex("g_i_el", &i_g_Iel, &j_g_Iel);
    SlowChIndex("g_p_roll", &i_g_Proll, &j_g_Proll);
    SlowChIndex("g_p_az", &i_g_Paz, &j_g_Paz);
    SlowChIndex("g_i_az", &i_g_Iaz, &j_g_Iaz);
    SlowChIndex("g_p_pivot", &i_g_pivot, &j_g_pivot);
    SlowChIndex("set_reac", &i_set_reac, &j_set_reac);
  }

  v_elev = GetVElev() * 6.0; // the 6.0 is to improve dynamic range.
  // It is removed in the DSP/ACS1 code.
  WriteFast(i_elVreq, 32768 + v_elev);
  v_az = GetVAz();
  WriteFast(i_azVreq, 32768 + v_az);

  /*** Send elevation angles to acs1 from acs2 ***/
  /* cos of el enc */
  el_rad = (M_PI / 180.0) * PointingData[point_index].el; // convert to radians
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  WriteFast(i_cos_el, ucos_el);

  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  WriteFast(i_sin_el, usin_el);

  /* zero motor gains if the pin is in */
  if (pinIsIn())
    elGainP = elGainI = 0;
  else {
    elGainP = CommandData.ele_gain.P;
    elGainI = CommandData.ele_gain.I;	
  }

  /*** adjust roll_gain ***/
  if (PointingData[point_index].gy_roll_amp>0.003) {
    rollGainP = 1000.0/PointingData[point_index].gy_roll_amp;
  } else {
    rollGainP = CommandData.roll_gain.P;
  }

  if (rollGainP>CommandData.roll_gain.P) rollGainP = CommandData.roll_gain.P;

  /*** Send Gains ****/
  /* proportional term for el motor */
  WriteSlow(i_g_Pel, j_g_Pel, elGainP);
  /* integral term for el_motor */
  WriteSlow(i_g_Iel, j_g_Iel, elGainI);
  /* p term for roll motor */
  WriteSlow(i_g_Proll, j_g_Proll, rollGainP);
  /* p term for az motor */
  WriteSlow(i_g_Paz, j_g_Paz, CommandData.azi_gain.P);
  /* p term for pivot motor */
  WriteSlow(i_g_pivot, j_g_pivot, CommandData.pivot_gain.P);
  /* setpoint for reaction wheel */
  WriteSlow(i_set_reac, j_set_reac, CommandData.pivot_gain.SP + 32768);
}

/************************************************************************
 *                                                                      *
 *  WriteAux: write aux data, like cpu time, temperature, fan speed     *
 *                                                                      *
 ************************************************************************/
void WriteAux(unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_fan = -1, j_fan = -1;
  static int i_t_cpu = -1, j_t_cpu = -1;
  static int i_time = -1, j_time = -1;
  static int i_samiam = -1, j_samiam = -1;
  time_t t;

  if (i_fan == -1) {
    SlowChIndex("cpu_fan", &i_fan, &j_fan);
    SlowChIndex("t_cpu", &i_t_cpu, &j_t_cpu);
    SlowChIndex("cpu_time", &i_time, &j_time);
    SlowChIndex("sam_i_am", &i_samiam, &j_samiam);
  }

  t = time(NULL);

  WriteSlow(i_time, j_time, t >> 16);
  WriteSlow(i_time + 1, j_time, t);

  WriteSlow(i_fan, j_fan, CommandData.fan);
  WriteSlow(i_t_cpu, j_t_cpu, CommandData.T);

  WriteSlow(i_samiam, j_samiam, SamIAm);
}

/************************************************************************
 *                                                                      *
 * PhaseControl: set phase shifts for DAS cards                         *
 *                                                                      *
 ************************************************************************/
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

/***********************************************************************
 *                                                                     *
 * CryoControl: Set heaters to values contained within the CommandData *
 *                                                                     *
 ***********************************************************************/
void CryoControl (unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW])
{
  static int i_cryoout1 = -1, j_cryoout1 = -1;
  static int i_cryoout2 = -1, j_cryoout2 = -1;
  static int i_cryoout3 = -1, j_cryoout3 = -1;
  static int cryostateCh = -1, cryostateInd = -1;
  int cryoout3 = 0, cryoout2 = 0, cryostate = 0;

  /************** Set indices first time around *************/
  if (i_cryoout3 == -1) {
    SlowChIndex("cryoout1", &i_cryoout1, &j_cryoout1);
    SlowChIndex("cryoout2", &i_cryoout2, &j_cryoout2);
    SlowChIndex("cryoout3", &i_cryoout3, &j_cryoout3);
    SlowChIndex("cryostate", &cryostateCh, &cryostateInd);
  }

  /********** Set Output Bits **********/
  if (CommandData.Cryo.heliumLevel == 0) {
    cryoout3 |= 0x02;
  } else {
    cryoout3 |= 0x01;
    cryostate |= 0x01;
  }
  if (CommandData.Cryo.charcoalHeater == 0) {
    cryoout3 |= 0x08;
  } else {
    cryoout3 |= 0x04;
    cryostate |= 0x02;
  }
  if (CommandData.Cryo.coldPlate == 0) {
    cryoout3 |= 0x20;
  } else {
    cryoout3 |= 0x10;
    cryostate |= 0x04;
  }
  if (CommandData.Cryo.JFETHeat == 0) {
    cryoout3 |= 0x80;
  } else {
    cryoout3 |= 0x40;
    cryostate |= 0x08;
  }
  if (CommandData.Cryo.heatSwitch == 0) {
    cryoout2 |= 0x20;
  } else {
    cryoout2 |= 0x10;
    cryostate |= 0x10;
  }
  if (CommandData.Cryo.heliumThree == 0) {
    cryoout2 |= 0x80;
  } else {
    cryoout2 |= 0x40;
    cryostate |= 0x20;
  }

  WriteSlow(i_cryoout3, j_cryoout3, cryoout3);
  WriteSlow(i_cryoout2, j_cryoout2, cryoout2);
  WriteSlow(cryostateCh, cryostateInd, cryostate);
}

/************************************************************************
 *                                                                      *
 *    ControlGyroHeat:  Controls gyro box temp by turning heater bit in *
 *    ACS1 on and off.  Also calculates gyro offsets.                   *
 *                                                                      *
 ************************************************************************/
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
  WriteSlow(i_T_GY_SET, j_T_GY_SET, (unsigned short)(CommandData.t_gybox_setpoint * 32768.0 / 100.0));

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

/************************************************************************
 *                                                                      *
 *    ControlISCHeat:  Controls ISC box temp by turning heater bit in   *
 *    ACS0 on and off.                                                  *
 *                                                                      *
 ************************************************************************/
int ControlISCHeat(unsigned int *Txframe,  unsigned short *Rxframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_T_ISC = -1;

  static int i_T_ISC_SET = -1, j_T_ISC_SET = -1;
  static int i_G_PISCH = -1, j_G_PISCH = -1;
  static int i_G_IISCH = -1, j_G_IISCH = -1;
  static int i_G_DISCH = -1, j_G_DISCH = -1;

  static int p_on = 0;
  static int p_off = -1;

  float error = 0, set_point;
  static float integral = 0;
  static float deriv = 0;
  static float error_last = 0;
  float P, I, D;

  /******** Obtain correct indexes the first time here ***********/
  if (i_T_ISC == -1) {
    FastChIndex("t_isc", &i_T_ISC);

    SlowChIndex("t_isc_set", &i_T_ISC_SET, &j_T_ISC_SET);
    SlowChIndex("g_p_ischeat", &i_G_PISCH, &j_G_PISCH);
    SlowChIndex("g_i_ischeat", &i_G_IISCH, &j_G_IISCH);
    SlowChIndex("g_d_ischeat", &i_G_DISCH, &j_G_DISCH);
  }

  /* send down the setpoints and gains values */
  WriteSlow(i_T_ISC_SET, j_T_ISC_SET, (unsigned short)(CommandData.t_isc_setpoint * 32768.0 / 100.0));
  WriteSlow(i_G_PISCH, j_G_PISCH, CommandData.isc_heat_gain.P);
  WriteSlow(i_G_IISCH, j_G_IISCH, CommandData.isc_heat_gain.I);
  WriteSlow(i_G_DISCH, j_G_DISCH, CommandData.isc_heat_gain.D);

  /* control the heat */
  set_point = (CommandData.t_isc_setpoint - 136.45) / (-9.5367431641e-08);
  P = CommandData.isc_heat_gain.P * (-1.0 / 1000000.0);
  I = CommandData.isc_heat_gain.I * (-1.0 / 110000.0);
  D = CommandData.isc_heat_gain.D * ( 1.0 / 1000.0);

  /********* if end of pulse, calculate next pulse *********/
  if (p_off < 0) {

    error = set_point -
      ((unsigned int)(Rxframe[i_T_ISC + 1]<< 16 | Rxframe[i_T_ISC]));

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

  /******** return the pulse *****/
  if (p_on > 0) {
    return ISC_HEAT;
    p_on--;
  } else {
    return ISC_NOHEAT;
    p_off--;
  }
}

/************************************************************************
 *                                                                      *
 *   BiasControl: Digital IO with the Bias Generator Card               *
 *                                                                      *
 ************************************************************************/
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
  static unsigned short lastBias1 = 0xff , lastBias2 = 0xff, lastBias3 = 0xff;
  static int hold = 0, ch = 0;

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
      //fprintf(stderr, "to DC\n");
    }
  } else { /* Bias is currently DC */
    if (CommandData.Bias.biasAC == 1) { /* it should be AC */
      biasout1 |= 0x02;
      //fprintf(stderr, "to AC\n");
    }
  }

  /********** set Bias Internal/External (ramp)  *******/
  if (isBiasRamp) { /* Bias is currently external Ramp */
    if (CommandData.Bias.biasRamp == 0) { /* it should be internal/fixed */
      biasout1 |= 0x40;
      //fprintf(stderr, "to fixed\n");
    }
  } else { /* Bias is currently internal (fixed) */
    if (CommandData.Bias.biasRamp == 1) { /* it should be external/Ramp */
      biasout1 |= 0x80;
      //fprintf(stderr, "to ramp\n");
    }
  }

  /********** set Clock Internal/External *******/
  if (isBiasClockInternal) { /* Bias is currently internal */
    if (CommandData.Bias.clockInternal == 0) { /* it should be external */
      biasout1 |= 0x10;
      //fprintf(stderr, "to external\n");
    }
  } else { /* Bias clock is currenly external */
    if (CommandData.Bias.clockInternal == 1) { /* it should be internal */
      biasout1 |= 0x20;
      //fprintf(stderr, "to internal\n");
    }
  }

  /************* Set the Bias Levels *******/
  if (hold > FAST_PER_SLOW + 2) { /* hold data with write low */
    hold--;
  } else if (hold > 0) {  /* hold data with write high */
    biasout2 |= 0x0f;
    hold--;
  } else if (ch==0) {
    biasout2 = ((CommandData.Bias.bias1 << 4) & 0xf0) | 0x03;
    lastBias1 = CommandData.Bias.bias1;
    hold = 2 * FAST_PER_SLOW + 4;
    ch++;
  } else if (ch==1) {
    biasout2 = ((CommandData.Bias.bias2 << 4) & 0xf0) | 0x05;
    lastBias2 = CommandData.Bias.bias2;
    hold = 2 * FAST_PER_SLOW + 4;
    ch++;
  } else if (ch==2) {
    biasout2 = ((CommandData.Bias.bias3 << 4) & 0xf0) | 0x06;
    lastBias3 = CommandData.Bias.bias3;
    hold = 2 * FAST_PER_SLOW + 4;
    ch = 0;
  }

  /******************** set the outputs *********************/
  WriteSlow(i_BIASOUT1, j_BIASOUT1, biasout1 & 0xffff);
  WriteSlow(i_BIASOUT2, j_BIASOUT2, (~biasout2) & 0xff);
  WriteSlow(Bias_lev1Ch, Bias_lev1Ind, CommandData.Bias.bias1);
  WriteSlow(Bias_lev2Ch, Bias_lev2Ind, CommandData.Bias.bias2);
  WriteSlow(Bias_lev3Ch, Bias_lev3Ind, CommandData.Bias.bias3);
}

/******************************************************************
 *                                                                *
 * Balance: control balance system                                *
 *                                                                *
 ******************************************************************/
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
  error = slow_data[iElCh][iElInd] - 32758 - CommandData.pumps.bal_target;

  if (error > 0) {
    iscBits |= BAL1_REV;  /* set reverse bit */
  } else {
    iscBits &= (0xFF - BAL1_REV);  /* clear reverse bit */
    error = -error;
  }

  pumppwm = 2047 - error / 10 - 300;

  if (pumppwm < 800) {
    pumppwm = 800;
  } else if (pumppwm > 2047) {
    pumppwm = 2047;
  }

  if (error > CommandData.pumps.bal_on) {
    pumpon = 1;
  } else if (error < CommandData.pumps.bal_off) {
    pumpon = 0;
    if (CommandData.pumps.bal_veto >= 0) CommandData.pumps.bal_veto = BAL_OFF_VETO;
  }

  if (pumpon) {
    iscBits |= BAL1_ON; /* turn on pump */
  } else {
    iscBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

  /*  fprintf(stderr, "Balance: error = %6i, pwm = %4i, reverse = %i, on = %i\n", error, pumppwm, (iscBits & 0x02) >> 1, pumpon); */

  WriteSlow(balPwm1Ch, balPwm1Ind, pumppwm);

  return iscBits;
}

/************************************************************************
 *                                                                      *
 *    Do Lock Logic: check status, determine if we are locked, etc      *
 *                                                                      *
 ************************************************************************/
#define MOVE_COUNTS 200
#define SEARCH_COUNTS 500
int GetLockBits(int acs0bits) {
  static int closing = 0;
  static int opening = 0;
  static int searching = 0;

  // check for commands from CommandData
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
  } else if ((acs0bits & 0xc0) == 0) { // if motor is on, these bits are 0
    return (LOKMOT_OFF);
  } else { // motor is off - we can read position
    if (acs0bits & 64) pin_is_in = 1;
    else pin_is_in = 0;
    return (0);
  }

}

/*****************************************************************
 *                                                               *
 *   Control the pumps and the lock                              *
 *                                                               *
 *****************************************************************/
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
  static int iscBitsCh;

  int pumpBits = 0;
  int iscBits = ControlISCHeat(Txframe, Rxframe, slowTxFields);

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
    SlowChIndex("bal_veto", &balVetoCh, &balVetoInd);
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

  WriteSlow(pumpBitsCh, pumpBitsInd, pumpBits);
  WriteSlow(pumpPwm2Ch, pumpPwm2Ind, CommandData.pumps.pwm2 & 0x7ff);
  WriteSlow(pumpPwm3Ch, pumpPwm3Ind, CommandData.pumps.pwm3 & 0x7ff);
  WriteSlow(pumpPwm4Ch, pumpPwm4Ind, CommandData.pumps.pwm4 & 0x7ff);
  WriteSlow(balOnCh, balOnInd, (int)CommandData.pumps.bal_on);
  WriteSlow(balOffCh, balOffInd, (int)CommandData.pumps.bal_off);
  WriteSlow(balVetoCh, balVetoInd, (int)CommandData.pumps.bal_veto);
  WriteSlow(balTargetCh, balTargetInd, (int)CommandData.pumps.bal_target);
  WriteFast(iscBitsCh, iscBits);

}

/*****************************************************************
 *                                                               *
 * SyncADC: check to see if any boards need to be synced and     *
 *    send the sync bit if they do.  Only one board can be       *
 *    synced in each superframe.                                 *
 *                                                               *
 *****************************************************************/
void SyncADC (int TxIndex,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int syncCh = -1, syncInd, nextInd;
  static int statusInd[17];
  static int statusCh[17];
  char buffer[9];

  int k, l;

  /******** Obtain correct indexes the first time here ***********/
  if (syncCh == -1) {
    SlowChIndex("sync", &syncCh, &syncInd);
    nextInd = (syncInd + 1) % FAST_PER_SLOW;
    for (k = 0; k < 17; ++k) {
      sprintf(buffer, "status%02i", k);
      SlowChIndex(buffer, &statusCh[k], &statusInd[k]);
    }
  }

  /* are we currently syncing? */
  if (slowTxFields[syncCh][syncInd] & BBC_ADC_SYNC) {
    /* if yes, turn off sync bit if we sent the sync last frame */
    if (TxIndex == nextInd) {
      slowTxFields[syncCh][syncInd] = BBC_WRITE | BBC_NODE(17) | BBC_CH(56);
    }
  } else {
    /* if not, check to see if we need to sync a board */
    for (k = 0; k < 17; ++k) {
      /* read board status */
      if (slow_data[statusCh[k]][statusInd[k]] == 0x0001) {
        /* board needs to be synced */
        l = (k == 0) ? 21 : k;
        slowTxFields[syncCh][syncInd] =
          BBC_WRITE | BBC_NODE(l) | BBC_CH(56) |
          BBC_ADC_SYNC | 0xa5a3;
        k = 17;  /* ABORT! ABORT! */
      }
    }
  }
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

char *StringToLower(char *s) {
  int i, len;
  static char ls[256];

  len = strlen(s);
  if (len > 255) len = 255;

  for (i = 0; i < len; i++) {
    ls[i] = tolower(s[i]);
  }
  ls[len] = '\0';
  return(ls);
}

char *StringToUpper(char *s) {
  int i, len;
  static char us[256];

  len = strlen(s);

  for (i = 0; i < len; i++) {
    us[i] = toupper(s[i]);
  }

  us[len] = '\0';
  return(us);
}

/************************************************************************
 *                                                                      *
 *    Store derived acs and pointing data in frame                      *
 *                                                                      *
 ************************************************************************/
void StoreData(unsigned int* Txframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW]) {
  static int i_V_COL = -1, j_V_COL = -1;
  static int i_V_ROW = -1, j_V_ROW = -1;
  static int i_V_MAG = -1, j_V_MAG = -1;
  static int i_V_FRA = -1, j_V_FRA = -1;
  static int arsCh = -1, arsInd, ersCh, ersInd, prinCh, prinInd;

  static int i_az = -1, i_el = -1;
  int i_vsc;
  int i_ss;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  if (i_V_COL == -1) {
    FastChIndex("az", &i_az);
    FastChIndex("el", &i_el);

    SlowChIndex("vsc_col", &i_V_COL, &j_V_COL);
    SlowChIndex("vsc_row", &i_V_ROW, &j_V_ROW);
    SlowChIndex("vsc_mag", &i_V_MAG, &j_V_MAG);
    SlowChIndex("vsc_fra", &i_V_FRA, &j_V_FRA);
    SlowChIndex("az_rel_sun", &arsCh, &arsInd);
    SlowChIndex("el_rel_sun", &ersCh, &ersInd);
    SlowChIndex("ss_prin", &prinCh, &prinInd);

  }

  /********** VSC Data **********/
  i_vsc = GETREADINDEX(vsc_index);
  WriteSlow(i_V_COL, j_V_COL, (int)(VSCData[i_vsc].col * 100));
  WriteSlow(i_V_ROW, j_V_ROW, (int)(VSCData[i_vsc].row * 100));
  WriteSlow(i_V_MAG, j_V_MAG, (int)(VSCData[i_vsc].mag * 100));
  WriteSlow(i_V_FRA, j_V_FRA, VSCData[i_vsc].sf_frame);

  /********** Sun Sensor Data **********/
  i_ss = GETREADINDEX(ss_index);
  WriteSlow(arsCh, arsInd, SunSensorData[i_ss].raw_az);
  WriteSlow(ersCh, ersInd, SunSensorData[i_ss].raw_el);
  WriteSlow(prinCh, prinInd, SunSensorData[i_ss].prin);

  /************* processed pointing data *************/
  i_point = GETREADINDEX(point_index);
  WriteFast(i_az, (unsigned int)(PointingData[i_point].az * 65536.0/360.0));
  WriteFast(i_el, (unsigned int)(PointingData[i_point].el * 65536.0/360.0));
}


/******************************************************************
 *                                                                *
 * IsNewFrame: returns true if d is a begining of frame marker,   *
 *    unless this is the first beginning of frame.                *
 *                                                                *
 ******************************************************************/
int IsNewFrame(unsigned int d) {
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | FILETYPE));
  if (is_bof && first_bof) {
    is_bof = 0; first_bof = 0;
  }

  return (is_bof);
}

void do_Tx_frame(int bbc_fp, unsigned int *Txframe,
    unsigned int slowTxFields[N_SLOW][FAST_PER_SLOW],
    unsigned short *Rxframe, int reset) {
  static int firsttime = 1;
  static int index = 0;

  static int BBC_Fsync_Word = BBC_FSYNC;
  int i = 0, j;

  if (firsttime | reset) {
    Txframe[0] = BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | FILETYPE; /* BOF */
    Txframe[1] = BBC_WRITE | BBC_NODE(63) | BBC_CH(1); /* f_num lsb */
    Txframe[2] = BBC_WRITE | BBC_NODE(63) | BBC_CH(2); /* f_num msb */
    Txframe[3] = BBC_WRITE | BBC_NODE(63) | BBC_CH(3); /* index */

    for (j = 0; j < N_SLOW; j++) {
      for (i = 0; i < FAST_PER_SLOW; i++) {
        if (SlowChList[j][i].rw=='r') {
          slowTxFields[j][i] =
            BBC_READ |
            BBC_NODE(SlowChList[j][i].node) |
            BBC_CH(SlowChList[j][i].adr);
        } else {
          slowTxFields[j][i] =
            BBC_WRITE |
            BBC_NODE(SlowChList[j][i].node) |
            BBC_CH(SlowChList[j][i].adr);
        }
      }
    }

    for (i = 0; i < N_FASTCHLIST; i++) {
      if (FastChList[i].rw =='r') {
        Txframe[i + FAST_OFFSET] =
          BBC_READ | BBC_NODE(FastChList[i].node) | BBC_CH(FastChList[i].adr);
      } else {
        Txframe[i + FAST_OFFSET] =
          BBC_WRITE| BBC_NODE(FastChList[i].node) | BBC_CH(FastChList[i].adr);
      }
    }
    firsttime = 0;
  }

  /*** update frame num ***/
  WriteFast(1, frame_num);
  WriteFast(2, frame_num >> 16);
  frame_num++;

  /*** update mplex fields  ***/
  WriteFast(3, index);
  for (j = 0; j < N_SLOW; j++) {
    Txframe[j + 4] = slowTxFields[j][index];
  }
  index++;
  if (index >= FAST_PER_SLOW) index = 0;

  /*** do Controls ***/
#ifndef BOLOTEST
  StoreData(Txframe, slowTxFields);
  ControlGyroHeat(Txframe, Rxframe, slowTxFields);
  ControlISCHeat(Txframe, Rxframe, slowTxFields);
  WriteMot(index, Txframe, Rxframe, slowTxFields);
#endif
  BiasControl(Txframe, Rxframe, slowTxFields);
  SyncADC(index, slowTxFields);

  /*** do slow Controls ***/
  if (index == 0) {
    WriteAux(slowTxFields);
  }
#ifndef BOLOTEST
  ControlAuxMotors(Txframe, Rxframe, slowTxFields);
#endif
  CryoControl(slowTxFields);
  PhaseControl(slowTxFields);

  SetReadBits(Txframe);

  /*** write FSync ***/
  write(bbc_fp, (void*)&BBC_Fsync_Word, sizeof(unsigned int));
  /*** Write Frame ***/
  write(bbc_fp, (void*)Txframe, TxFrameBytes());
}
