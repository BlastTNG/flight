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

/* Cryostat digital signals (G2 and G3 outputs) */
#define CRYO_COLDPLATE_ON    0x10 /* N3G3 - cryoout3 */
#define CRYO_COLDPLATE_OFF   0x20 /* N3G3 */
#define CRYO_CALIBRATOR_ON   0x40 /* N3G3 */
#define CRYO_CALIBRATOR_OFF  0x80 /* N3G3 */
#define CRYO_HELIUMLEVEL_ON  0x01 /* N3G3 */
#define CRYO_HELIUMLEVEL_OFF 0x02 /* N3G3 */
#define CRYO_CHARCOAL_ON     0x04 /* N3G3 */
#define CRYO_CHARCOAL_OFF    0x08 /* N3G3 */

#define CRYO_LNVALVE_ON      0x10 /* N3G2 - cryoout2 */
#define CRYO_LNVALVE_OPEN    0x40 /* N3G2 Group two of the cryo card */
#define CRYO_LNVALVE_CLOSE   0x80 /* N3G2 appears to have its nybbles */
#define CRYO_LHeVALVE_ON     0x01 /* N3G2 backwards */
#define CRYO_LHeVALVE_OPEN   0x04 /* N3G2 */
#define CRYO_LHeVALVE_CLOSE  0x08 /* N3G2 */

/* CryoState bitfield */
#define CS_HELIUMLEVEL   0x0001
#define CS_CHARCOAL      0x0002
#define CS_COLDPLATE     0x0004
#define CS_CALIBRATOR    0x0008
#define CS_LNVALVE_ON    0x0010
#define CS_LNVALVE_OPEN  0x0020
#define CS_LHeVALVE_ON   0x0040
#define CS_LHeVALVE_OPEN 0x0080

#define BAL_OFF_VETO  1000            /* # of frames to veto balance system
                                         after turning off pump */

#define DPS2GYU (66.7 * 65536.0/4000.0)

extern struct CommandDataStruct CommandData;
extern short int SamIAm;

extern unsigned short slow_data[N_SLOW][FAST_PER_SLOW];

double round(double x);
double LockPosition(double elevation); // defined in commands.c

int frame_num;
int pin_is_in = 1;

struct AxesModeStruct {
  int az_mode;
  int el_mode;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
};

struct AxesModeStruct axes_mode; // low level velocity mode

double getlst(time_t t, double lon);
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
                double *el);

void DoSched();

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
  int i_point;

  i_point = GETREADINDEX(point_index);

  if (axes_mode.el_mode == AXIS_VEL) {
    vel = axes_mode.el_vel;
  } else if (axes_mode.el_mode == AXIS_POSITION) {
    vel = (axes_mode.el_dest - PointingData[i_point].el)
	  * 0.36;
  } else if (axes_mode.el_mode == AXIS_LOCK) {
    /* for the lock, only use the elevation encoder */
    vel = (axes_mode.el_dest - ACSData.enc_elev) * 0.64;
  }
  
  /* correct offset and convert to Gyro Units */
  vel -= PointingData[i_point].gy1_offset;

  vel *= DPS2GYU; 

  /* Limit Maximim speed */
  if (vel > DPS2GYU/2.0) vel = DPS2GYU/2.0;
  if (vel < -DPS2GYU/2.0) vel = -DPS2GYU/2.0;

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
  int i_point;
  double vel_offset;
  
  i_point = GETREADINDEX(point_index);

  if (axes_mode.az_mode == AXIS_VEL) {
    vel = axes_mode.az_vel;
  } else if (axes_mode.az_mode == AXIS_POSITION) {
    vel = -(PointingData[i_point].az - axes_mode.az_dest)
	  * 0.36;
  }

  
  //-[V5-GYRO2]*cos([el_rad-sv]) - [V6-GYRO3]*sin([el_rad-sv])
  //vel_offset = -GY2_TMP_OFFSET*cos(PointingData[i_point].el) -
  //       GY3_TMP_OFFSET*sin(PointingData[i_point].el);
  vel_offset =
    -PointingData[i_point].gy2_offset*cos(PointingData[i_point].el*M_PI/180.0) -
    PointingData[i_point].gy3_offset*sin(PointingData[i_point].el*M_PI/180.0);
  
  vel -= vel_offset;
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

  static int wait = 100; // wait 20 frames before controlling.
  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, elGainP, elGainI, rollGainP;
  int azGainP, azGainI, pivGainP;
  int i_point;

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
  
  i_point = GETREADINDEX(point_index);

  /***************************************************/
  /**           Elevation Drive Motors              **/
  /* elevation speed */
  v_elev = GetVElev() * 6.0; // the 6.0 is to improve dynamic range.
  if (v_elev>32767) v_elev = 32767;
  if (v_elev < -32768) v_elev = -32768;  
  WriteFast(i_elVreq, 32768 + v_elev);

  /* zero motor gains if the pin is in */
  if (pinIsIn() || CommandData.disable_el) {
    elGainP = elGainI = 0;
  } else {
    elGainP = CommandData.ele_gain.P;
    elGainI = CommandData.ele_gain.I;	
  }
  /* proportional term for el motor */
  WriteSlow(i_g_Pel, j_g_Pel, elGainP);
  /* integral term for el_motor */
  WriteSlow(i_g_Iel, j_g_Iel, elGainI);

  
  /***************************************************/
  /*** Send elevation angles to acs1 from acs2 ***/
  /* cos of el enc */
  el_rad = (M_PI / 180.0) * PointingData[i_point].el; // convert to radians
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  WriteFast(i_cos_el, ucos_el);
  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  WriteFast(i_sin_el, usin_el);

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = GetVAz()*6.0; // the 6.0 is to improve dynamic range.
  if (v_az>32767) v_az = 32767;
  if (v_az < -32768) v_az = -32768;  
  WriteFast(i_azVreq, 32768 + v_az);

  if ((CommandData.disable_az) || (wait>0)) {
    azGainP = 0;
    azGainI = 0;
    pivGainP = 0;
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    pivGainP = CommandData.pivot_gain.P;
  }

  /* p term for az motor */
  WriteSlow(i_g_Paz, j_g_Paz, azGainP);
  /* I term for az motor */
  WriteSlow(i_g_Iaz, j_g_Iaz, azGainI);
  /* p term for pivot motor */
  WriteSlow(i_g_pivot, j_g_pivot, pivGainP);
  /* setpoint for reaction wheel */
  WriteSlow(i_set_reac, j_set_reac, CommandData.pivot_gain.SP + 32768);


  /***************************************************/
  /**                Roll Drive Motors              **/  
  if (PointingData[i_point].gy_roll_amp>0.003) { 
    rollGainP = 1000.0/PointingData[i_point].gy_roll_amp;
  } else {
    rollGainP = CommandData.roll_gain.P;
  }
  if (rollGainP>CommandData.roll_gain.P) rollGainP = CommandData.roll_gain.P;

  if (wait>0) rollGainP = 0;
  
  /* p term for roll motor */
  WriteSlow(i_g_Proll, j_g_Proll, rollGainP);

  if (wait>0) wait--;
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
  static int i_df = -1, j_df = -1;
  time_t t;

  if (i_fan == -1) {
    SlowChIndex("cpu_fan", &i_fan, &j_fan);
    SlowChIndex("t_cpu", &i_t_cpu, &j_t_cpu);
    SlowChIndex("cpu_time", &i_time, &j_time);
    SlowChIndex("sam_i_am", &i_samiam, &j_samiam);
    SlowChIndex("disk_free", &i_df, &j_df);
  }

  t = time(NULL);

  WriteSlow(i_time, j_time, t >> 16);
  WriteSlow(i_time + 1, j_time, t);

  WriteSlow(i_fan, j_fan, CommandData.fan);
  WriteSlow(i_t_cpu, j_t_cpu, CommandData.T);

  WriteSlow(i_samiam, j_samiam, SamIAm);
  WriteSlow(i_df, j_df, CommandData.df);
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
  static int i_cryoin = -1, j_cryoin = -1;
  static int i_cryoout2 = -1, j_cryoout2 = -1;
  static int i_cryoout3 = -1, j_cryoout3 = -1;
  static int cryostateCh, cryostateInd;
  static int he3pwmCh, he3pwmInd;
  static int jfetpwmCh, jfetpwmInd;
  static int hspwmCh, hspwmInd;
  static int cryopwmCh, cryopwmInd;
  int cryoout3 = 0, cryoout2 = 0;
  static int cryostate = 0;

  /************** Set indices first time around *************/
  if (i_cryoout3 == -1) {
    SlowChIndex("cryoin", &i_cryoin, &j_cryoin);
    SlowChIndex("cryoout2", &i_cryoout2, &j_cryoout2);
    SlowChIndex("cryoout3", &i_cryoout3, &j_cryoout3);
    SlowChIndex("cryostate", &cryostateCh, &cryostateInd);
    SlowChIndex("he3pwm", &he3pwmCh, &he3pwmInd);
    SlowChIndex("jfetpwm", &jfetpwmCh, &jfetpwmInd);
    SlowChIndex("hspwm", &hspwmCh, &hspwmInd);
    SlowChIndex("cryopwm", &cryopwmCh, &cryopwmInd);
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
  if (CommandData.Cryo.calibrator == 0) {
    cryoout3 |= CRYO_CALIBRATOR_OFF;
    cryostate &= 0xFFFF - CS_CALIBRATOR;
  } else {
    cryoout3 |= CRYO_CALIBRATOR_ON;
    cryostate |= CS_CALIBRATOR;
  }

  cryoout2 = CRYO_LNVALVE_OPEN | CRYO_LNVALVE_CLOSE | 
   CRYO_LHeVALVE_OPEN | CRYO_LHeVALVE_CLOSE;

  /* Control motorised valves -- latching relays */
  if (CommandData.Cryo.lnvalve_open > 0) {
    cryoout2 &= ~CRYO_LNVALVE_OPEN;
    cryostate |= CS_LNVALVE_OPEN;
    CommandData.Cryo.lnvalve_open--;
  } else if (CommandData.Cryo.lnvalve_close > 0) {
    cryoout2 &= ~CRYO_LNVALVE_CLOSE;
    cryostate &= 0xFFFF - CS_LNVALVE_OPEN;
    CommandData.Cryo.lnvalve_close--;
  }
  if (CommandData.Cryo.lnvalve_on) {
    cryoout2 |= CRYO_LNVALVE_ON;
    cryostate |= CS_LNVALVE_ON;
  } else
    cryostate &= 0xFFFF - CS_LNVALVE_ON;

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

  WriteSlow(i_cryoout3, j_cryoout3, cryoout3);
  WriteSlow(i_cryoout2, j_cryoout2, cryoout2);
  WriteSlow(cryostateCh, cryostateInd, cryostate);
  WriteSlow(he3pwmCh, he3pwmInd, CommandData.Cryo.heliumThree);
  WriteSlow(hspwmCh, hspwmInd, CommandData.Cryo.heatSwitch);
  WriteSlow(jfetpwmCh, jfetpwmInd, CommandData.Cryo.JFETHeat);
  WriteSlow(cryopwmCh, cryopwmInd, CommandData.Cryo.sparePwm);
}

/***************************************************************************
 *                                                                         *
 * PulseCalibrator: Set heaters to values contained within the CommandData *
 *                                                                         *
 ***************************************************************************/
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
  WriteSlow(i_T_GY_SET, j_T_GY_SET,
      (unsigned short)(CommandData.t_gybox_setpoint * 32768.0 / 100.0));

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
  } else if (ch == 0) {
    if (CommandData.Bias.SetLevel1) {
      biasout2 = ((CommandData.Bias.bias1 << 4) & 0xf0) | 0x03;
      hold = 2 * FAST_PER_SLOW + 4;
      CommandData.Bias.SetLevel1 = 0;
    }
    ch++;
  } else if (ch == 1) {
    if (CommandData.Bias.SetLevel2) {
      biasout2 = ((CommandData.Bias.bias2 << 4) & 0xf0) | 0x05;
      hold = 2 * FAST_PER_SLOW + 4;
      CommandData.Bias.SetLevel2 = 0;
    }
    ch++;
  } else if (ch == 2) {
    if (CommandData.Bias.SetLevel3) {
      biasout2 = ((CommandData.Bias.bias3 << 4) & 0xf0) | 0x06;
      hold = 2 * FAST_PER_SLOW + 4;
      CommandData.Bias.SetLevel3 = 0;
    }
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
    if (CommandData.pumps.bal_veto >= 0)
      CommandData.pumps.bal_veto = BAL_OFF_VETO;
  }

  if (pumpon) {
    iscBits |= BAL1_ON; /* turn on pump */
  } else {
    iscBits &= (0xFF - BAL1_ON); /* turn off pump */
  }

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

#define ISC_TRIG_LEN 3
#define ISC_TRIG_PER 100
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
  static int i_lockpin, j_lockpin;
  static int isc_trigger_count = 0;

  int iscBits = 0;
  int pumpBits = 0;

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
    SlowChIndex("lokmot_pin", &i_lockpin, &j_lockpin);
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

  if (isc_trigger_count<ISC_TRIG_LEN) {
    iscBits|=ISC_TRIGGER;
  }
  isc_trigger_count++;
  if (isc_trigger_count>=ISC_TRIG_PER) {
    isc_trigger_count = 0;
  }
  
  WriteSlow(i_lockpin, j_lockpin, pin_is_in);
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
  static int i_AZ_REL_SUN = -1, j_AZ_REL_SUN, i_EL_REL_SUN, j_EL_REL_SUN;
  static int i_SS_PRIN, j_SS_PRIN;
  static int i_SIP_LAT, i_SIP_LON, i_SIP_ALT, i_SIP_TIME;
  static int j_SIP_LAT, j_SIP_LON, j_SIP_ALT, j_SIP_TIME;
  static int i_LST, j_LST;
  static int i_LAT, i_LON;
  static int j_LAT, j_LON;

  /** pointing mode indexes **/
  static int i_AZ_MODE, j_AZ_MODE, i_EL_MODE, j_EL_MODE;
  static int i_AZ1, j_AZ1, i_AZ2, j_AZ2;
  static int i_EL1, j_EL1, i_EL2, j_EL2;
  static int i_AZ_VEL, j_AZ_VEL, i_EL_VEL, j_EL_VEL;
  static int i_RA, j_RA, i_DEC, j_DEC, i_R, j_R;
  static int i_SVETO, j_SVETO;
  static int i_MAG_MODEL, j_MAG_MODEL;
  
  /** dgps fields **/
  static int i_dgps_time, j_dgps_time;
  static int i_dgps_lat, j_dgps_lat;  
  static int i_dgps_lon, j_dgps_lon;  
  static int i_dgps_alt, j_dgps_alt;  
  static int i_dgps_speed, j_dgps_speed;  
  static int i_dgps_dir, j_dgps_dir;  
  static int i_dgps_climb, j_dgps_climb;
  static int i_dgps_att_ok, j_dgps_att_ok;
  static int i_dgps_att_index, j_dgps_att_index;
  static int i_dgps_pos_index, j_dgps_pos_index;
  static int i_dgps_n_sat, j_dgps_n_sat;

  /** isc fields **/
  static int blob0_xCh, blob0_xInd;
  static int blob1_xCh, blob1_xInd;
  static int blob2_xCh, blob2_xInd;
  static int blob0_yCh, blob0_yInd;
  static int blob1_yCh, blob1_yInd;
  static int blob2_yCh, blob2_yInd;
  static int blob0_fluxCh, blob0_fluxInd;
  static int blob1_fluxCh, blob1_fluxInd;
  static int blob2_fluxCh, blob2_fluxInd;
  static int isc_errorCh, isc_errorInd;
  static int isc_exposeCh, isc_exposeInd;
  static int isc_rotCh, isc_rotInd;
  static int isc_raCh, isc_raInd;
  static int isc_decCh, isc_decInd;
  static int isc_apertCh, isc_apertInd;
  static int isc_pscaleCh, isc_pscaleInd;
  static int isc_gainCh, isc_gainInd;
  static int isc_cenboxCh, isc_cenboxInd;
  static int isc_apboxCh, isc_apboxInd;
  static int isc_mdistCh, isc_mdistInd;
  static int isc_nblobsCh, isc_nblobsInd;
  static int isc_focusCh, isc_focusInd;
  static int isc_offsetCh, isc_offsetInd;
  static int isc_mapmeanCh, isc_mapmeanInd;
  static int isc_threshCh, isc_threshInd;
  static int isc_gridCh, isc_gridInd;

  
  static int blob_index = 0;

  time_t t;

  static int i_az = -1, i_el = -1, i_MAG_AZ;
  int i_vsc;
  int i_ss;
  int i_point;
  int i_dgps;
  int sensor_veto;
  int i_isc = GETREADINDEX(iscdata_index);

  /******** Obtain correct indexes the first time here ***********/
  if (i_V_COL == -1) {
    FastChIndex("az", &i_az);
    FastChIndex("el", &i_el);
    FastChIndex("mag_az", &i_MAG_AZ);

    SlowChIndex("vsc_col", &i_V_COL, &j_V_COL);
    SlowChIndex("vsc_row", &i_V_ROW, &j_V_ROW);
    SlowChIndex("vsc_mag", &i_V_MAG, &j_V_MAG);
    SlowChIndex("vsc_fra", &i_V_FRA, &j_V_FRA);
    SlowChIndex("az_rel_sun", &i_AZ_REL_SUN, &j_AZ_REL_SUN);
    SlowChIndex("el_rel_sun", &i_EL_REL_SUN, &j_EL_REL_SUN);
    SlowChIndex("ss_prin", &i_SS_PRIN, &j_SS_PRIN);
    SlowChIndex("sip_lat", &i_SIP_LAT, &j_SIP_LAT);
    SlowChIndex("sip_lon", &i_SIP_LON, &j_SIP_LON);
    SlowChIndex("sip_alt", &i_SIP_ALT, &j_SIP_ALT);
    SlowChIndex("sip_time", &i_SIP_TIME, &j_SIP_TIME);
    SlowChIndex("lst", &i_LST, &j_LST);
    SlowChIndex("lat", &i_LAT, &j_LAT);
    SlowChIndex("lon", &i_LON, &j_LON);
    SlowChIndex("mag_model", &i_MAG_MODEL, &j_MAG_MODEL);
    
    SlowChIndex("p_az_mode", &i_AZ_MODE, &j_AZ_MODE);
    SlowChIndex("p_el_mode", &i_EL_MODE, &j_EL_MODE);
    SlowChIndex("p_az1", &i_AZ1, &j_AZ1);
    SlowChIndex("p_az2", &i_AZ2, &j_AZ2);
    SlowChIndex("p_el1", &i_EL1, &j_EL1);
    SlowChIndex("p_el2", &i_EL2, &j_EL2);
    SlowChIndex("p_az_vel", &i_AZ_VEL, &j_AZ_VEL);
    SlowChIndex("p_el_vel", &i_EL_VEL, &j_EL_VEL);
    SlowChIndex("p_ra", &i_RA, &j_RA);
    SlowChIndex("p_dec", &i_DEC, &j_DEC);
    SlowChIndex("p_r", &i_R, &j_R);
    SlowChIndex("sensor_veto", &i_SVETO, &j_SVETO);

    SlowChIndex("dgps_time", &i_dgps_time, &j_dgps_time);
    SlowChIndex("dgps_lat", &i_dgps_lat, &j_dgps_lat);
    SlowChIndex("dgps_lon", &i_dgps_lon, &j_dgps_lon);
    SlowChIndex("dgps_alt", &i_dgps_alt, &j_dgps_alt);
    SlowChIndex("dgps_speed", &i_dgps_speed, &j_dgps_speed);
    SlowChIndex("dgps_dir", &i_dgps_dir, &j_dgps_dir);
    SlowChIndex("dgps_climb", &i_dgps_climb, &j_dgps_climb);
    SlowChIndex("dgps_n_sat", &i_dgps_n_sat, &j_dgps_n_sat);
    SlowChIndex("dgps_pos_index", &i_dgps_pos_index, &j_dgps_pos_index);
    SlowChIndex("dgps_att_ok", &i_dgps_att_ok, &j_dgps_att_ok);
    SlowChIndex("dgps_att_index", &i_dgps_att_index, &j_dgps_att_index);
    
    SlowChIndex("blob0_x", &blob0_xCh, &blob0_xInd);
    SlowChIndex("blob1_x", &blob1_xCh, &blob1_xInd);
    SlowChIndex("blob2_x", &blob2_xCh, &blob2_xInd);
    SlowChIndex("blob0_y", &blob0_yCh, &blob0_yInd);
    SlowChIndex("blob1_y", &blob1_yCh, &blob1_yInd);
    SlowChIndex("blob2_y", &blob2_yCh, &blob2_yInd);
    SlowChIndex("blob0_flux", &blob0_fluxCh, &blob0_fluxInd);
    SlowChIndex("blob1_flux", &blob1_fluxCh, &blob1_fluxInd);
    SlowChIndex("blob2_flux", &blob2_fluxCh, &blob2_fluxInd);
    SlowChIndex("isc_error", &isc_errorCh, &isc_errorInd);
    SlowChIndex("isc_expose", &isc_exposeCh, &isc_exposeInd);
    SlowChIndex("isc_rot", &isc_rotCh, &isc_rotInd);
    SlowChIndex("isc_ra", &isc_raCh, &isc_raInd);
    SlowChIndex("isc_dec", &isc_decCh, &isc_decInd);
    SlowChIndex("isc_apert", &isc_apertCh, &isc_apertInd);
    SlowChIndex("isc_pscale", &isc_pscaleCh, &isc_pscaleInd);
    SlowChIndex("isc_gain", &isc_gainCh, &isc_gainInd);
    SlowChIndex("isc_cenbox", &isc_cenboxCh, &isc_cenboxInd);
    SlowChIndex("isc_apbox", &isc_apboxCh, &isc_apboxInd);
    SlowChIndex("isc_mdist", &isc_mdistCh, &isc_mdistInd);
    SlowChIndex("isc_nblobs", &isc_nblobsCh, &isc_nblobsInd);
    SlowChIndex("isc_focus", &isc_focusCh, &isc_focusInd);
    SlowChIndex("isc_offset", &isc_offsetCh, &isc_offsetInd);
    SlowChIndex("isc_mapmean", &isc_mapmeanCh, &isc_mapmeanInd);
    SlowChIndex("isc_thresh", &isc_threshCh, &isc_threshInd);
    SlowChIndex("isc_grid", &isc_gridCh, &isc_gridInd);
  }

  /********** VSC Data **********/
  i_vsc = GETREADINDEX(vsc_index);
  WriteSlow(i_V_COL, j_V_COL, (int)(VSCData[i_vsc].col * 100));
  WriteSlow(i_V_ROW, j_V_ROW, (int)(VSCData[i_vsc].row * 100));
  WriteSlow(i_V_MAG, j_V_MAG, (int)(VSCData[i_vsc].mag * 100));
  WriteSlow(i_V_FRA, j_V_FRA, VSCData[i_vsc].sf_frame);

  /********** Sun Sensor Data **********/
  i_ss = GETREADINDEX(ss_index);
  WriteSlow(i_AZ_REL_SUN, j_AZ_REL_SUN, SunSensorData[i_ss].raw_az);
  WriteSlow(i_EL_REL_SUN, j_EL_REL_SUN, SunSensorData[i_ss].raw_el);
  WriteSlow(i_SS_PRIN, j_SS_PRIN, SunSensorData[i_ss].prin);

  /********** SIP GPS Data **********/
  WriteSlow(i_SIP_LAT, j_SIP_LAT, (int)(SIPData.GPSpos.lat*DEG2I));
  WriteSlow(i_SIP_LON, j_SIP_LON, (int)(SIPData.GPSpos.lon*DEG2I));
  WriteSlow(i_SIP_ALT, j_SIP_ALT, (int)(SIPData.GPSpos.lat*0.25));
  t = SIPData.GPStime.UTC;
  WriteSlow(i_SIP_TIME, j_SIP_TIME, t >> 16);
  WriteSlow(i_SIP_TIME + 1, j_SIP_TIME, t);


  /************* processed pointing data *************/
  i_point = GETREADINDEX(point_index);
  WriteFast(i_az, (unsigned int)(PointingData[i_point].az * 65536.0/360.0));
  WriteFast(i_el, (unsigned int)(PointingData[i_point].el * 65536.0/360.0));
  WriteFast(i_MAG_AZ,
	    (unsigned int)(PointingData[i_point].mag_az * 65536.0/360.0));
  t = PointingData[i_point].lst;
  WriteSlow(i_LST, j_LST, t >> 16);
  WriteSlow(i_LST + 1, j_LST, t);
  WriteSlow(i_LAT, j_LAT, (int)(PointingData[i_point].lat * DEG2I));
  WriteSlow(i_LON, j_LON, (int)(PointingData[i_point].lon * DEG2I));

  WriteSlow(i_MAG_MODEL, j_MAG_MODEL,
	    (int)(PointingData[i_point].mag_model *DEG2I));
  
  /************* Pointing mode fields *************/
  WriteSlow(i_AZ_MODE, j_AZ_MODE, (int)(CommandData.pointing_mode.az_mode));
  WriteSlow(i_EL_MODE, j_EL_MODE, (int)(CommandData.pointing_mode.el_mode));
  WriteSlow(i_AZ1, j_AZ1, (int)(CommandData.pointing_mode.az1 * DEG2I));
  WriteSlow(i_AZ2, j_AZ2, (int)(CommandData.pointing_mode.az2 * DEG2I));
  WriteSlow(i_EL1, j_EL1, (int)(CommandData.pointing_mode.el1 * DEG2I));
  WriteSlow(i_EL2, j_EL2, (int)(CommandData.pointing_mode.el2 * DEG2I));
  WriteSlow(i_AZ_VEL, j_AZ_VEL,
	    (int)(CommandData.pointing_mode.az_vel * VEL2I));
  WriteSlow(i_EL_VEL, j_EL_VEL,
	    (int)(CommandData.pointing_mode.el_vel * VEL2I));
  WriteSlow(i_RA, j_RA, (int)(CommandData.pointing_mode.ra * H2I));	
  WriteSlow(i_DEC, j_DEC, (int)(CommandData.pointing_mode.dec * DEG2I));
  WriteSlow(i_R, j_R, (int)(CommandData.pointing_mode.r * DEG2I));

  sensor_veto = (!CommandData.use_sun) | ((!CommandData.use_isc)<<1) |
		((!CommandData.use_vsc)<<2) |
		((!CommandData.use_mag)<<3) |
		((!CommandData.use_gps)<<4);

  if (PointingData[i_point].t >= CommandData.pointing_mode.t_start_sched) {
    sensor_veto |= (1 << 5);
  }
  
  WriteSlow(i_SVETO, j_SVETO, sensor_veto);

  /************* dgps fields *************/
  t = DGPSTime;
  WriteSlow(i_dgps_time, j_dgps_time, t >> 16);
  WriteSlow(i_dgps_time + 1, j_dgps_time, t);
  if (t<100) printf("t spike %lu\n", t);
  
  /** Pos fields **/
  i_dgps = GETREADINDEX(dgpspos_index);
  WriteSlow(i_dgps_lat, j_dgps_lat, (int)(DGPSPos[i_dgps].lat * DEG2I));
  WriteSlow(i_dgps_lon, j_dgps_lon, (int)(DGPSPos[i_dgps].lon * DEG2I));
  WriteSlow(i_dgps_alt, j_dgps_alt, (int)(DGPSPos[i_dgps].alt));
  WriteSlow(i_dgps_speed, j_dgps_speed, (int)(DGPSPos[i_dgps].speed * DEG2I));
  WriteSlow(i_dgps_dir, j_dgps_dir, (int)(DGPSPos[i_dgps].direction * DEG2I));
  WriteSlow(i_dgps_climb, j_dgps_climb, (int)(DGPSPos[i_dgps].climb * DEG2I));
  WriteSlow(i_dgps_n_sat, j_dgps_n_sat, DGPSPos[i_dgps].n_sat);
  WriteSlow(i_dgps_pos_index, j_dgps_pos_index, i_dgps);

  /** Att fields **/
  i_dgps = GETREADINDEX(dgpsatt_index);  
  WriteSlow(i_dgps_att_ok, j_dgps_att_ok, DGPSAtt[i_dgps].att_ok);
  WriteSlow(i_dgps_att_index, j_dgps_att_index, i_dgps);

  /** ISC Fields **/
  if (blob_index == 0) {
    i_isc = GETREADINDEX(iscdata_index); 
  }

  /*** Blobs ***/
  WriteSlow(blob0_xCh, blob0_xInd,
      (int)(ISCData[i_isc].az_blobs[blob_index * 3 + 0] * 40.));
  WriteSlow(blob1_xCh, blob1_xInd,
      (int)(ISCData[i_isc].az_blobs[blob_index * 3 + 1] * 40.));
  WriteSlow(blob2_xCh, blob2_xInd,
      (int)(ISCData[i_isc].az_blobs[blob_index * 3 + 2] * 40.));

  WriteSlow(blob0_yCh, blob0_yInd,
      (int)(ISCData[i_isc].el_blobs[blob_index * 3 + 0] * 40.));
  WriteSlow(blob1_yCh, blob1_yInd,
      (int)(ISCData[i_isc].el_blobs[blob_index * 3 + 1] * 40.));
  WriteSlow(blob2_yCh, blob2_yInd,
      (int)(ISCData[i_isc].el_blobs[blob_index * 3 + 2] * 40.));

  WriteSlow(blob0_fluxCh, blob0_fluxInd,
      (int)(ISCData[i_isc].flux_blobs[blob_index * 3 + 0] / 32.));
  WriteSlow(blob1_fluxCh, blob1_fluxInd,
      (int)(ISCData[i_isc].flux_blobs[blob_index * 3 + 1] / 32.));
  WriteSlow(blob2_fluxCh, blob2_fluxInd,
      (int)(ISCData[i_isc].flux_blobs[blob_index * 3 + 2] / 32.));

  if (blob_index >= 5)
    blob_index = 0;

  /*** Camera Info ***/
  WriteSlow(isc_errorCh, isc_errorInd, (int)ISCData[i_isc].error);
  WriteSlow(isc_exposeCh, isc_exposeInd, (int)ISCData[i_isc].exposure);
  WriteSlow(isc_rotCh, isc_rotInd, (int)(ISCData[i_isc].rot * DEG2I));
  WriteSlow(isc_raCh, isc_raInd, (int)(ISCData[i_isc].ra * DEG2LI));
  WriteSlow(isc_raCh + 1, isc_raInd, (int)(ISCData[i_isc].ra * DEG2LI) >> 16);
  WriteSlow(isc_decCh, isc_decInd, (int)(ISCData[i_isc].dec * DEG2LI));
  WriteSlow(isc_decCh + 1, isc_decInd, (int)(ISCData[i_isc].dec * DEG2LI) >>16);
  WriteSlow(isc_apertCh, isc_apertInd, (int)ISCData[i_isc].aperturePosition);
  WriteSlow(isc_pscaleCh, isc_pscaleInd,
      (int)(ISCData[i_isc].platescale * 1000.));
  WriteSlow(isc_gainCh, isc_gainInd, (int)ISCData[i_isc].gain);
  WriteSlow(isc_cenboxCh, isc_cenboxInd, (int)ISCData[i_isc].cenbox);
  WriteSlow(isc_apboxCh, isc_apboxInd, (int)ISCData[i_isc].apbox);
  WriteSlow(isc_mdistCh, isc_mdistInd, (int)ISCData[i_isc].multiple_dist);
  WriteSlow(isc_nblobsCh, isc_nblobsInd, (int)ISCData[i_isc].nblobs);
  WriteSlow(isc_focusCh, isc_focusInd, (int)ISCData[i_isc].focusPosition);
  WriteSlow(isc_offsetCh, isc_offsetInd, (int)ISCData[i_isc].offset);
  WriteSlow(isc_mapmeanCh, isc_mapmeanInd, (int)(ISCData[i_isc].mapmean * 4.));
  WriteSlow(isc_threshCh, isc_threshInd, (int)(ISCData[i_isc].threshold * 10.));
  WriteSlow(isc_gridCh, isc_gridInd, (int)ISCData[i_isc].grid);
}


#define AZ_ACCEL (0.001)
#define AZ_MARGIN 0.5
#define MIN_SCAN 0.2
void DoAzScanMode() {
  double az, p1, p2, v;
  int i_point;

  i_point = GETREADINDEX(point_index);
  az = PointingData[i_point].az; // FIXME - extrapolate velocity

  p1 = CommandData.pointing_mode.az1;
  p2 = CommandData.pointing_mode.az2;
  v = CommandData.pointing_mode.az_vel;
  
  while (az-p2>180.0) az-=360.0;
  while (p1-az>180.0) az+=360.0;

  if (axes_mode.az_vel < -v) axes_mode.az_vel = -v;
  if (axes_mode.az_vel > v) axes_mode.az_vel = v;
  
  if (p1-az>AZ_MARGIN) { // out of range: move to p1
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = p1;
    axes_mode.az_vel = 0.0;
  } else if (az-p2>AZ_MARGIN) { // out of range - move to p2
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = p2;
    axes_mode.az_vel = 0.0;
  } else if (az<p1) {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel < v) axes_mode.az_vel+=AZ_ACCEL;
  } else if (az>p2) {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel > -v) axes_mode.az_vel-=AZ_ACCEL;
  } else {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel>0) axes_mode.az_vel = v;
    else axes_mode.az_vel = -v;
  } 
}

#define MIN_EL 20.0
#define MAX_EL 50.0
#define EL_BORDER 0.1

void DoRasterMode() {
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt;
  double lst;
  int i_point;
  double x, y, r,v;
  double x2, xmin, xmax;
  
  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;
  while (el> 180.0) el-=360.0;
  while (el<-180.0) el += 360.0;
  if (el>80) el = 80; // very bad situation - don't know how this can happen
  if (el<-10) el = -10; // very bad situation - don't know how this can happen

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.ra, CommandData.pointing_mode.dec,
 	     lst, PointingData[i_point].lat,
 	     &caz, &cel);
  radec2azel(CommandData.pointing_mode.ra, CommandData.pointing_mode.dec,
 	     lst+1.0, PointingData[i_point].lat,
 	     &az2, &el2);
  daz_dt = az2 - caz;
  del_dt = el2 - cel;

  /* get elevation limits */
  if (cel < MIN_EL) cel = MIN_EL;
  if (cel > MAX_EL) cel = MAX_EL;
  r = CommandData.pointing_mode.r;
  el1 = CommandData.pointing_mode.el1 = cel + r;
  el2 = CommandData.pointing_mode.el2 = cel - r;
  if (el1>MAX_EL) el1 = MAX_EL;
  if (el2<MIN_EL) el2 = MIN_EL;

  CommandData.pointing_mode.az1 = caz;
  CommandData.pointing_mode.az2 = caz;
  
  // check for out of range in el
  if (el > el1+EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    CommandData.pointing_mode.el_vel = fabs(CommandData.pointing_mode.el_vel);
    return;
  } else if (el < el2-EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    CommandData.pointing_mode.el_vel = -fabs(CommandData.pointing_mode.el_vel);
    return;
  } else if (el> el1) { // turn around
    CommandData.pointing_mode.el_vel = fabs(CommandData.pointing_mode.el_vel);
  } else if (el < el2) { // turn around
    CommandData.pointing_mode.el_vel = -fabs(CommandData.pointing_mode.el_vel);
  }    

  // we must be in range for elevation - go to el-vel mode
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = CommandData.pointing_mode.el_vel + del_dt;

  /** Get x (ie, az*cos_el-caz) **/
  x = az - caz;
  while (x>180.0) x-=360.0;
  while (x<-180.0) x+=360.0;
  x*=cos(el * M_PI/180.0);
  
  /** Get x limits **/
  y = el - cel;
  x2 = r*r-y*y;
  if (x2<0) {
    xmin = xmax = 0.0;
  } else {
    xmax = sqrt(x2);
  }
  if (xmax < MIN_SCAN) xmax = MIN_SCAN;
  xmin = -xmax;

  /* set az v */
  v = CommandData.pointing_mode.az_vel/cos(el * M_PI/180.0);
  
  /* set az mode */
  if (axes_mode.az_vel < -v) axes_mode.az_vel = -v;
  if (axes_mode.az_vel > v) axes_mode.az_vel = v;
  if ((x>xmax+AZ_MARGIN) || (x < xmin - AZ_MARGIN)) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
  } else if (x < xmin) {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel < v) axes_mode.az_vel+=AZ_ACCEL;
  } else if (x > xmax) {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel > -v) axes_mode.az_vel-=AZ_ACCEL;
  } else {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel>0) axes_mode.az_vel = v;
    else axes_mode.az_vel = -v;
  }  
}

void DoRaDecGotoMode() {
  double caz, cel;
  double lst;
  int i_point;
  
  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;

  radec2azel(CommandData.pointing_mode.ra, CommandData.pointing_mode.dec,
 	     lst, PointingData[i_point].lat,
 	     &caz, &cel);

  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = caz;
  axes_mode.az_vel = 0.0;
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = cel;
  axes_mode.el_vel = 0.0;
}

/******************************************************************
 *                                                                *
 * Update Axis Modes: Set axes_mode based on                      *
 *    CommandData.pointing_mode                                   *
 *                                                                *
 ******************************************************************/
void UpdateAxesMode() {
  switch (CommandData.pointing_mode.el_mode) {
  case POINT_VEL:
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = CommandData.pointing_mode.el_vel;
    break;
  case POINT_POINT:
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = CommandData.pointing_mode.el1;
    axes_mode.el_vel = 0.0;
    break;
  case POINT_RASTER:
    DoRasterMode();
    break;
  case POINT_RADEC_GOTO:
    DoRaDecGotoMode();
    break;
  case POINT_LOCK:
    axes_mode.el_mode = AXIS_LOCK;
    axes_mode.el_dest = CommandData.pointing_mode.el1;
    axes_mode.el_vel = 0.0;
    break;
  default:
    fprintf(stderr, "Unknown Elevation Pointing Mode %d: stopping\n",
	    CommandData.pointing_mode.el_mode);
    CommandData.pointing_mode.el_mode = POINT_VEL;
    CommandData.pointing_mode.el_vel = 0.0;
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = 0.0;
    break;
  }

  switch (CommandData.pointing_mode.az_mode) {
  case POINT_VEL:
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = CommandData.pointing_mode.az_vel;
    break;
  case POINT_POINT:
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = CommandData.pointing_mode.az1;
    axes_mode.az_vel = 0.0;
    break;
  case POINT_RASTER:
    // already done in el mode test....
    break;
  case POINT_RADEC_GOTO:
    // alread done
    break;
  case POINT_SCAN:
    DoAzScanMode();
    break;
  default:
    CommandData.pointing_mode.az_mode = POINT_VEL;
    CommandData.pointing_mode.az_vel = 0.0;
    fprintf(stderr, "Unknown Az Pointing Mode %d: stopping\n",
	    CommandData.pointing_mode.az_mode);
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = 0.0;
    break;
  }
    
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
  DoSched();
  UpdateAxesMode();
  StoreData(Txframe, slowTxFields);
  ControlGyroHeat(Txframe, Rxframe, slowTxFields);
  WriteMot(index, Txframe, Rxframe, slowTxFields);
#endif
  BiasControl(Txframe, Rxframe, slowTxFields);
  SyncADC(index, slowTxFields);

  /*** do slow Controls ***/
  if (index == 0) {
    WriteAux(slowTxFields);
    PhaseControl(slowTxFields);
  }
#ifndef BOLOTEST
  ControlAuxMotors(Txframe, Rxframe, slowTxFields);
#endif
  CryoControl(slowTxFields);

  SetReadBits(Txframe);

  /*** write FSync ***/
  write(bbc_fp, (void*)&BBC_Fsync_Word, sizeof(unsigned int));
  /*** Write Frame ***/
  write(bbc_fp, (void*)Txframe, TxFrameBytes());
}
