#include <stdio.h>

#include "tx_struct.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"

#define MIN_EL 20.0
#define MAX_EL 65.0

struct AxesModeStruct axes_mode; /* low level velocity mode */

int pinIsIn(void);  /* auxcontrol.c */
void UnwindDiff(double ref, double *A); /* in pointing.c */
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
                double *el);


/************************************************************************/
/*                                                                      */
/*   GetVElev: get the current elevation velocity, given current        */
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
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
  vel -= (PointingData[i_point].gy1_offset - PointingData[i_point].gy1_earth);

  if (ACSData.enc_elev < MIN_EL) vel = 0.2; // go up
  if (ACSData.enc_elev > MAX_EL) vel = -0.2; // go down

  vel *= DPS_TO_ADU1; 

  /* Limit Maximim speed to 0.5 dps*/
  if (vel > 0.5 * DPS_TO_ADU1) vel = 0.5 * DPS_TO_ADU1;
  if (vel < -0.5 * DPS_TO_ADU1) vel = -0.5 * DPS_TO_ADU1;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv) vel = last_vel + max_dv;
  if (dvel < -max_dv) vel = last_vel - max_dv;
  last_vel = vel;

  return (vel);
}

/************************************************************************/
/*                                                                      */
/*   GetVAz: get the current az velocity, given current                 */
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
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
    vel = -drem(PointingData[i_point].az - axes_mode.az_dest, 360.0)
	  * 0.36;
  }
  
  vel_offset =
    -(PointingData[i_point].gy2_offset- PointingData[i_point].gy2_earth)*
    cos(PointingData[i_point].el*M_PI/180.0) -
    (PointingData[i_point].gy3_offset- PointingData[i_point].gy3_earth)*
    sin(PointingData[i_point].el*M_PI/180.0);
  
  vel -= vel_offset;
  vel *= DPS_TO_ADU2; /* convert to gyro units FIXME: use elevation info*/
  
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

/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
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

  static int wait = 100; /* wait 20 frames before controlling. */
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
  v_elev = GetVElev() * 6.0; /* the 6.0 is to improve dynamic range. */
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
  el_rad = (M_PI / 180.0) * PointingData[i_point].el; /* convert to radians */
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  WriteFast(i_cos_el, ucos_el);
  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  WriteFast(i_sin_el, usin_el);

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = GetVAz()*6.0; /* the 6.0 is to improve dynamic range. */
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

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define AZ_ACCEL (0.001)
#define AZ_MARGIN 0.5
#define MIN_SCAN 0.2
void SetAzScanMode(double az, double left, double right, double v, double D) {
  if (axes_mode.az_vel < -v+D) axes_mode.az_vel = -v+D;
  if (axes_mode.az_vel > v+D) axes_mode.az_vel = v+D;

  if (az < left - AZ_MARGIN) { /* out of range: move to left */
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = left;
    axes_mode.az_vel = 0.0;
    isc_pulses.is_fast = 1;
  } else if (az > right + AZ_MARGIN) { /* out of range - move to p2 */
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = right;
    axes_mode.az_vel = 0.0;
    isc_pulses.is_fast = 1;
  } else if (az<left) {
    isc_pulses.is_fast = 0;
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel < v+D) axes_mode.az_vel+=AZ_ACCEL;
  } else if (az > right) {
    isc_pulses.is_fast = 0;
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel > -v+D) axes_mode.az_vel-=AZ_ACCEL;
  } else {
    axes_mode.az_mode = AXIS_VEL;
    if (axes_mode.az_vel>0) {
      axes_mode.az_vel = v+D;
      if (az > right - v) isc_pulses.is_fast = 0;/* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    } else {
      axes_mode.az_vel = -v+D;
      if (az < left + v) isc_pulses.is_fast = 0; /* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    }
  }
}

void DoAzScanMode() {
  double az, left, right, v,w;
  int i_point;

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel  = 0.0;

  i_point = GETREADINDEX(point_index);
  az = PointingData[i_point].az; /* FIXME - extrapolate velocity */

  w = CommandData.pointing_mode.w;
  right = CommandData.pointing_mode.X + w/2;
  left = CommandData.pointing_mode.X - w/2;
  UnwindDiff(left, &az);

  v = CommandData.pointing_mode.vaz;

  SetAzScanMode(az, left, right, v, 0);
}

#define EL_BORDER 0.15
void DoVCapMode() {
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, r,v;
  double x2, xw;
  double left, right;
  static int dir = 1;
  
  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el>80) el = 80; /* very bad situation - dont know how this can happen */
  if (el<-10) el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst+1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  /* get elevation limits */
  if (cel < MIN_EL) cel = MIN_EL;
  if (cel > MAX_EL) cel = MAX_EL;
  r = CommandData.pointing_mode.w;
  el1 = cel + r;
  el2 = cel - r;
  if (el1>MAX_EL) el1 = MAX_EL;
  if (el2<MIN_EL) el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1+EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    dir = -1;
    return;
  } else if (el < el2-EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    dir = 1;
    return;
  } else if (el> el1) { /* turn around */
    dir = -1;
  } else if (el < el2) { /* turn around */
    dir = 1;
  }    
  v_el = CommandData.pointing_mode.del * dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x (ie, (az-caz)*cos_el) **/
/*   x = drem(az - caz, 360.0); */
/*   x*=cos(el * M_PI/180.0); */

  /** Get x limits **/
  y = el - cel;
  x2 = r*r-y*y;
  if (x2<0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN) xw = MIN_SCAN;
  xw /= cos(el * M_PI/180.0);
  left = caz - xw;
  right = caz + xw;

  /* set az v */
  v = CommandData.pointing_mode.vaz/cos(el * M_PI/180.0);
  SetAzScanMode(az, left, right, v, daz_dt);

}

void DoVBoxMode() {
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, x, v;
  double left, right;
  static int dir = 1;
  
  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el>80) el = 80; /* very bad situation - dont know how this can happen */
  if (el<-10) el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst+1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  /* get elevation limits */
  if (cel < MIN_EL) cel = MIN_EL;
  if (cel > MAX_EL) cel = MAX_EL;
  y = CommandData.pointing_mode.h/2.0;
  el1 = cel + y;
  el2 = cel - y;
  if (el1>MAX_EL) el1 = MAX_EL;
  if (el2<MIN_EL) el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1+EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    dir = -1;
    return;
  } else if (el < el2-EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    dir = 1;
    return;
  } else if (el> el1) { /* turn around */
    dir = -1;
  } else if (el < el2) { /* turn around */
    dir = 1;
  }    
  v_el = CommandData.pointing_mode.del * dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x limits **/
  x = CommandData.pointing_mode.w/2.0;
  x = x/cos(el * M_PI/180.0);
  
  left = caz - x;
  right = caz + x;

  /* set az v */
  v = CommandData.pointing_mode.vaz/cos(el * M_PI/180.0);
  SetAzScanMode(az, left, right, v, daz_dt);

}

void DoRaDecGotoMode() {
  double caz, cel;
  double lst;
  int i_point;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;

  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);

  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = caz;
  axes_mode.az_vel = 0.0;
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = cel;
  axes_mode.el_vel = 0.0;
  isc_pulses.is_fast = 0;
}

void DoBoxMode() {
  double caz, cel, sw_2, h_2;
  double bottom, top, left, right;
  double az, az2, el, el2;
  double daz_dt;
  double lst;
  double v;
  int i_point;

  static struct PointingModeStruct last_pm = {
    P_BOX, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0};
  
  static struct {
    double el; /* el of current scan, relative to cel */
    int eldir; /* 1 = going up, -1 = going down */
    double azdir; /* current scan direction */
    int new;
  } S = {0,1,1,1};

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;
  
  if (el>80) el = 80; /* very bad situation - dont know how this can happen */
  if (el<-10) el = -10; /* very bad situation - dont know how this can happen */
  
  v = fabs(CommandData.pointing_mode.vaz/cos(el * M_PI/180.0));

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst+1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);

  UnwindDiff(az, &caz); 

  h_2 = 0.5*CommandData.pointing_mode.h;
  bottom = cel - h_2;
  top = cel + h_2;
  
  sw_2 = 0.5*CommandData.pointing_mode.w / cos(el*M_PI/180.0);
  left = caz - sw_2;
  right = caz + sw_2;
  
  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_pm.X) ||
      (CommandData.pointing_mode.Y != last_pm.Y)) {
    S.new = 1;
    last_pm.X = CommandData.pointing_mode.X;
    last_pm.Y = CommandData.pointing_mode.Y;
  }

  /* if a new command, go to bottom left corner */
  if (S.new) {
    S.el = -h_2;
    S.eldir = 1;
    S.azdir = 1;
    if ( (fabs(az - (left)) < 0.1) &&
	(fabs(el - (bottom)) < 0.1)) {
      S.new = 0;
    } else {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      isc_pulses.is_fast = 1;
      return;
    }
  }

  axes_mode.az_mode = AXIS_VEL;
  if (axes_mode.az_vel < -v + daz_dt) axes_mode.az_vel = -v + daz_dt;
  if (axes_mode.az_vel > v + daz_dt) axes_mode.az_vel = v + daz_dt;
  if (az<left) {
    if (S.azdir<0) {
      S.azdir = 1; 
      S.el += CommandData.pointing_mode.del * S.eldir; /* step up */
    }
    if (axes_mode.az_vel < v + daz_dt) axes_mode.az_vel += AZ_ACCEL;
    isc_pulses.is_fast = 0;
  } else if (az>right) {
    if (S.azdir>0) {
      S.azdir = -1;
      S.el += CommandData.pointing_mode.del * S.eldir; /* step up */
    }
    if (axes_mode.az_vel > -v + daz_dt) axes_mode.az_vel -= AZ_ACCEL;
    isc_pulses.is_fast = 0;
  } else {
    axes_mode.az_vel = v * (double)S.azdir + daz_dt;
    if (S.azdir>0) {
      if (az > right - v) isc_pulses.is_fast = 0; /* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    } else {
      if (az < left + v) isc_pulses.is_fast = 0; /* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    }
  }

  if (S.el> h_2) {
    S.el = h_2;
    S.eldir = -1;
    S.new = 1; /* back to bottom left corner: comment out to go up/down */
  }	
  if (S.el<-h_2) {
    S.el = -h_2;
    S.eldir = 1;
  }
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = S.el + cel;
}

void DoCapMode() {
  double caz, cel, r, x2, y, xw; 
  double bottom, top, left, right;
  double az, az2, el, el2;
  double daz_dt;
  double lst;
  double v;
  int i_point;
  double turn_around;

  static struct PointingModeStruct last_pm = {
    P_BOX, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0};
  
  static struct {
    double el; /* el of current scan, relative to cel */
    int eldir; /* 1 = going up, -1 = going down */
    double azdir; /* current scan direction */
    int new;
  } S = {0,1,1,1};

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;
  
  
  if (el>67.0) el = 67.0; /* very bad situation - dont know how this can happen */
  if (el<22.0) el = 22.0; /* very bad situation - dont know how this can happen */

  v = fabs(CommandData.pointing_mode.vaz/cos(el*M_PI/180.0));
  
  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst+1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);

  UnwindDiff(az, &caz); 

  r = CommandData.pointing_mode.w;
  bottom = cel - r;
  top = cel + r;
    
  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_pm.X) ||
      (CommandData.pointing_mode.Y != last_pm.Y)) {
    S.new = 1;
    last_pm.X = CommandData.pointing_mode.X;
    last_pm.Y = CommandData.pointing_mode.Y;
  }

  /* if a new command, go to bottom */
  if (S.new) {
    S.el = -r;
    S.eldir = 1;
    S.azdir = 1;
    if ( (fabs(az - (caz)) < MIN_SCAN) &&
	(fabs(el - (bottom)) < 0.1)) {
      S.new = 0;
    } else {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = caz;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      isc_pulses.is_fast = 1;
      return;
    }
  }

  /** Get x limits **/
  y = el - cel;
  x2 = r*r-y*y;
  if (x2<0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  
  turn_around = v*0.3; // FRAME_MARGIN * 10 ms - time delay
  turn_around += (v*v*(0.5)/(AZ_ACCEL*100.0))*0.5;  // 
  xw -= turn_around;
  
  if (xw < MIN_SCAN) xw = MIN_SCAN;
  xw /= cos(el * M_PI/180.0);

  left = caz - xw;
  right = caz + xw;

  axes_mode.az_mode = AXIS_VEL;
  if (axes_mode.az_vel < -v + daz_dt) axes_mode.az_vel = -v + daz_dt;
  if (axes_mode.az_vel > v + daz_dt) axes_mode.az_vel = v + daz_dt;
  isc_pulses.is_fast = 0;
  if (az<left) {
    if (S.azdir<0) {
      S.azdir = 1; 
      S.el += CommandData.pointing_mode.del * S.eldir; /* step up */
    }
    if (axes_mode.az_vel < v + daz_dt) axes_mode.az_vel += AZ_ACCEL;
  } else if (az>right) {
    if (S.azdir>0) {
      S.azdir = -1;
      S.el += CommandData.pointing_mode.del * S.eldir; /* step up */
    }
    if (axes_mode.az_vel > -v + daz_dt) axes_mode.az_vel -= AZ_ACCEL;
  } else {
    axes_mode.az_vel = v * (double)S.azdir + daz_dt;
    if (S.azdir>0) {
      if (az > right - v) isc_pulses.is_fast = 0; /* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    } else {
      if (az < left + v) isc_pulses.is_fast = 0; /* within 1s of turnaround */
      else isc_pulses.is_fast = 1;
    }
  }

  if (S.el> r) {
    S.el = r;
    S.eldir = -1;
    S.new = 1; /* back to bottom left corner: comment out to go up/down */
  }	
  if (S.el<-r) {
    S.el = -r;
    S.eldir = 1;
  }
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = S.el + cel;
}

/******************************************************************/
/*                                                                */
/* Update Axis Modes: Set axes_mode based on                      */
/*    CommandData.pointing_mode                                   */
/*                                                                */
/******************************************************************/
void UpdateAxesMode() {
  switch (CommandData.pointing_mode.mode) {
  case P_DRIFT:
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = CommandData.pointing_mode.del;
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = CommandData.pointing_mode.vaz;
    isc_pulses.is_fast = 0;
    break;
  case P_AZEL_GOTO:
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = CommandData.pointing_mode.Y;
    axes_mode.el_vel = 0.0;
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = CommandData.pointing_mode.X;
    axes_mode.az_vel = 0.0;
    isc_pulses.is_fast = 0;
    break;
  case P_AZ_SCAN:
    DoAzScanMode();
    break;
  case P_VCAP:
    DoVCapMode();
    break;
  case P_VBOX:
    DoVBoxMode();
    break;
  case P_BOX:
    DoBoxMode();
    break;
  case P_CAP:
    DoCapMode();
    break;
  case P_RADEC_GOTO:
    DoRaDecGotoMode();
    break;
  case P_LOCK:
    axes_mode.el_mode = AXIS_LOCK;
    axes_mode.el_dest = CommandData.pointing_mode.Y;
    axes_mode.el_vel = 0.0;
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = 0.0;
    isc_pulses.is_fast = 0;
    break;
  default:
    fprintf(stderr, "Unknown Elevation Pointing Mode %d: stopping\n",
	    CommandData.pointing_mode.mode);
    CommandData.pointing_mode.mode = P_DRIFT;
    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;      
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = 0.0;
    axes_mode.az_mode = AXIS_VEL;
    axes_mode.az_vel = 0.0;
    isc_pulses.is_fast = 0;
    break;
  }
}
