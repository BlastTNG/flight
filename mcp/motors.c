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

#include <stdio.h>

#include "channels.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"

#define MIN_EL 22.5
#define MAX_EL 65.0

struct AxesModeStruct axes_mode = {
  .el_dir = 1,
  .az_dir = 0
}; /* low level velocity mode */

int pinIsIn(void);  /* auxcontrol.c */
void SetSafeDAz(double ref, double *A); /* in pointing.c */
void SetSafeDAzC(double ref, double *A, double *C); /* in pointing.c */
void UnwindDiff(double ref, double *A);

void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
    double *el);

/* in radbox.c */
void radbox_endpoints( double az[4], double el[4], double el_in,
    double *az_left, double *az_right, double *min_el,
    double *max_el, double *az_of_bot );


static int last_mode = -1;

/************************************************************************/
/*                                                                      */
/*   GetVElev: get the current elevation velocity, given current        */
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
static double GetVElev(void)
{
  double vel = 0;
  static double last_vel = 0;
  double dvel;
  int i_point;
  double max_dv = 1.6;

  i_point = GETREADINDEX(point_index);

  if (axes_mode.el_mode == AXIS_VEL) {
    vel = axes_mode.el_vel;
  } else if (axes_mode.el_mode == AXIS_POSITION) {
    vel = (axes_mode.el_dest - PointingData[i_point].el) * 0.36;
  } else if (axes_mode.el_mode == AXIS_LOCK) {
    /* for the lock, only use the elevation encoder */
    vel = (axes_mode.el_dest - ACSData.enc_elev) * 0.64;
  }

  /* correct offset and convert to Gyro Units */
  vel -= (PointingData[i_point].gy1_offset - PointingData[i_point].gy1_earth);

  if (ACSData.enc_elev < MIN_EL)
    vel = 0.2; // go up
  if (ACSData.enc_elev > MAX_EL)
    vel = -0.2; // go down

  /* Limit Maximim speed to 0.5 dps*/
  if (vel > 0.5)
    vel = 0.5;
  if (vel < -0.5)
    vel = -0.5;

  vel *= DPS_TO_GY16;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv)
    vel = last_vel + max_dv;
  if (dvel < -max_dv)
    vel = last_vel - max_dv;
  last_vel = vel;

  return (vel);
}

/************************************************************************/
/*                                                                      */
/*   GetVAz: get the current az velocity, given current                 */
/*   pointing mode, etc..                                               */
/*                                                                      */
/************************************************************************/
static int GetVAz(void)
{
  double vel = 0;
  static int last_vel = 0;
  int dvel;
  int i_point;
  double vel_offset;
  double az, az_dest;
  double max_dv = 20;

  i_point = GETREADINDEX(point_index);

  if (axes_mode.az_mode == AXIS_VEL) {
    vel = axes_mode.az_vel;
  } else if (axes_mode.az_mode == AXIS_POSITION) {
    az = PointingData[i_point].az;
    az_dest = axes_mode.az_dest;
    SetSafeDAz(az, &az_dest);
    vel = -(az - az_dest) * 0.36;
  }

  vel_offset =
    -(PointingData[i_point].gy2_offset- PointingData[i_point].gy2_earth)*
    cos(PointingData[i_point].el * M_PI / 180.0) -
    (PointingData[i_point].gy3_offset- PointingData[i_point].gy3_earth)*
    sin(PointingData[i_point].el * M_PI / 180.0);

  vel -= vel_offset;
  vel *= DPS_TO_GY16;

  /* Limit Maximim speed */
  if (vel > 2000)
    vel = 2000;
  if (vel < -2000)
    vel = -2000;

  /* limit Maximum acceleration */
  dvel = vel - last_vel;
  if (dvel > max_dv)
    vel = last_vel + max_dv;
  if (dvel < -max_dv)
    vel = last_vel - max_dv;
  last_vel = vel;

  return (vel);
}

/************************************************************************/
/*                                                                      */
/*    WriteMot: motors, and, for convenience, the inner frame lock      */
/*                                                                      */
/************************************************************************/
void WriteMot(int TxIndex, unsigned short *RxFrame)
{
  static struct NiosStruct* elVreqAddr;
  static struct NiosStruct* azVreqAddr;
  static struct NiosStruct* cosElAddr;
  static struct NiosStruct* sinElAddr;

  static struct NiosStruct* gPElAddr;
  static struct NiosStruct* gIElAddr;
  static struct NiosStruct* gPRollAddr;
  static struct NiosStruct* gPAzAddr;
  static struct NiosStruct* gIAzAddr;
  static struct NiosStruct* gPPivotAddr;
  static struct NiosStruct* setReacAddr;
  static struct NiosStruct* emfGainAddr;
  static struct NiosStruct* emfOffsetAddr;
  static struct NiosStruct* useAnalogueAddr;

  static int wait = 100; /* wait 20 frames before controlling. */
  double el_rad;
  unsigned int ucos_el;
  unsigned int usin_el;

  int v_elev, v_az, elGainP, elGainI, rollGainP;
  int azGainP, azGainI, pivGainP;
  int i_point;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    elVreqAddr = GetNiosAddr("el_vreq");
    azVreqAddr = GetNiosAddr("az_vreq");
    cosElAddr = GetNiosAddr("cos_el");
    sinElAddr = GetNiosAddr("sin_el");

    gPElAddr = GetNiosAddr("g_p_el");
    gIElAddr = GetNiosAddr("g_i_el");
    gPRollAddr = GetNiosAddr("g_p_roll");
    gPAzAddr = GetNiosAddr("g_p_az");
    gIAzAddr = GetNiosAddr("g_i_az");
    gPPivotAddr = GetNiosAddr("g_p_pivot");
    setReacAddr = GetNiosAddr("set_reac");
    emfGainAddr = GetNiosAddr("emf_gain");
    emfOffsetAddr = GetNiosAddr("emf_offset");

    useAnalogueAddr = GetNiosAddr("use_analogue");
  }

  i_point = GETREADINDEX(point_index);

  /***************************************************/
  /**           Elevation Drive Motors              **/
  /* elevation speed */
  v_elev = floor(GetVElev() * 6.0 + 0.5);
  /* the 6.0 is to improve dynamic range on the elevation speeds. */
  if (v_elev > 32767)
    v_elev = 32767;
  if (v_elev < -32768)
    v_elev = -32768;
  WriteData(elVreqAddr, 32768 + v_elev, NIOS_QUEUE);

  /* zero motor gains if the pin is in */
  if ((pinIsIn() && !CommandData.force_el) || CommandData.disable_el) {
    elGainP = elGainI = 0;
  } else {
    elGainP = CommandData.ele_gain.P;
    elGainI = CommandData.ele_gain.I;	
  }
  /* proportional term for el motor */
  WriteData(gPElAddr, elGainP, NIOS_QUEUE);
  /* integral term for el_motor */
  WriteData(gIElAddr, elGainI, NIOS_QUEUE);


  /***************************************************/
  /*** Send elevation angles to acs1 from acs2 ***/
  /* cos of el enc */
  el_rad = (M_PI / 180.0) * PointingData[i_point].el; /* convert to radians */
  ucos_el = (unsigned int)((cos(el_rad) + 1.0) * 32768.0);
  WriteData(cosElAddr, ucos_el, NIOS_QUEUE);
  /* sin of el enc */
  usin_el = (unsigned int)((sin(el_rad) + 1.0) * 32768.0);
  WriteData(sinElAddr, usin_el, NIOS_QUEUE);

  /***************************************************/
  /**            Azimuth Drive Motors              **/
  v_az = GetVAz() * 6.0; /* the 6.0 is to improve dynamic range. */
  if (v_az > 32767)
    v_az = 32767;
  if (v_az < -32768)
    v_az = -32768;
  WriteData(azVreqAddr, 32768 + v_az, NIOS_QUEUE);

  if ((CommandData.disable_az) || (wait > 0)) {
    azGainP = 0;
    azGainI = 0;
    pivGainP = 0;
  } else {
    azGainP = CommandData.azi_gain.P;
    azGainI = CommandData.azi_gain.I;
    pivGainP = CommandData.pivot_gain.P;
  }

  /* p term for az motor */
  WriteData(gPAzAddr, azGainP, NIOS_QUEUE);
  /* I term for az motor */
  WriteData(gIAzAddr, azGainI, NIOS_QUEUE);
  /* p term for pivot motor */
  WriteData(gPPivotAddr, pivGainP, NIOS_QUEUE);
  /* setpoint for reaction wheel */
  WriteData(setReacAddr, CommandData.pivot_gain.SP, NIOS_QUEUE);

  /* reaction wheel back-EMF gain correction */
  WriteData(emfGainAddr, CommandData.emf_gain, NIOS_QUEUE);
  /* reaction wheel back-EMF offset correction */
  WriteData(emfOffsetAddr, CommandData.emf_offset + 32767, NIOS_QUEUE);

  /***************************************************/
  /**                Roll Drive Motors              **/
  if (PointingData[i_point].gy_roll_amp > 0.003) {
    rollGainP = 2200.0 / PointingData[i_point].gy_roll_amp;
    rollGainP *= (CommandData.roll_gain.P / 32768.0);
  } else {
    rollGainP = CommandData.roll_gain.P;
  }
  if (rollGainP > CommandData.roll_gain.P)
    rollGainP = CommandData.roll_gain.P;

  if (wait > 0)
    rollGainP = 0;

  /* p term for roll motor */
  WriteData(gPRollAddr, rollGainP, NIOS_QUEUE);

  if (wait > 0)
    wait--;

  /* Gyro selection code */
  WriteData(useAnalogueAddr, CommandData.use_analogue_gyros, NIOS_FLUSH);
}

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/
#define AZ_ACCEL (0.001)
#define MIN_SCAN 0.2
static void SetAzScanMode(double az, double left, double right, double v,
    double D)
{
    if (axes_mode.az_vel < -v + D)
      axes_mode.az_vel = -v + D;
    if (axes_mode.az_vel > v + D)
      axes_mode.az_vel = v + D;

    if (az < left) {
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel < v + D)
        axes_mode.az_vel += AZ_ACCEL;
    } else if (az > right) {
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > -v + D)
        axes_mode.az_vel -= AZ_ACCEL;
    } else {
      axes_mode.az_mode = AXIS_VEL;
      if (axes_mode.az_vel > 0) {
        axes_mode.az_vel = v + D;
        if (az > right - v) /* within 1 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      } else {
        axes_mode.az_vel = -v + D;
        if (az < left + v) /* within 1 sec of turnaround */
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
        else
          isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      }
    }
  }

static void DoAzScanMode(void)
{
  static double last_x=0, last_w = 0;
  double az, left, right, v,w;
  int i_point;

  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = CommandData.pointing_mode.Y;
  axes_mode.el_vel  = 0.0;

  i_point = GETREADINDEX(point_index);
  az = PointingData[i_point].az; /* FIXME - extrapolate velocity */

  w = CommandData.pointing_mode.w;
  right = CommandData.pointing_mode.X + w / 2;
  left = CommandData.pointing_mode.X - w / 2;

  SetSafeDAz(left, &az);

  v = CommandData.pointing_mode.vaz;

  if (last_x!= CommandData.pointing_mode.X || last_w != w) {
    if (az < left) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else if (az > right) {
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = right;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
    } else {
      // once we are within the new az/w range, we can mark this as 'last'.
      last_x = CommandData.pointing_mode.X;
      last_w = w;
      SetAzScanMode(az, left, right, v, 0);
    }
  } else {
    SetAzScanMode(az, left, right, v, 0);
  }
}

#define EL_BORDER 1.0
static void DoVCapMode(void)
{
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, r,v;
  double x2, xw;
  double left, right;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el > 80)
    el = 80; /* very bad situation - dont know how this can happen */
  if (el < -10)
    el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;
  SetSafeDAz(az, &caz);

  /* get elevation limits */
  if (cel < MIN_EL)
    cel = MIN_EL;
  if (cel > MAX_EL)
    cel = MAX_EL;
  r = CommandData.pointing_mode.w;
  el1 = cel + r;
  el2 = cel - r;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    axes_mode.el_dir = -1;
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    axes_mode.el_dir = 1;
    return;
  } else if (el > el1) { /* turn around */
    axes_mode.el_dir = -1;
  } else if (el < el2) { /* turn around */
    axes_mode.el_dir = 1;
  }
  v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x limits **/
  y = el - cel;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  left = caz - xw;
  right = caz + xw;

  /* set az v */
  v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v, daz_dt);
}

static void DoVBoxMode(void)
{
  double caz, cel;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt, v_el;
  double lst;
  int i_point;
  double y, x, v;
  double left, right;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  if (el > 80)
    el = 80; /* very bad situation - dont know how this can happen */
  if (el < -10)
    el = -10; /* very bad situation - dont know how this can happen */

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, PointingData[i_point].lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;
  SetSafeDAz(az, &caz);

  /* get elevation limits */
  if (cel < MIN_EL)
    cel = MIN_EL;
  if (cel > MAX_EL)
    cel = MAX_EL;
  y = CommandData.pointing_mode.h / 2.0;
  el1 = cel + y;
  el2 = cel - y;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    axes_mode.el_dir = -1;
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    axes_mode.el_dir = 1;
    return;
  } else if (el> el1) { /* turn around */
    axes_mode.el_dir = -1;
  } else if (el < el2) { /* turn around */
    axes_mode.el_dir = 1;
  }
  v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

  /* we must be in range for elevation - go to el-vel mode */
  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

  /** Get x limits **/
  x = CommandData.pointing_mode.w / 2.0;
  x = x / cos(el * M_PI / 180.0);

  left = caz - x;
  right = caz + x;

  /* set az v */
  v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v, daz_dt);
}

static void DoRaDecGotoMode(void)
{
  double caz, cel;
  double lst, az;
  int i_point;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;

  az = PointingData[i_point].az;

  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, PointingData[i_point].lat,
      &caz, &cel);
  SetSafeDAz(az, &caz);

  axes_mode.az_mode = AXIS_POSITION;
  axes_mode.az_dest = caz;
  axes_mode.az_vel = 0.0;
  axes_mode.el_mode = AXIS_POSITION;
  axes_mode.el_dest = cel;
  axes_mode.el_vel = 0.0;
  isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
}

static void DoNewCapMode(void)
{
  double caz, cel, r, x2, y, xw;
  double bottom, top, left, right;
  double next_left, next_right, az_distance;
  double az, az2, el, el1, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i_point;
  int new_step = 0;

  static double last_X=0, last_Y=0, last_w=0;
  static double v_el = 0;
  static double targ_el=0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  r = CommandData.pointing_mode.w;
  bottom = cel - r;
  top = cel + r;

  // FIXME: reboot proofing...

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (last_mode != P_CAP)) {
    if ( (fabs(az - (caz)) < 0.1) &&
        (fabs(el - (bottom)) < 0.05)) {
      last_X = CommandData.pointing_mode.X;
      last_Y = CommandData.pointing_mode.Y;
      last_w = CommandData.pointing_mode.w;
    } else {
      last_w = 0; // remember we are moving...
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = caz;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = -r;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  /** Get x limits at the next elevation row **/
  y = targ_el; //el - cel + CommandData.pointing_mode.del*el_dir;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  next_left = caz - xw;
  next_right = caz + xw;

  /** Get x limits at the current elevation **/
  y = el - cel;
  x2 = r * r - y * y;
  if (x2 < 0) {
    xw = 0.0;
  } else {
    xw = sqrt(x2);
  }
  if (xw < MIN_SCAN)
    xw = MIN_SCAN;
  xw /= cos(el * M_PI / 180.0);
  left = caz - xw;
  right = caz + xw;

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      az_distance = next_right - left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>=r) {
      targ_el = r;
      axes_mode.el_dir=-1;
    } else if (targ_el<=-r) {
      targ_el = -r;
      axes_mode.el_dir = 1;
    }
  }

  el1 = cel + r;
  el2 = cel - r;
  if (el1 > MAX_EL)
    el1 = MAX_EL;
  if (el2 < MIN_EL)
    el2 = MIN_EL;

  /* check for out of range in el */
  if (el > el1 + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el1;
    axes_mode.el_dir = -1;
    return;
  } else if (el < el2 - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = el2;
    axes_mode.el_dir = 1;
    return;
  }
  /* else if (el > el1) { */
  /*     axes_mode.el_dir = -1; */
  /*   } else if (el < el2) {  */
  /*     axes_mode.el_dir = 1; */
  /*   }     */

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

static void DoNewBoxMode(void)
{
  double caz, cel, w, h;
  double bottom, top, left, right;
  double az, az2, el, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i_point;
  int new_step = 0;
  int new = 0;

  static double last_X=0, last_Y=0, last_w=0, last_h = 0;
  static double v_el = 0;
  static double targ_el=0.0;

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

  /* get raster center and sky drift speed */
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst, lat,
      &caz, &cel);
  radec2azel(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
      lst + 1.0, lat,
      &az2, &el2);
  daz_dt = drem(az2 - caz, 360.0);
  del_dt = el2 - cel;

  SetSafeDAz(az, &caz);

  w = CommandData.pointing_mode.w/cos(el * M_PI / 180.0);
  h = CommandData.pointing_mode.h;
  bottom = cel - h*0.5;
  top = cel + h*0.5;
  left = caz - w*0.5;
  right = caz + w*0.5;

  if (top > MAX_EL)
    top = MAX_EL;
  if (bottom < MIN_EL)
    bottom = MIN_EL;

  // FIXME: reboot proofing...

  new = 0;

  /* If a new command, reset to bottom row */
  if ((CommandData.pointing_mode.X != last_X) ||
      (CommandData.pointing_mode.Y != last_Y) ||
      (CommandData.pointing_mode.w != last_w) ||
      (CommandData.pointing_mode.h != last_h) ||
      (last_mode != P_BOX)) {
    new = 1;
  }
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  /* If a new command, reset to bottom row */
  if (new) {
    if ( (fabs(az - left) < 0.1) &&
        (fabs(el - bottom) < 0.05)) {
      last_X = CommandData.pointing_mode.X;
      last_Y = CommandData.pointing_mode.Y;
      last_w = CommandData.pointing_mode.w;
      last_h = CommandData.pointing_mode.h;
    } else {
      last_w = 0; // remember we are moving...
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = left;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = -h*0.5;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      t = w/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      t = w/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el - (el-cel))/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>h*0.5) {
      targ_el = h*0.5;
      axes_mode.el_dir=-1;
    } else if (targ_el<-h*0.5) {
      targ_el = -h*0.5;
      axes_mode.el_dir = 1;
    }
  }
  /* check for out of range in el */
  if (el > top + EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = top;
    axes_mode.el_dir = -1;
    return;
  } else if (el < bottom - EL_BORDER) {
    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_vel = 0.0;
    axes_mode.el_dest = bottom;
    axes_mode.el_dir = 1;
    return;
  }

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

void DoQuadMode(void) // aka radbox
{
  double bottom, top, left, right, next_left, next_right, az_distance;
  double az, az2, el, el2;
  double daz_dt, del_dt;
  double lst, lat;
  double v_az, t=1;
  int i, i_point;
  int new_step = 0;
  double c_az[4], c_el[4]; // corner az and corner el
  double az_of_bot;
  int new;
  //int i_top, i_bot, new;

  static double last_ra[4] = {0,0,0,0}, last_dec[4] = {0,0,0,0};
  static double v_el = 0;
  static double targ_el=0.0; // targ_el is in degrees from bottom

  i_point = GETREADINDEX(point_index);
  lst = PointingData[i_point].lst;
  lat = PointingData[i_point].lat;
  az = PointingData[i_point].az;
  el = PointingData[i_point].el;

  /* convert ra/decs to az/el */
  for (i=0; i<4; i++) {
    radec2azel(CommandData.pointing_mode.ra[i],
        CommandData.pointing_mode.dec[i],
        lst, lat,
        c_az+i, c_el+i);
  }

  /* get sky drift speed */
  radec2azel(CommandData.pointing_mode.ra[0],
      CommandData.pointing_mode.dec[0],
      lst+1.0, lat,
      &az2, &el2);

  UnwindDiff(az, &az2);

  daz_dt = drem(az2 - c_az[0], 360.0);
  del_dt = el2 - c_el[0];

  radbox_endpoints(c_az, c_el, el, &left, &right, &bottom, &top, &az_of_bot);

  SetSafeDAz(az, &left); // don't cross the sun
  UnwindDiff(left, &right);

  SetSafeDAz(az, &az_of_bot); // correct left

  if (right-left < MIN_SCAN) {
    left = (left+right)/2.0 - MIN_SCAN/2.0;
    right = left + MIN_SCAN;
  }

  new = 0;
  if (last_mode != P_QUAD) new = 1;
  if (el < bottom - 0.5) new = 1;
  if (el > top + 0.5) new = 1;
  if (az < left - 2.0) new = 1;
  if (az > right + 2.0) new = 1;

  for (i=0; i<4; i++) {
    if (CommandData.pointing_mode.ra[i] != last_ra[i]) new = 1;
    if (CommandData.pointing_mode.dec[i] != last_dec[i]) new = 1;
  }

  if (new) {
    if ( (fabs(az - az_of_bot) < 0.1) &&
        (fabs(el - bottom) < 0.05)) {
      for (i=0; i<4; i++) {
        last_ra[i] = CommandData.pointing_mode.ra[i];
        last_dec[i] = CommandData.pointing_mode.dec[i];
      }
    } else {
      last_dec[0] = -99.9745; // remember it is new....
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = az_of_bot;
      axes_mode.az_vel = 0.0;
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = bottom;
      axes_mode.el_vel = 0.0;
      v_el = 0.0;
      targ_el = 0.0;
      axes_mode.el_dir = 1;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 1;
      return;
    }
  }

  if (targ_el<0) {
    targ_el = 0;
  }
  if (targ_el>top-bottom) {
    targ_el = top-bottom;
  }

  radbox_endpoints(c_az, c_el, targ_el+bottom, &next_left,
      &next_right, &bottom, &top, &az_of_bot);


  // make next close to this...
  UnwindDiff(left, &next_left);
  UnwindDiff(left, &next_right);

  if (next_right-next_left < MIN_SCAN) {
    next_left = (next_left+next_right)/2.0 - MIN_SCAN/2.0;
    next_right = next_left + MIN_SCAN;
  }

  /* set az v */
  v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
  SetAzScanMode(az, left, right, v_az, daz_dt);

  /** set El V **/
  new_step = 0;
  if (az<left) {
    if (axes_mode.az_dir < 0) {
      az_distance = next_right - left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = 1;
  } else if (az>right) {
    if (axes_mode.az_dir > 0) {
      az_distance = right - next_left;
      t = az_distance/v_az + 2.0*v_az/(AZ_ACCEL * SR);
      new_step = 1;
    }
    axes_mode.az_dir = -1;
  }

  if (new_step) {
    // set v for this step
    v_el = (targ_el+bottom - el)/t;
    // set targ_el for the next step
    targ_el += CommandData.pointing_mode.del*axes_mode.el_dir;
    if (targ_el>top-bottom) {
      targ_el = top-bottom;
      axes_mode.el_dir=-1;
    } else if (targ_el<0) {
      targ_el = 0;
      axes_mode.el_dir = 1;
    }
  }

  axes_mode.el_mode = AXIS_VEL;
  axes_mode.el_vel = v_el + del_dt;

}

/******************************************************************
 *                                                                *
 * Update Axis Modes: Set axes_mode based on                      *
 *    CommandData.pointing_mode                                   *
 *                                                                *
 ******************************************************************/
void UpdateAxesMode(void)
{
  switch (CommandData.pointing_mode.mode) {
    case P_DRIFT:
      axes_mode.el_mode = AXIS_VEL;
      axes_mode.el_vel = CommandData.pointing_mode.del;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = CommandData.pointing_mode.vaz;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
    case P_AZEL_GOTO:
      axes_mode.el_mode = AXIS_POSITION;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_POSITION;
      axes_mode.az_dest = CommandData.pointing_mode.X;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
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
      DoNewBoxMode();
      break;
    case P_CAP:
      DoNewCapMode();
      break;
    case P_RADEC_GOTO:
      DoRaDecGotoMode();
      break;
    case P_QUAD: // aka radbox
      DoQuadMode();
      break;
    case P_LOCK:
      axes_mode.el_mode = AXIS_LOCK;
      axes_mode.el_dest = CommandData.pointing_mode.Y;
      axes_mode.el_vel = 0.0;
      axes_mode.az_mode = AXIS_VEL;
      axes_mode.az_vel = 0.0;
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
    default:
      bprintf(warning, "Pointing: Unknown Elevation Pointing Mode %d: "
          "stopping\n", CommandData.pointing_mode.mode);
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
      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
      break;
  }
  last_mode = CommandData.pointing_mode.mode;
}
