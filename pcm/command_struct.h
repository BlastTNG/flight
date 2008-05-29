/* mcp: the Spider master control program
 *
 * command_struct.h: global definitions for command system
 * 
 * This software is copyright (C) 2002-2007 University of Toronto
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

#ifndef COMMAND_STRUCT_H
#define COMMAND_STRUCT_H

#include "command_list.h"
#include "channels.h"
#include <time.h>

struct GainStruct {
  unsigned short int P;
  unsigned short int I;
  unsigned short int D;
  unsigned short int SP;
};

enum PointingMode {scan, spin, point};

struct SpinStruct {
  double dps; // Target Gondola Spin Speed
};

struct PointStruct {
  double az; // Target azimuth
  double tol; // Tolerence (how far can we drift from the 
              // requested azimuth before action is required?
};

struct ScanStruct {
  double C; // Azimuth Scan Centre
  double P; // Scan Period in seconds
  double W; // Scan Width zero to Peak (degrees)
  double Wcrit; // Switch to constant accel mode 
               // when |theta-C| > Wcrit
};

// Stores the motor gains
struct MotorGainStruct {
  // Spin mode gains
  double sp_r1; // prop to velocity error term
  double sp_r2; // prop to gondola acceleration
  double sp_p1; // prop to velocity error term
  double sp_p2; // prop to RW speed


  // Scan mode gains
  double sc_r1;  // prop to velocity error term
  double sc_r2; // prop to gondola acceleration
  double sc_p1; // prop to velocity error term
  double sc_p2; // prop to reaction wheel speed
};

// mode        X     Y    vaz   del    w    h
// LOCK              el
// AZEL_GOTO   az    el
// AZ_SCAN     az    el   vaz
// DRIFT                  vaz   vel
// RADEC_GOTO  ra    dec
// VCAP        ra    dec  vaz   vel    r
// CAP         ra    dec  vaz   elstep r
// BOX         ra    xdec  vaz   elstep w    h
struct PointingModeStruct {
  int nw; /* used for gy-offset veto during slews */
  int mode;
  double X;
  double Y;
  double vaz;
  double del;
  double w;
  double h;
  time_t t;
  double ra[4]; // the RAs for radbox (ie, quad)
  double dec[4]; // the decs for radbox (ie, quad)
  // lmf: Need to add in reaction wheel velocity!  
};

struct StarcamCommandData {
  //camera and lens configuration
  short int forced;  //are lens moves forced?
  int expInt;        //exposure interval (ms) (0=triggered)
  int expTime;       //exposure duration (ms)
  int focusRes;      //steps to divide lens range into for focus
  int moveTol;       //precision (ticks) for lens moves

  //image processing configuration
  int maxBlobs;      //max number of blobs to find
  int grid;          //search grid cell size (pix)
  double threshold;  //# sigma threshold for star finding
  int minBlobDist;   //min dist (pix) between blobs
};

struct CommandDataStruct {
  struct StarcamCommandData cam;
  double tableRelMove;     //relative angle to move table by (deg)
  double tableMoveGain;             //P
  struct GainStruct tableGain;      //PID

  //TODO is this the best way to organize this data?
  enum PointingMode spiderMode;     // Are we going to spin, point or scan?
  struct SpinStruct spiderSpin;     // Stores Spin Speed
  struct PointStruct spiderPoint;     // Stores Spin Speed
  struct ScanStruct spiderScan;     // Define Scan Centre, period and width 
  struct MotorGainStruct spiderGain;  // Sets the gain factors for the pivot 
                                      // and reaction wheel.
};

struct ScheduleEvent {
  int t;
  int is_multi;
  int command;
  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];
};

struct ScheduleType {
  int n_sched;
  time_t t0;
  struct ScheduleEvent* event;
};

#endif   //COMMAND_STRUCT_H

void InitCommandData();
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;
