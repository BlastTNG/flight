/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2006 University of Toronto
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <limits.h>

#include "ezstep.h"
#include "mcp.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "hwpr.h"

void nameThread(const char*);		/* mcp.c */
double LockPosition(double elevation);	/* commands.c */
extern short int InCharge;		/* tx.c */

/* actuator bus setup paramters */
#define ACTBUS_CHATTER	EZ_CHAT_ACT    // EZ_CHAT_ACT (normal) | EZ_CHAT_BUS (debugging)
#define ACT_BUS "/dev/ttySI15"
#define NACT 6
#define LOCKNUM 3
#define HWPRNUM 4
#define SHUTTERNUM 5
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
				 "Lock Motor", HWPR_NAME, "Shutter"};
static const int id[NACT] = {EZ_WHO_S1, EZ_WHO_S2, EZ_WHO_S3, 
			     EZ_WHO_S5, HWPR_ADDR, EZ_WHO_S8};
#define ID_ALL_ACT  EZ_WHO_G1_4
//set microstep resolution
#define LOCK_PREAMBLE "j256"
#define SHUTTER_PREAMBLE "j64"
//set encoder/microstep ratio (aE25600), coarse correction band (aC50),
//fine correction tolerance (ac%d), stall retries (au5), 
//enable encoder feedback mode (n8)
//NB: this is a printf template now, requires a move tolerance (ac) to be set
#define ACT_PREAMBLE  "aE25600aC50ac%dau5n8"
static struct ezbus bus;

#define POLL_TIMEOUT 25			/* 5s @ 5Hz */
#define	MAX_SERIAL_ERRORS 20		/* after this many, repoll bus */
static int poll_timeout = POLL_TIMEOUT; /* track time since last repoll */
static int actbus_reset = 1;		/* 1 means actbus is on */
static unsigned int actuators_init = 0;	/* bitfield for when actuators usable */

/* Lock motor parameters and data */
#define LOCK_MOTOR_DATA_TIMER 100   /* 1 second */
#define DRIVE_TIMEOUT 3000	    /* 30 seconds */
int lock_timeout = -1;

#define LOCK_MIN_POT 300      //actual min stop: ~120 (fully extended)
#define LOCK_MAX_POT 16368    //max stop at saturation: 16368 (fully retracted)
#define LOCK_POT_RANGE 300

static struct lock_struct {
  int pos;		  //raw step count
  unsigned short adc[4];  //ADC readout (including pot)
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

// Shutter motor parameters and data

#define  SHUTTER_MOTOR_DATA_TIMER 100 /* 1 second */
#define  SHUTTER_TIMEOUT 3000         /* 30 seconds */
#define  SHUTTER_CLOSED_BIT 0x04      // /7?4 returns 15 when shutter is closed and
                                      // returns 11 when shutter is not closed
//#define  SHUTTER_OPEN 7               // The choice of 7 is arbitrary
#define SHUTTER_SLEEP 100000 /* 100 milliseconds */
//#define  SHUTTER_SLEEP 50000
#define  SHUTTER_IS_CLOSED 2
#define  SHUTTER_IS_UNK    1
#define  SHUTTER_IS_OPEN   0
#define  SHUTTER_POLARITY  0  // If polarity on one pair of windings is reversed
                              // then motor will turn in opposite direction
int shutter_timeout = -1;

static struct shutter_struct {
  int pos;
  int out;               // shutter not out (presumed in) = 0
                         // shutter out (limit switch depressed) = 1
  int in;                // shutter in = 1 (opto-switch blocked)
                         // shutter not in = 0 (opto-switch not blocked)
  unsigned int  state;
} shutter_data = { .state = SHUTTER_UNK };

/* Secondary actuator data and parameters */
#define LVDT_FILT_LEN 25      //5s @ 5Hz
#define DEFAULT_DR    32768   //value to use if reading file fails
#define MIN_ENC	      1000    //minimum acceptable encoder vlaue, load dr below
#define ACTBUS_TRIM_WAIT  3*LVDT_FILT_LEN //thrice LVDT_FILT_LEN
					  //wait between trims, and after moves

static struct act_struct {
  int pos;	//raw step count
  int enc;	//encoder reading
  int lvdt;	//lvdt-inferred position of this motor
  int dr;	//dead reckoning (best-guess absolute position)
} act_data[3];

static unsigned int actbus_flags = 0;
static int act_trim_wait = 0;
static int act_trim_flag_wait = 0;

#define ACT_FL_TRIM_WAIT  0x001
#define ACT_FL_TRIMMED	  0x002
#define ACT_FL_BUSY0	  0x004
#define ACT_FL_BUSY1	  0x008
#define ACT_FL_BUSY2	  0x010
#define ACT_FL_BUSY(i)	  (ACT_FL_BUSY0 << i)
#define ACT_FL_BUSY_MASK  (ACT_FL_BUSY0 | ACT_FL_BUSY1 | ACT_FL_BUSY2)
#define ACT_FL_BAD_MOVE	  0x020

/* Secondary focus crap */
static double t_primary = -1, t_secondary = -1;
static double focus = -1;	  /* set in ab thread, read in fc thread */
static double correction = 0;     /* set in fc thread, read in ab thread */

/************************************************************************/
/*                                                                      */
/*    Actuator Logic: servo focus based on thermal model/commands       */
/*                                                                      */
/************************************************************************/
//simple check for encoder in a well-initialized state
static inline int encOK(int enc)
{
  return (enc > MIN_ENC);
}

//write DR to disk
static void WriteDR()
{
  int fp, n, i;

  /** write the default file */
  fp = open("/data/etc/blast/act.dr", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "act.dr open()");
    return;
  }
  for (i=0; i<3; i++) {
    if ((n = write(fp, &act_data[i].dr, sizeof(int))) < 0) {
      berror(err, "act.dr write()");
      return;
    }
  }
  if ((n = close(fp)) < 0) {
    berror(err, "act.dr close()");
    return;
  }
}

//on initialization, or bad enc detection, read DR from disk
void ReadDR()
{
  int fp, n_read = 0, read_fail = 0, i;

  if ((fp = open("/data/etc/blast/act.dr", O_RDONLY)) < 0) {
    read_fail = 1;
    berror(err, "Unable to open act.dr file for reading");
  } else {
    for (i=0; i<3; i++) {
      if ((n_read = read(fp, &act_data[i].dr, sizeof(int))) < 0) {
	//read failed
	read_fail = 1;
	berror(err, "act.dr read()");
	break;
      } else if (n_read != sizeof(int)) {
	//short read
	read_fail = 1;
	bprintf(err, "act.dr read(): wrong number of bytes");
	break;
      } else if (!encOK(act_data[i].dr)) {
	//data not reasonable
	read_fail = 1;
	bprintf(err, "act.dr read(): bad encoder data");
	break;
      }
    }
    if (close(fp) < 0)
      berror(err, "act.dr close()");
  }

  if (read_fail) {
    bprintf(info, "Read of act.dr failed. Using default value %d", DEFAULT_DR);
    for (i=0; i<3; i++) act_data[i].dr = DEFAULT_DR;
  }
}

static int CheckMove(int goal0, int goal1, int goal2)
{
  int maxE, minE;

  int lvdt_low = CommandData.actbus.lvdt_low;
  int lvdt_high = CommandData.actbus.lvdt_high;
  int lvdt_delta = CommandData.actbus.lvdt_delta;

  if (goal0 < goal1) {
   maxE = goal1;
   minE = goal0;
  } else {
   maxE = goal0;
   minE = goal1;
  }

  if (goal2 > maxE)
    maxE = goal2;
  else if (goal2 < minE)
    minE = goal2;

  bprintf(info, "%d %d %d | %d %d | %d %d | %d %d", goal0, goal1, goal2, 
      minE, maxE, lvdt_low, lvdt_high, maxE - minE, lvdt_delta);

  if (minE < lvdt_low || maxE > lvdt_high || maxE - minE > lvdt_delta) {
    bputs(warning, "Move Out of Range.");
    actbus_flags |= ACT_FL_BAD_MOVE;
  } else
    actbus_flags &= ~ACT_FL_BAD_MOVE;

  return actbus_flags & ACT_FL_BAD_MOVE;
}

static char preamble_buf[EZ_BUS_BUF_LEN];
static inline char* actPreamble(unsigned short tol)
{
  sprintf(preamble_buf, ACT_PREAMBLE, tol);
  return preamble_buf;
}

static void ReadActuator(int num)
{
  if (!EZBus_IsUsable(&bus, id[num])) return;

  EZBus_ReadInt(&bus, id[num], "?0", &act_data[num].pos);
  EZBus_ReadInt(&bus, id[num], "?8", &act_data[num].enc);
}

//Set both dead reckoning and encoder to trim value, dump dr to disk
void actEncTrim(int *trim)
{
  int i;
  char buffer[EZ_BUS_BUF_LEN];

  bprintf(info, "trim enc and dr to (%d, %d, %d)", trim[0], trim[1], trim[2]);

  for (i=0; i<3; i++) {
    /* set the dr */
    act_data[i].dr = trim[i];
    /* Set the encoder */
    EZBus_Comm(&bus, id[i], 
	EZBus_StrComm(&bus, id[i], buffer, "z%iR", act_data[i].dr));
  }

  WriteDR();
}

//check encoders for sane values, and update if not good
void CheckEncoders()
{
  int i, do_trim = 0, trim[3];

  //check busy state of actuators, update flag
  for (i=0; i<3; i++) {
    if (EZBus_IsBusy(&bus, id[i])) {
      act_trim_wait = ACTBUS_TRIM_WAIT;
      actbus_flags |= ACT_FL_BUSY(i);
      actbus_flags |= ACT_FL_TRIM_WAIT;
    }
    else actbus_flags &= ~ACT_FL_BUSY(i);
  }

  //read encoders, and check if values need updating
  for (i = 0; i < 3; ++i) {
    ReadActuator(i);
    //for bad encoders, prepare to do a trim
    if (!encOK(act_data[i].enc)) do_trim = 1;
    //for bad DR, reload from file
    if (!encOK(act_data[i].dr)) ReadDR();
    trim[i] = act_data[i].dr;
  }

  //if busy, or waiting, do not trim
  if (actbus_flags & (ACT_FL_BUSY_MASK | ACT_FL_TRIM_WAIT)) return;
  else if (do_trim) {
    actEncTrim(trim);
    act_trim_flag_wait = act_trim_wait = ACTBUS_TRIM_WAIT;
    actbus_flags |= ACT_FL_TRIMMED | ACT_FL_TRIM_WAIT;
  }
}

//before moving, update dead reckoning to new goal
static void UpdateDR(int* goal)
{
  int i;
  for (i=0; i<3; i++) act_data[i].dr = goal[i];
  WriteDR();
}

//NB has less checks than servoing used to, but I don't see how they were useful
static void ServoActuators(int* goal)
{
  int i;
  char buf[EZ_BUS_BUF_LEN];

  if (CheckMove(goal[0], goal[1], goal[2]))
    return;

  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return;

  EZBus_Take(&bus, ID_ALL_ACT);

  UpdateDR(goal);

  //stop any current action
  EZBus_Stop(&bus, ID_ALL_ACT); /* terminate all strings */

  for (i = 0; i < 3; ++i) {
    //send command to each actuator, but don't run yet
    EZBus_Comm(&bus, id[i], EZBus_StrComm(&bus, id[i], buf, "A%d", goal[i]));
  }
  EZBus_Comm(&bus, ID_ALL_ACT, "R");	  //run all act commands at once

  EZBus_Release(&bus, ID_ALL_ACT);
}

static void DeltaActuators(void)
{
  int i, goal[3];

  for (i = 0; i < 3; ++i)
    goal[i] = CommandData.actbus.delta[i] + act_data[i].enc;

  ServoActuators(goal);
}

static int ThermalCompensation(void)
{
  /* if panicking, continue to do so */
  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return ACTBUS_FM_PANIC;

  /* Do nothing if vetoed or autovetoed */
  if (CommandData.actbus.tc_mode != TC_MODE_ENABLED)
    return ACTBUS_FM_SLEEP;

  /* Do nothing if we haven't timed out */
  if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait)
    return ACTBUS_FM_SLEEP;

  /* Do nothing if the offset is below the threshold */
  if (correction < CommandData.actbus.tc_step
      && -correction < CommandData.actbus.tc_step)
    return ACTBUS_FM_SLEEP;

  /* Do something! */
  CommandData.actbus.focus = focus - correction;
  CommandData.actbus.sf_time = 0;

  return ACTBUS_FM_THERMO;
}

static void DoActuators(void)
{
  int delta;

  EZBus_SetVel(&bus, ID_ALL_ACT, CommandData.actbus.act_vel);
  EZBus_SetAccel(&bus, ID_ALL_ACT, CommandData.actbus.act_acc);
  EZBus_SetIMove(&bus, ID_ALL_ACT, CommandData.actbus.act_move_i);
  EZBus_SetIHold(&bus, ID_ALL_ACT, CommandData.actbus.act_hold_i);
  EZBus_SetPreamble(&bus, ID_ALL_ACT, actPreamble(CommandData.actbus.act_tol));

  switch(CommandData.actbus.focus_mode) {
    case ACTBUS_FM_PANIC:
      bputs(warning, "Actuator Panic");
      EZBus_Stop(&bus, ID_ALL_ACT); /* terminate all strings */
      EZBus_Comm(&bus, ID_ALL_ACT, "n0R");	/* also stop fine correction */
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
      break;
    case ACTBUS_FM_DELTA:
      DeltaActuators();
      break;
    case ACTBUS_FM_DELFOC:
      CommandData.actbus.focus += focus;
      /* Fallthough */
    case ACTBUS_FM_THERMO:
    case ACTBUS_FM_FOCUS:
      bprintf(info, "changing focus %g to %d", focus, CommandData.actbus.focus);
      delta = CommandData.actbus.focus - focus;
      CommandData.actbus.goal[0] = act_data[0].enc + delta;
      CommandData.actbus.goal[1] = act_data[1].enc + delta;
      CommandData.actbus.goal[2] = act_data[2].enc + delta;
      /* fallthrough */
    case ACTBUS_FM_SERVO:
	ServoActuators(CommandData.actbus.goal);
	break;
    case ACTBUS_FM_TRIM:
	actEncTrim(CommandData.actbus.trim);
	break;
    case ACTBUS_FM_SLEEP:
	break;
    default:
	bprintf(err, "Unknown Focus Mode (%i), sleeping", 
	    CommandData.actbus.focus_mode);
	CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
  }

  CommandData.actbus.focus_mode = ThermalCompensation();

  CheckEncoders();

  focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc)/3.0;
}

//adjust focus offset so that new gains don't change focus thermal correction
void RecalcOffset(double new_gp, double new_gs)
{
  if (t_primary < 0 || t_secondary < 0)
    return;

  CommandData.actbus.sf_offset += ( (new_gp - CommandData.actbus.g_primary) *
    (t_primary - T_PRIMARY_FOCUS) - (new_gs - CommandData.actbus.g_secondary) *
    (t_secondary - T_SECONDARY_FOCUS) ) / ACTENC_TO_UM;
}


static int InitialiseActuator(struct ezbus* thebus, char who)
{
  char buffer[EZ_BUS_BUF_LEN];
  int i;

  for (i=0; i<3; i++) {
    if (id[i] == who) {	  //only operate on actautors
      //bprintf(info, "Initialising %s...", name[i]);
      ReadDR();	  //inefficient, 

      /* Set the encoder */
      if ( EZBus_Comm(thebus, who, 
	    EZBus_StrComm(thebus, who, buffer, "z%iR", act_data[i].dr))
	  == EZ_ERR_OK ) return 1;
      else {
	bprintf(warning, "Initialising %s failed...", name[i]);
	return 0;
      }
    }
  }
  return 1;
}


/******************************************************************************/
/*                                                                            */
/*    Do shutter logic: check status, determine if shutter is in or out       */
/*    or in between                                                           */
/*                                                                            */
/******************************************************************************/

static void TurnOffShutter()
{
  // Set hold current to zero
  bputs(info, "TurnOffShutter...");
  if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0R") != EZ_ERR_OK)
    bputs(warning, "TurnOffShutter: Error turning off shutter");
}


static void InitializeShutter()
{
  // Set move current and speed
  bputs(info, "InitializeShutter:...");
  if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64m100l100h50R") != EZ_ERR_OK)
  //if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64m100l100v10h50R") != EZ_ERR_OK)
    bputs(info, "InitializeShutter: Error initializing shutter");
  CommandData.actbus.shutter_step = 4224;
  CommandData.actbus.shutter_step_slow = 300;
  //CommandData.actbus.shutter_move_i = 100;
  //CommandData.actbus.shutter_move_i = 50;
  //CommandData.actbus.shutter_vel = 20;
  //CommandData.actbus.shutter_acc = 1;
  // Set microstepping on j64N1
  // Wait 2 seconds M2000
  // Set hold current to 50 h50
  // Set position to 5000 z5000
  // Move to activate limit switch D424
  // Set position to 0 z0
  //EZBus_Comm(&bus, id[SHUTTERNUM], "j64h50R");
  //EZBus_SetIMove(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_move_i);
  //EZBus_SetIHold(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_hold_i);
  //EZBus_SetVel(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_vel);
  //EZBus_SetAccel(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_acc);
}


// If shutter failed to close completely, this causes shutter to
// open by setting hold current to 0 and then moving to limit switch.

static void ResetShutter()
{
  //if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0M2000h50z5000D424z0R") != EZ_ERR_OK)
  if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0M2000h50z5000P424z0R") != EZ_ERR_OK)
    bputs(info, "ResetShutter: Error resetting shutter");
}


static void OpenCloseShutter()
{

  char    cmd[80];

  bputs(info, "OpenCloseShutter...");

  if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.in) != EZ_ERR_OK)
    bputs(info, "OpenCloseShutter: Error polling opto switch");
  usleep(SHUTTER_SLEEP);
  EZBus_Stop(&bus, id[SHUTTERNUM]);

  if ((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
    bputs(info, "OpenCloseShutter: doing action");
    //if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0z5000h50V10000D424P4224R") != EZ_ERR_OK)
    sprintf(cmd, "h0z5000h50V10000P424D%dR", CommandData.actbus.shutter_step);
    if (EZBus_Comm(&bus, id[SHUTTERNUM], cmd) != EZ_ERR_OK) {
       bputs(warning, "OpenCloseShutter: EZ Bus error");
       usleep(20*SHUTTER_SLEEP);
    }
  }
  else
    bputs(info, "OpenCloseShutter: Shutter is already closed");

}


#define SHUTTER_CLOSE_TIMEOUT 20000000


static void CloseShutter()
{
  char        cmd[80];

  int  shutter_timeout = 0;

  if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.in) != EZ_ERR_OK)
    bputs(info, "CloseShutter: 1. Error polling opto switch");
  else
    ;

  // This code does new style closing of the shutter:
  // If the shutter is not closed, then turn of the shutter (it will fall
  // open), drive against limit switch then close quickly.
  if ((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
    bputs(info, "CloseShutter: closing shutter...");
    usleep(SHUTTER_SLEEP);
    sprintf(cmd, "z0V10000h0M2000h50P424z5000D%dR", CommandData.actbus.shutter_step);
    if (EZBus_Comm(&bus, id[SHUTTERNUM], cmd) != EZ_ERR_OK)
      bputs(warning, "CloseShutter: EZ Bus error");
    usleep(SHUTTER_SLEEP);
    bputs(info, "start wait..");
    usleep(5000000);   // Wait 5 seconds
    EZBus_Stop(&bus, id[SHUTTERNUM]);
    bputs(info, "end wait");
  }
  else {  // Shutter is closed according to opto switch
    shutter_data.state = SHUTTER_CLOSED;
    //bputs(info, "CloseShutter: shutter is closed");
  }

  if (shutter_timeout >= SHUTTER_CLOSE_TIMEOUT)
    bputs(warning, "CloseShutter: Closing shutter timed out");

}


#define SHUTTER_CLOSE_SLOW_TIMEOUT 50000000

static void CloseSlowShutter()
{
  int  shutter_timeout = 0;

  char  cmd[80];

  if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.in) != EZ_ERR_OK)
    bputs(info, "CloseShutter: 1. Error polling opto switch");
  else
    ;

  //bprintf(info, "%d %d %d", shutter_data.in, shutter_data.in & SHUTTER_CLOSED_BIT,
  //        SHUTTER_CLOSED_BIT);
  
  // This code does the old style clsing of the shutter:
  // Close shutter a little, check the opto, close the shutter a little,
  // check the opto... until shutter is closed
  if ((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {

    bputs(info, "CloseSlowShutter: Closing shutter...");

    while (((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) &
            (shutter_timeout < SHUTTER_CLOSE_TIMEOUT)) {
      if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
        if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.in) != EZ_ERR_OK)
          bputs(warning, "CloseShutter: 2. Error polling opto switch");
        usleep(SHUTTER_SLEEP);
        if ((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
          //if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64z0h50V1000P300R") != EZ_ERR_OK)
          sprintf(cmd, "j64z5000h50V1000D%dR", CommandData.actbus.shutter_step_slow);
          if (EZBus_Comm(&bus, id[SHUTTERNUM], cmd) != EZ_ERR_OK)
            bputs(warning, "CloseShutter: EZ Bus error");
        }
      }
      shutter_timeout += SHUTTER_SLEEP;
    }
  }
  else {  // Shutter is closed according to opto switch
    shutter_data.state = SHUTTER_CLOSED;
    //bputs(info, "CloseShutter: shutter is closed");
  }

  if (shutter_timeout >= SHUTTER_CLOSE_SLOW_TIMEOUT)
    bputs(warning, "CloseSlowShutter: Closing shutter timed out");

}


static void OpenShutter()
{
  char  cmd[80];
  //EZBus_Comm(&bus, id[SHUTTERNUM], "z5000V10000D4224R");
  sprintf(cmd, "z0V10000P%dR", CommandData.actbus.shutter_step);
  EZBus_Comm(&bus, id[SHUTTERNUM], cmd);
}


static void GetShutterData(int *position)
{

  *position = SHUTTER_IS_UNK;

  // This position is only where the step controller thinks the shutter
  // is.  There is no direct feedback from the shutter other than the
  // limit switch.
  if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
    if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.in) != EZ_ERR_OK)
      bputs(warning, "GetShutterData: EZBus is busy.");
    else {
      if ((shutter_data.in & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT)
        *position = SHUTTER_IS_CLOSED;
    }
  }
  
  // NEED TO CHECK SIGN OF THIS!!
  if (CommandData.actbus.shutter_out < 15000)
    *position = SHUTTER_IS_OPEN;

}


#define SHUTTER_EXIT       0
#define SHUTTER_DO_OFF     1
#define SHUTTER_DO_CLOSE   2
#define SHUTTER_DO_CLOSE_SLOW   3
#define SHUTTER_DO_OPEN    4
#define SHUTTER_DO_OPEN_CLOSE 6
#define SHUTTER_DO_RESET   7
#define SHUTTER_DO_INIT    8
#define SHUTTER_DO_NOP     9

static void DoShutter(void)
{
  int action = SHUTTER_EXIT;
  static int  shutter_pos;

  if (shutter_data.state == SHUTTER_UNK) {
    //bputs(info, "Initializing shutter...");
    EZBus_Take(&bus, id[SHUTTERNUM]);
    EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
    InitializeShutter();
    EZBus_Release(&bus, id[SHUTTERNUM]);
    shutter_data.state = SHUTTER_OPEN;
    CommandData.actbus.shutter_goal = SHUTTER_NOP;
  }

  GetShutterData(&shutter_pos);
  shutter_data.pos = shutter_pos;

  switch (CommandData.actbus.shutter_goal) {
    case  SHUTTER_OPEN:
      action = SHUTTER_DO_OPEN;
      shutter_data.state = SHUTTER_OPEN;
      CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break;
    case  SHUTTER_CLOSED:
      action = SHUTTER_DO_CLOSE;
      //shutter_data.state = SHUTTER_CLOSED;
      //CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break;
    case  SHUTTER_CLOSED_SLOW:
      action = SHUTTER_DO_CLOSE_SLOW;
      break;
    case  SHUTTER_CLOSED2:
      action = SHUTTER_DO_OPEN_CLOSE;
      //CommandData.actbus.shutter_goal = SHUTTER_CLOSED;
      CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break; 
    case  SHUTTER_INIT:
      action = SHUTTER_DO_INIT;
      shutter_data.state = SHUTTER_OPEN;
      CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break;
    case  SHUTTER_RESET:
      action = SHUTTER_DO_RESET;
      shutter_data.state = SHUTTER_OPEN;
      CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break;
    case  SHUTTER_OFF:
      action = SHUTTER_DO_OFF;
      shutter_data.state = SHUTTER_OPEN;
      CommandData.actbus.shutter_goal = SHUTTER_NOP;
      break;
      //case  SHUTTER_NOP:
      //action = SHUTTER_DO_NOP;
      //break;
  }
    
  /* Figure out what to do... */
  switch (action) {
    case SHUTTER_DO_OFF:
      bputs(warning, "Turning off shutter.  Shutter will open.");
      EZBus_Take(&bus, id[SHUTTERNUM]);
      EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
      TurnOffShutter();
      EZBus_Release(&bus, id[SHUTTERNUM]);
      break;
    case SHUTTER_DO_CLOSE:
      //shutter_timeout = DRIVE_TIMEOUT;
      //bputs(warning, "1. Closing shutter.");
      if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
        EZBus_Take(&bus, id[SHUTTERNUM]);
        //EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
        CloseShutter();
        EZBus_Release(&bus, id[SHUTTERNUM]);
      }
      break;
    case SHUTTER_DO_CLOSE_SLOW:
      if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
          EZBus_Take(&bus, id[SHUTTERNUM]);
          //EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
          CloseSlowShutter();
          EZBus_Release(&bus, id[SHUTTERNUM]);
      }
      break;
    case SHUTTER_DO_OPEN_CLOSE:
      //if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
        EZBus_Take(&bus, id[SHUTTERNUM]);
        //EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
        OpenCloseShutter();
        EZBus_Release(&bus, id[SHUTTERNUM]);
	//}
	//else
        //bputs(warning, "EZBus busy --- not calling OpenCloseShutter");
      break;
    case SHUTTER_DO_OPEN:
      //shutter_timeout = DRIVE_TIMEOUT;
      bputs(warning, "Opening shutter.");
      EZBus_Take(&bus, id[SHUTTERNUM]);
      EZBus_Stop(&bus, id[SHUTTERNUM]); /* stop current action first */
      OpenShutter();
      EZBus_Release(&bus, id[SHUTTERNUM]);
      break;
    case SHUTTER_DO_INIT:
      //shutter_timeout = DRIVE_TIMEOUT;
      bputs(warning, "Intializing shutter.  Shutter will open.");
      EZBus_Take(&bus, id[SHUTTERNUM]);
      EZBus_Stop(&bus, id[SHUTTERNUM]);  /* stop current action first */
      InitializeShutter();
      EZBus_Release(&bus, id[SHUTTERNUM]);
      break;
    case SHUTTER_DO_RESET:
      //shutter_timeout = DRIVE_TIMEOUT;
      bputs(warning, "Resetting shutter.  Shutter will open.");
      EZBus_Take(&bus, id[SHUTTERNUM]);
      EZBus_Stop(&bus, id[SHUTTERNUM]);  /* stop current action first */
      ResetShutter();
      EZBus_Release(&bus, id[SHUTTERNUM]);
      break;
    case SHUTTER_DO_NOP:
      break;
  }

  action = SHUTTER_EXIT;

}


/************************************************************************/
/*                                                                      */
/*    Do Lock Logic: check status, determine if we are locked, etc      */
/*                                                                      */
/************************************************************************/
static void GetLockData()
{
  static int counter = 0;
  //when lock motor not active, take data more slowly
  if (EZBus_IsTaken(&bus, id[LOCKNUM]) != EZ_ERR_OK
      && counter++ < LOCK_MOTOR_DATA_TIMER)
    return;
  counter = 0;

  EZBus_ReadInt(&bus, id[LOCKNUM], "?0", &lock_data.pos);
  EZBus_Comm(&bus, id[LOCKNUM], "?aa");
  sscanf(bus.buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1],
      &lock_data.adc[2], &lock_data.adc[3]);
}

/* The NiC MCC does this via the BlastBus to give it a chance to know what's
 * going on.  The ICC reads it directly to get more promptly the answer
 * (since all these fields are slow). */
static void SetLockState(int nic)
{
  static int firsttime = 1;
  int pot;
  unsigned int state;

  static struct BiPhaseStruct* potLockAddr;
  static struct BiPhaseStruct* stateLockAddr;

  if (firsttime) {
    firsttime = 0;
    potLockAddr = GetBiPhaseAddr("pot_lock");
    stateLockAddr = GetBiPhaseAddr("state_lock");
  }

  //get lock data
  if (nic) {
    //use bbus when nic
    pot = slow_data[potLockAddr->index][potLockAddr->channel];
    state = slow_data[stateLockAddr->index][stateLockAddr->channel];
    lock_data.adc[1] = pot;
  } else {
    //otherwise (in charge) use lock_data
    pot = lock_data.adc[1];
    state = lock_data.state;
  }

  //update the NIC on pot state

  //set the EZBus move parameters
  EZBus_SetVel(&bus, id[LOCKNUM], CommandData.actbus.lock_vel);
  EZBus_SetAccel(&bus, id[LOCKNUM], CommandData.actbus.lock_acc);
  EZBus_SetIMove(&bus, id[LOCKNUM], CommandData.actbus.lock_move_i);
  EZBus_SetIHold(&bus, id[LOCKNUM], CommandData.actbus.lock_hold_i);

  state &= LS_DRIVE_MASK; /* zero everything but drive info */

  if (pot <= LOCK_MIN_POT)
    state |= LS_CLOSED;
  else if (pot >= LOCK_MAX_POT) {
    state |= LS_OPEN;
  } else if ((pot < LOCK_MIN_POT + LOCK_POT_RANGE)
      || (pot > LOCK_MAX_POT - LOCK_POT_RANGE))
    state |= lock_data.state & (LS_OPEN | LS_CLOSED);

  if (fabs(ACSData.enc_raw_el - LockPosition(CommandData.pointing_mode.Y)) <= 0.5)
    state |= LS_EL_OK;

  /* Assume the pin is out unless we're all the way closed */
  if (state & LS_CLOSED)
    CommandData.pin_is_in = 1;
  else
    CommandData.pin_is_in = 0;

  lock_data.state = state;
}

#define SEND_SLEEP 100000 /* 100 milliseconds */
#define WAIT_SLEEP 50000 /* 50 milliseconds */
#define LA_EXIT    0
#define LA_STOP    1
#define LA_WAIT    2
#define LA_EXTEND  3
#define LA_RETRACT 4
static void DoLock(void)
{
  int action = LA_EXIT;

  do {
    GetLockData();

    /* Fix weird states */
    if ((lock_data.state & (LS_DRIVE_EXT | LS_DRIVE_RET | LS_DRIVE_UNK)
          && lock_data.state & LS_DRIVE_OFF)
        || CommandData.actbus.lock_goal & LS_DRIVE_FORCE) {
      lock_data.state &= ~LS_DRIVE_MASK | LS_DRIVE_UNK;
      CommandData.actbus.lock_goal &= ~LS_DRIVE_FORCE;
      bprintf(warning, "Reset lock motor state.");
    }

    SetLockState(0);

    /* compare goal to current state -- only 3 goals are supported:
     * open + off, closed + off and off */
    if ((CommandData.actbus.lock_goal & 0x7) == (LS_OPEN | LS_DRIVE_OFF)) {
      /*                                       ORe -.
       * cUe -+-(stp)- cFe -(ext)- cXe -(---)- OXe -+-(stp)- OFe ->
       * cRe -'                                OUe -'
       */
      if ((lock_data.state & (LS_OPEN | LS_DRIVE_OFF))
          == (LS_OPEN | LS_DRIVE_OFF))
        action = LA_EXIT;
      else if (lock_data.state & LS_OPEN)
        action = LA_STOP;
      else if (lock_data.state & (LS_DRIVE_RET))
        action = LA_WAIT;
      else if (lock_data.state & LS_DRIVE_OFF)
        action = LA_RETRACT;
      else
        action = LA_STOP;
    } else if ((CommandData.actbus.lock_goal & 0x7) == (LS_CLOSED
          | LS_DRIVE_OFF)) {
      /* oX -.         oUE -(stp)-.              CRe -(stp)-+
       * oR -+-(stp) - oF  -(---)-+- oFE -(ret)- oRE -(---)-+- CFe ->
       * oU -'         oXE -(stp)-'              CUe -(stp)-+
       *                                         CXe -(stp)-'
       */
      if ((lock_data.state & (LS_CLOSED | LS_DRIVE_OFF))
          == (LS_CLOSED | LS_DRIVE_OFF)) 
        action = LA_EXIT;
      else if (lock_data.state & LS_CLOSED)
        action = LA_STOP;
      else if (lock_data.state & LS_EL_OK
          || CommandData.actbus.lock_goal & LS_IGNORE_EL) { /* el in range */
        if ((lock_data.state & (LS_OPEN | LS_DRIVE_STP))
            == (LS_OPEN | LS_DRIVE_STP))
          action = LA_WAIT;
        else if (lock_data.state & LS_DRIVE_EXT)
          action = LA_WAIT;
        else if (lock_data.state & LS_DRIVE_STP)
          action = LA_STOP;
        else if (lock_data.state & LS_DRIVE_OFF)
          action = LA_EXTEND;
        else
          action = LA_STOP;
      } else { /* el out of range */
        action = (lock_data.state & LS_DRIVE_OFF) ? LA_WAIT : LA_STOP;
      }
    } else if ((CommandData.actbus.lock_goal & 0x7) == LS_DRIVE_OFF) {
      /* ocXe -.
       * ocRe -+-(stp)- ocFe ->
       * ocUe -+
       * ocSe -'
       */
      action = (lock_data.state & LS_DRIVE_OFF) ? LA_EXIT : LA_STOP;
    } else {
      bprintf(warning, "Unhandled lock goal (%x) ignored.",
          CommandData.actbus.lock_goal);
      CommandData.actbus.lock_goal = LS_DRIVE_OFF;
    }

    /* Timeout check */
    if (lock_timeout == 0) {
      bputs(warning, "Lock Motor drive timeout.");
      action = LA_STOP;
    }
    /* Seize the bus */
    if (action == LA_EXIT)
      EZBus_Release(&bus, id[LOCKNUM]);
    else
      EZBus_Take(&bus, id[LOCKNUM]);

    /* Figure out what to do... */
    switch (action) {
      case LA_STOP:
        lock_timeout = -1;
        bputs(info, "Stopping lock motor.");
        EZBus_Stop(&bus, id[LOCKNUM]); /* terminate all strings */
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_OFF;
        break;
      case LA_EXTEND:
        lock_timeout = DRIVE_TIMEOUT;
        bputs(info, "Extending lock motor.");
        EZBus_Stop(&bus, id[LOCKNUM]); /* stop current action first */
	EZBus_RelMove(&bus, id[LOCKNUM], INT_MAX);
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        lock_timeout = DRIVE_TIMEOUT;
        bputs(info, "Retracting lock motor.");
        EZBus_Stop(&bus, id[LOCKNUM]); /* stop current action first */
	EZBus_RelMove(&bus, id[LOCKNUM], INT_MIN);
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      case LA_WAIT:
	usleep(WAIT_SLEEP); /* wait for a bit */
	break;
    }

    //quit if timeout
    if (lock_timeout == 0) {
      lock_timeout = -1;
      action = LA_EXIT;
    }

  } while (action != LA_EXIT);
}

/************************************************************************/
/*                                                                      */
/*    Frame Logic: Write data to the frame, called from main thread     */
/*                                                                      */
/************************************************************************/
#define N_FILT_TEMP 2	    //number of temperatures to filter
#define TEMP_FILT_LEN 300   //60s @ 5Hz
static double filterTemp(int num, double data)
{
  static double temp_buf[N_FILT_TEMP][TEMP_FILT_LEN] = {}; //init to 0
  static double temp_sum[N_FILT_TEMP] = {};
  static int ibuf[N_FILT_TEMP] = {};

  temp_sum[num] += (data - temp_buf[num][ibuf[num]]);
  temp_buf[num][ibuf[num]] = data;
  ibuf[num] = (ibuf[num] + 1) % TEMP_FILT_LEN;
  return temp_sum[num]/TEMP_FILT_LEN;
}

/* decide on primary and secondary temperature, write focus-related fields */
void SecondaryMirror(void)
{
  static int firsttime = 1;

  static struct NiosStruct* correctionSfAddr;
  static struct NiosStruct* ageSfAddr;
  static struct NiosStruct* offsetSfAddr;
  static struct NiosStruct* tPrimeSfAddr;
  static struct NiosStruct* tSecondSfAddr;

  static struct BiPhaseStruct* t1PrimeAddr;
  static struct BiPhaseStruct* t1SecondAddr;
  static struct BiPhaseStruct* t2PrimeAddr;
  static struct BiPhaseStruct* t2SecondAddr;
  double t_primary1, t_secondary1;
  double t_primary2, t_secondary2;

  double correction_temp = 0;
  if (firsttime) {
    firsttime = 0;
    t1PrimeAddr = GetBiPhaseAddr("vt_1_prime");
    t1SecondAddr = GetBiPhaseAddr("t_1_second");
    t2PrimeAddr = GetBiPhaseAddr("vt_2_prime");
    t2SecondAddr = GetBiPhaseAddr("t_2_second");
    correctionSfAddr = GetNiosAddr("correction_sf");
    ageSfAddr = GetNiosAddr("age_sf");
    offsetSfAddr = GetNiosAddr("offset_sf");
    tPrimeSfAddr = GetNiosAddr("t_prime_sf");
    tSecondSfAddr = GetNiosAddr("t_second_sf");
  }

  t_primary1 = CalibrateThermister(
      slow_data[t1PrimeAddr->index][t1PrimeAddr->channel]);
  t_primary2 = CalibrateThermister(
      slow_data[t2PrimeAddr->index][t2PrimeAddr->channel]);

  t_secondary1 = CalibrateAD590(
      slow_data[t1SecondAddr->index][t1SecondAddr->channel]);
  t_secondary2 = CalibrateAD590(
      slow_data[t2SecondAddr->index][t2SecondAddr->channel]);

  if (t_primary1 < 0 || t_primary2 < 0)
    t_primary = -1; /* autoveto */
  else if (fabs(t_primary1 - t_primary2) < CommandData.actbus.tc_spread) {
    if (t_primary1 >= 0 && t_primary2 >= 0)
      t_primary = filterTemp(0, (t_primary1 + t_primary2)/2);
    else if (t_primary1 >= 0)
      t_primary = filterTemp(0, t_primary1);
    else
      t_primary = filterTemp(0, t_primary2);
  } else {
    if (t_primary1 >= 0 && CommandData.actbus.tc_prefp == 1)
      t_primary = filterTemp(0, t_primary1);
    else if (t_primary2 >= 0 && CommandData.actbus.tc_prefp == 2)
      t_primary = filterTemp(0, t_primary2);
    else
      t_primary = -1; /* autoveto */
  }

  if (t_secondary1 < 0 || t_secondary2 < 0)
    t_secondary = -1; /* autoveto */
  else if (fabs(t_secondary1 - t_secondary2) < CommandData.actbus.tc_spread) {
    if (t_secondary1 >= 0 && t_secondary2 >= 0)
      t_secondary = filterTemp(1, (t_secondary1 + t_secondary2)/2);
    else if (t_secondary1 >= 0)
      t_secondary = filterTemp(1, t_secondary1);
    else
      t_secondary = filterTemp(1, t_secondary2);
  } else {
    if (t_secondary1 >= 0 && CommandData.actbus.tc_prefs == 1)
      t_secondary = filterTemp(1, t_secondary1);
    else if (t_secondary2 >= 0 && CommandData.actbus.tc_prefs == 2)
      t_secondary = filterTemp(1, t_secondary2);
    else
      t_secondary = -1; /* autoveto */
  }

  if (CommandData.actbus.tc_mode != TC_MODE_VETOED &&
      (t_primary < 0 || t_secondary < 0)) {
    if (CommandData.actbus.tc_mode == TC_MODE_ENABLED)
      bputs(info, "Thermal Compensation: Autoveto raised.");
    CommandData.actbus.tc_mode = TC_MODE_AUTOVETO;
  } else if (CommandData.actbus.tc_mode == TC_MODE_AUTOVETO) {
    bputs(info, "Thermal Compensation: Autoveto lowered.");
    CommandData.actbus.tc_mode = TC_MODE_ENABLED;
  }

  correction_temp = CommandData.actbus.g_primary * (t_primary - T_PRIMARY_FOCUS)
    - CommandData.actbus.g_secondary * (t_secondary - T_SECONDARY_FOCUS);

  /* convert to counts */
  correction_temp /= ACTENC_TO_UM;

  /* re-adjust */
  correction_temp += focus - POSITION_FOCUS - CommandData.actbus.sf_offset;

  correction = correction_temp;  //slightly more thread safe

  if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait)
    CommandData.actbus.sf_time++;

  WriteData(tPrimeSfAddr, t_primary/M_16_AD590 + B_16_AD590, NIOS_QUEUE);
  WriteData(tSecondSfAddr, t_secondary/M_16_AD590 + B_16_AD590, NIOS_QUEUE);
  WriteData(correctionSfAddr, correction, NIOS_QUEUE);
  WriteData(ageSfAddr, CommandData.actbus.sf_time / 10., NIOS_QUEUE);
  WriteData(offsetSfAddr, CommandData.actbus.sf_offset, NIOS_FLUSH);
}

static char name_buffer[100];
static inline struct NiosStruct* GetActNiosAddr(int i, const char* field)
{
  sprintf(name_buffer, "%s_%i_act", field, i);

  return GetNiosAddr(name_buffer);
}

static int filterLVDT(int num, int data)
{
  static int lvdt_buf[3][LVDT_FILT_LEN] = {}; //init to 0
  static int lvdt_sum[3] = {0, 0, 0};
  static int ibuf = 0;

  lvdt_sum[num] += (data - lvdt_buf[num][ibuf]);
  lvdt_buf[num][ibuf] = data;
  ibuf = (ibuf + 1) % LVDT_FILT_LEN;
  return (int)((double)lvdt_sum[num]/LVDT_FILT_LEN + 0.5);
}

//handle counters in a well-timed frame synchronous manner
void UpdateActFlags()
{
  //count down timeout on ACT_FL_TRIMMED indicator flag
  if (act_trim_flag_wait > 0) {
    act_trim_flag_wait--;
    actbus_flags |= ACT_FL_TRIMMED;
  } else {
    actbus_flags &= ~ACT_FL_TRIMMED;
  }

  //Check if waiting before trimming again
  if (act_trim_wait > 0) {
    act_trim_wait--;
    actbus_flags |= ACT_FL_TRIM_WAIT;
  }
  else actbus_flags &= ~ACT_FL_TRIM_WAIT;

  if (poll_timeout > 0) poll_timeout--;

  if (lock_timeout > 0) lock_timeout--;

}

void StoreActBus(void)
{
  int j;
  static int firsttime = 1;
  int lvdt_filt[3];

  static struct BiPhaseStruct* lvdt63ActAddr;
  static struct BiPhaseStruct* lvdt64ActAddr;
  static struct BiPhaseStruct* lvdt65ActAddr;

  static struct NiosStruct* busResetActAddr;
  static struct NiosStruct* posLockAddr;
  static struct NiosStruct* stateLockAddr;
  static struct NiosStruct* goalLockAddr;
  static struct NiosStruct* seizedActAddr;
  static struct NiosStruct* potLockAddr;
  static struct NiosStruct* pinInLockAddr;

  static struct NiosStruct* velLockAddr;
  static struct NiosStruct* accLockAddr;
  static struct NiosStruct* iMoveLockAddr;
  static struct NiosStruct* iLockHoldAddr;

  static struct NiosStruct* posShutterAddr;
  static struct NiosStruct* outShutterAddr;
  static struct NiosStruct* stepShutterAddr;
  static struct NiosStruct* stepSlowShutterAddr;

  static struct NiosStruct* velActAddr;
  static struct NiosStruct* accActAddr;
  static struct NiosStruct* iMoveActAddr;
  static struct NiosStruct* iHoldActAddr;
  static struct NiosStruct* tolActAddr;
  static struct NiosStruct* flagsActAddr;
  static struct NiosStruct* modeActAddr;

  static struct NiosStruct* posActAddr[3];
  static struct NiosStruct* encActAddr[3];
  static struct NiosStruct* lvdtActAddr[3];
  static struct NiosStruct* offsetActAddr[3];
  static struct NiosStruct* goalActAddr[3];
  static struct NiosStruct* drActAddr[3];

  static struct NiosStruct* lvdtSpreadActAddr;
  static struct NiosStruct* lvdtLowActAddr;
  static struct NiosStruct* lvdtHighActAddr;

  static struct NiosStruct* gPrimeSfAddr;
  static struct NiosStruct* gSecondSfAddr;
  static struct NiosStruct* stepSfAddr;
  static struct NiosStruct* waitSfAddr;
  static struct NiosStruct* modeSfAddr;
  static struct NiosStruct* spreadSfAddr;
  static struct NiosStruct* prefTpSfAddr;
  static struct NiosStruct* prefTsSfAddr;
  static struct NiosStruct* goalSfAddr;
  static struct NiosStruct* focusSfAddr;

  static struct NiosStruct* statusActbusAddr;

  if (firsttime) {
    firsttime = 0;

    lvdt63ActAddr = GetBiPhaseAddr("lvdt_63_act");
    lvdt64ActAddr = GetBiPhaseAddr("lvdt_64_act");
    lvdt65ActAddr = GetBiPhaseAddr("lvdt_65_act");

    busResetActAddr = GetNiosAddr("bus_reset_act");
    pinInLockAddr = GetNiosAddr("pin_in_lock");
    posLockAddr = GetNiosAddr("pos_lock");
    stateLockAddr = GetNiosAddr("state_lock");
    seizedActAddr = GetNiosAddr("seized_act");
    goalLockAddr = GetNiosAddr("goal_lock");
    potLockAddr = GetNiosAddr("pot_lock");

    for (j = 0; j < 3; ++j) {
      posActAddr[j] = GetActNiosAddr(j, "pos");
      encActAddr[j] = GetActNiosAddr(j, "enc");
      lvdtActAddr[j] = GetActNiosAddr(j, "lvdt");
      offsetActAddr[j] = GetActNiosAddr(j, "offset");
      goalActAddr[j] = GetActNiosAddr(j, "goal");
      drActAddr[j] = GetActNiosAddr(j, "dr");
    }

    gPrimeSfAddr = GetNiosAddr("g_prime_sf");
    gSecondSfAddr = GetNiosAddr("g_second_sf");
    stepSfAddr = GetNiosAddr("step_sf");
    waitSfAddr = GetNiosAddr("wait_sf");
    modeSfAddr = GetNiosAddr("mode_sf");
    spreadSfAddr = GetNiosAddr("spread_sf");
    prefTpSfAddr = GetNiosAddr("pref_tp_sf");
    prefTsSfAddr = GetNiosAddr("pref_ts_sf");
    goalSfAddr = GetNiosAddr("goal_sf");
    focusSfAddr = GetNiosAddr("focus_sf");

    lvdtSpreadActAddr = GetNiosAddr("lvdt_spread_act");
    lvdtLowActAddr = GetNiosAddr("lvdt_low_act");
    lvdtHighActAddr = GetNiosAddr("lvdt_high_act");

    velActAddr = GetNiosAddr("vel_act");
    accActAddr = GetNiosAddr("acc_act");
    iMoveActAddr = GetNiosAddr("i_move_act");
    iHoldActAddr = GetNiosAddr("i_hold_act");
    tolActAddr = GetNiosAddr("tol_act");
    flagsActAddr = GetNiosAddr("flags_act");
    modeActAddr = GetNiosAddr("mode_act");

    velLockAddr = GetNiosAddr("vel_lock");
    accLockAddr = GetNiosAddr("acc_lock");
    iMoveLockAddr = GetNiosAddr("i_move_lock");
    iLockHoldAddr = GetNiosAddr("i_hold_lock");

    posShutterAddr = GetNiosAddr("pos_shutter");
    outShutterAddr = GetNiosAddr("out_shutter");
    stepShutterAddr = GetNiosAddr("steps_shutter");
    stepSlowShutterAddr = GetNiosAddr("steps_slow_shutter");

    statusActbusAddr = GetNiosAddr("status_actbus");
  }

  UpdateActFlags();

  //filter the LVDTs, scale into encoder units, rotate to motor positions
  lvdt_filt[0] = filterLVDT(0, 
      slow_data[lvdt63ActAddr->index][lvdt63ActAddr->channel]);
  lvdt_filt[1] = filterLVDT(1, 
      slow_data[lvdt64ActAddr->index][lvdt64ActAddr->channel]);
  lvdt_filt[2] = filterLVDT(2, 
      slow_data[lvdt65ActAddr->index][lvdt65ActAddr->channel]);
  lvdt_filt[0] = (int)((double)lvdt_filt[0] * LVDT63_ADC_TO_ENC + LVDT63_ZERO);
  lvdt_filt[1] = (int)((double)lvdt_filt[1] * LVDT64_ADC_TO_ENC + LVDT64_ZERO);
  lvdt_filt[2] = (int)((double)lvdt_filt[2] * LVDT65_ADC_TO_ENC + LVDT65_ZERO);
  act_data[0].lvdt = (int)(
      (double)(-lvdt_filt[2] + 2 * lvdt_filt[0] + 2 * lvdt_filt[1]) / 3.0);
  act_data[1].lvdt = (int)(
      (double)(-lvdt_filt[0] + 2 * lvdt_filt[1] + 2 * lvdt_filt[2]) / 3.0);
  act_data[2].lvdt = (int)(
      (double)(-lvdt_filt[1] + 2 * lvdt_filt[2] + 2 * lvdt_filt[0]) / 3.0);

  if (CommandData.actbus.off) {
    if (CommandData.actbus.off > 0) CommandData.actbus.off--;
    actbus_reset = 0;   //turn actbus off
  } else actbus_reset = 1;
  WriteData(busResetActAddr, actbus_reset, NIOS_QUEUE);

  WriteData(pinInLockAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (j = 0; j < 3; ++j) {
    WriteData(posActAddr[j], 
	act_data[j].pos - CommandData.actbus.offset[j], NIOS_QUEUE);
    WriteData(encActAddr[j], 
	act_data[j].enc - CommandData.actbus.offset[j], NIOS_QUEUE);
    WriteData(lvdtActAddr[j], 
	act_data[j].lvdt - CommandData.actbus.offset[j], NIOS_QUEUE);
    WriteData(offsetActAddr[j], CommandData.actbus.offset[j], NIOS_QUEUE);
    WriteData(goalActAddr[j], 
	CommandData.actbus.goal[j] - CommandData.actbus.offset[j], NIOS_QUEUE);
    WriteData(drActAddr[j], 
	act_data[j].dr - CommandData.actbus.offset[j], NIOS_QUEUE);
  }
  WriteData(focusSfAddr, 
      (int)focus - POSITION_FOCUS - CommandData.actbus.sf_offset, NIOS_QUEUE);

  WriteData(potLockAddr, lock_data.adc[1], NIOS_QUEUE);
  WriteData(stateLockAddr, lock_data.state, NIOS_QUEUE);
  WriteData(seizedActAddr, bus.seized, NIOS_QUEUE);
  WriteData(goalLockAddr, CommandData.actbus.lock_goal, NIOS_QUEUE);
  WriteData(posLockAddr, lock_data.pos, NIOS_QUEUE);
  
  WriteData(velActAddr, CommandData.actbus.act_vel, NIOS_QUEUE);
  WriteData(accActAddr, CommandData.actbus.act_acc, NIOS_QUEUE);
  WriteData(iMoveActAddr, CommandData.actbus.act_move_i, NIOS_QUEUE);
  WriteData(iHoldActAddr, CommandData.actbus.act_hold_i, NIOS_QUEUE);
  WriteData(tolActAddr, CommandData.actbus.act_tol, NIOS_QUEUE);
  WriteData(flagsActAddr, actbus_flags, NIOS_QUEUE);
  WriteData(modeActAddr, CommandData.actbus.focus_mode, NIOS_QUEUE);

  WriteData(lvdtSpreadActAddr, CommandData.actbus.lvdt_delta, NIOS_QUEUE);
  WriteData(lvdtLowActAddr, CommandData.actbus.lvdt_low+5000, NIOS_QUEUE);
  WriteData(lvdtHighActAddr, CommandData.actbus.lvdt_high+5000, NIOS_QUEUE);

  WriteData(velLockAddr, CommandData.actbus.lock_vel / 100, NIOS_QUEUE);
  WriteData(accLockAddr, CommandData.actbus.lock_acc, NIOS_QUEUE);
  WriteData(iMoveLockAddr, CommandData.actbus.lock_move_i, NIOS_QUEUE);
  WriteData(iLockHoldAddr, CommandData.actbus.lock_hold_i, NIOS_QUEUE);

  // Shutter data
  WriteData(stepShutterAddr, CommandData.actbus.shutter_step, NIOS_QUEUE);
  WriteData(stepSlowShutterAddr, CommandData.actbus.shutter_step_slow, NIOS_QUEUE);
  WriteData(posShutterAddr, (shutter_data.in & SHUTTER_CLOSED_BIT), NIOS_QUEUE);
  WriteData(outShutterAddr, shutter_data.pos, NIOS_QUEUE);

  WriteData(gPrimeSfAddr, CommandData.actbus.g_primary * 100., NIOS_QUEUE);
  WriteData(gSecondSfAddr, CommandData.actbus.g_secondary*100., NIOS_QUEUE);
  WriteData(modeSfAddr, CommandData.actbus.tc_mode, NIOS_QUEUE);
  WriteData(stepSfAddr, CommandData.actbus.tc_step, NIOS_QUEUE);
  WriteData(spreadSfAddr, CommandData.actbus.tc_spread * 500., NIOS_QUEUE);
  WriteData(prefTpSfAddr, CommandData.actbus.tc_prefp, NIOS_QUEUE);
  WriteData(prefTsSfAddr, CommandData.actbus.tc_prefs, NIOS_QUEUE);
  WriteData(waitSfAddr, CommandData.actbus.tc_wait / 10., NIOS_QUEUE);
  WriteData(goalSfAddr, CommandData.actbus.focus, NIOS_FLUSH);

  WriteData(statusActbusAddr, actuators_init, NIOS_FLUSH);
}

/************************************************************************/
/*                                                                      */
/*    Act Thread: initialize bus and command lock/secondary steppers    */
/*                                                                      */
/************************************************************************/
//for NICC to get DR from bus and save to disk
void SyncDR()
{
  int i;
  static int firsttime = 1;
  static struct BiPhaseStruct* drActAddr[3];
  static struct BiPhaseStruct* offsetActAddr[3];
  short dr;
  unsigned short offset;

  if (firsttime) {
    firsttime = 0;
    drActAddr[0] = GetBiPhaseAddr("dr_0_act");
    drActAddr[1] = GetBiPhaseAddr("dr_1_act");
    drActAddr[2] = GetBiPhaseAddr("dr_2_act");
    offsetActAddr[0] = GetBiPhaseAddr("offset_0_act");
    offsetActAddr[1] = GetBiPhaseAddr("offset_1_act");
    offsetActAddr[2] = GetBiPhaseAddr("offset_2_act");
  }

  //get dead reckoning data
  for (i=0; i<3; i++) {
    dr = slow_data[drActAddr[i]->index][drActAddr[i]->channel];
    offset = slow_data[offsetActAddr[i]->index][offsetActAddr[i]->channel];
    act_data[i].dr = dr + offset;
  }

  WriteDR();
}

void ActuatorBus(void)
{
  int all_ok = 0;
  int i;
  int j=0; // Used for debugging print statements.  Delete later.
  int my_cindex = 0;
  int caddr_match = 0;
  int is_init = 0;
  int first_time=1;
  int sf_ok;


  nameThread("ActBus");
  bputs(startup, "ActuatorBus startup.");

  while (!InCharge) {
    if (first_time) {
      bprintf(info,"Not in charge.  Waiting.");
      first_time = 0;
    }
    usleep(1000000);
    CommandData.actbus.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
    if (BLASTBusUseful) {
      SetLockState(1); /* to ensure the NiC MCC knows the pin state */
      SyncDR();	     /* get encoder absolute state from the ICC */
    }
    CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP; /* ignore all commands */
    CommandData.actbus.caddr[my_cindex] = 0; /* prevent commands from executing 
                                                twice if we switch to ICC */
  }
  first_time = 1;
  while (!is_init) {
    if (first_time) {
      bprintf(info,"In Charge! Attempting to initalize.");
      first_time = 0;
    }
    if (EZBus_Init(&bus, ACT_BUS, "", ACTBUS_CHATTER) == EZ_ERR_OK)
      is_init = 1;
    usleep(10000);
    if (is_init) {
      bprintf(info,"Bus initialized on %ith attempt",j);
    }
    j++;
  }

  for (i=0; i<NACT; i++) {
    EZBus_Add(&bus, id[i], name[i]);
    if (i == LOCKNUM) EZBus_SetPreamble(&bus, id[i], LOCK_PREAMBLE);
    else if (i == SHUTTERNUM) EZBus_SetPreamble(&bus, id[i], SHUTTER_PREAMBLE);
    else if (id[i] == HWPR_ADDR) EZBus_SetPreamble(&bus, id[i], HWPR_PREAMBLE);
    else 
      EZBus_SetPreamble(&bus, id[i], actPreamble(CommandData.actbus.act_tol));
  }

  all_ok = !(EZBus_PollInit(&bus, InitialiseActuator) & EZ_ERR_POLL);

  for (;;) {
    /* Repoll bus if necessary */
    if (CommandData.actbus.force_repoll || bus.err_count > MAX_SERIAL_ERRORS) {
      for (i=0; i<NACT; i++)
	EZBus_ForceRepoll(&bus, id[i]);
      poll_timeout = 0;
      all_ok = 0;
      CommandData.actbus.force_repoll = 0;
    }
    
    if (poll_timeout <= 0 && !all_ok && actbus_reset) {
      //suppress non-error messages during repoll
      bus.chatter = EZ_CHAT_ERR;
      all_ok = !(EZBus_PollInit(&bus, InitialiseActuator) & EZ_ERR_POLL);
      bus.chatter = ACTBUS_CHATTER;
      poll_timeout = POLL_TIMEOUT;
    }    

    /* Send the uplinked command, if any */
    my_cindex = GETREADINDEX(CommandData.actbus.cindex);
    caddr_match = 0;
    for (i=0; i<NACT; i++)
      if (CommandData.actbus.caddr[my_cindex] == id[i]) caddr_match = 1;
    if (caddr_match) {
      bprintf(info, "Sending command %s to Act %c\n",
	      CommandData.actbus.command[my_cindex], 
	      CommandData.actbus.caddr[my_cindex]);
      //increase print level for uplinked manual commands
      bus.chatter = EZ_CHAT_BUS;
      EZBus_Comm(&bus, CommandData.actbus.caddr[my_cindex],
		 CommandData.actbus.command[my_cindex]);
      CommandData.actbus.caddr[my_cindex] = 0;
      bus.chatter = ACTBUS_CHATTER;
    }
    
    if (EZBus_IsUsable(&bus, id[LOCKNUM])) {
      // DoLock temporarily disabled.  7/18/2012
      DoLock(); 
      actuators_init |= 0x1 << LOCKNUM;
    } else {
      EZBus_ForceRepoll(&bus, id[LOCKNUM]);
      all_ok = 0;
      actuators_init &= ~(0x1 << LOCKNUM);
    }

    if (EZBus_IsUsable(&bus, id[SHUTTERNUM])) {

      DoShutter(); 
      actuators_init |= 0x1 << SHUTTERNUM;
    } else {
      EZBus_ForceRepoll(&bus, id[SHUTTERNUM]);
      all_ok = 0;
      actuators_init &= ~(0x1 << SHUTTERNUM);
    }
    
    sf_ok = 1;
    for (i=0; i<3; i++) {
      if (EZBus_IsUsable(&bus, id[i])) {
	actuators_init |= 0x1 << i;
      } else {
	EZBus_ForceRepoll(&bus, id[i]);
	all_ok = 0;
	sf_ok = 0;
	actuators_init &= ~(0x1 << i);
      }
    }
    if (sf_ok) DoActuators();

    if (EZBus_IsUsable(&bus, HWPR_ADDR)) {
     DoHWPR(&bus);
     actuators_init |= 0x1 << HWPRNUM;
    } else {
      EZBus_ForceRepoll(&bus, HWPR_ADDR);
      all_ok = 0;
     actuators_init &= ~(0x1 << HWPRNUM);
    }

    usleep(10000);
    
  }
}
