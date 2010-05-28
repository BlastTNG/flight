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

void nameThread(const char*);		/* mcp.c */
double LockPosition(double elevation);	/* commands.c */
extern short int InCharge;		/* tx.c */

/* actuator bus setup paramters */
#define ACTBUS_CHATTER	EZ_CHAT_ACT
#define ACT_BUS "/dev/ttySI15"
#define NACT 4
#define LOCKNUM 3
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor"};
static const int id[NACT] = {EZ_WHO_S1, EZ_WHO_S2, EZ_WHO_S3, EZ_WHO_S5};
#define ID_ALL_ACT  EZ_WHO_G1_4
#define LOCK_PREAMBLE "j256"
#define ACT_PREAMBLE  "aE25600n8"
static struct ezbus bus;
#define POLL_TIMEOUT 30000	    /* 5 minutes */

/* Lock motor parameters and data */
#define LOCK_MOTOR_DATA_TIMER 100   /* 1 second */
#define DRIVE_TIMEOUT 3000	    /* 30 seconds */

#define LOCK_MIN_POT 800
#define LOCK_MAX_POT 3750
#define LOCK_POT_RANGE 100

static struct lock_struct {
  int pos;		  //raw step count
  unsigned short adc[4];  //ADC readout (including pot_
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

/* Secondary actuator data and parameters */
#define LVDT_FILT_LEN 25   //5s @ 5Hz
#define ACTBUS_MAX_ENC_ERR  50	  //maximum difference between enc and lvdt
#define ACTBUS_TRIM_WAIT    20*2*LVDT_FILT_LEN  //twice LVDT_FILT_LEN, @ 100Hz
					  //wait between trims, and after moves
static struct act_struct {
  int pos;	//raw step count
  int enc;	//encoder reading
  int lvdt;	//lvdt-inferred position of this motor
} act_data[3];

static int actbus_flags = 0;
static int bad_move = 0;

#define ACTBUS_FL_LOST     0x001
#define ACTBUS_FL_DR_PS    0x002
#define ACTBUS_FL_DR_LG    0x004
#define ACTBUS_FL_LG_PS    0x008
#define ACTBUS_FL_DR_LV    0x010
#define ACTBUS_FL_LG_LV    0x020
#define ACTBUS_FL_PS_LV    0x040
#define ACTBUS_FL_BAD_MOVE 0x080
#define ACTBUS_FL_FAIL0    0x100
#define ACTBUS_FL_FAIL1    0x200
#define ACTBUS_FL_FAIL2    0x400

/* Secondary focus crap */
static double t_primary = -1, t_secondary = -1;
static double focus = -1;	  /* set in ab thread, read in fc thread */
static double correction = 0;     /* set in fc thread, read in ab thread */

/* Thermal model numbers, from MD and MV */
#define T_PRIMARY_FOCUS   258.15 /* = -15C */
#define T_SECONDARY_FOCUS 243.15 /* = -30C */
#define POSITION_FOCUS     11953 /* absolute counts */

/************************************************************************/
/*                                                                      */
/*    Actuator Logic: servo focus based on thermal model/commands       */
/*                                                                      */
/************************************************************************/
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
    bad_move = ACTBUS_FL_BAD_MOVE;
  } else
    bad_move = 0;

  return bad_move;
}

//if encoder values disagree with LVDTs, trim encoders to LVDT
static void trimActEnc()
{
  static int wait = ACTBUS_TRIM_WAIT;
  int i, do_trim = 0;
  char buf[EZ_BUS_BUF_LEN];

  for (i=0; i<3; i++) {
    //TODO should make business a global flag, and write to frame
    if (EZBus_IsBusy(&bus, id[i])) {
      wait = ACTBUS_TRIM_WAIT;
      return;
    }
  }
  if (wait-- > 0) return;

  for (i=0; i<3; i++) {
    if (!EZBus_IsBusy(&bus, id[i]) && 
	abs(act_data[i].enc - act_data[i].lvdt) > ACTBUS_MAX_ENC_ERR) {
      do_trim |= 1<<i;
    }
  }
  if (do_trim) {
    bputs(info, "Trimming actuator encoders to LVDTs");
    for (i=0; i<3; i++) {
      if (do_trim & 1<<i) {
	sprintf(buf, "z%dR", act_data[i].lvdt);
	EZBus_Comm(&bus, id[i], buf, 0);
      }
    }
    wait = ACTBUS_TRIM_WAIT;
  }
}

static void ReadActuator(int num)
{
  if (!EZBus_IsUsable(&bus, id[num])) return;

  EZBus_ReadInt(&bus, id[num], "?0", &act_data[num].pos);
  EZBus_ReadInt(&bus, id[num], "?8", &act_data[num].enc);
}

//TODO probably want to add back some of the checks from old ServoActuators
//TODO ServoActuators with aE25600n8 results in lots of command overflows (busy)
//  could maybe add a business check before other commands
//TODO should maybe add calls to EZBus_Stop before trying various commands
//TODO servo_actuators command needs bounds fixed
static void ServoActuators(int* goal)
{
  int i;

  if (CheckMove(goal[0], goal[1], goal[2]))
    return;

  if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC)
    return;

  EZBus_Take(&bus, ID_ALL_ACT);

  for (i = 0; i < 3; ++i) {
    EZBus_Goto(&bus, id[i], goal[i]);
  }

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
  //TODO for now always just go to sleep
  return ACTBUS_FM_SLEEP;

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
  int i;
  //int update_dr = 1;

  trimActEnc();
  EZBus_SetVel(&bus, ID_ALL_ACT, CommandData.actbus.act_vel);
  EZBus_SetAccel(&bus, ID_ALL_ACT, CommandData.actbus.act_acc);
  EZBus_SetIMove(&bus, ID_ALL_ACT, CommandData.actbus.act_move_i);
  EZBus_SetIHold(&bus, ID_ALL_ACT, CommandData.actbus.act_hold_i);

  switch(CommandData.actbus.focus_mode) {
    case ACTBUS_FM_PANIC:
      bputs(warning, "Actuator Panic");
      EZBus_Stop(&bus, ID_ALL_ACT); /* terminate all strings */
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
      break;
    case ACTBUS_FM_DELTA:
      DeltaActuators();
      if (CommandData.actbus.focus_mode != ACTBUS_FM_PANIC)
	CommandData.actbus.focus_mode = ThermalCompensation();
      break;
#if 0
    case ACTBUS_FM_DELFOC:
      CommandData.actbus.focus += focus;
      /* Fallthough */
    case ACTBUS_FM_THERMO:
    case ACTBUS_FM_FOCUS:
      update_dr = 0;
      if (SetNewFocus())
	/* fallthrough */
#endif
    case ACTBUS_FM_SERVO:
	ServoActuators(CommandData.actbus.goal/*, update_dr*/);
	if (CommandData.actbus.focus_mode != ACTBUS_FM_PANIC)
	  CommandData.actbus.focus_mode = ThermalCompensation();
	break;
#if 0
    case ACTBUS_FM_OFFSET:
	SetOffsets(CommandData.actbus.offset);
	/* fallthrough */
#endif
    case ACTBUS_FM_SLEEP:
	if (CommandData.actbus.focus_mode != ACTBUS_FM_PANIC)
	  CommandData.actbus.focus_mode = ThermalCompensation();
	break;
    default:
	bputs(err, "Unknown Focus Mode (%i), sleeping");
  }

#if 0
  UpdateFlags();
#endif

  for (i = 0; i < 3; ++i)
    ReadActuator(i);

  focus = (act_data[0].lvdt + act_data[1].lvdt + act_data[2].lvdt) / 3;


#if 0
  if (CommandData.actbus.reset_dr) {
    int i;

    for (i = 0; i < 3; ++i)
      CommandData.actbus.dead_reckon[i] = act_data[i].enc;

    CommandData.actbus.reset_dr = 0;
  }
#endif
}

void RecalcOffset(double new_gp, double new_gs)
{
  if (t_primary < 0 || t_secondary < 0)
    return;

  //TODO if new offset is to ensure correction doesn't change with gain change
  //then it's missing a scaling factor 1.0/ACTENC_TO_UM
  CommandData.actbus.sf_offset = (new_gp - CommandData.actbus.g_primary) *
    (t_primary - T_PRIMARY_FOCUS) - (new_gs - CommandData.actbus.g_secondary) *
    (t_secondary - T_SECONDARY_FOCUS) + CommandData.actbus.sf_offset;
}

static void InitialiseActuator(struct ezbus* thebus, char who)
{
  char buffer[EZ_BUS_BUF_LEN];
  int i;

  for (i=0; i<3; i++) {
    if (id[i] == who) {	  //only operate on actautors
      bprintf(info, "Initialising %s...", name[i]);

      /* Set the encoder */
      sprintf(buffer, ACT_PREAMBLE "z%iR", act_data[i].lvdt);  
      EZBus_Comm(thebus, who, buffer, 0);
      return;
    }
  }
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
  EZBus_Comm(&bus, id[LOCKNUM], "?aa", 0);
  sscanf(bus.buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1],
      &lock_data.adc[2], &lock_data.adc[3]);
}

/* The NiC MCC does this via the BlastBus to give it a chance to know what's
 * going on.  The ICC reads it directly to get more promptly the answer
 * (since all these fields are slow). */
static void SetLockState(int nic)
{
  static int firsttime = 1;
  int pot = lock_data.adc[1];
  unsigned int state = lock_data.state; 

  static struct BiPhaseStruct* lockPotAddr;
  static struct BiPhaseStruct* lockStateAddr;

  if (firsttime) {
    firsttime = 0;
    lockPotAddr = GetBiPhaseAddr("lock_pot");
    lockStateAddr = GetBiPhaseAddr("lock_state");
  }

  //set the EZBus move parameters
  EZBus_SetVel(&bus, id[LOCKNUM], CommandData.actbus.lock_vel);
  EZBus_SetAccel(&bus, id[LOCKNUM], CommandData.actbus.lock_acc);
  EZBus_SetIMove(&bus, id[LOCKNUM], CommandData.actbus.lock_move_i);
  EZBus_SetIHold(&bus, id[LOCKNUM], CommandData.actbus.lock_hold_i);

  //TODO lock state should probably be taken from bus even for ICC
  if (nic) {
    pot = slow_data[lockPotAddr->index][lockPotAddr->channel];
    state = slow_data[lockStateAddr->index][lockStateAddr->channel];
  }

  state &= LS_DRIVE_MASK; /* zero everything but drive info */

  if (pot < LOCK_MIN_POT)
    state |= LS_CLOSED;
  else if (pot > LOCK_MAX_POT) {
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

#define SEND_SLEEP 100000 /* 100 miliseconds */
#define WAIT_SLEEP 50000 /* 50 miliseconds */
#define LA_EXIT    0
#define LA_STOP    1
#define LA_WAIT    2
#define LA_EXTEND  3
#define LA_RETRACT 4
static void DoLock(void)
{
  int action = LA_EXIT;
  static int drive_timeout = 0;

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
    if (drive_timeout == 1) {
      bputs(warning, "Lock Motor drive timeout.");
      action = LA_STOP;
    }
    if (drive_timeout > 0)
      --drive_timeout;

    /* Seize the bus */
    if (action == LA_EXIT)
      EZBus_Release(&bus, id[LOCKNUM]);
    else
      EZBus_Take(&bus, id[LOCKNUM]);

    /* Figure out what to do... */
    switch (action) {
      case LA_STOP:
        drive_timeout = 0;
        bputs(info, "Stopping lock motor.");
        EZBus_Stop(&bus, id[LOCKNUM]); /* terminate all strings */
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_OFF;
        break;
      case LA_EXTEND:
        drive_timeout = DRIVE_TIMEOUT;
        bputs(info, "Extending lock motor.");
	EZBus_RelMove(&bus, id[LOCKNUM], INT_MAX);
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_EXT;
        break;
      case LA_RETRACT:
        drive_timeout = DRIVE_TIMEOUT;
        bputs(info, "Retracting lock motor.");
	EZBus_RelMove(&bus, id[LOCKNUM], INT_MIN);
	usleep(SEND_SLEEP); /* wait for a bit */
        lock_data.state &= ~LS_DRIVE_MASK;
        lock_data.state |= LS_DRIVE_RET;
        break;
      case LA_WAIT:
	usleep(WAIT_SLEEP); /* wait for a bit */
	break;
    }

  } while (action != LA_EXIT);
}

/************************************************************************/
/*                                                                      */
/*    Frame Logic: Write data to the frame, called from main thread     */
/*                                                                      */
/************************************************************************/
static double CalibrateAD590(int counts)
{
  double t = M_16T * (counts + B_16T);

  /* if t < -73C or t > 67C, assume AD590 is broken */
  if (t < 170)
    t = -1;
  else if (t > 360)
    t = -2;

  return t;
}

#define N_FILT_TEMP 2	    //number of temperatures to filter
#define TEMP_FILT_LEN 300   //60s @ 5Hz
static int filterTemp(int num, int data)
{
  static int temp_buf[N_FILT_TEMP][TEMP_FILT_LEN] = {}; //init to 0
  static int temp_sum[N_FILT_TEMP] = {};
  static int ibuf = 0;

  temp_sum[num] += (data - temp_buf[num][ibuf]);
  temp_buf[num][ibuf] = data;
  ibuf = (ibuf + 1) % TEMP_FILT_LEN;
  return temp_sum[num]/TEMP_FILT_LEN;
}

/* decide on primary and secondary temperature, write focus-related fields */
void SecondaryMirror(void)
{
  static int firsttime = 1;

  static struct NiosStruct* sfCorrectionAddr;
  static struct NiosStruct* sfAgeAddr;
  static struct NiosStruct* sfOffsetAddr;
  static struct NiosStruct* tPrimeFidAddr;
  static struct NiosStruct* tSecondFidAddr;

  static struct BiPhaseStruct* tPrimary1Addr;
  static struct BiPhaseStruct* tSecondary1Addr;
  static struct BiPhaseStruct* tPrimary2Addr;
  static struct BiPhaseStruct* tSecondary2Addr;
  double t_primary1, t_secondary1;
  double t_primary2, t_secondary2;

  double correction_temp = 0;

  if (firsttime) {
    firsttime = 0;
    tPrimary1Addr = GetBiPhaseAddr("t_primary_1");
    tSecondary1Addr = GetBiPhaseAddr("t_secondary_1");
    tPrimary2Addr = GetBiPhaseAddr("t_primary_2");
    tSecondary2Addr = GetBiPhaseAddr("t_secondary_2");
    sfCorrectionAddr = GetNiosAddr("sf_correction");
    sfAgeAddr = GetNiosAddr("sf_age");
    sfOffsetAddr = GetNiosAddr("sf_offset");
    tPrimeFidAddr = GetNiosAddr("t_prime_fid");
    tSecondFidAddr = GetNiosAddr("t_second_fid");
  }

#if 0 //TODO will need to change or remove this check for haven't heard from act
  /* Do nothing if we haven't heard from the actuators */
  if (focus < -ACTENC_OFFSET / 2)
    return;
#endif

  t_primary1 = CalibrateAD590(
      slow_data[tPrimary1Addr->index][tPrimary1Addr->channel]
      ) + AD590_CALIB_PRIMARY_1;
  t_primary2 = CalibrateAD590(
      slow_data[tPrimary2Addr->index][tPrimary2Addr->channel]
      ) + AD590_CALIB_PRIMARY_2;

  t_secondary1 = CalibrateAD590(
      slow_data[tSecondary1Addr->index][tSecondary1Addr->channel]
      ) + AD590_CALIB_SECONDARY_1;
  t_secondary2 = CalibrateAD590(
      slow_data[tSecondary2Addr->index][tSecondary2Addr->channel]
      ) + AD590_CALIB_SECONDARY_2;

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
  correction_temp = correction_temp + focus - POSITION_FOCUS -
    CommandData.actbus.sf_offset;

  correction = correction_temp;  //slightly more thread safe

  if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait)
    CommandData.actbus.sf_time++;

  WriteData(tPrimeFidAddr, (t_primary - 273.15) * 500, NIOS_QUEUE);
  WriteData(tSecondFidAddr, (t_secondary - 273.15) * 500, NIOS_QUEUE);
  WriteData(sfCorrectionAddr, correction, NIOS_QUEUE);
  WriteData(sfAgeAddr, CommandData.actbus.sf_time / 10., NIOS_QUEUE);
  WriteData(sfOffsetAddr, CommandData.actbus.sf_offset, NIOS_FLUSH);
}

static char name_buffer[100];
static inline struct NiosStruct* GetActNiosAddr(int i, const char* field)
{
  sprintf(name_buffer, "act%i_%s", i, field);

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
  return lvdt_sum[num]/LVDT_FILT_LEN;
}

void StoreActBus(void)
{
  int j;
  static int firsttime = 1;
  int actbus_reset = 1;   //1 means actbus is on
  int lvdt_filt[3];

  static struct BiPhaseStruct* lvdt63RawAddr;    //used to be 11
  static struct BiPhaseStruct* lvdt64RawAddr;    //used to be 13
  static struct BiPhaseStruct* lvdt65RawAddr;    //used to be 10

  static struct NiosStruct* actbusResetAddr;
  static struct NiosStruct* lockPosAddr;
  static struct NiosStruct* lockStateAddr;
  static struct NiosStruct* lockGoalAddr;
  static struct NiosStruct* seizedBusAddr;
  static struct NiosStruct* lockPotAddr;
  static struct NiosStruct* lokmotPinAddr;

  static struct NiosStruct* lockVelAddr;
  static struct NiosStruct* lockAccAddr;
  static struct NiosStruct* lockMoveIAddr;
  static struct NiosStruct* lockHoldIAddr;

  static struct NiosStruct* actVelAddr;
  static struct NiosStruct* actAccAddr;
  static struct NiosStruct* actMoveIAddr;
  static struct NiosStruct* actHoldIAddr;
  static struct NiosStruct* actFlagsAddr;

  static struct NiosStruct* actPosAddr[3];
  static struct NiosStruct* actEncAddr[3];
  static struct NiosStruct* actLVDTAddr[3];

  static struct NiosStruct* lvdtSpreadAddr;
  static struct NiosStruct* lvdtLowAddr;
  static struct NiosStruct* lvdtHighAddr;

  static struct NiosStruct* tcGPrimAddr;
  static struct NiosStruct* tcGSecAddr;
  static struct NiosStruct* tcStepAddr;
  static struct NiosStruct* tcWaitAddr;
  static struct NiosStruct* tcModeAddr;
  static struct NiosStruct* tcSpreadAddr;
  static struct NiosStruct* tcPrefTpAddr;
  static struct NiosStruct* tcPrefTsAddr;
  static struct NiosStruct* secGoalAddr;
  static struct NiosStruct* absFocusAddr;

  if (firsttime) {
    firsttime = 0;

    lvdt63RawAddr = GetBiPhaseAddr("lvdt_63_raw");
    lvdt64RawAddr = GetBiPhaseAddr("lvdt_64_raw");
    lvdt65RawAddr = GetBiPhaseAddr("lvdt_65_raw");

    actbusResetAddr = GetNiosAddr("actbus_reset");
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
    lockPosAddr = GetNiosAddr("lock_pos");
    lockStateAddr = GetNiosAddr("lock_state");
    seizedBusAddr = GetNiosAddr("seized_bus");
    lockGoalAddr = GetNiosAddr("lock_goal");
    lockPotAddr = GetNiosAddr("lock_pot");

    for (j = 0; j < 3; ++j) {
      actPosAddr[j] = GetActNiosAddr(j, "pos");
      actEncAddr[j] = GetActNiosAddr(j, "enc");
      actLVDTAddr[j] = GetActNiosAddr(j, "lvdt");
    }

    tcGPrimAddr = GetNiosAddr("tc_g_prim");
    tcGSecAddr = GetNiosAddr("tc_g_sec");
    tcStepAddr = GetNiosAddr("tc_step");
    tcWaitAddr = GetNiosAddr("tc_wait");
    tcModeAddr = GetNiosAddr("tc_mode");
    tcSpreadAddr = GetNiosAddr("tc_spread");
    tcPrefTpAddr = GetNiosAddr("tc_pref_tp");
    tcPrefTsAddr = GetNiosAddr("tc_pref_ts");
    secGoalAddr = GetNiosAddr("sec_goal");
    absFocusAddr = GetNiosAddr("abs_focus");

    lvdtSpreadAddr = GetNiosAddr("lvdt_spread");
    lvdtLowAddr = GetNiosAddr("lvdt_low");
    lvdtHighAddr = GetNiosAddr("lvdt_high");

    actVelAddr = GetNiosAddr("act_vel");
    actAccAddr = GetNiosAddr("act_acc");
    actMoveIAddr = GetNiosAddr("act_move_i");
    actHoldIAddr = GetNiosAddr("act_hold_i");
    actFlagsAddr = GetNiosAddr("act_flags");

    lockVelAddr = GetNiosAddr("lock_vel");
    lockAccAddr = GetNiosAddr("lock_acc");
    lockMoveIAddr = GetNiosAddr("lock_move_i");
    lockHoldIAddr = GetNiosAddr("lock_hold_i");
  }

  //filter the LVDTs, scale into encoder units, rotate to motor positions
  lvdt_filt[0] = filterLVDT(0, 
      slow_data[lvdt63RawAddr->index][lvdt63RawAddr->channel]);
  lvdt_filt[1] = filterLVDT(1, 
      slow_data[lvdt64RawAddr->index][lvdt64RawAddr->channel]);
  lvdt_filt[2] = filterLVDT(2, 
      slow_data[lvdt65RawAddr->index][lvdt65RawAddr->channel]);
  lvdt_filt[0] = (int)((double)lvdt_filt[0] * LVDT63_ADC_TO_ENC + LVDT63_ZERO);
  lvdt_filt[1] = (int)((double)lvdt_filt[1] * LVDT64_ADC_TO_ENC + LVDT64_ZERO);
  lvdt_filt[2] = (int)((double)lvdt_filt[2] * LVDT65_ADC_TO_ENC + LVDT65_ZERO);
  act_data[0].lvdt = (-lvdt_filt[2] + 2 * lvdt_filt[0] + 2 * lvdt_filt[1]) / 3;
  act_data[1].lvdt = (-lvdt_filt[0] + 2 * lvdt_filt[1] + 2 * lvdt_filt[2]) / 3;
  act_data[2].lvdt = (-lvdt_filt[1] + 2 * lvdt_filt[2] + 2 * lvdt_filt[0]) / 3;

  if (CommandData.actbus.off) {
    if (CommandData.actbus.off > 0) CommandData.actbus.off--;
    actbus_reset = 0;   //turn actbus off
  }
  WriteData(actbusResetAddr, actbus_reset, NIOS_QUEUE);

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  for (j = 0; j < 3; ++j) {
    WriteData(actPosAddr[j], act_data[j].pos, NIOS_QUEUE);
    WriteData(actEncAddr[j], act_data[j].enc, NIOS_QUEUE);
    WriteData(actLVDTAddr[j], act_data[j].lvdt, NIOS_QUEUE);
  }
  WriteData(absFocusAddr, focus, NIOS_QUEUE);

  WriteData(lockPotAddr, lock_data.adc[1], NIOS_QUEUE);
  WriteData(lockStateAddr, lock_data.state, NIOS_QUEUE);
  WriteData(seizedBusAddr, bus.seized, NIOS_QUEUE);
  WriteData(lockGoalAddr, CommandData.actbus.lock_goal, NIOS_QUEUE);
  WriteData(lockPosAddr, lock_data.pos, NIOS_QUEUE);

  WriteData(actVelAddr, CommandData.actbus.act_vel, NIOS_QUEUE);
  WriteData(actAccAddr, CommandData.actbus.act_acc, NIOS_QUEUE);
  WriteData(actMoveIAddr, CommandData.actbus.act_move_i, NIOS_QUEUE);
  WriteData(actHoldIAddr, CommandData.actbus.act_hold_i, NIOS_QUEUE);
  WriteData(actFlagsAddr, actbus_flags, NIOS_QUEUE);

  WriteData(lvdtSpreadAddr, CommandData.actbus.lvdt_delta, NIOS_QUEUE);
  WriteData(lvdtLowAddr, CommandData.actbus.lvdt_low+5000, NIOS_QUEUE);
  WriteData(lvdtHighAddr, CommandData.actbus.lvdt_high+5000, NIOS_QUEUE);

  WriteData(lockVelAddr, CommandData.actbus.lock_vel / 100, NIOS_QUEUE);
  WriteData(lockAccAddr, CommandData.actbus.lock_acc, NIOS_QUEUE);
  WriteData(lockMoveIAddr, CommandData.actbus.lock_move_i, NIOS_QUEUE);
  WriteData(lockHoldIAddr, CommandData.actbus.lock_hold_i, NIOS_QUEUE);

  WriteData(tcGPrimAddr, CommandData.actbus.g_primary * 100., NIOS_QUEUE);
  WriteData(tcGSecAddr, CommandData.actbus.g_secondary * 100., NIOS_QUEUE);
  WriteData(tcModeAddr, CommandData.actbus.tc_mode, NIOS_QUEUE);
  WriteData(tcStepAddr, CommandData.actbus.tc_step, NIOS_QUEUE);
  WriteData(tcSpreadAddr, CommandData.actbus.tc_spread * 500., NIOS_QUEUE);
  WriteData(tcPrefTpAddr, CommandData.actbus.tc_prefp, NIOS_QUEUE);
  WriteData(tcPrefTsAddr, CommandData.actbus.tc_prefs, NIOS_QUEUE);
  WriteData(tcWaitAddr, CommandData.actbus.tc_wait / 10., NIOS_QUEUE);
  WriteData(secGoalAddr, CommandData.actbus.focus, NIOS_FLUSH);
}

/************************************************************************/
/*                                                                      */
/*    Act Thread: initialize bus and command lock/secondary steppers    */
/*                                                                      */
/************************************************************************/
void ActuatorBus(void)
{
  int poll_timeout = POLL_TIMEOUT;
  int all_ok = 0;
  int i;
  int my_cindex = 0;
  int caddr_match = 0;

  nameThread("ActBus");
  bputs(startup, "ActuatorBus startup.");

  //temporary partian fix to NIC problems
  while (!InCharge) usleep(1000000);

  if (EZBus_Init(&bus, ACT_BUS, "", ACTBUS_CHATTER) != EZ_ERR_OK)
    berror(tfatal, "failed to connect");

  for (i=0; i<NACT; i++) {
    EZBus_Add(&bus, id[i], name[i]);
    if (i == LOCKNUM) EZBus_SetPreamble(&bus, id[i], LOCK_PREAMBLE);
    else EZBus_SetPreamble(&bus, id[i], ACT_PREAMBLE);
  }

  all_ok = !(EZBus_PollInit(&bus, InitialiseActuator) & EZ_ERR_POLL);

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      CommandData.actbus.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
      EZBus_Recv(&bus);
      SetLockState(1); /* to ensure the NiC MCC knows the pin state */
      //CopyActuators(); /* let the NiC MCC know what's going on */
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP; /* ignore all commands */
      /* no need to sleep -- BusRecv does that for us */
      CommandData.actbus.caddr[my_cindex] = 0;
    }

    /* Repoll bus if necessary */
    if (CommandData.actbus.force_repoll) {
      for (i=0; i<NACT; i++)
	EZBus_ForceRepoll(&bus, id[i]);
      poll_timeout = POLL_TIMEOUT;
      all_ok = !(EZBus_PollInit(&bus, InitialiseActuator) & EZ_ERR_POLL);
      CommandData.actbus.force_repoll = 0;
    }

    if (poll_timeout == 0 && !all_ok) {
      all_ok = !(EZBus_PollInit(&bus, InitialiseActuator) & EZ_ERR_POLL);
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0)
      poll_timeout--;

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
	  CommandData.actbus.command[my_cindex], 0);
      CommandData.actbus.caddr[my_cindex] = 0;
      bus.chatter = ACTBUS_CHATTER;
    }

    DoLock(); /* Lock motor stuff -- this will seize the bus until
                 the lock motor's state has settled */

    DoActuators(); /* Actuator stuff -- this may seize the bus */

    usleep(10000);
  }
}
