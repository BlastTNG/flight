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

/* actuator bus setup paramters */
#define ACTBUS_CHATTER	EZ_CHAT_ACT
#define ACT_BUS "/dev/ttySI15"
#define NACT 4
#define LOCKNUM 3
static const char *name[NACT] = {"Actuator #0", "Actuator #1", "Actuator #2",
  "Lock Motor"};
static const int id[NACT] = {EZ_WHO_S1, EZ_WHO_S2, EZ_WHO_S3, EZ_WHO_S5};
static struct ezbus bus;

#define POLL_TIMEOUT 30000	    /* 5 minutes */
#define LOCK_MOTOR_DATA_TIMER 100   /* 1 second */
#define DRIVE_TIMEOUT 3000	    /* 30 seconds */

/* Lock motor thresholds */
#define LOCK_MIN_POT 800
#define LOCK_MAX_POT 3750
#define LOCK_POT_RANGE 100

void nameThread(const char*);		/* mcp.c */
double LockPosition(double elevation);	/* commands.c */
extern short int InCharge;		/* tx.c */


static struct lock_struct {
  int pos;
  unsigned short adc[4];
  unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

static double lvdt[3];

static int actbus_flags = 0;

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

/************************************************************************/
/*                                                                      */
/*    Actuator Logic: servo focus based on thermal model/commands       */
/*                                                                      */
/************************************************************************/
static void DoActuators(void)
{
  return;
}

//TODO this can maybe be removed, but it's command need this to compile
void ActPotTrim(void)
{
  return;
}

void RecalcOffset(double new_gp, double new_gs)
{
  /* TODO removed until actuators reimplemented
  if (t_primary < 0 || t_secondary < 0)
    return;

  CommandData.actbus.sf_offset = (new_gp - CommandData.actbus.g_primary) *
    (t_primary - T_PRIMARY_FOCUS) - (new_gs - CommandData.actbus.g_secondary) *
    (t_secondary - T_SECONDARY_FOCUS) + CommandData.actbus.sf_offset;
  */
}

void CopyActuators(void)
{
  static struct BiPhaseStruct* act0LGoodAddr;
  static struct BiPhaseStruct* act1LGoodAddr;
  static struct BiPhaseStruct* act2LGoodAddr;
  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;
    act0LGoodAddr = GetBiPhaseAddr("act0_l_good");
    act1LGoodAddr = GetBiPhaseAddr("act1_l_good");
    act2LGoodAddr = GetBiPhaseAddr("act2_l_good");
  }

  CommandData.actbus.last_good[0] = CommandData.actbus.dead_reckon[0] =
    slow_data[act0LGoodAddr->index][act0LGoodAddr->channel];
  CommandData.actbus.last_good[1] = CommandData.actbus.dead_reckon[1] =
    slow_data[act1LGoodAddr->index][act1LGoodAddr->channel];
  CommandData.actbus.last_good[2] = CommandData.actbus.dead_reckon[2] =
    slow_data[act2LGoodAddr->index][act2LGoodAddr->channel];
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

  //set the move parameters
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
void SecondaryMirror(void)
{
  return;
}

static char name_buffer[100];
static inline struct NiosStruct* GetActNiosAddr(int i, const char* field)
{
  sprintf(name_buffer, "act%i_%s", i, field);

  return GetNiosAddr(name_buffer);
}

void StoreActBus(void)
{
  int j;
  static int firsttime = 1;
  int actbus_reset = 1;   //1 means actbus is on

  static struct BiPhaseStruct* lvdt65Addr;    //used to be 10
  static struct BiPhaseStruct* lvdt63Addr;    //used to be 11
  static struct BiPhaseStruct* lvdt64Addr;    //used to be 13

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
  static struct NiosStruct* actPostrimAddr[3];
  static struct NiosStruct* actEncAddr[3];
  static struct NiosStruct* actDeadRecAddr[3];
  static struct NiosStruct* actLGoodAddr[3];

  static struct NiosStruct* lvdtSpreadAddr;
  static struct NiosStruct* lvdtLowAddr;
  static struct NiosStruct* lvdtHighAddr;

  static struct NiosStruct* tcGPrimAddr;
  static struct NiosStruct* tcGSecAddr;
  static struct NiosStruct* tcStepAddr;
  static struct NiosStruct* tcWaitAddr;
  static struct NiosStruct* tcFilterAddr;
  static struct NiosStruct* tcModeAddr;
  static struct NiosStruct* tcSpreadAddr;
  static struct NiosStruct* tcPrefTpAddr;
  static struct NiosStruct* tcPrefTsAddr;
  static struct NiosStruct* secGoalAddr;
  static struct NiosStruct* absFocusAddr;

  if (firsttime) {
    firsttime = 0;

    lvdt63Addr = GetBiPhaseAddr("lvdt_63");
    lvdt64Addr = GetBiPhaseAddr("lvdt_64");
    lvdt65Addr = GetBiPhaseAddr("lvdt_65");

    actbusResetAddr = GetNiosAddr("actbus_reset");
    lokmotPinAddr = GetNiosAddr("lokmot_pin");
    lockPosAddr = GetNiosAddr("lock_pos");
    lockStateAddr = GetNiosAddr("lock_state");
    seizedBusAddr = GetNiosAddr("seized_bus");
    lockGoalAddr = GetNiosAddr("lock_goal");
    lockPotAddr = GetNiosAddr("lock_pot");

    for (j = 0; j < 3; ++j) {
      actPosAddr[j] = GetActNiosAddr(j, "pos");
      actPostrimAddr[j] = GetActNiosAddr(j, "postrim");
      actEncAddr[j] = GetActNiosAddr(j, "enc");
      actLGoodAddr[j] = GetActNiosAddr(j, "l_good");
      actDeadRecAddr[j] = GetActNiosAddr(j, "dead_rec");
    }

    tcGPrimAddr = GetNiosAddr("tc_g_prim");
    tcGSecAddr = GetNiosAddr("tc_g_sec");
    tcStepAddr = GetNiosAddr("tc_step");
    tcWaitAddr = GetNiosAddr("tc_wait");
    tcFilterAddr = GetNiosAddr("tc_filter");
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

  //old naming: lvdt[0] = lvdt_10, lvdt[1] = lvdt_11, lvdt[2] = lvdt_13;
  lvdt[0] = slow_data[lvdt63Addr->index][lvdt63Addr->channel] *
    LVDT63_ADC_TO_ENC + LVDT63_ZERO;
  lvdt[1] = slow_data[lvdt64Addr->index][lvdt64Addr->channel] *
    LVDT64_ADC_TO_ENC + LVDT64_ZERO;
  lvdt[2] = slow_data[lvdt65Addr->index][lvdt65Addr->channel] *
    LVDT65_ADC_TO_ENC + LVDT65_ZERO;

  if (CommandData.actbus.off) {
    if (CommandData.actbus.off > 0) CommandData.actbus.off--;
    actbus_reset = 0;   //turn actbus off
  }
  WriteData(actbusResetAddr, actbus_reset, NIOS_QUEUE);

  WriteData(lokmotPinAddr, CommandData.pin_is_in, NIOS_QUEUE);

  /* TODO removed until actbus code rewritten
  for (j = 0; j < 3; ++j) {
    WriteData(actPosAddr[j], act_data[j].pos + CommandData.actbus.pos_trim[j],
        NIOS_QUEUE);
    WriteData(actPostrimAddr[j], CommandData.actbus.pos_trim[j], NIOS_QUEUE);
    WriteData(actEncAddr[j], act_data[j].enc, NIOS_QUEUE);
    WriteData(actLGoodAddr[j], CommandData.actbus.last_good[j] - ACTENC_OFFSET,
        NIOS_QUEUE);
    WriteData(actDeadRecAddr[j], CommandData.actbus.dead_reckon[j]
        - ACTENC_OFFSET, NIOS_QUEUE);
  }
  WriteData(absFocusAddr, focus, NIOS_QUEUE);
  */

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
  WriteData(lvdtLowAddr, CommandData.actbus.lvdt_low, NIOS_QUEUE);
  WriteData(lvdtHighAddr, CommandData.actbus.lvdt_high, NIOS_QUEUE);

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
  WriteData(tcFilterAddr, CommandData.actbus.tc_filter, NIOS_QUEUE);
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
    if (i == LOCKNUM) EZBus_SetPreamble(&bus, id[i], "j256");
    else EZBus_SetPreamble(&bus, id[i], "aE25600");
  }

  all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      CommandData.actbus.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
      EZBus_Recv(&bus);
      SetLockState(1); /* to ensure the NiC MCC knows the pin state */
      CopyActuators(); /* let the NiC MCC know what's going on */
      CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP; /* ignore all commands */
      /* no need to sleep -- BusRecv does that for us */
      CommandData.actbus.caddr[my_cindex] = 0;
    }

    /* Repoll bus if necessary */
    if (CommandData.actbus.force_repoll) {
      for (i=0; i<NACT; i++)
	EZBus_ForceRepoll(&bus, id[i]);
      poll_timeout = POLL_TIMEOUT;
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      CommandData.actbus.force_repoll = 0;
    }

    if (poll_timeout == 0 && !all_ok) {
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0)
      poll_timeout--;

    /* Send the uplinked command, if any */
    my_cindex = GETREADINDEX(CommandData.actbus.cindex);
    caddr_match = 0;
    for (i=0; i<NACT; i++)
      if (CommandData.actbus.caddr[my_cindex] == id[i]) caddr_match = 1;
    if (caddr_match) {
      EZBus_Comm(&bus, CommandData.actbus.caddr[my_cindex],
	  CommandData.actbus.command[my_cindex], 0);
      CommandData.actbus.caddr[my_cindex] = 0;
    }

    DoLock(); /* Lock motor stuff -- this will seize the bus until
                 the lock motor's state has settled */

    DoActuators(); /* Actuator stuff -- this may seize the bus */

    /* TODO removed until actuators reimplemented
    for (i = 0; i < 3; ++i)
      ReadActuator(i, 1);

    focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3
      - ACTENC_OFFSET;
    */

    usleep(10000);
  }
}
