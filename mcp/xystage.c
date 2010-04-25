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

#include "mcp.h"

#ifdef USE_XY_THREAD

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>

#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "ezstep.h"

#ifdef USE_XY_STAGE
#  error Both USE_XY_STAGE and USE_XY_THREAD defined.
#endif

/* EZBus setup parameters */
#define STAGE_BUS_TTY "/dev/ttyXYSTAGE"
#define STAGE_BUS_CHATTER EZ_CHAT_ACT
#define STAGEX_NAME "XY Stage X"
#define STAGEY_NAME "XY Stage Y"
#define STAGEX_ID EZ_WHO_S6
#define STAGEY_ID EZ_WHO_S7

#define STAGE_BUS_ACCEL 2
#define STAGE_BUS_IHOLD 20
#define STAGE_BUS_IMOVE 30

#define STAGEXNUM 0
#define STAGEYNUM 1
#define POLL_TIMEOUT 30000 /* 5 minutes */

extern short int InCharge; /* tx.c */
void nameThread(const char*);	/* mcp.c */

static struct stage_struct {
  int xpos, ypos;
  int xlim, ylim;
  int xstp, ystp;
  int xstr, ystr;
  int xvel, yvel;
} stage_data;


static void ReadStage(struct ezbus* bus)
{
  static int counter = 0;
  if (!EZBus_IsUsable(bus, STAGEX_ID) || !EZBus_IsUsable(bus, STAGEY_ID))
    return;

  EZBus_ReadInt(bus, STAGEX_ID, "?0", &stage_data.xpos);
  EZBus_ReadInt(bus, STAGEY_ID, "?0", &stage_data.ypos);

  if (counter == 0)
    EZBus_ReadInt(bus, STAGEX_ID, "?1", &stage_data.xstr);
  else if (counter == 1)
    EZBus_ReadInt(bus, STAGEX_ID, "?3", &stage_data.xstp);
  else if (counter == 2)
    EZBus_ReadInt(bus, STAGEX_ID, "?4", &stage_data.xlim);
  else if (counter == 3)
    EZBus_ReadInt(bus, STAGEX_ID, "?5", &stage_data.xvel);
  else if (counter == 4)
    EZBus_ReadInt(bus, STAGEY_ID, "?1", &stage_data.ystr);
  else if (counter == 5)
    EZBus_ReadInt(bus, STAGEY_ID, "?3", &stage_data.ystp);
  else if (counter == 6)
    EZBus_ReadInt(bus, STAGEY_ID, "?5", &stage_data.yvel);
  else if (counter == 7)
    EZBus_ReadInt(bus, STAGEY_ID, "?4", &stage_data.ylim);

  counter = (counter + 1) % 8;
}

/* This function is called by the frame control thread */
void StoreStageBus(int index)
{
  static int firsttime = 1;

  static struct NiosStruct* stageXAddr;
  static struct NiosStruct* stageXLimAddr;
  static struct NiosStruct* stageXStpAddr;
  static struct NiosStruct* stageXStrAddr;
  static struct NiosStruct* stageXVelAddr;

  static struct NiosStruct* stageYAddr;
  static struct NiosStruct* stageYLimAddr;
  static struct NiosStruct* stageYStpAddr;
  static struct NiosStruct* stageYStrAddr;
  static struct NiosStruct* stageYVelAddr;

  if (firsttime) {
    firsttime = 0;

    stageXAddr = GetNiosAddr("stage_x");
    stageXLimAddr = GetNiosAddr("stage_x_lim");
    stageXStrAddr = GetNiosAddr("stage_x_str");
    stageXStpAddr = GetNiosAddr("stage_x_stp");
    stageXVelAddr = GetNiosAddr("stage_x_vel");
    stageYAddr = GetNiosAddr("stage_y");
    stageYLimAddr = GetNiosAddr("stage_y_lim");
    stageYStrAddr = GetNiosAddr("stage_y_str");
    stageYStpAddr = GetNiosAddr("stage_y_stp");
    stageYVelAddr = GetNiosAddr("stage_y_vel");
  }

  WriteData(stageXAddr, stage_data.xpos/2, NIOS_QUEUE);
  WriteData(stageYAddr, stage_data.ypos/2, NIOS_QUEUE);
  if (index == 0) {
    WriteData(stageXLimAddr, stage_data.xlim, NIOS_QUEUE);
    WriteData(stageXStrAddr, stage_data.xstr, NIOS_QUEUE);
    WriteData(stageXStpAddr, stage_data.xstp, NIOS_QUEUE);
    WriteData(stageXVelAddr, stage_data.xvel, NIOS_QUEUE);
    WriteData(stageYStrAddr, stage_data.ystr, NIOS_QUEUE);
    WriteData(stageYStpAddr, stage_data.ystp, NIOS_QUEUE);
    WriteData(stageYVelAddr, stage_data.yvel, NIOS_QUEUE);
    WriteData(stageYLimAddr, stage_data.ylim, NIOS_FLUSH);
  }
}

void GoWait(struct ezbus *bus, int dest, int vel, int is_y)
{
  int now;
  char who = (is_y) ? STAGEY_ID : STAGEX_ID;

  if (CommandData.xystage.mode == XYSTAGE_PANIC || vel == 0)
    return;

  if (dest < 0)
    dest = 0;

  bprintf(info, "Move %c to %i at speed %i and wait", 
      (is_y) ? 'Y' : 'X', dest, vel);
  EZBus_GotoVel(bus, who, dest, vel);

  do {
    if (CommandData.xystage.mode == XYSTAGE_PANIC)
      return;
    usleep(10000);

    ReadStage(bus);

    now = (is_y) ? stage_data.ypos : stage_data.xpos;
  } while (now != dest);
}

void Raster(struct ezbus *bus, int start, int end, int is_y, int y, 
    int ymin, int ymax, int vel, int ss)
{
  int x;
  int step = (start > end) ? -ss : ss;
  bprintf(info, "Raster %i %i %i\n", start, end, step); 
  for (x = start; x != end + step; x += step) {
    if (step < 0) {
      if (x < end)
        x = end;
    } else if (x > end)
      x = end;
    GoWait(bus, x, vel, is_y);
    if (y == ymin)
      y = ymax;
    else
      y = ymin;
    GoWait(bus, y, vel, !is_y);
  }
}

void ControlXYStage(struct ezbus* bus)
{
  int my_cindex = 0;

  /* Send the uplinked command, if any */
  my_cindex = GETREADINDEX(CommandData.actbus.cindex);
  if (CommandData.actbus.caddr[my_cindex] == STAGEX_ID 
      || CommandData.actbus.caddr[my_cindex] == STAGEY_ID) {
    EZBus_Comm(bus, CommandData.actbus.caddr[my_cindex],
	CommandData.actbus.command[my_cindex], 0);
    CommandData.actbus.caddr[my_cindex] = 0;
  }

  /* respond to normal movement commands */
  if (CommandData.xystage.is_new) {
    /* PANIC! */
    if (CommandData.xystage.mode == XYSTAGE_PANIC) {
      bputs(info, "Panic");
      EZBus_Stop(bus, STAGEX_ID);
      EZBus_Stop(bus, STAGEY_ID);
      CommandData.xystage.is_new = 0;
    /* GOTO */
    } else if (CommandData.xystage.mode == XYSTAGE_GOTO) {
      if (CommandData.xystage.xvel > 0) {
	bprintf(info, "Move X to %i at speed %i",
	    CommandData.xystage.x1, CommandData.xystage.xvel);
	EZBus_GotoVel(bus, STAGEX_ID, CommandData.xystage.x1, 
	    CommandData.xystage.xvel);
      }
      if (CommandData.xystage.yvel > 0) {
	bprintf(info, "Move Y to %i at speed %i",
	    CommandData.xystage.y1, CommandData.xystage.yvel);
	EZBus_GotoVel(bus, STAGEY_ID, CommandData.xystage.y1,
	    CommandData.xystage.yvel);
      }
    /* JUMP */
    } else if (CommandData.xystage.mode == XYSTAGE_JUMP) {
      if (CommandData.xystage.xvel > 0 && CommandData.xystage.x1 != 0) {
	bprintf(info, "Jump X by %i at speed %i",
	    CommandData.xystage.x1, CommandData.xystage.xvel);
	EZBus_RelMoveVel(bus, STAGEX_ID, CommandData.xystage.x1,
	    CommandData.xystage.xvel);
      }
      if (CommandData.xystage.yvel > 0 && CommandData.xystage.y1 != 0) {
	bprintf(info, "Jump Y by %i at speed %i",
	    CommandData.xystage.y1, CommandData.xystage.yvel);
	EZBus_RelMoveVel(bus, STAGEY_ID, CommandData.xystage.y1,
	    CommandData.xystage.yvel);
      }
    /* SCAN */
    } else if (CommandData.xystage.mode == XYSTAGE_SCAN) {
      if (CommandData.xystage.xvel > 0 && CommandData.xystage.x2 > 0) {
	GoWait(bus, CommandData.xystage.x1, CommandData.xystage.xvel, 0);
	GoWait(bus, CommandData.xystage.x1 + CommandData.xystage.x2,
	    CommandData.xystage.xvel, 0);
	GoWait(bus, CommandData.xystage.x1 - CommandData.xystage.x2,
	    CommandData.xystage.xvel, 0);
	GoWait(bus, CommandData.xystage.x1, CommandData.xystage.xvel, 0);
      } else if (CommandData.xystage.yvel > 0 && CommandData.xystage.y2 > 0) {
	GoWait(bus, CommandData.xystage.y1, CommandData.xystage.yvel, 1);
	GoWait(bus, CommandData.xystage.y1 + CommandData.xystage.y2,
	    CommandData.xystage.yvel, 1);
	GoWait(bus, CommandData.xystage.y1 - CommandData.xystage.y2,
	    CommandData.xystage.yvel, 1);
	GoWait(bus, CommandData.xystage.y1, CommandData.xystage.yvel, 1);
      }
    /* RASTER */
    } else if (CommandData.xystage.mode == XYSTAGE_RASTER) {
      if (CommandData.xystage.xvel > 0) {
	int vel = CommandData.xystage.xvel;
	int xcent = CommandData.xystage.x1;
	int ycent = CommandData.xystage.y1;
	int size = CommandData.xystage.x2;
	int step = CommandData.xystage.y2;
	GoWait(bus, xcent, vel, 0);
	GoWait(bus, ycent - size, vel, 1);
	Raster(bus, xcent, xcent + size, 0, ycent + size, ycent - size,
	    ycent + size, vel, step);
	Raster(bus, ycent + size, ycent - size, 1, xcent + size, xcent - size,
	    xcent + size, vel, step);
	Raster(bus, xcent - size, xcent, 0, ycent - size, ycent - size,
	    ycent + size, vel, step);
	GoWait(bus, xcent, vel, 0);
	GoWait(bus, ycent, vel, 1);
      }
    }
    if (CommandData.xystage.mode != XYSTAGE_PANIC)
      CommandData.xystage.is_new = 0;
  }

}

void StageBus(void)
{
  int poll_timeout = POLL_TIMEOUT;
  int all_ok = 0;
  struct ezbus bus;

  nameThread("XYBus");
  bputs(startup, "startup.");

  //TODO need to make steppers serial port safe on nicc
  if (EZBus_Init(&bus, STAGE_BUS_TTY, "", STAGE_BUS_CHATTER) != EZ_ERR_OK)
    //TODO should EZBus_Init fail be tfatal??
    berror(tfatal, "failed to connect");

  EZBus_Add(&bus, STAGEX_ID, STAGEX_NAME);
  EZBus_Add(&bus, STAGEY_ID, STAGEY_NAME);

  EZBus_SetAccel(&bus, STAGEX_ID, STAGE_BUS_ACCEL);
  EZBus_SetAccel(&bus, STAGEY_ID, STAGE_BUS_ACCEL);
  EZBus_SetIHold(&bus, STAGEX_ID, STAGE_BUS_IHOLD);
  EZBus_SetIHold(&bus, STAGEY_ID, STAGE_BUS_IHOLD);
  EZBus_SetIMove(&bus, STAGEX_ID, STAGE_BUS_IMOVE);
  EZBus_SetIMove(&bus, STAGEY_ID, STAGE_BUS_IMOVE);

  all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      EZBus_ForceRepoll(&bus, STAGEX_ID);
      EZBus_ForceRepoll(&bus, STAGEY_ID);
      EZBus_Recv(&bus); /* this is a blocking call - clear the recv buffer */
      /* no need to sleep -- EZBus_Recv does that for us */
    }

    /* Repoll bus if necessary */
    if (CommandData.xystage.force_repoll) {
      EZBus_ForceRepoll(&bus, STAGEX_ID);
      EZBus_ForceRepoll(&bus, STAGEY_ID);
      poll_timeout = POLL_TIMEOUT;
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      CommandData.xystage.force_repoll = 0;
    }

    if (poll_timeout == 0 && !all_ok) {
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0)
      poll_timeout--;

    ControlXYStage(&bus);

    ReadStage(&bus);

    usleep(10000);
  }
}
#endif
