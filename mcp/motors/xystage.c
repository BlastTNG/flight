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
#include <errno.h>
#include <math.h>

#include "phenom/sysutil.h"

#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"
#include "ezstep.h"
#include "mcp.h"
#include "xystage.h"


/* EZBus setup parameters */
#define STAGE_BUS_TTY "/dev/ttystage"
#define STAGE_BUS_CHATTER EZ_CHAT_BUS
#define STAGEX_NAME "XY Stage X"
#define STAGEY_NAME "XY Stage Y"
#define STAGEX_ID EZ_WHO_S6
#define STAGEY_ID EZ_WHO_S7

#define STAGE_BUS_ACCEL 2
#define STAGE_BUS_IHOLD 20
#define STAGE_BUS_IMOVE 50

#define STAGEXNUM 0
#define STAGEYNUM 1
#define POLL_TIMEOUT 500 /* 15 seconds */

extern int16_t InCharge; /* tx.c */
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

  static channel_t* xStageAddr;
  static channel_t* xLimStageAddr;
  static channel_t* xStpStageAddr;
  static channel_t* xStrStageAddr;
  static channel_t* xVelStageAddr;

  static channel_t* yStageAddr;
  static channel_t* yLimStageAddr;
  static channel_t* yStpStageAddr;
  static channel_t* yStrStageAddr;
  static channel_t* yVelStageAddr;

  if (firsttime) {
    firsttime = 0;

    xStageAddr = channels_find_by_name("x_stage");
    xLimStageAddr = channels_find_by_name("x_lim_stage");
    xStrStageAddr = channels_find_by_name("x_str_stage");
    xStpStageAddr = channels_find_by_name("x_stp_stage");
    xVelStageAddr = channels_find_by_name("x_vel_stage");
    yStageAddr = channels_find_by_name("y_stage");
    yLimStageAddr = channels_find_by_name("y_lim_stage");
    yStrStageAddr = channels_find_by_name("y_str_stage");
    yStpStageAddr = channels_find_by_name("y_stp_stage");
    yVelStageAddr = channels_find_by_name("y_vel_stage");
  }

  SET_VALUE(xStageAddr, stage_data.xpos/2);
  SET_VALUE(yStageAddr, stage_data.ypos/2);
  if (index == 0) {  // writing channels depends on argument in mcp.c
    SET_VALUE(xLimStageAddr, stage_data.xlim);
    SET_VALUE(xStrStageAddr, stage_data.xstr);
    SET_VALUE(xStpStageAddr, stage_data.xstp);
    SET_VALUE(xVelStageAddr, stage_data.xvel);
    SET_VALUE(yStrStageAddr, stage_data.ystr);
    SET_VALUE(yStpStageAddr, stage_data.ystp);
    SET_VALUE(yVelStageAddr, stage_data.yvel);
    SET_VALUE(yLimStageAddr, stage_data.ylim);
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

  blast_info("Move %c to %i at speed %i and wait",
      (is_y) ? 'Y' : 'X', dest, vel);
  EZBus_GotoVel(bus, who, dest, vel);

  do {
    if (CommandData.xystage.mode == XYSTAGE_PANIC)
      return;
    usleep(10000);

//    ReadStage(bus);

    EZBus_ReadInt(bus, STAGEX_ID, "?0", &stage_data.xpos);
    EZBus_ReadInt(bus, STAGEY_ID, "?0", &stage_data.ypos);

    now = (is_y) ? stage_data.ypos : stage_data.xpos;
    } while (now != dest);
}

void Raster(struct ezbus *bus, int start, int end, int is_y, int y,
    int ymin, int ymax, int xvel, int yvel, int ss)
{
  int x;
  int step = (start > end) ? -ss : ss;
  blast_info("Raster %i %i %i\n", start, end, step);
  for (x = start; x != end + step; x += step) {
    if (step < 0) {
      if (x < end)
        x = end;
    } else if (x > end) {
      x = end;
    }
    GoWait(bus, x, (is_y)?yvel:xvel, is_y);
    if (y == ymin)
      y = ymax;
    else
      y = ymin;
    GoWait(bus, y, (!is_y)?yvel:xvel, !is_y);
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
	CommandData.actbus.command[my_cindex]);
    CommandData.actbus.caddr[my_cindex] = 0;
  }

  /* respond to normal movement commands */
  if (CommandData.xystage.is_new) {
    /* PANIC! */
    if (CommandData.xystage.mode == XYSTAGE_PANIC) {
      bputs(warning, "XY Stage Panic");
      EZBus_Stop(bus, STAGEX_ID);
      EZBus_Stop(bus, STAGEY_ID);
      CommandData.xystage.is_new = 0;
    /* GOTO */
    } else if (CommandData.xystage.mode == XYSTAGE_GOTO) {
      if (CommandData.xystage.xvel > 0) {
	blast_info("Move X to %i at speed %i",
	    CommandData.xystage.x1, CommandData.xystage.xvel);
	EZBus_GotoVel(bus, STAGEX_ID, CommandData.xystage.x1,
	    CommandData.xystage.xvel);
      }
      if (CommandData.xystage.yvel > 0) {
	blast_info("Move Y to %i at speed %i",
	    CommandData.xystage.y1, CommandData.xystage.yvel);
	EZBus_GotoVel(bus, STAGEY_ID, CommandData.xystage.y1,
	    CommandData.xystage.yvel);
      }
    /* JUMP */
    } else if (CommandData.xystage.mode == XYSTAGE_JUMP) {
      if (CommandData.xystage.xvel > 0 && CommandData.xystage.x1 != 0) {
	blast_info("Jump X by %i at speed %i",
	    CommandData.xystage.x1, CommandData.xystage.xvel);
	EZBus_RelMoveVel(bus, STAGEX_ID, CommandData.xystage.x1,
	    CommandData.xystage.xvel);
      }
      if (CommandData.xystage.yvel > 0 && CommandData.xystage.y1 != 0) {
	blast_info("Jump Y by %i at speed %i",
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
	int xcent = CommandData.xystage.x1;
	int xsize = CommandData.xystage.x2;
	int ycent = CommandData.xystage.y1;
	int ysize = CommandData.xystage.y2;
	int xvel = CommandData.xystage.xvel;
	int yvel = CommandData.xystage.yvel;
	int step = CommandData.xystage.step;
	GoWait(bus, xcent, xvel, 0);
	GoWait(bus, ycent - ysize, yvel, 1);
	Raster(bus, xcent, xcent + xsize, 0, ycent + ysize, ycent - ysize,
	    ycent + ysize, xvel, yvel, step);
	Raster(bus, ycent + ysize, ycent - ysize, 1, xcent + xsize,
	    xcent - xsize, xcent + xsize, xvel, yvel, step);
	Raster(bus, xcent - xsize, xcent, 0, ycent - ysize, ycent - ysize,
	    ycent + ysize, xvel, yvel, step);
	GoWait(bus, xcent, xvel, 0);
	GoWait(bus, ycent, yvel, 1);
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
  unsigned conn_attempt = 1;
  int first_time = 1;
  struct ezbus bus;
  int chat_temp;

  ph_library_init();
  nameThread("XYBus");
  bputs(startup, "startup.");
  while (!InCharge) {
    if (first_time) {
      blast_info("Not in charge.  Waiting.");
      first_time = 0;
    }
    usleep(500000);
  }
  while (1) {
    if (EZBus_Init(&bus, STAGE_BUS_TTY, "", STAGE_BUS_CHATTER) == EZ_ERR_OK) {
      blast_info("Connected to port %d on attempt %u.", STAGE_BUS_TTY, conn_attempt);
      break;
    }
    conn_attempt++;
    sleep(2);
  }
  CommandData.xystage.mode = XYSTAGE_PANIC; // make sure no command executes at startup
  EZBus_Add(&bus, STAGEX_ID, STAGEX_NAME);
  EZBus_Add(&bus, STAGEY_ID, STAGEY_NAME);

  EZBus_Stop(&bus, STAGEX_ID);
  EZBus_Stop(&bus, STAGEY_ID);
  EZBus_SetAccel(&bus, STAGEX_ID, STAGE_BUS_ACCEL);
  EZBus_SetAccel(&bus, STAGEY_ID, STAGE_BUS_ACCEL);
  EZBus_SetIHold(&bus, STAGEX_ID, STAGE_BUS_IHOLD);
  EZBus_SetIHold(&bus, STAGEY_ID, STAGE_BUS_IHOLD);
  EZBus_SetIMove(&bus, STAGEX_ID, STAGE_BUS_IMOVE);
  EZBus_SetIMove(&bus, STAGEY_ID, STAGE_BUS_IMOVE);

  EZBus_SetPreamble(&bus, STAGEX_ID, XYSTAGE_PREAMBLE);
  EZBus_SetPreamble(&bus, STAGEY_ID, XYSTAGE_PREAMBLE);

  EZBus_Comm(&bus, STAGEX_ID, "z10000R"); // Wake up at position 10000 so we have space to move in either direction.
  EZBus_Comm(&bus, STAGEY_ID, "z10000R"); // May want to use home command to re-zero on limit switches.
  all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);

  for (;;) {
    /* LMF: I'm pretty sure this next while loop should be removed*/
    /* Otherwise it will attempt to open the serial port for the XY stage even if it is not in charge*/
    /* Anyway I've added a while (!Incharge) loop above that should make this redundant */
    while (!InCharge) { /* NiC MCC traps here */
      EZBus_ForceRepoll(&bus, STAGEX_ID);
      EZBus_ForceRepoll(&bus, STAGEY_ID);
      EZBus_Recv(&bus); /* this is a blocking call - clear the recv buffer */
      /* no need to sleep -- EZBus_Recv does that for us */
    }

    /* Repoll bus if necessary */
    if (CommandData.xystage.force_repoll) {
      blast_info("XYBus: repolling");
      EZBus_ForceRepoll(&bus, STAGEX_ID);
      EZBus_ForceRepoll(&bus, STAGEY_ID);
      poll_timeout = POLL_TIMEOUT;
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      blast_info("all_ok = %i", all_ok);
      CommandData.xystage.force_repoll = 0;
    }

    if (poll_timeout == 0 && !all_ok) {
      chat_temp = bus.chatter;
      // bus.chatter = EZ_CHAT_NONE;
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      bus.chatter = chat_temp;
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0) {
      poll_timeout--;
    }

    ControlXYStage(&bus);
    ReadStage(&bus);

    usleep(10000);
  }
}
