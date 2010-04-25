/* mcp: hwpr: part of the BLAST master control program
 *
 * This software is copyright (C) 2010 Matthew Truch
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

#include <unistd.h>
#include <stdlib.h>

#include "hwpr.h"
#include "ezstep.h"
#include "command_struct.h"
#include "tx.h" /* InCharge */

/* Define this symbol to have mcp log all hwpr bus traffic */
#define HWPRBUS_CHATTER EZ_CHAT_ACT

#define HWPR_BUS "/dev/ttyHWPR"
#define HWPR_ADDR EZ_WHO_S13
#define POLL_TIMEOUT 30000 /* 5 minutes */

#define GP_LEN 255

void nameThread(const char*);	/* mcp.c */

static struct hwpr_struct {
  int pos;
  int enc;
  int vel;
  int acc;
  int move;
  int hold;
} hwpr_data;

void MonitorHWPR(struct ezbus *bus)
{
  EZBus_ReadInt(bus, HWPR_ADDR, "?0", &hwpr_data.pos);
  hwpr_data.pos /= HWPR_STEPS_PER_MOTENC;
  EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
#if 0
  EZBus_ReadInt(bus, HWPR_ADDR, "?V", hwpr_data.vel);
  EZBus_ReadInt(bus, HWPR_ADDR, "?L", hwpr_data.acc);
  EZBus_ReadInt(bus, HWPR_ADDR, "?m", hwpr_data.move);
  EZBus_ReadInt(bus, HWPR_ADDR, "?h", hwpr_data.hold);
#endif
}

/* Called by frame writer in tx.c */
void StoreHWPRBus(void)
{
  static int firsttime = 1;
  static struct NiosStruct* hwprVelAddr;
  static struct NiosStruct* hwprAccAddr;
  static struct NiosStruct* hwprMoveIAddr;
  static struct NiosStruct* hwprHoldIAddr;
  static struct NiosStruct* hwprPosAddr;
  static struct NiosStruct* hwprEncAddr;

  if (firsttime)
  {
    firsttime = 0;
    hwprVelAddr = GetNiosAddr("hwpr_vel");
    hwprAccAddr = GetNiosAddr("hwpr_acc");
    hwprMoveIAddr = GetNiosAddr("hwpr_move_i");
    hwprHoldIAddr = GetNiosAddr("hwpr_hold_i");
    hwprPosAddr = GetNiosAddr("hwpr_pos");
    hwprEncAddr = GetNiosAddr("hwpr_enc");
  }

  WriteData(hwprVelAddr, CommandData.hwpr.vel / 100, NIOS_QUEUE);
  WriteData(hwprAccAddr, CommandData.hwpr.acc, NIOS_QUEUE);
  WriteData(hwprMoveIAddr, CommandData.hwpr.move_i, NIOS_QUEUE);
  WriteData(hwprHoldIAddr, CommandData.hwpr.hold_i, NIOS_QUEUE);
  WriteData(hwprPosAddr, hwpr_data.pos, NIOS_QUEUE);
  WriteData(hwprEncAddr, hwpr_data.enc, NIOS_FLUSH);
}

void ControlHWPR(struct ezbus *bus)
{
  int my_cindex = 0;
  char gp_buffer[GP_LEN+1];
  gp_buffer[GP_LEN] = '\0';

  /* Send the uplinked (general) command, if any */
  my_cindex = GETREADINDEX(CommandData.actbus.cindex);
  if ((CommandData.actbus.caddr[my_cindex] == HWPR_ADDR)) {
    EZBus_Comm(bus, CommandData.actbus.caddr[my_cindex],
        CommandData.actbus.command[my_cindex], 0);
    CommandData.actbus.caddr[my_cindex] = 0;
  }

  if (CommandData.hwpr.is_new) {
    if (CommandData.hwpr.mode == HWPR_PANIC) {
      bputs(info, "Panic");
      EZBus_Stop(bus, HWPR_ADDR);
    } else if ((CommandData.hwpr.mode == HWPR_GOTO)) {
      EZBus_Goto(bus, HWPR_ADDR, 
	  CommandData.hwpr.target * HWPR_STEPS_PER_MOTENC);
      CommandData.hwpr.is_new = 0;
    } else if ((CommandData.hwpr.mode == HWPR_JUMP)) {
      EZBus_RelMove(bus, HWPR_ADDR, 
	  CommandData.hwpr.target * HWPR_STEPS_PER_MOTENC);
      CommandData.hwpr.is_new = 0;
    }
  }
}


extern short int InCharge; /* tx.c */

void HWPRBus(void)
{
  int poll_timeout = POLL_TIMEOUT;
  int all_ok;
  struct ezbus bus;

  nameThread("HWPR");
  bputs(startup, "HWPRBus startup.");

  if (EZBus_Init(&bus, HWPR_BUS, "", HWPRBUS_CHATTER) != EZ_ERR_OK)
    berror(tfatal, "failed to connect");

  EZBus_Add(&bus, HWPR_ADDR, "HWPR");

  all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);

  /* initialize hwpr_data */
  hwpr_data.pos = 0;
  hwpr_data.enc = 0;

  for (;;) {
    while (!InCharge) { /* NiC MCC traps here */
      CommandData.hwpr.force_repoll = 1; /* repoll bus as soon as gaining
                                              control */
      EZBus_Recv(&bus); /* this is a blocking call - clear the recv buffer */
      /* no need to sleep -- EZBus_Recv does that for us */
    }

    if (CommandData.hwpr.force_repoll) {
      EZBus_ForceRepoll(&bus, HWPR_ADDR);
      poll_timeout = POLL_TIMEOUT;
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      CommandData.hwpr.force_repoll = 0;
    }

    if (poll_timeout == 0 && !all_ok) {
      all_ok = !(EZBus_Poll(&bus) & EZ_ERR_POLL);
      poll_timeout = POLL_TIMEOUT;
    } else if (poll_timeout > 0)
      poll_timeout--;

    /* update the HWPR move parameters */
    EZBus_SetVel(&bus, HWPR_ADDR, CommandData.hwpr.vel);
    EZBus_SetAccel(&bus, HWPR_ADDR, CommandData.hwpr.acc);
    EZBus_SetIMove(&bus, HWPR_ADDR, CommandData.hwpr.move_i);
    EZBus_SetIHold(&bus, HWPR_ADDR, CommandData.hwpr.hold_i);

    ControlHWPR(&bus);

    MonitorHWPR(&bus);

    usleep(10000);
  }

}
