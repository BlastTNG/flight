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

static struct hwpr_struct {
  int pos;
  int enc;
} hwpr_data;

void MonitorHWPR(struct ezbus *bus)
{
  EZBus_ReadInt(bus, HWPR_ADDR, "?0", &hwpr_data.pos);
  EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
}

//counter incremented in StoreHWPRBus to better time tep_repeat mode
static int hwpr_wait_cnt = 0;

/* Called by frame writer in tx.c */
void StoreHWPRBus(void)
{
  static int firsttime = 1;
  static struct NiosStruct* velHwprAddr;
  static struct NiosStruct* accHwprAddr;
  static struct NiosStruct* iMoveHwprAddr;
  static struct NiosStruct* iHoldHwprAddr;
  static struct NiosStruct* posHwprAddr;
  static struct NiosStruct* encHwprAddr;
  static struct NiosStruct* pos0HwprAddr;
  static struct NiosStruct* pos1HwprAddr;
  static struct NiosStruct* pos2HwprAddr;
  static struct NiosStruct* pos3HwprAddr;
  static struct NiosStruct* overshootHwprAddr;
  static struct NiosStruct* iposHwprAddr;

  if (firsttime)
  {
    firsttime = 0;
    velHwprAddr = GetNiosAddr("vel_hwpr");
    accHwprAddr = GetNiosAddr("acc_hwpr");
    iMoveHwprAddr = GetNiosAddr("i_move_hwpr");
    iHoldHwprAddr = GetNiosAddr("i_hold_hwpr");
    posHwprAddr = GetNiosAddr("pos_hwpr");
    encHwprAddr = GetNiosAddr("enc_hwpr");
    overshootHwprAddr = GetNiosAddr("overshoot_hwpr");
    pos0HwprAddr = GetNiosAddr("pos0_hwpr");
    pos1HwprAddr = GetNiosAddr("pos1_hwpr");
    pos2HwprAddr = GetNiosAddr("pos2_hwpr");
    pos3HwprAddr = GetNiosAddr("pos3_hwpr");
    iposHwprAddr = GetNiosAddr("i_pos_hwpr");
  }

  hwpr_wait_cnt--;

  WriteData(velHwprAddr, CommandData.hwpr.vel, NIOS_QUEUE);
  WriteData(accHwprAddr, CommandData.hwpr.acc, NIOS_QUEUE);
  WriteData(iMoveHwprAddr, CommandData.hwpr.move_i, NIOS_QUEUE);
  WriteData(iHoldHwprAddr, CommandData.hwpr.hold_i, NIOS_QUEUE);
  WriteData(posHwprAddr, hwpr_data.pos, NIOS_QUEUE);
  WriteData(encHwprAddr, hwpr_data.enc, NIOS_FLUSH);
  WriteData(overshootHwprAddr, CommandData.hwpr.overshoot, NIOS_FLUSH);
  WriteData(pos0HwprAddr, CommandData.hwpr.pos[0]*65535, NIOS_FLUSH);
  WriteData(pos1HwprAddr, CommandData.hwpr.pos[1]*65535, NIOS_FLUSH);
  WriteData(pos2HwprAddr, CommandData.hwpr.pos[2]*65535, NIOS_FLUSH);
  WriteData(pos3HwprAddr, CommandData.hwpr.pos[3]*65535, NIOS_FLUSH);
  WriteData(iposHwprAddr, CommandData.hwpr.i_pos, NIOS_FLUSH);
}

void ControlHWPR(struct ezbus *bus)
{
  static int repeat_pos_cnt = 0;
  static int overshooting = 0;
  int overshoot = 0;
  if (CommandData.hwpr.mode == HWPR_PANIC) {
    bputs(info, "Panic");
    EZBus_Stop(bus, HWPR_ADDR);
    CommandData.hwpr.mode = HWPR_SLEEP;
  } else if (CommandData.hwpr.is_new) {
    if ((CommandData.hwpr.mode == HWPR_GOTO)) {
      EZBus_Goto(bus, HWPR_ADDR, CommandData.hwpr.target);
      CommandData.hwpr.mode = HWPR_SLEEP;
    } else if ((CommandData.hwpr.mode == HWPR_JUMP)) {
      EZBus_RelMove(bus, HWPR_ADDR, CommandData.hwpr.target);
      CommandData.hwpr.mode = HWPR_SLEEP;
    } else if ((CommandData.hwpr.mode == HWPR_STEP)) {
      bprintf(info,"ControlHWPR: HWPR step requested.  Once someone actually writes a step mode this will do something awesome.");
    } else if ((CommandData.hwpr.mode == HWPR_REPEAT)) {
      //just received, initialize HWPR_REPEAT variables
      repeat_pos_cnt = CommandData.hwpr.n_pos;
      hwpr_wait_cnt = CommandData.hwpr.step_wait;
    }
    CommandData.hwpr.is_new = 0;
  }

  //repeat mode
  if( CommandData.hwpr.mode == HWPR_REPEAT) {
    if (CommandData.hwpr.step_size > 0) {
      overshoot = CommandData.hwpr.overshoot;
    } else {
      overshoot = (-1)*CommandData.hwpr.overshoot;
    }

    if ( hwpr_wait_cnt <= 0
	     //TODO I don't know why having a shorter wait for the overshoot fails
	     /*|| (overshooting && hwpr_wait_cnt >= 10)*/ ) {
      hwpr_wait_cnt = CommandData.hwpr.step_wait;
      if (overshooting) {
	overshooting = 0;
	EZBus_RelMove(bus, HWPR_ADDR, CommandData.hwpr.overshoot);
      } else if (CommandData.hwpr.repeats-- <= 0) { //done stepping
	CommandData.hwpr.mode = HWPR_SLEEP;
      } else {    //step the HWPR
	if (--repeat_pos_cnt > 0)	 { //move to next step
	  EZBus_RelMove(bus, HWPR_ADDR, CommandData.hwpr.step_size);
	} else {						   //reset to first step
	  repeat_pos_cnt = CommandData.hwpr.n_pos;
	  hwpr_wait_cnt *= CommandData.hwpr.n_pos;  //wait extra long
	  overshooting = 1;
	  EZBus_RelMove(bus, HWPR_ADDR, 
			-CommandData.hwpr.step_size * (CommandData.hwpr.n_pos - 1)
			- CommandData.hwpr.overshoot);
	}
      }
    }
    else if (hwpr_wait_cnt == 10) {
      //pulse the potentiometer
      CommandData.Cryo.hwprPos = 50;
    }
  }
}

void DoHWPR(struct ezbus* bus)
{
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    /* initialize hwpr_data */
    hwpr_data.pos = 0;
    hwpr_data.enc = 0;
  }

  /* update the HWPR move parameters */
  EZBus_SetVel(bus, HWPR_ADDR, CommandData.hwpr.vel);
  EZBus_SetAccel(bus, HWPR_ADDR, CommandData.hwpr.acc);
  EZBus_SetIMove(bus, HWPR_ADDR, CommandData.hwpr.move_i);
  EZBus_SetIHold(bus, HWPR_ADDR, CommandData.hwpr.hold_i);

  ControlHWPR(bus);

  MonitorHWPR(bus);
}
