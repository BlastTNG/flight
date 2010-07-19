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
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */

#define HWPR_READ_WAIT 5

static struct hwpr_struct {
  int pos;
  int enc;
  double pot;
} hwpr_data;

enum move_type {none,enc,pot,step};
enum move_status {not_yet,moving,at_overshoot,is_done};
enum read_pot {yes,no,reading,done};

static struct hwpr_control_struct {
  enum move_type go;
  enum move_status move_cur;
  enum read_pot read_before;
  enum read_pot read_after;
  int read_wait_cnt;
  int done_move;
  int done_all;
  int rel_move;

} hwpr_control;

void MonitorHWPR(struct ezbus *bus)
{
  EZBus_ReadInt(bus, HWPR_ADDR, "?0", &hwpr_data.pos);
  EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
  hwpr_data.pot = ((double) ACSData.hwpr_pot)/65535.0;
}

 
/* Clear out the hwpr_control structure*/
void ResetControlHWPR (void) {
  hwpr_control.go = none;
  hwpr_control.move_cur = not_yet;
  hwpr_control.read_before = no;
  hwpr_control.read_after = no;
  hwpr_control.read_wait_cnt = 0;
  hwpr_control.done_move = 0;
  hwpr_control.done_all = 0;
  hwpr_control.rel_move = 0;

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
  static struct NiosStruct* readWaitHwprAddr;

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
    readWaitHwprAddr = GetNiosAddr("read_wait_hwpr");
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
  WriteData(readWaitHwprAddr, hwpr_control.read_wait_cnt, NIOS_FLUSH);
}

void ControlHWPR(struct ezbus *bus)
{
  static int repeat_pos_cnt = 0;
  static int overshooting = 0;
  static int first_time = 1;
  int overshoot = 0;

  if (first_time) {

    /* Initialize hwpr_controls */
    ResetControlHWPR();

    first_time = 0;

  }

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
      bprintf(info,"ControlHWPR: HWPR step requested.  Someone is in the process of writing this code and someday this will do something awesome.");
      if(!CommandData.hwpr.no_step) {
	ResetControlHWPR();
	hwpr_control.go = step;
	hwpr_control.move_cur = not_yet;
	hwpr_control.read_before = yes;
	hwpr_control.read_after = yes;
      } else {
	bprintf(warning,"Cannot step half wave plate.  hwpr_step_off is set.");
	CommandData.hwpr.mode = HWPR_SLEEP;
      }
    } else if ((CommandData.hwpr.mode == HWPR_REPEAT)) {
      //just received, initialize HWPR_REPEAT variables
      repeat_pos_cnt = CommandData.hwpr.n_pos;
      hwpr_wait_cnt = CommandData.hwpr.step_wait;
    } else if (CommandData.hwpr.mode == HWPR_GOTO_I) {
    }
    CommandData.hwpr.is_new = 0;
  }

  /*** Begin fall through control ***/
  
  /* if are doing anything with the HWPR other than sleeping, panicing or repeating */
  /* TODO: Eventually we'll want to rewrite repeat mode so that it can be incorporated in to the fall through code */ 
  if(CommandData.hwpr.mode >= HWPR_GOTO && CommandData.hwpr.mode != HWPR_REPEAT) {
    
    /* Do we want a pot reading first? Should always be yes for step mode.*/
    if (hwpr_control.read_before == yes) {
 
      /* pulse the potentiometer */
      bprintf(info,"Pulsing the pot before we do anything else...");
      CommandData.Cryo.hwprPos = 50;
      hwpr_control.read_before = reading;
      hwpr_control.read_wait_cnt = HWPR_READ_WAIT;

    } else if (hwpr_control.read_before == reading) { // we are reading...

      hwpr_control.read_wait_cnt--;
      if (hwpr_control.read_wait_cnt <= 0) 
	hwpr_control.read_before = done;
      
    } else { // we are either done or we don't want a reading

      /*** Do we want to move? ***/

      if(hwpr_control.go != none ) { // yes, move

	/*** Step Mode ***/
	if(hwpr_control.go == step && hwpr_control.done_move == 0) {

	  if(hwpr_control.move_cur == not_yet) {  // we haven't yet started a move...
	    bprintf(info,"We haven't yet started to move!");

	    if(((hwpr_data.pot > HWPR_POT_MIN) ||
		(hwpr_data.pot < HWPR_POT_MAX)) &&
	       (CommandData.hwpr.use_pot)) { // use pot

	      /* calculate rel move from pot lut*/
	      bprintf(info,"This is where I will calculate the relative step from the pot value.");
	      hwpr_control.rel_move = 420;
              CommandData.hwpr.mode = HWPR_SLEEP;
	      return;
	    } else { // don't use pot

	      /* assume rel move of ~22.5 degrees*/
	      hwpr_control.rel_move = HWPR_DEFAULT_STEP;
	      bprintf(info,"The pot is dead! Use default HWPR step of %i",hwpr_control.rel_move);
	      CommandData.hwpr.mode = HWPR_SLEEP;
	      return;
	    }
	  }
	} else if (hwpr_control.go == pot) {
	  bprintf(info,"Go to pot position code isn't written yet.  Sorry.");
	  CommandData.hwpr.mode = HWPR_SLEEP;
	  return;  // break out of loop!         
	} else if (hwpr_control.go == enc) {
	  bprintf(info,"Go to encoder position code isn't written yet. Sorry");
	  CommandData.hwpr.mode = HWPR_SLEEP; 
	  return; // break out of loop!
	} else {
	  bprintf(info,"This state should be impossible.");
	  CommandData.hwpr.mode = HWPR_SLEEP; 
	  return; // break out of loop!
	}
	
      } else { // ...we're done!
        bprintf(info,"Nothing left to do!");
	hwpr_control.done_all = 1;
	CommandData.hwpr.mode = HWPR_SLEEP; 
        return;
      }
      
      
    }
  }

  /*** end fall through control ***/

  //repeat mode - TODO LMF: steve wrote this.  May want to incorporate it into my code later. 
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
