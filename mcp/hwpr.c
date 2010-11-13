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
#include "lut.h"
#include "ezstep.h"
#include "command_struct.h"
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */

//#define DEBUG_HWPR

#define HWPR_READ_WAIT 10
#define HWPR_MOVE_TIMEOUT 3 

static struct hwpr_struct {
  int pos;
  int enc;
  double pot;
} hwpr_data;

enum move_type {none=0,pot,ind,step};
enum move_status {not_yet=0,ready,moving,at_overshoot,is_done};
enum read_pot {no=0,yes,reading,done};

static struct hwpr_control_struct {
  enum move_type go;
  enum move_status move_cur;
  enum read_pot read_before;
  enum read_pot read_after;
  int read_wait_cnt; // Added
  int done_move;
  int done_all;
  int rel_move; //Added
  int i_next_step; //Added
  int do_overshoot;
  int stop_cnt; //Added

} hwpr_control;

void MonitorHWPR(struct ezbus *bus)
{

  EZBus_ReadInt(bus, HWPR_ADDR, "?0", &hwpr_data.pos);
  EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
  //TODO why is pot acquired from acsdata? it's in das.c
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
  hwpr_control.do_overshoot = 0;
  hwpr_control.stop_cnt = 0;
}


//counter incremented in StoreHWPRBus to better time tep_repeat mode
static int hwpr_wait_cnt = 0;

/* Called by frame writer in tx.c */
void StoreHWPRBus(void)
{
  int hwpr_stat_field = 0;
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
  static struct NiosStruct* iposRqHwprAddr;
  static struct NiosStruct* iposHwprAddr;
  static struct NiosStruct* readWaitHwprAddr;
  static struct NiosStruct* stopCntHwprAddr;
  static struct NiosStruct* relMoveHwprAddr;
  static struct NiosStruct* statControlHwprAddr;
  static struct NiosStruct* potTargHwprAddr;

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
    iposRqHwprAddr = GetNiosAddr("i_pos_rq_hwpr");
    iposHwprAddr = GetNiosAddr("i_pos_hwpr");
    readWaitHwprAddr = GetNiosAddr("read_wait_hwpr");
    stopCntHwprAddr = GetNiosAddr("stop_cnt_hwpr");
    relMoveHwprAddr = GetNiosAddr("rel_move_hwpr");
    statControlHwprAddr = GetNiosAddr("stat_control_hwpr");
    potTargHwprAddr = GetNiosAddr("pot_targ_hwpr");
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
  WriteData(iposRqHwprAddr, CommandData.hwpr.i_pos, NIOS_FLUSH);
  WriteData(potTargHwprAddr, CommandData.hwpr.pot_targ, NIOS_FLUSH);
  WriteData(iposHwprAddr, hwpr_control.i_next_step, NIOS_FLUSH);
  WriteData(readWaitHwprAddr, hwpr_control.read_wait_cnt, NIOS_FLUSH);
  WriteData(stopCntHwprAddr, hwpr_control.stop_cnt, NIOS_FLUSH);
  WriteData(relMoveHwprAddr, hwpr_control.rel_move/2, NIOS_FLUSH);

  /* Make HWPR status bit field */
  hwpr_stat_field |= (hwpr_control.go) & 0x0007 ;
  hwpr_stat_field |= ((hwpr_control.move_cur) & 0x0007)<<3 ;
  hwpr_stat_field |= ((hwpr_control.read_before) & 0x0003)<<6 ;
  hwpr_stat_field |= ((hwpr_control.read_after) & 0x0003)<<8 ;
  hwpr_stat_field |= ((hwpr_control.do_overshoot) & 0x0001)<<10 ;
  hwpr_stat_field |= ((hwpr_control.done_move) & 0x0001)<<11 ;
  hwpr_stat_field |= ((hwpr_control.done_all) & 0x0001)<<12 ;

  WriteData(statControlHwprAddr, hwpr_stat_field, NIOS_FLUSH);

}

// From the current potentiometer reading. Figure out the nearest
// hwpr step position, and return the index
int GetHWPRi(double pot_val)
{
  int i; 
  int i_min = 0;
  double d_pot; 
  double d_pot_min = 2.0;

  for(i=0;i<=3;i++) {
    d_pot = fabs(pot_val-CommandData.hwpr.pos[i]);
    if(d_pot < d_pot_min) {
      d_pot_min = d_pot;
      i_min = i;
    }
    //    bprintf(info,"GetHWPRi: i=%i,d_pot=%f,pot_val=%f,CommandData.hwpr.pos[i]=%f,d_pot_min=%f,i_min=%i",i,d_pot,pot_val,CommandData.hwpr.pos[i],d_pot_min,i_min);
  }

#ifdef DEBUG_HWPR
  bprintf(info,"GetHWPRi: Returning %i",i_min);
#endif
  // i is the closest index to where we are.  return the next index (i+1)%4
  return i_min;
}

void ControlHWPR(struct ezbus *bus)
{
  static int repeat_pos_cnt = 0;
  static int overshooting = 0;
  static int first_time = 1;
  static int last_enc = 0;

  int hwpr_enc_cur, hwpr_enc_dest;
  int i_step, i_next_step; // index of the current step

  int overshoot = 0;

  static struct LutType HwprPotLut = {"/data/etc/hwpr_pot.lut", 0, NULL, NULL, 0};

  if (HwprPotLut.n == 0)
    LutInit(&HwprPotLut);

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
    } else if ((CommandData.hwpr.mode == HWPR_GOTO_I)) {
      bprintf(info,"ControlHWPR: Attempting to go to HWPR position %i",CommandData.hwpr.i_pos);
      ResetControlHWPR();
      hwpr_control.go = ind;
      hwpr_control.move_cur = not_yet;
      hwpr_control.read_before = yes;
      hwpr_control.read_after = yes;
    } else if (CommandData.hwpr.mode == HWPR_GOTO_POT) {
      bprintf(info,"ControlHWPR: Attempting to go to HWPR potentiometer position %f",CommandData.hwpr.pot_targ);
      ResetControlHWPR();
      hwpr_control.go = pot;
      hwpr_control.move_cur = not_yet;
      hwpr_control.read_before = yes;
      hwpr_control.read_after = yes;
    } else if ((CommandData.hwpr.mode == HWPR_STEP)) {
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

#ifdef DEBUG_HWPR
      bprintf(info,"Waiting : CommandData.Cryo.hwprPos = %i",CommandData.Cryo.hwprPos );
#endif
      if (CommandData.Cryo.hwprPos <= 0) 
	hwpr_control.read_before = done;
      
    } else { // we are either done or we don't want a reading

      /*** Do we want to move? ***/

      if (hwpr_control.go != none) { // yes, move

	/*** Figure out how many encoder counts we want to step ***/
	if (hwpr_control.move_cur == not_yet) {
	  if (hwpr_control.go == step) {

	    
	    if (((hwpr_data.pot > HWPR_POT_MIN) &&
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot
	      
	      /* calculate rel move from pot lut*/
	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);

	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
	      
	      /*get index of the closest hwpr_step position, and find the encoder position for the next step pos*/

	      i_step = GetHWPRi(hwpr_data.pot);
	      i_next_step = (i_step +1)%4;
	      hwpr_control.i_next_step = i_next_step;
	      hwpr_enc_dest = LutCal(&HwprPotLut, CommandData.hwpr.pos[i_next_step]);
	      
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
	      
#ifdef DEBUG_HWPR
	      bprintf(info,"Nearest step position: i = %i, encoder_lut = %i, pot = %f",i_step,hwpr_enc_cur,hwpr_data.pot);
	      bprintf(info,"Destination step position: i = %i, encoder_lut = %i, pot = %f",hwpr_control.i_next_step,hwpr_enc_dest,CommandData.hwpr.pos[i_next_step]);
#endif
	    } else { // don't use pot
	      
	      /* assume rel move of ~22.5 degrees*/
	      //TODO pot brokenness should be written the the frame (in bitfield)
	      hwpr_control.rel_move = HWPR_DEFAULT_STEP;
	      bprintf(info,"The pot is dead! Use default HWPR step of %i",hwpr_control.rel_move);
	    }
	    
	    hwpr_control.move_cur = ready; 


	  } else if (hwpr_control.go == ind) {
	    //TODO I don't like repeated code, and this chunk is VERY similar to the above one
	    //TODO can set i_next_step depending on mode, and then have common code shared
	    //TODO or can make a function that calculates move to any new index
	    if (((hwpr_data.pot > HWPR_POT_MIN) ||  //TODO why is this OR?
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot

	      bprintf(info,"This is where I calculate the relative step from the pot value.");
	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);
	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
             
              i_next_step = CommandData.hwpr.i_pos;
	      hwpr_enc_dest = LutCal(&HwprPotLut, CommandData.hwpr.pos[i_next_step]);
	      
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
	      //	      bprintf(info,"Destination is index %i, pot value = %f, required rel encoder move is %i:",CommandData.hwpr.i_pos,CommandData.hwpr.pos[i_next_step],hwpr_control.rel_move);
	    } else { // don't use pot
	      
	      /* can't step to a hwp position, because we don't know where it is */
	      hwpr_control.rel_move = 0;
	      bprintf(warning, "The pot is dead! Don't know where to move. rel_move = %i", hwpr_control.rel_move);
	      CommandData.hwpr.mode = HWPR_SLEEP;
	      return;
	    }

	    hwpr_control.move_cur = ready; 

	  } else if (hwpr_control.go == pot) {
	    //TODO as above, if you're sufficiently crafty, much of this code could be shared with the above two modes
	    if (((hwpr_data.pot > HWPR_POT_MIN) ||  //TODO why is this OR?
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot

	      bprintf(info,"This is where I calculate the relative step from the pot value.");
	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);
	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
             
	      hwpr_enc_dest = LutCal(&HwprPotLut, CommandData.hwpr.pot_targ);
	      
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
	      //	      bprintf(info,"Destination is index %i, pot value = %f, required rel encoder move is %i:",CommandData.hwpr.i_pos,CommandData.hwpr.pos[i_next_step],hwpr_control.rel_move);
	    } else { // don't use pot
	      
	      /* can't step to a hwp position, because we don't know where it is */
	      bprintf(warning, "The pot is dead! Don't know where to move.");
	      CommandData.hwpr.mode = HWPR_SLEEP;
	      return;
	    }

	    hwpr_control.move_cur = ready; 

	  } else {
	    bprintf(info,"This state should be impossible.");
	    CommandData.hwpr.mode = HWPR_SLEEP; 
	    return; // break out of loop!
	  }

	  /*** Once we are ready to move send ActBus Command ***/
	  //TODO putting this in an else means it won't execute until next time through all this. Why not now?
	} else if (hwpr_control.move_cur == ready) {

	  /* Is the hwpr move negative?  
	     If so check if we need to add an overshoot for backlash correction */
	  if (hwpr_control.rel_move < 0) {
	    if (CommandData.hwpr.overshoot > 0) {
	      hwpr_control.rel_move-= CommandData.hwpr.overshoot;
	      hwpr_control.do_overshoot = 1;
	      bprintf(info,"ControlHWPR: Overshoot of %i requested.", CommandData.hwpr.overshoot);
	    }
	  } else if (hwpr_control.rel_move == 0) {
	    bprintf(info,"ControlHWPR: Requested a move of 0.  Ignoring.");
            hwpr_control.done_move = 1;
            hwpr_control.move_cur = done;
	  } 
	  bprintf(info,"ControlHWPR: Here's where I will send a relative move command of %i",hwpr_control.rel_move);
	  EZBus_RelMove(bus, HWPR_ADDR, hwpr_control.rel_move);
	  hwpr_control.move_cur = moving;
	  hwpr_control.stop_cnt = 0;

	  /*** We are moving.  Wait until we are done. ***/
	} else if (hwpr_control.move_cur == moving) {

	  if (hwpr_data.enc == last_enc) {
	    hwpr_control.stop_cnt ++;
	  } else {
	    hwpr_control.stop_cnt = 0 ;
	  }

	  //	  bprintf(info,"ControlHWPR: We are moving! hwpr_data.enc = %i, last_enc = %i", hwpr_data.enc, last_enc);

	  //TODO, you have to be careful with this, sometimes mcp gets into a state where the encoders aren't read out for a while (esp on palestine)
	  //TODO though maybe it's okay since such cases probably cause this whole thread to not execute
	  if (hwpr_control.stop_cnt >=HWPR_MOVE_TIMEOUT) {
	    bprintf(info,"We've stopped!");

	    if(hwpr_control.do_overshoot) {
	      hwpr_control.move_cur = at_overshoot;
	    } else { // we're done moving
	      hwpr_control.move_cur = is_done;
	      bprintf(info,"We're done moving!");
	    }
	    
	  }

	  last_enc = hwpr_data.enc;

	} else if (hwpr_control.move_cur == at_overshoot) {
	    bprintf(info,"At the overshoot.");

	    hwpr_control.rel_move = CommandData.hwpr.overshoot;
	    bprintf(info,"ControlHWPR: Sending overshoot move command of %i",hwpr_control.rel_move);
	    EZBus_RelMove(bus, HWPR_ADDR, hwpr_control.rel_move);
	    hwpr_control.move_cur = moving;
	    hwpr_control.stop_cnt = 0;
	    hwpr_control.do_overshoot = 0;
	} else if (hwpr_control.move_cur == is_done) {

	    /* Do we want to read the pot?*/
	    if (hwpr_control.read_after == yes) {
	      
	      /* pulse the potentiometer */
	      bprintf(info,"Pulsing the pot...");
	      CommandData.Cryo.hwprPos = 50;
	      hwpr_control.read_after = reading;
	      hwpr_control.read_wait_cnt = HWPR_READ_WAIT;
	      
	    } else if (hwpr_control.read_after == reading) { // we are reading...
	      
	      //TODO why don't you just check CommandData.Cryo.hwprPos?
	      hwpr_control.read_wait_cnt--;
	      if (hwpr_control.read_wait_cnt <= 0) 
		hwpr_control.read_after = done;
      
	    } else {
	      hwpr_control.done_all = 1;
	    }
	}

      } else { // hwpr_control.go == none
        // ...we're done!
        bprintf(info,"Nothing left to do!");
	hwpr_control.done_all = 1;
      }
      
    }
    if(hwpr_control.done_all == 1) {
      bprintf(info,"ControlHWPR: HWPR command complete");
      CommandData.hwpr.mode = HWPR_SLEEP; 
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
    hwpr_control.i_next_step = 0;
 }

  /* update the HWPR move parameters */
  EZBus_SetVel(bus, HWPR_ADDR, CommandData.hwpr.vel);
  EZBus_SetAccel(bus, HWPR_ADDR, CommandData.hwpr.acc);
  EZBus_SetIMove(bus, HWPR_ADDR, CommandData.hwpr.move_i);
  EZBus_SetIHold(bus, HWPR_ADDR, CommandData.hwpr.hold_i);

  ControlHWPR(bus);

  MonitorHWPR(bus);
}
