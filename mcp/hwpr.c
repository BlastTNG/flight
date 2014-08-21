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

#define DEBUG_HWPR

#define HWPR_READ_WAIT 10
#define HWPR_MOVE_TIMEOUT 3 

static struct hwpr_struct {
  int pos;
  int enc;
  double pot;
} hwpr_data;

enum move_type {no_move=0,pot,ind,step};
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
  int enc_targ; //Added
  int enc_err; //Added
  double pot_targ; //Added
  double pot_err; //Added
  int dead_pot; //Added
  int do_calpulse; 
  int reset_enc; 
} hwpr_control;

int hwpr_calpulse_flag = 0;

void MonitorHWPR(struct ezbus *bus)
{

  EZBus_ReadInt(bus, HWPR_ADDR, "?0", &hwpr_data.pos);
  EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
  hwpr_data.pot = ((double) ACSData.hwpr_pot)/65535.0;
}

 
/* Clear out the hwpr_control structure*/
void ResetControlHWPR (void) {
  hwpr_control.go = no_move;
  hwpr_control.move_cur = not_yet;
  hwpr_control.read_before = no;
  hwpr_control.read_after = no;
  hwpr_control.read_wait_cnt = 0;
  hwpr_control.done_move = 0;
  hwpr_control.done_all = 0;
  hwpr_control.rel_move = 0;
  hwpr_control.do_overshoot = 0;
  hwpr_control.do_calpulse = no;
  hwpr_control.stop_cnt = 0;
  hwpr_control.enc_targ = 0;
  hwpr_control.enc_err = 0;
  hwpr_control.pot_targ = 0;
  hwpr_control.pot_err = 0;
  hwpr_control.reset_enc = 0;
}


//counter incremented in StoreHWPRBus to better time tep_repeat mode
static int hwpr_wait_cnt = 0;

/* Called by frame writer in tx.c */
void StoreHWPRBus(void)
{
  int hwpr_stat_field = 0;
  static int firsttime = 1;
  static channel_t* velHwprAddr;
  static channel_t* accHwprAddr;
  static channel_t* iMoveHwprAddr;
  static channel_t* iHoldHwprAddr;
  static channel_t* posHwprAddr;
  static channel_t* encHwprAddr;
  static channel_t* pos0HwprAddr;
  static channel_t* pos1HwprAddr;
  static channel_t* pos2HwprAddr;
  static channel_t* pos3HwprAddr;
  static channel_t* overshootHwprAddr;
  static channel_t* iposRqHwprAddr;
  static channel_t* iposHwprAddr;
  static channel_t* readWaitHwprAddr;
  static channel_t* stopCntHwprAddr;
  static channel_t* relMoveHwprAddr;
  static channel_t* statControlHwprAddr;
  static channel_t* potTargHwprAddr;
  static channel_t* encTargHwprAddr;
  static channel_t* encErrHwprAddr;
  static channel_t* potErrHwprAddr;

  if (firsttime)
  {
    firsttime = 0;
    velHwprAddr = channels_find_by_name("vel_hwpr");
    accHwprAddr = channels_find_by_name("acc_hwpr");
    iMoveHwprAddr = channels_find_by_name("i_move_hwpr");
    iHoldHwprAddr = channels_find_by_name("i_hold_hwpr");
    posHwprAddr = channels_find_by_name("pos_hwpr");
    encHwprAddr = channels_find_by_name("enc_hwpr");
    overshootHwprAddr = channels_find_by_name("overshoot_hwpr");
    pos0HwprAddr = channels_find_by_name("pos0_hwpr");
    pos1HwprAddr = channels_find_by_name("pos1_hwpr");
    pos2HwprAddr = channels_find_by_name("pos2_hwpr");
    pos3HwprAddr = channels_find_by_name("pos3_hwpr");
    iposRqHwprAddr = channels_find_by_name("i_pos_rq_hwpr");
    iposHwprAddr = channels_find_by_name("i_pos_hwpr");
    readWaitHwprAddr = channels_find_by_name("read_wait_hwpr");
    stopCntHwprAddr = channels_find_by_name("stop_cnt_hwpr");
    relMoveHwprAddr = channels_find_by_name("rel_move_hwpr");
    statControlHwprAddr = channels_find_by_name("stat_control_hwpr");
    potTargHwprAddr = channels_find_by_name("pot_targ_hwpr");
    encTargHwprAddr = channels_find_by_name("enc_targ_hwpr");
    encErrHwprAddr = channels_find_by_name("enc_err_hwpr");
    potErrHwprAddr = channels_find_by_name("pot_err_hwpr");
  }

  hwpr_wait_cnt--;

  SET_VALUE(velHwprAddr, CommandData.hwpr.vel);
  SET_VALUE(accHwprAddr, CommandData.hwpr.acc);
  SET_VALUE(iMoveHwprAddr, CommandData.hwpr.move_i);
  SET_VALUE(iHoldHwprAddr, CommandData.hwpr.hold_i);
  SET_VALUE(posHwprAddr, hwpr_data.pos);
  SET_VALUE(encHwprAddr, hwpr_data.enc);
  SET_VALUE(overshootHwprAddr, CommandData.hwpr.overshoot);
  SET_VALUE(pos0HwprAddr, CommandData.hwpr.pos[0]*65535);
  SET_VALUE(pos1HwprAddr, CommandData.hwpr.pos[1]*65535);
  SET_VALUE(pos2HwprAddr, CommandData.hwpr.pos[2]*65535);
  SET_VALUE(pos3HwprAddr, CommandData.hwpr.pos[3]*65535);
  SET_VALUE(iposRqHwprAddr, CommandData.hwpr.i_pos);
  SET_VALUE(potTargHwprAddr, hwpr_control.pot_targ*65535);
  SET_VALUE(iposHwprAddr, hwpr_control.i_next_step);
  SET_VALUE(readWaitHwprAddr, hwpr_control.read_wait_cnt);
  SET_VALUE(stopCntHwprAddr, hwpr_control.stop_cnt);
  SET_VALUE(relMoveHwprAddr, hwpr_control.rel_move/2);
  SET_VALUE(encTargHwprAddr, hwpr_control.enc_targ);
  SET_VALUE(encErrHwprAddr, hwpr_control.enc_err);
  SET_VALUE(potErrHwprAddr, hwpr_control.pot_err*32767);

  /* Make HWPR status bit field */
  hwpr_stat_field |= (hwpr_control.go) & 0x0007 ;
  hwpr_stat_field |= ((hwpr_control.move_cur) & 0x0007)<<3 ;
  hwpr_stat_field |= ((hwpr_control.read_before) & 0x0003)<<6 ;
  hwpr_stat_field |= ((hwpr_control.read_after) & 0x0003)<<8 ;
  hwpr_stat_field |= ((hwpr_control.do_overshoot) & 0x0001)<<10 ;
  hwpr_stat_field |= ((hwpr_control.done_move) & 0x0001)<<11 ;
  hwpr_stat_field |= ((hwpr_control.done_all) & 0x0001)<<12 ;
  hwpr_stat_field |= ((hwpr_control.dead_pot) & 0x0001)<<13 ;
  hwpr_stat_field |= ((hwpr_control.do_calpulse) & 0x0001)<<14 ;
  hwpr_stat_field |= ((hwpr_control.reset_enc) & 0x0001)<<15 ;

  SET_VALUE(statControlHwprAddr, hwpr_stat_field);

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
  static int cal_wait_cnt = 0; 
  static int overshooting = 0;
  static int first_time = 1;
  static int last_enc = 0;
  
  int hwpr_enc_cur, hwpr_enc_dest;
  int i_step;  // index of the current step
  int i_next_step=0;

  static struct LutType HwprPotLut = {"/data/etc/blast/hwpr_pot.lut", 0, NULL, NULL, 0};

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
      hwpr_control.reset_enc = 1;
      if (CommandData.Cryo.calib_pulse == repeat) hwpr_control.do_calpulse = yes;
    } else if (CommandData.hwpr.mode == HWPR_GOTO_POT) {
      bprintf(info,"ControlHWPR: Attempting to go to HWPR potentiometer position %f",CommandData.hwpr.pot_targ);
      ResetControlHWPR();
      hwpr_control.go = pot;
      hwpr_control.move_cur = not_yet;
      hwpr_control.read_before = yes;
      hwpr_control.read_after = yes;
      hwpr_control.reset_enc = 1;
      if (CommandData.Cryo.calib_pulse == repeat) hwpr_control.do_calpulse = yes;
    } else if ((CommandData.hwpr.mode == HWPR_STEP)) {
      if(!CommandData.hwpr.no_step) {
	ResetControlHWPR();
	hwpr_control.go = step;
	hwpr_control.move_cur = not_yet;
	hwpr_control.read_before = yes;
	hwpr_control.read_after = yes;
	hwpr_control.do_calpulse = yes;
	hwpr_control.reset_enc = 1;
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
  if(CommandData.hwpr.mode >= HWPR_GOTO && CommandData.hwpr.mode != HWPR_REPEAT) {
    if (hwpr_control.do_calpulse && cal_wait_cnt <= 0) {
      hwpr_calpulse_flag=1;
      cal_wait_cnt = CommandData.Cryo.calib_pulse/20;
#ifdef DEBUG_HWPR
      bprintf(info,"Setting calpulse flag...cal_wait_cnt=%i",cal_wait_cnt);
#endif
      //      hwpr_control.do_calpulse = waiting;
      //Set flag for calpulse
    } else if (cal_wait_cnt > 0) {
      cal_wait_cnt--;
      if (cal_wait_cnt <= 0) hwpr_control.do_calpulse = 0;
#ifdef DEBUG_HWPR
      bprintf(info,"Waiting for calpulse to be done...cal_wait_cnt=%i, hwpr_control.do_calpulse=%i",cal_wait_cnt,hwpr_control.do_calpulse);
#endif
   /* Do we want a pot reading first? Should always be yes for step mode.*/
    } else 
if (hwpr_control.read_before == yes) {
 
      /* pulse the potentiometer */
#ifdef DEBUG_HWPR
      bprintf(info,"Pulsing the pot before we do anything else...");
#endif
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

      if (hwpr_control.go != no_move) { // yes, move

	/*** Figure out how many encoder counts we want to step ***/
	if (hwpr_control.move_cur == not_yet) {
	  if (hwpr_control.go == step) {

	    
	    if (((hwpr_data.pot > HWPR_POT_MIN) &&
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot
	      hwpr_control.dead_pot = 0;	      
	      /* calculate rel move from pot lut*/
	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);

#ifdef DEBUG_HWPR
	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
#endif
	      
	      /*get index of the closest hwpr_step position, and find the encoder position for the next step pos*/

	      i_step = GetHWPRi(hwpr_data.pot);
	      i_next_step = (i_step +1)%4;
	      hwpr_control.i_next_step = i_next_step;

              hwpr_control.pot_targ = CommandData.hwpr.pos[i_next_step];	      
	      hwpr_enc_dest = LutCal(&HwprPotLut, hwpr_control.pot_targ);
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
	      
#ifdef DEBUG_HWPR
	      bprintf(info,"Nearest step position: i = %i, encoder_lut = %i, pot = %f",i_step,hwpr_enc_cur,hwpr_data.pot);
	      bprintf(info,"Destination step position: i = %i, encoder_lut = %i, pot_targ = %f, CommandData.hwpr.pos[i_next_step] = %f",hwpr_control.i_next_step,hwpr_enc_dest,hwpr_control.pot_targ,CommandData.hwpr.pos[i_next_step]);
#endif
	    } else { // don't use pot
	      hwpr_control.dead_pot = 1;      
	      
	      /* assume rel move of ~22.5 degrees*/
	      hwpr_control.rel_move = HWPR_DEFAULT_STEP;
	      bprintf(info,"The pot is dead! Use default HWPR step of %i",hwpr_control.rel_move);
	    }
	    
	    hwpr_control.move_cur = ready; 


	  } else if (hwpr_control.go == ind) {
            // Not changing for flight.  Fix if we fly again. -lmf
	    if (((hwpr_data.pot > HWPR_POT_MIN) &&  
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot
	      hwpr_control.dead_pot = 0;      

	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);
#ifdef DEBUG_HWPR
	      bprintf(info,"This is where I calculate the relative step from the pot value.");
	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
#endif
             
              i_next_step = CommandData.hwpr.i_pos;
	      hwpr_control.i_next_step = i_next_step;
	      hwpr_control.pot_targ = CommandData.hwpr.pos[i_next_step];
	      hwpr_enc_dest = LutCal(&HwprPotLut, hwpr_control.pot_targ);
	      
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
	      bprintf(info,"Destination is index %i, pot value = %f, required rel encoder move is %i:",CommandData.hwpr.i_pos,CommandData.hwpr.pos[i_next_step],hwpr_control.rel_move);
	    } else { // don't use pot
	      hwpr_control.dead_pot = 1;	      
	      
	      /* can't step to a hwp position, because we don't know where it is */
	      hwpr_control.rel_move = 0;
	      bprintf(warning, "The pot is dead! Don't know where to move. rel_move = %i", hwpr_control.rel_move);
	      CommandData.hwpr.mode = HWPR_SLEEP;
	      return;
	    }

	    hwpr_control.move_cur = ready; 

	  } else if (hwpr_control.go == pot) {

	    if (((hwpr_data.pot > HWPR_POT_MIN) &&  
		 (hwpr_data.pot < HWPR_POT_MAX)) &&
		(CommandData.hwpr.use_pot)) { // use pot
	      hwpr_control.dead_pot = 0;	      

	      hwpr_enc_cur = LutCal(&HwprPotLut, hwpr_data.pot);
#ifdef DEBUG_HWPR
	      bprintf(info,"This is where I calculate the relative step from the pot value.");
	      bprintf(info,"Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %i", hwpr_data.pot, hwpr_enc_cur);
#endif
              hwpr_control.pot_targ = CommandData.hwpr.pot_targ;
	      hwpr_enc_dest = LutCal(&HwprPotLut, hwpr_control.pot_targ);
	      
	      hwpr_control.rel_move = hwpr_enc_dest - hwpr_enc_cur;
#ifdef DEBUG_HWPR
	      	      bprintf(info,"Destination is index %i, pot value = %f, required rel encoder move is %i:",CommandData.hwpr.i_pos,CommandData.hwpr.pos[i_next_step],hwpr_control.rel_move);
#endif

	    } else { // don't use pot
	      
	      hwpr_control.dead_pot = 1;	      
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

	} else if (hwpr_control.move_cur == ready) {

	  /* Is the hwpr move negative?  
	     If so check if we need to add an overshoot for backlash correction */
	  if (hwpr_control.rel_move < 0) {
	    if (CommandData.hwpr.overshoot > 0) {
	      hwpr_control.rel_move-= CommandData.hwpr.overshoot;
	      hwpr_control.do_overshoot = 1;
#ifdef DEBUG_HWPR
	      bprintf(info,"ControlHWPR: Overshoot of %i requested.", CommandData.hwpr.overshoot);
#endif
	    }
	  } else if (hwpr_control.rel_move == 0) {
	    bprintf(info,"ControlHWPR: Requested a move of 0.  Ignoring.");
            hwpr_control.done_move = 1;
            hwpr_control.move_cur = done;
	  } 
#ifdef DEBUG_HWPR
	  bprintf(info,"ControlHWPR: Here's where I will send a relative move command of %i",hwpr_control.rel_move);
#endif
	  EZBus_RelMove(bus, HWPR_ADDR, hwpr_control.rel_move);
	  hwpr_control.move_cur = moving;
	  hwpr_control.stop_cnt = 0;
          hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move;
	  /*** We are moving.  Wait until we are done. ***/
	} else if (hwpr_control.move_cur == moving) {

	  if (hwpr_data.enc == last_enc) {
	    hwpr_control.stop_cnt ++;
	  } else {
	    hwpr_control.stop_cnt = 0 ;
	  }

	  //	  bprintf(info,"ControlHWPR: We are moving! hwpr_data.enc = %i, last_enc = %i", hwpr_data.enc, last_enc);

	  if (hwpr_control.stop_cnt >=HWPR_MOVE_TIMEOUT) {
#ifdef DEBUG_HWPR
	    bprintf(info,"We've stopped!");
#endif
	    hwpr_control.enc_err = hwpr_control.enc_targ - hwpr_data.enc;

	    if(hwpr_control.do_overshoot) {
	      hwpr_control.move_cur = at_overshoot;
	    } else { // we're done moving
	      hwpr_control.move_cur = is_done;
#ifdef DEBUG_HWPR
	      bprintf(info,"We're done moving!");
#endif
	    }
	    
	  }

	  last_enc = hwpr_data.enc;

	} else if (hwpr_control.move_cur == at_overshoot) {
#ifdef DEBUG_HWPR
	    bprintf(info,"At the overshoot.");
#endif
	    hwpr_control.rel_move = CommandData.hwpr.overshoot;

#ifdef DEBUG_HWPR
	    bprintf(info,"ControlHWPR: Sending overshoot move command of %i",hwpr_control.rel_move);
#endif
	    EZBus_RelMove(bus, HWPR_ADDR, hwpr_control.rel_move);
	    hwpr_control.move_cur = moving;
	    hwpr_control.stop_cnt = 0;
	    hwpr_control.do_overshoot = 0;
	    hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move;

	} else if (hwpr_control.move_cur == is_done) {

	    /* Have to tell the HWPR that it is at it's current warm encoder position
	     * or else it will sometimes oscillate 
	     */
	  if (hwpr_control.reset_enc) {
#ifdef DEBUG_HWPR
	    bprintf(info,"ControlHWPR: Telling the HWPR that it is at the current encoder position of %i",hwpr_data.enc);
#endif
            EZBus_SetEnc(bus, HWPR_ADDR, hwpr_data.enc);
	    hwpr_control.reset_enc = 0;
	  }
	    /* Do we want to read the pot?*/
	    if (hwpr_control.read_after == yes) {
	      
	      /* pulse the potentiometer */
#ifdef DEBUG_HWPR
	      bprintf(info,"Pulsing the pot...");
#endif
	      CommandData.Cryo.hwprPos = 50;
	      hwpr_control.read_after = reading;
	      hwpr_control.read_wait_cnt = HWPR_READ_WAIT;
	      
	    } else if (hwpr_control.read_after == reading) { // we are reading...
	      
	      hwpr_control.read_wait_cnt--;
	      if (hwpr_control.read_wait_cnt <= 0) { 
		hwpr_control.read_after = done;
		hwpr_control.pot_err = hwpr_control.pot_targ - hwpr_data.pot;
	      }
      
	    } else {
	      hwpr_control.done_all = 1;
	    }
	}

      } else { // hwpr_control.go == no_move
        // ...we're done!
#ifdef DEBUG_HWPR
        bprintf(info,"Nothing left to do!");
#endif
	hwpr_control.done_all = 1;
      }
      
    }
    if(hwpr_control.done_all == 1) {
#ifdef DEBUG_HWPR
      bprintf(info,"ControlHWPR: HWPR command complete");
#endif
      CommandData.hwpr.mode = HWPR_SLEEP; 
    }
  }

  /*** end fall through control ***/

  /* This is historic and won't be used for flight.*/
  if( CommandData.hwpr.mode == HWPR_REPEAT) {

    if ( hwpr_wait_cnt <= 0
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
    } else if (hwpr_wait_cnt == 10) {
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
