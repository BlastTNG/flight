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

#include <unistd.h>
#include <stdlib.h>

#include "mcp.h"
#include "hwpr.h"
#include "lut.h"
#include "ezstep.h"
#include "command_struct.h"
#include "pointing_struct.h" /* To access ACSData */
#include "tx.h" /* InCharge */
#include "ec_motors.h"
#include "actuators.h"
#include "ethercattype.h"

#undef DEBUG_HWPR

#define HWPR_READ_WAIT 10
#define HWPR_MOVE_TIMEOUT 3
#define DEG_TO_STEPS 14222.2222 // 100 (input_rev/output_rev) * 51200 (usteps/input rev) / 360 (deg/output_rev)
#define ENC_TO_DEG 0.00044 // 3.6 (output_deg/input_rev) / 8192 (enc_units/input_rev)

static struct hwpr_struct {
  int addr;
  int32_t pos;
  double enc;
  double pot;
  float enc_real_hwpr;
} hwpr_data = {0};


enum move_type
{
    no_move = 0, pot, ind, step, goto_abs, goto_rel
};
enum move_status
{
    not_yet = 0, ready, moving, at_overshoot, is_done, needs_backoff, engage
};
/*enum read_pot
{
    no = 0, yes, reading, done
};*/

static struct hwpr_control_struct
{
    enum move_type go;
    enum move_status move_cur;
    // enum read_pot read_before;
    // enum read_pot read_after;
    // int read_wait_cnt; // added
    int done_move;
    int done_all;
    int32_t rel_move;
    int i_next_step;
    int do_overshoot;
	int do_disengage;
	int do_main_move;
    int stop_cnt;
    float enc_targ;
	float enc_real_targ;
    float enc_err;
	float enc_real_err;
    // double pot_targ;
    // double pot_err;
    // int dead_pot;
    // int do_calpulse;
    // int reset_enc;
	float margin; // shouldn't be reset before each move
	int engaged; // shouldn't be reset before each move
	int32_t engage_move;
	int32_t disengage_move;
	float overshoot;
	int index;
} hwpr_control;

int hwpr_calpulse_flag = 0;

void MonitorHWPR(struct ezbus *bus)
{
  EZBus_ReadInt(bus, hwpr_data.addr, "?0", &hwpr_data.pos);
  hwpr_control.index = GetHWPRIndex(hwpr_data.enc);
}

/* Clear out the hwpr_control structure*/
void ResetControlHWPR(void) {
  hwpr_control.go = no_move;
  hwpr_control.move_cur = not_yet;
  // hwpr_control.read_before = no;
  // hwpr_control.read_after = no;
  // hwpr_control.read_wait_cnt = 0;
  hwpr_control.done_move = 0;
  hwpr_control.done_all = 0;
  hwpr_control.rel_move = 0;
  hwpr_control.do_overshoot = 0;
  hwpr_control.do_disengage = 0;
  hwpr_control.do_main_move = 0;
  // hwpr_control.do_calpulse = no;
  hwpr_control.stop_cnt = 0;
  hwpr_control.enc_targ = 0;
  hwpr_control.enc_err = 0;
  // hwpr_control.pot_targ = 0;
  // hwpr_control.pot_err = 0;
  // hwpr_control.reset_enc = 0;
  hwpr_control.enc_real_targ = 0;
  hwpr_control.enc_real_err = 0;
  hwpr_control.disengage_move = 0;
}

// counter incremented in StoreHWPRBus to better time tep_repeat mode
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
  static channel_t* overshootHwprAddr;
  static channel_t* iposRqHwprAddr;
  static channel_t* iposHwprAddr;
  // static channel_t* readWaitHwprAddr;
  static channel_t* stopCntHwprAddr;
  static channel_t* relMoveHwprAddr;
  static channel_t* statControlHwprAddr;
  // static channel_t* potTargHwprAddr;
  static channel_t* encTargHwprAddr;
  static channel_t* encErrHwprAddr;
  static channel_t* encRealHwprAddr;
  static channel_t* encRealTargHwprAddr;
  static channel_t* encRealErrHwprAddr;
  static channel_t* marginHwprAddr;
  static channel_t* backoffHwprAddr;
  // static channel_t* potErrHwprAddr;

  if (firsttime) {
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
    iposRqHwprAddr = channels_find_by_name("i_pos_rq_hwpr");
    iposHwprAddr = channels_find_by_name("i_pos_hwpr");
    // readWaitHwprAddr = channels_find_by_name("read_wait_hwpr");
    stopCntHwprAddr = channels_find_by_name("stop_cnt_hwpr");
    relMoveHwprAddr = channels_find_by_name("rel_move_hwpr");
    statControlHwprAddr = channels_find_by_name("stat_control_hwpr");
    // potTargHwprAddr = channels_find_by_name("pot_targ_hwpr");
    encTargHwprAddr = channels_find_by_name("enc_targ_hwpr");
    encErrHwprAddr = channels_find_by_name("enc_err_hwpr");
    // potErrHwprAddr = channels_find_by_name("pot_err_hwpr");
	encRealHwprAddr = channels_find_by_name("enc_real_hwpr");
	encRealTargHwprAddr = channels_find_by_name("enc_real_targ_hwpr");
	marginHwprAddr = channels_find_by_name("margin_hwpr");
	backoffHwprAddr = channels_find_by_name("backoff_hwpr");
	encRealErrHwprAddr = channels_find_by_name("enc_real_err_hwpr");
  }

  hwpr_wait_cnt--;

  SET_UINT32(velHwprAddr, CommandData.hwpr.vel);
  SET_UINT16(accHwprAddr, CommandData.hwpr.acc);
  SET_UINT8(iMoveHwprAddr, CommandData.hwpr.move_i);
  SET_UINT8(iHoldHwprAddr, CommandData.hwpr.hold_i);
  SET_INT32(posHwprAddr, hwpr_data.pos);
  SET_FLOAT(encHwprAddr, hwpr_data.enc);
  SET_FLOAT(overshootHwprAddr, CommandData.hwpr.overshoot);
  SET_FLOAT(pos0HwprAddr, CommandData.hwpr.pos[0]);
  SET_FLOAT(pos1HwprAddr, CommandData.hwpr.pos[1]);
  SET_INT8(iposRqHwprAddr, hwpr_control.i_next_step);
  // SET_VALUE(potTargHwprAddr, hwpr_control.pot_targ*65535);
  SET_INT8(iposHwprAddr, hwpr_control.index);
  // SET_VALUE(readWaitHwprAddr, hwpr_control.read_wait_cnt);
  SET_UINT16(stopCntHwprAddr, hwpr_control.stop_cnt);
  SET_INT32(relMoveHwprAddr, hwpr_control.rel_move);
  SET_FLOAT(encTargHwprAddr, hwpr_control.enc_targ);
  SET_FLOAT(encErrHwprAddr, hwpr_control.enc_err);
  // SET_VALUE(potErrHwprAddr, hwpr_control.pot_err*32767);
  SET_FLOAT(encRealHwprAddr, hwpr_data.enc_real_hwpr);
  SET_FLOAT(encRealTargHwprAddr, hwpr_control.enc_real_targ);
  SET_FLOAT(marginHwprAddr, hwpr_control.margin);
  SET_FLOAT(backoffHwprAddr, CommandData.hwpr.backoff);
  SET_FLOAT(encRealErrHwprAddr, hwpr_control.enc_real_err);

  /* Make HWPR status bit field */
  hwpr_stat_field |= (hwpr_control.go) & 0x0007;
  hwpr_stat_field |= ((hwpr_control.move_cur) & 0x0007) << 3;
  hwpr_stat_field |= ((hwpr_control.engaged) & 0x0001) << 6;
  hwpr_stat_field |= ((hwpr_control.do_overshoot) & 0x0001) << 7;
  hwpr_stat_field |= ((hwpr_control.done_move) & 0x0001) << 8;
  hwpr_stat_field |= ((hwpr_control.done_all) & 0x0001) << 9;
  hwpr_stat_field |= ((hwpr_control.do_disengage) & 0x0001) << 10;
  hwpr_stat_field |= ((hwpr_control.do_main_move) & 0x0001) << 11;

  SET_INT16(statControlHwprAddr, hwpr_stat_field);
}

// DEPRECATED, use GetHWPRIndex for two position HWPR - PAW 2018/11/25
// From the current potentiometer reading. Figure out the nearest
// hwpr step position, and return the index
int GetHWPRi(double pot_val)
{
    int i;
    int i_min = 0;
    double d_pot;
    double d_pot_min = 2.0;

    for (i = 0; i <= 3; i++) {
        d_pot = fabs(pot_val - CommandData.hwpr.pos[i]);
        if (d_pot < d_pot_min) {
            d_pot_min = d_pot;
            i_min = i;
        }
//            blast_info("GetHWPRi: i=%i,d_pot=%f,pot_val=%f,CommandData.hwpr.pos[i]=%f,d_pot_min=%f,i_min=%i",
//                       i,d_pot,pot_val,CommandData.hwpr.pos[i],d_pot_min,i_min);
    }

#ifdef DEBUG_HWPR
  blast_info("GetHWPRi: Returning %i", i_min);
#endif
  // i is the closest index to where we are.  return the next index (i+1)%4
  return i_min;
}

int GetHWPRIndex(double enc_val)
// From the current encoder reading, and a specified margin, return the position of the HWP
{
	int index;

	hwpr_control.margin = CommandData.hwpr.margin;

	// if encoder is within hwpr_margin of position 0
	if ((enc_val < (CommandData.hwpr.pos[0] + hwpr_control.margin))
			&& (enc_val > (CommandData.hwpr.pos[0] - hwpr_control.margin))) {
		index = 0;
	// if encoder is within hwpr_margin of position 1
	} else if ((enc_val < (CommandData.hwpr.pos[1] + hwpr_control.margin))
			&& (enc_val > (CommandData.hwpr.pos[1] - hwpr_control.margin))) {
		index = 1;
	} else {
		index = -1; // not at a defined position, next position will be zero
	}

#ifdef DEBUG_HWPR
    blast_info("GetHWPRIndex: Returning position %i", i_min);
#endif

	return index;
}

void ControlHWPR(struct ezbus *bus)
{
    static int repeat_pos_cnt = 0;
    // static int cal_wait_cnt = 0;
    static int overshooting = 0;
    static int first_time = 1;
    static float last_enc = 0;

    double hwpr_enc_cur = 0.0;
    // float hwpr_enc_dest = 0.0;
    int i_step;  // index of the current step
    int i_next_step = 0;
    uint16_t enc_state = 0;

    // static struct LutType HwprPotLut = { "/data/etc/blast/hwpr_pot.lut", 0, NULL, NULL, 0 };

    // if (HwprPotLut.n == 0) LutInit(&HwprPotLut);

    if (first_time) {
        /* Initialize hwpr_controls */
        ResetControlHWPR();
		hwpr_control.engaged = 0;
        first_time = 0;
    }

    if (CommandData.hwpr.mode == HWPR_PANIC) {
        blast_info("HWPR Panic mode, stopping and then sleeping");
        EZBus_Stop(bus, hwpr_data.addr);
        CommandData.hwpr.mode = HWPR_SLEEP;
    } else if (CommandData.hwpr.is_new) {
        if ((CommandData.hwpr.mode == HWPR_GOTO)) {
            blast_info("HWPR GOTO: %f", CommandData.hwpr.target); // DEBUG PCA
            ResetControlHWPR();
			hwpr_control.go = goto_abs;
			hwpr_control.move_cur = not_yet;
	    	// EZBus_Goto(bus, hwpr_data.addr, CommandData.hwpr.target);
            // CommandData.hwpr.mode = HWPR_SLEEP;
        } else if ((CommandData.hwpr.mode == HWPR_GOTO_REL)) {
            blast_info("HWPR GOTO REL: %f", CommandData.hwpr.target); // DEBUG PCA
            ResetControlHWPR();
			hwpr_control.go = goto_rel;
			hwpr_control.move_cur = not_yet;
	    	// EZBus_RelMove(bus, hwpr_data.addr, CommandData.hwpr.target);
            // CommandData.hwpr.mode = HWPR_SLEEP;
        } else if ((CommandData.hwpr.mode == HWPR_GOTO_I)) {
            blast_info("ControlHWPR: Attempting to go to HWPR position %i", CommandData.hwpr.i_pos);
            ResetControlHWPR();
            hwpr_control.go = ind;
            hwpr_control.move_cur = not_yet;
        } else if (CommandData.hwpr.mode == HWPR_GOTO_POT) {
            blast_info("ControlHWPR: Attempting to go to HWPR potentiometer position %f", CommandData.hwpr.pot_targ);
            ResetControlHWPR();
            hwpr_control.go = pot;
            hwpr_control.move_cur = not_yet;
        } else if ((CommandData.hwpr.mode == HWPR_STEP)) {
            if (!CommandData.hwpr.no_step) {
                ResetControlHWPR();
                hwpr_control.go = step;
                hwpr_control.move_cur = not_yet;
            } else {
                blast_warn("Cannot step half wave plate.  hwpr_step_off is set.");
                CommandData.hwpr.mode = HWPR_SLEEP;
            }
        } else if ((CommandData.hwpr.mode == HWPR_REPEAT)) {
            // just received, initialize HWPR_REPEAT variables
            repeat_pos_cnt = CommandData.hwpr.n_pos;
            hwpr_wait_cnt = CommandData.hwpr.step_wait;
        }
        CommandData.hwpr.is_new = 0;
    }
    /*** Begin fall through control ***/

	// TODO(paul): check units (encoder vs deg vs motor steps) in control code
    /* if are doing anything with the HWPR other than sleeping, panicing or repeating */
    if (CommandData.hwpr.mode >= HWPR_GOTO && CommandData.hwpr.mode != HWPR_REPEAT) {
            /*** Do we want to move? ***/

		if (hwpr_control.go != no_move) { // yes, move
        	/*** Figure out how many encoder counts we want to step ***/
            if (hwpr_control.move_cur == not_yet) {
				if (hwpr_control.go == step) {
                    enc_state = hwp_get_state();
					if (enc_state > 0) {
						hwpr_enc_cur = hwpr_data.enc;
#ifdef DEBUG_HWPR
                		blast_info("Current enc value: hwpr_enc_cur = %f", hwpr_enc_cur);
#endif
						/**
                		* get index of the closest hwpr_step position,
                		* and find the encoder position for the next step pos
						*
						* if not at a defined position, i_step = -1, so the next position will be position 0
                		*/

                		i_step = GetHWPRIndex(hwpr_enc_cur);
                		i_next_step = (i_step + 1) % 2;
                		hwpr_control.i_next_step = i_next_step;

                		hwpr_control.enc_targ = CommandData.hwpr.pos[i_next_step];
                		// hwpr_enc_dest = LutCal(&HwprPotLut, hwpr_control.pot_targ);
                		hwpr_control.rel_move = (int32_t)((hwpr_control.enc_targ - hwpr_enc_cur) * DEG_TO_STEPS);

						// engage fork before main move
						hwpr_control.move_cur = engage;
#ifdef DEBUG_HWPR
                		blast_info("Nearest step position: i = %i, encoder = %f", i_step,
                                       hwpr_enc_cur);
						blast_info("Destination step position: i = %i, "
                                       "CommandData.hwpr.pos[i_next_step] = %f",
                                        hwpr_control.i_next_step,
                                        CommandData.hwpr.pos[i_next_step]);
#endif
					} else {
                        /* can't move hwp, encoder is dead */
                        hwpr_control.rel_move = 0;
                        blast_warn("The encoder is dead! State = %d: Don't know where to move. rel_move = %i",
                                   enc_state, hwpr_control.rel_move);
                        CommandData.hwpr.mode = HWPR_SLEEP;
                        return;
                	}
				} else if (hwpr_control.go == ind) {
                	// Not changing for flight.  Fix if we fly again. -lmf
                    enc_state = hwp_get_state();
					if (enc_state > 0) {
					    hwpr_enc_cur = hwpr_data.enc;
#ifdef DEBUG_HWPR
                        blast_info("This is where I calculate the relative step from the pot value.");
                        blast_info("Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %f", hwpr_data.pot,
                                       hwpr_enc_cur);
#endif

                        i_next_step = CommandData.hwpr.i_pos;
                        hwpr_control.i_next_step = i_next_step;
                        hwpr_control.enc_targ = CommandData.hwpr.pos[i_next_step];

                        hwpr_control.rel_move = (int32_t)((hwpr_control.enc_targ - hwpr_enc_cur) * DEG_TO_STEPS);
						// engage fork before main move
                    	hwpr_control.move_cur = engage;
#ifdef DEBUG_HWPR
                        blast_info("Destination is index %i, pot value = %f, required rel motor step move is %ld:",
                                       CommandData.hwpr.i_pos, CommandData.hwpr.pos[i_next_step],
                                       hwpr_control.rel_move);
                        blast_info("target: %f, current: %f", hwpr_control.enc_targ, hwpr_enc_cur); // DEBUG
#endif
					} else { // don't use pot
                             // hwpr_control.dead_pot = 1;

                            /* can't step to a hwp position, because we don't know where it is */
                            hwpr_control.rel_move = 0;
                            blast_warn("The encoder is dead! State = %d: Don't know where to move. rel_move = %i",
                                       enc_state, hwpr_control.rel_move);
                            CommandData.hwpr.mode = HWPR_SLEEP;
                            return;
                    }
                } else if (hwpr_control.go == pot) {
                    enc_state = hwp_get_state();
					if (enc_state > 0) {
    				    hwpr_enc_cur = hwpr_data.enc;
#ifdef DEBUG_HWPR
                        blast_info("This is where I calculate the relative step from the pot value.");
                        blast_info("Current pot value: hwpr_data.pot = %f, hwpr_enc_cur = %f", hwpr_data.pot,
                                       hwpr_enc_cur);
#endif
                        hwpr_control.enc_targ = CommandData.hwpr.pot_targ; // pot_targ in deg, enc_targ is always deg
                        // hwpr_enc_dest = LutCal(&HwprPotLut, hwpr_control.pot_targ);

						//
                        hwpr_control.rel_move =
							(int32_t)((hwpr_control.enc_targ - hwpr_enc_cur) * DEG_TO_STEPS);
						// engage fork before main move
                        hwpr_control.move_cur = engage;
#ifdef DEBUG_HWPR
                        blast_info("Destination is index %i, pot value = %f, required rel encoder move is %i:",
                                       CommandData.hwpr.i_pos, CommandData.hwpr.pos[i_next_step],
                                       hwpr_control.rel_move);
#endif

                    } else { // don't use pot
                            // hwpr_control.dead_pot = 1;
                            /* can't step to a hwp position, because we don't know where it is */
                            blast_warn("The encoder is dead! State = %d: Don't know where to move.", enc_state);
                            CommandData.hwpr.mode = HWPR_SLEEP;
                            return;
                    }
                } else if (hwpr_control.go == goto_abs) {
					enc_state = hwp_get_state();
					if (enc_state > 0) {
    				    hwpr_enc_cur = hwpr_data.enc; // hwpr_enc_cur is in degrees

						/* enc_targ is the goal for the first part of the move, 
						 * but it could get modified to include overshoot
						 *
						 * enc_real_targ is where we want to leave the hwp after end of backlash correction, 
						 * but before thermal break backoff
						 */
						hwpr_control.enc_targ = CommandData.hwpr.target;
						hwpr_control.enc_real_targ = CommandData.hwpr.target;
						// rel_move in steps
                    	hwpr_control.rel_move = (int32_t)((hwpr_control.enc_targ - hwpr_enc_cur) * DEG_TO_STEPS);

						// engage fork before main move
						hwpr_control.move_cur = engage;

					} else { // encoder is dead
						blast_warn("The HWPR encoder is dead! State = %d: Not moving", enc_state);
						CommandData.hwpr.mode = HWPR_SLEEP;
						return;
					}

                } else if (hwpr_control.go == goto_rel) {
					enc_state = hwp_get_state();
					if (enc_state > 0) {
    				 	hwpr_enc_cur = hwpr_data.enc; // hwpr_enc_cur is in degrees

						/* enc_targ is the goal for the first part of the move, 
						 * but it could get modified to include overshoot
						 *
						 * enc_real_targ is where we want to leave the hwp after end of backlash correction, 
						 * but before thermal break backoff
						 */
						hwpr_control.enc_targ = hwpr_enc_cur + CommandData.hwpr.target;
						hwpr_control.enc_real_targ = hwpr_enc_cur + CommandData.hwpr.target;
						// rel_move in steps
                    	hwpr_control.rel_move = (int32_t)((hwpr_control.enc_targ - hwpr_enc_cur) * DEG_TO_STEPS);
						// engage fork before main move
						hwpr_control.move_cur = engage;

					} else { // encoder is dead
						blast_warn("The HWPR encoder is dead! State = %d: Not moving", enc_state);
						CommandData.hwpr.mode = HWPR_SLEEP;
						return;
					}
                } else {
                    blast_info("This state should be impossible.");
                    CommandData.hwpr.mode = HWPR_SLEEP;
                    return; // break out of loop!
                }

            /*** Once we are ready to move send ActBus Command ***/

            } else if (hwpr_control.move_cur == engage) {
				/* We should be ready, but we need to re-engage the fork at the cold end
				 * CommandData.hwpr.backoff is in deg on the input shaft, so we divide by 100 to find
				 * degrees on the hwpr (which is what DEG_TO_STEPS assumes)
				 *
				 * Engage move always in the same direction of the final move (before disengaging), so it is in  
				 * the opposite direction from the overshoot)
				 */

				hwpr_control.overshoot = CommandData.hwpr.overshoot;
				if (hwpr_control.overshoot > 0) {
					hwpr_control.engage_move = (-1) * (int32_t) (CommandData.hwpr.backoff * DEG_TO_STEPS / 100);
				} else if (hwpr_control.overshoot < 0) {
					hwpr_control.engage_move = (int32_t) (CommandData.hwpr.backoff * DEG_TO_STEPS / 100);
				}

				EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.engage_move);

				hwpr_control.move_cur = moving;
				// after engage move, go to main move
				hwpr_control.do_main_move = 1;
				hwpr_control.stop_cnt = 0;

            } else if (hwpr_control.move_cur == ready) {
            	/* Is the hwpr move negative?
                If so check if we need to add an overshoot for backlash correction */
		    	if (hwpr_control.rel_move < 0) {
                	if (hwpr_control.overshoot < 0) {
                    	hwpr_control.rel_move += (int32_t)(hwpr_control.overshoot * DEG_TO_STEPS);
                    	hwpr_control.do_overshoot = 1;
						// change target to include the overshoot
                		hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move / DEG_TO_STEPS;
#ifdef DEBUG_HWPR
                    	blast_info("ControlHWPR: Overshoot of %i requested.", hwpr_control.overshoot);
#endif
                	}
                }

				/* Or if the move is positive and the overshoot is also positive, then we overshoot
				 */
				if (hwpr_control.rel_move > 0) {
                	if (hwpr_control.overshoot > 0) {
                    	hwpr_control.rel_move += (int32_t)(hwpr_control.overshoot * DEG_TO_STEPS);
                    	hwpr_control.do_overshoot = 1;
						// change target to include the overshoot
                		hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move / DEG_TO_STEPS;
#ifdef DEBUG_HWPR
                    	blast_info("ControlHWPR: Overshoot of %i requested.", hwpr_control.overshoot);
#endif
                	}
                }
#ifdef DEBUG_HWPR
                blast_info("ControlHWPR: Here's where I will send a relative move command of %i",
                               hwpr_control.rel_move);
#endif
				// this is exactly the same as the line at the beginning of the ready block
				// but we need it to include the overshoot, if one is present
                // hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move / DEG_TO_STEPS;

                EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.rel_move);
                hwpr_control.move_cur = moving;
				hwpr_control.do_main_move = 0;
                hwpr_control.stop_cnt = 0;

				if (hwpr_control.rel_move == 0) {
                    blast_info("ControlHWPR: Requested a move of 0.  Ignoring.");
                    hwpr_control.done_move = 1;
                    hwpr_control.move_cur = is_done;
                }
				// only set do_disengage now if we are not overshooting, otherwise wait until after overshoot correction
				if (!hwpr_control.do_overshoot) {
					hwpr_control.do_disengage = 1;
				}

            /*** We are moving.  Wait until we are done. ***/
            } else if (hwpr_control.move_cur == moving) {
				// tolerance on "stopped moving" is 0.01 deg
            	if (fabs(hwpr_data.enc-last_enc) <= 0.01) {
                	hwpr_control.stop_cnt++;
                } else {
                	hwpr_control.stop_cnt = 0;
                }

                //  blast_info("ControlHWPR: We are moving! hwpr_data.enc = %f, last_enc = %f",
                //             hwpr_data.enc, last_enc);

                if (hwpr_control.stop_cnt >= HWPR_MOVE_TIMEOUT) {
#ifdef DEBUG_HWPR
                    blast_info("We've stopped!");
#endif
                    hwpr_control.enc_err = hwpr_control.enc_targ - hwpr_data.enc;

                    if (hwpr_control.do_overshoot) {
                        hwpr_control.move_cur = at_overshoot;
					} else if (hwpr_control.do_main_move) {
						hwpr_control.engaged = 1;
						hwpr_control.move_cur = ready; // go to main move part
					} else if (hwpr_control.do_disengage) {
						hwpr_control.move_cur = needs_backoff;
                    } else { // we're done moving
                        hwpr_control.move_cur = is_done;
						hwpr_control.done_move = 1;
						hwpr_control.engaged = 0;
#ifdef DEBUG_HWPR
                        blast_info("We're done moving!");
#endif
                    }
                }

                last_enc = hwpr_data.enc;

            } else if (hwpr_control.move_cur == at_overshoot) {
#ifdef DEBUG_HWPR
            	blast_info("At the overshoot.");
#endif
				hwpr_enc_cur = hwpr_data.enc;
				/* Undo overshoot: moves are precise, so just use the opposite of whatever the overshoot is
				 * We know that this goes in the correct direction, because if we got here we did the overshoot
				 * according to its sign already
				 */
				hwpr_control.rel_move = (-1) * (int32_t)(hwpr_control.overshoot * DEG_TO_STEPS);
#ifdef DEBUG_HWPR
                blast_info("ControlHWPR: Sending overshoot move command of %i", hwpr_control.rel_move);
		    	blast_info("ControlHWPR: Currently at %f, target is %f", hwpr_data.enc, hwpr_control.enc_targ);
#endif
                EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.rel_move);
                hwpr_control.move_cur = moving;
                hwpr_control.stop_cnt = 0;
                hwpr_control.do_overshoot = 0;
				hwpr_control.do_disengage = 1; // after overshoot correction, we should backoff
				// So we can keep track of move accuracy, calculate enc_targ for this step
                hwpr_control.enc_targ = hwpr_enc_cur + hwpr_control.rel_move / DEG_TO_STEPS;

			} else if (hwpr_control.move_cur == needs_backoff) {
#ifdef DEBUG_HWPR
				blast_info("Going to backoff to break thermal link");
#endif
				hwpr_data.enc_real_hwpr = hwpr_data.enc;
				// calculate how far we are from the spot we really want to be now, before backoff
				hwpr_control.enc_real_err = hwpr_control.enc_real_targ - hwpr_data.enc_real_hwpr;
				/* again, backoff in deg on input shaft, so divide by 100
				 *
				 * Again, check the sign of overshoot, disengage should be in the same direction
				 */
				if (hwpr_control.overshoot < 0) {
					hwpr_control.disengage_move = (-1) * (int32_t) (CommandData.hwpr.backoff * DEG_TO_STEPS / 100);
				} else if (hwpr_control.overshoot > 0) {
					hwpr_control.disengage_move = (int32_t) (CommandData.hwpr.backoff * DEG_TO_STEPS / 100);
				}

				EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.disengage_move);
				hwpr_control.move_cur = moving;
				hwpr_control.stop_cnt = 0;
				hwpr_control.do_disengage = 0;
				hwpr_control.enc_targ = hwpr_data.enc + hwpr_control.rel_move / DEG_TO_STEPS;

            } else if (hwpr_control.move_cur == is_done) {
                /* Have to tell the HWPR that it is at it's current warm encoder position
                 * or else it will sometimes oscillate
                 */
                /*if (hwpr_control.reset_enc) {
#ifdef DEBUG_HWPR
                    blast_info("ControlHWPR: Telling the HWPR that it is at the current encoder position of %i",
                               hwpr_data.enc);
#endif
                    EZBus_SetEnc(bus, hwpr_data.addr, hwpr_data.enc);
                    hwpr_control.reset_enc = 0;
                }*/
                    /* Do we want to read the pot?*/
                    /* if (hwpr_control.read_after == yes) {
                        // pulse the potentiometer 
#ifdef DEBUG_HWPR
                        blast_info("Pulsing the pot...");
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
                    }*/
            }

        } else { // hwpr_control.go == no_move
                // ...we're done!
#ifdef DEBUG_HWPR
                blast_info("Nothing left to do!");
#endif
                hwpr_control.done_all = 1;
        }
        if (hwpr_control.done_all == 1) {
#ifdef DEBUG_HWPR
            blast_info("ControlHWPR: HWPR command complete");
#endif
            CommandData.hwpr.mode = HWPR_SLEEP;
        }
    }

    /*** end fall through control ***/

    /* This is historic and won't be used for flight.*/
    if (CommandData.hwpr.mode == HWPR_REPEAT) {
        if (hwpr_wait_cnt <= 0
        /*|| (overshooting && hwpr_wait_cnt >= 10)*/) {
            hwpr_wait_cnt = CommandData.hwpr.step_wait;
            if (overshooting) {
                overshooting = 0;
                EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.overshoot);
            } else if (CommandData.hwpr.repeats-- <= 0) { // done stepping
                CommandData.hwpr.mode = HWPR_SLEEP;
            } else {    // step the HWPR
                if (--repeat_pos_cnt > 0) { // move to next step
                    EZBus_RelMove(bus, hwpr_data.addr, CommandData.hwpr.step_size);
                } else {						   // reset to first step
                    repeat_pos_cnt = CommandData.hwpr.n_pos;
                    hwpr_wait_cnt *= CommandData.hwpr.n_pos;  // wait extra long
                    overshooting = 1;
                    EZBus_RelMove(
                            bus, hwpr_data.addr,
                            -CommandData.hwpr.step_size * (CommandData.hwpr.n_pos - 1) - hwpr_control.overshoot);
                }
            }
        } else if (hwpr_wait_cnt == 10) {
            // pulse the potentiometer
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
        hwpr_data.addr = GetActAddr(HWPRNUM);
        hwpr_control.i_next_step = 0;
    }

    /* update the HWPR move parameters */
    EZBus_SetVel(bus, hwpr_data.addr, CommandData.hwpr.vel);
    EZBus_SetAccel(bus, hwpr_data.addr, CommandData.hwpr.acc);
    EZBus_SetIMove(bus, hwpr_data.addr, CommandData.hwpr.move_i);
    EZBus_SetIHold(bus, hwpr_data.addr, CommandData.hwpr.hold_i);
    ControlHWPR(bus);

    MonitorHWPR(bus);
}

// ReadHWPREnc called from the mcp 5Hz loop.  Reads from the EtherCat data structure.
void ReadHWPREnc(void)
{
    hwpr_data.enc = (double)(hwp_get_position() * ENC_TO_DEG);
}
