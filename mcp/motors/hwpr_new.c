/* mcp: hwpr: part of the BLAST master control program
 *
 * This software is copyright (C) 2018 Paul Williams
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

#define HWPR_DEG_TO_STEPS 14222.22222 // 100 (input_rev/output_rev) * 51200 (usteps/input_rev) / 360 (deg/output_rev)

static struct hwpr_struct {
  int addr;
  int32_t pos;
  float enc; // old file had enc and pot here, why? We should only need enc, old blast had only pot
} hwpr_data;

enum move_type
{
    no_move = 0, ind, step
};
enum move_status
{
    not_yet = 0, ready, moving, at_overshoot, is_done
};

static struct hwpr_control_struct
{
    enum move_type go;
    enum move_status move_cur;
    int done_move;
    int done_all;
    int32_t rel_move;
    int i_next_step;
    int do_overshoot;
    int stop_cnt;
    float enc_targ;
    float enc_err;
    int reset_enc;
	float angle_targ;
	float angle;
} hwpr_control;

void MonitorHWPR(struct ezbus *bus)
{
  EZBus_ReadInt(bus, hwpr_data.addr, "?0", &hwpr_data.pos);
  // EZBus_ReadInt(bus, HWPR_ADDR, "?8", &hwpr_data.enc);
  hwpr_data.enc = hwp_get_position() * ENC_TO_DEG; //  absolute degrees on hwpr
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
  // hwpr_control.do_calpulse = no;
  hwpr_control.stop_cnt = 0;
  hwpr_control.enc_targ = 0;
  hwpr_control.enc_err = 0;
  // hwpr_control.pot_targ = 0;
  // hwpr_control.pot_err = 0;
  hwpr_control.reset_enc = 0;
}

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
  // static channel_t* potTargHwprAddr;
  static channel_t* encTargHwprAddr;
  static channel_t* encErrHwprAddr;
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
    pos2HwprAddr = channels_find_by_name("pos2_hwpr");
    pos3HwprAddr = channels_find_by_name("pos3_hwpr");
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
  }

  hwpr_wait_cnt--;

  SET_VALUE(velHwprAddr, CommandData.hwpr.vel);
  SET_VALUE(accHwprAddr, CommandData.hwpr.acc);
  SET_VALUE(iMoveHwprAddr, CommandData.hwpr.move_i);
  SET_VALUE(iHoldHwprAddr, CommandData.hwpr.hold_i);
  SET_INT32(posHwprAddr, hwpr_data.pos);
  SET_FLOAT(encHwprAddr, hwpr_data.enc);
  SET_FLOAT(overshootHwprAddr, CommandData.hwpr.overshoot);
  SET_FLOAT(pos0HwprAddr, CommandData.hwpr.pos[0]);
  SET_FLOAT(pos1HwprAddr, CommandData.hwpr.pos[1]);
  SET_FLOAT(pos2HwprAddr, CommandData.hwpr.pos[2]);
  SET_FLOAT(pos3HwprAddr, CommandData.hwpr.pos[3]);
  SET_VALUE(iposRqHwprAddr, CommandData.hwpr.i_pos);
  // SET_VALUE(potTargHwprAddr, hwpr_control.pot_targ*65535);
  SET_VALUE(iposHwprAddr, hwpr_control.i_next_step);
  // SET_VALUE(readWaitHwprAddr, hwpr_control.read_wait_cnt);
  SET_VALUE(stopCntHwprAddr, hwpr_control.stop_cnt);
  SET_VALUE(relMoveHwprAddr, hwpr_control.rel_move/2); // ???
  SET_FLOAT(encTargHwprAddr, hwpr_control.enc_targ);
  SET_FLOAT(encErrHwprAddr, hwpr_control.enc_err);
  // SET_VALUE(potErrHwprAddr, hwpr_control.pot_err*32767);

  /* Make HWPR status bit field */
  hwpr_stat_field |= (hwpr_control.go) & 0x0007;
  hwpr_stat_field |= ((hwpr_control.move_cur) & 0x0007) << 3;
  // hwpr_stat_field |= ((hwpr_control.read_before) & 0x0003) << 6;
  // hwpr_stat_field |= ((hwpr_control.read_after) & 0x0003) << 8;
  hwpr_stat_field |= ((hwpr_control.do_overshoot) & 0x0001) << 10;
  hwpr_stat_field |= ((hwpr_control.done_move) & 0x0001) << 11;
  hwpr_stat_field |= ((hwpr_control.done_all) & 0x0001) << 12;
  // hwpr_stat_field |= ((hwpr_control.dead_pot) & 0x0001) << 13;
  hwpr_stat_field |= ((hwpr_control.do_calpulse) & 0x0001) << 14;
  hwpr_stat_field |= ((hwpr_control.reset_enc) & 0x0001) << 15;

  SET_VALUE(statControlHwprAddr, hwpr_stat_field);
}


void ControlHWPR(struct ezbus *bus)
{	
	hwpr_control = CommandData.hwpr.overshoot;

    if (CommandData.hwpr.mode == HWPR_PANIC) {
		blast_info("Panic"); // need to make this only print on change of state
		EZBus_Stop(bus, hwpr_data.addr);
		CommandData.hwpr.mode = HWPR_SLEEP;
    } else if (CommandData.hwpr.is_new == 1) {
		if (CommandData.hwpr.mode == HWPR_SLEEP) {
		} else if (CommandData.hwpr.mode == HWPR_GOTO) {
			hwpr_control.angle_targ = CommandData.hwpr.target;
			hwpr_control.angle_diff = hwpr_control.angle_targ - hwpr_control.angle;
			hwpr_control.rel_move = (int32) (hwpr_control.angle_diff * HWPR_DEG_TO_STEPS);
		} else if (CommandData.hwpr.mode == HWPR_JUMP) {
		} else if (CommandData.hwpr.mode == HWPR_STEP) {
			GetHWPRi(bus);
		} else if (CommandData.hwpr.mode == HWPR_REPEAT) {
		} else if (CommandData.hwpr.mode == HWPR_GOTO_I) {
		} else if (CommandData.hwpr.mode == HWPR_GOTO_POT) {
		}
	CommandData.hwpr_is_new = 0;
	}
	if (hwpr_control.rel_move > 0) {
		EZBus_RelMove(bus, hwpr_data.addr, hwpr_control.rel_move)
	} else if (hwpr_control.rel_move < 0) {
		hwpr_control.rel_move += overshoot;
	}









}
