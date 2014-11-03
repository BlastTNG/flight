/* MPC: MCE-PCM communicator
 *
 * This software is copyright (C) 2013 D. V. Wiebe
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#undef DEBUG_META

#include "mpc.h"
#include "blast.h"
#include <stdlib.h>
#include <string.h>

/* goal and mode names */
const char *const goal_string[] = { GOAL_STRINGS };
const char *const moda_string[] = { MODA_STRINGS };

/* the current mode */
unsigned int state = 0;

/* goal and moda */
enum modas moda = md_none;

struct gl_data goal = {gl_acq};
struct gl_data new_goal;
int change_goal;

uint32_t meta_tk = st_drives; /* might as well do this right away */

static int meta_veto = 0;

/* handle a new goal */
static int set_new_goal(enum goals *working_goal)
{
  if (change_goal) {
    /* we can't actually change goal until the current moda stops */
    *working_goal = (memory.squidveto) ? gl_stop : new_goal.goal;
    if (moda == md_none) {
      bprintf(info, "New goal: %s", goal_string[new_goal.goal]);
      memcpy(&goal, &new_goal, sizeof(goal));
      change_goal = 0;
      return 0;
    }
  } else if (memory.squidveto) {
    /* veto: force stop */
    *working_goal = gl_stop;
    if (moda == md_none)
      return 0;
  } else if (~state & st_drives) {
    /* must stop running to fix drives */
    *working_goal = goal.goal;
    if (moda == md_none)
      return 0;
  } else {
    /* normal operation: no need to stop */
    *working_goal = goal.goal;
    return 0;
  }

#ifdef DEBUG_META
  bprintf(info, "Stop moda: %s for goal %s", moda_string[moda],
      goal_string[*working_goal]);
#endif
  meta_tk = STOP_TK | (moda << MODA_SHIFT);
  return 1;
}

/* Meta's job is to run the goal and mode */
void meta(void)
{
  enum goals working_goal;

  /* we're vetoed or something's going on */
  if (meta_veto || meta_tk)
    return;

  /* change goals, if necessary; returns non-zero if it has set meta_tk */
  if (set_new_goal(&working_goal))
    return;

  /* now, moda is either md_none, or one of the allowed modas for this goal */

  /* if we're waiting for a state that the current goal doesn't need, stop it
   */
  if (wait_state == st_biased) {
    if (working_goal < gl_acq) /* don't need this */
      meta_tk = STOP_TK;
    else
      return; /* wait */
  } else if (working_goal == gl_stop ||
      ((state & ~ST_DRIVE_IGNORE) && (~state & st_drives)))
  {
    /* stop stuff */
    if (state & st_retdat)
      meta_tk = st_retdat | STOP_TK;
    else if (state & st_acqcnf)
      meta_tk = st_acqcnf | STOP_TK;
    else if (state & st_config)
      meta_tk = st_config | STOP_TK;
    else if (working_goal == gl_stop && state & st_active)
      meta_tk = st_active | STOP_TK;
  /* low-level stuff everyone but gl_stop wants */
  } else if (~state & st_drives) /* everyone wants this */
    meta_tk = st_drives;
  else if (~state & st_active)
    meta_tk = st_active;
  else if (~state & st_mcecom)
    meta_tk = st_mcecom;
  else if (~state & st_config) {
    /* need to stop an acquisition to do this */
    if (moda != md_none)
      meta_tk = STOP_TK | (moda << MODA_SHIFT);
    else if (state & st_retdat)
      meta_tk = st_retdat | STOP_TK;
    else if (state & st_acqcnf)
      meta_tk = st_acqcnf | STOP_TK;
    else
      meta_tk = st_config;
  } else if (working_goal >= gl_acq) { /* turn acq on */
    if (~state & st_biased)
      meta_tk = st_biased;
    else if (~state & st_acqcnf)
      meta_tk = st_acqcnf;
    else if (~state & st_retdat)
      meta_tk = st_retdat;
  } else { /* turn acq off */
    if (state & st_retdat)
      meta_tk = st_retdat | STOP_TK;
    else if (state & st_acqcnf)
      meta_tk = st_acqcnf | STOP_TK;
  }

  if (!meta_tk && !dt_going) /* now run the goal */
    switch (working_goal) {
      case gl_stop: /* nothing to do */
      case gl_pause:
        break;
      case gl_cycle:
        goal.goal = gl_acq; /* immediate change goal */
        break;
      case gl_acq:
        if (moda == md_none)
          meta_tk = md_running << MODA_SHIFT;
        break;
      case gl_tune:
        if (moda == md_none)
          meta_tk = md_tuning << MODA_SHIFT;
        break;
      case gl_iv:
        if (moda == md_none)
          meta_tk = md_iv_curve << MODA_SHIFT;
        break;
      case gl_partial:
        if (moda == md_none)
          meta_tk = md_partial << MODA_SHIFT;
        break;
      case gl_lcloop:
        if (moda == md_none)
          meta_tk = md_lcloop << MODA_SHIFT;
        break;
      case gl_bstep:
        if (moda == md_none)
          meta_tk = md_bstep << MODA_SHIFT;
        break;
      case gl_bramp:
        if (moda == md_none)
          meta_tk = md_bramp << MODA_SHIFT;
        break;
    }

#ifdef DEBUG_META
  if (meta_tk) {
    bprintf(info, "M: goal: %s; moda: %s; state: 0x%04X wait: 0x%04X %s",
        goal_string[goal.goal], moda_string[moda], state, wait_state,
        memory.squidveto ? "vetoed" : "");
    if ((meta_tk & ~STOP_TK) >= (1U << MODA_SHIFT)) {
      bprintf(info, "M: meta_tk: %s %s", (meta_tk & STOP_TK) ? "stop" : "start",
          moda_string[(meta_tk & ~STOP_TK) >> MODA_SHIFT]);
    }
    else 
      bprintf(info, "M: meta_tk: %s 0x%04X",
          (meta_tk & STOP_TK) ? "stop" : "start", meta_tk & ~STOP_TK);
  }
#endif
}

/* return to normal operation */
void meta_goal_complete(int need_reconfig)
{
  meta_veto = 1;
  if (need_reconfig)
    state &= ~st_config;
  new_goal.goal = gl_acq;
  change_goal = 1;
  meta_veto = 0;
}
