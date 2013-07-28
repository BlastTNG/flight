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

/* goal and mode names */
const char *const goal_string[] = { GOAL_STRINGS };
const char *const moda_string[] = { MODA_STRINGS };

/* the current mode */
unsigned int state = 0;

/* goal and moda */
enum modas moda = md_none;
enum goals goal = gl_acq;

uint32_t meta_tk = 0;

static int meta_veto = 0;

/* stop a mode, if necessary; returns non-zero if we needed to stop something */
static int stop_moda(enum goals working_goal)
{
  if (moda == md_none)
    return 0;

  switch (working_goal) {
    case gl_ready:
    case gl_stop:
      break;
    case gl_tune:
      if (moda == md_tuning)
        return 0;
      break;
    case gl_iv:
      if (moda == md_iv_curve)
        return 0;
      break;
    case gl_lcloop:
      if (moda == md_lcloop)
        return 0;
      break;
    case gl_acq:
      if (moda == md_running)
        return 0;
      break;
    case gl_bstep:
      if (moda == md_bstep)
        return 0;
      break;
  }

#ifdef DEBUG_META
  bprintf(info, "Stop moda: %s for goal %s", moda_string[moda],
      goal_string[working_goal]);
#endif
  meta_tk = STOP_TK | (moda << MODA_SHIFT);
  return 1;
}

/* returns true if this moda wants and acquisition running */
int need_acq(enum goals goal)
{
  if (goal == gl_acq)
    return 1;
  return 0;
}

/* Meta's job is to run the goal and mode */
void meta(void)
{
  enum goals working_goal = (memory.squidveto) ? gl_stop : goal;

  /* something's going on */
  if (meta_veto || meta_tk)
    return;

  /* stop an old mode, if necessary */
  if (stop_moda(working_goal))
    return;

  /* now, moda is either md_none, or one of the allowed modas for this goal */

  if (~state & st_drives) /* everyone wants this */
    meta_tk = st_drives;
  else if (working_goal == gl_stop) { /* stop stuff */
    if (state & st_retdat)
      meta_tk = st_retdat | STOP_TK;
    else if (state & st_acqcnf)
      meta_tk = st_acqcnf | STOP_TK;
    else if (state & st_config)
      meta_tk = st_config | STOP_TK;
    else if (state & st_active)
      meta_tk = st_active | STOP_TK;
  /* low-level stuff everyone but gl_stop wants */
  } else if (~state & st_active)
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
  } else if (need_acq(working_goal)) { /* turn acq on */
    if (~state & st_acqcnf)
      meta_tk = st_acqcnf;
    else if (~state & st_retdat)
      meta_tk = st_retdat;
  } else { /* turn acq off */
    if (state & st_retdat)
      meta_tk = st_retdat | STOP_TK;
    else if (state & st_acqcnf)
      meta_tk = st_acqcnf | STOP_TK;
  }

  if (!meta_tk) /* now run the goal */
    switch (working_goal) {
      case gl_stop: /* nothing to do */
      case gl_ready:
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
      case gl_lcloop:
        if (moda == md_none)
          meta_tk = md_lcloop << MODA_SHIFT;
      case gl_bstep:
        if (moda == md_none)
          meta_tk = md_bstep << MODA_SHIFT;
    }

#ifdef DEBUG_META
  if (meta_tk) {
    bprintf(info, "M: goal: %s; moda: %s; state: 0x%04X %s",
        goal_string[goal], moda_string[moda], state,
        memory.squidveto ? "vetoed" : "");
    if ((meta_tk & ~STOP_TK) >= (1U << MODA_SHIFT))
      bprintf(info, "M: meta_tk: %s %s", (meta_tk & STOP_TK) ? "stop" : "start",
          moda_string[(meta_tk & ~STOP_TK) >> MODA_SHIFT]);
    else 
      bprintf(info, "M: meta_tk: %s 0x%04X",
          (meta_tk & STOP_TK) ? "stop" : "start", meta_tk & ~STOP_TK);
  }
#endif
}

/* change mode, state and/or goal without screwing up the director */
void meta_safe_update(enum goals new_goal, enum modas new_moda,
    unsigned int new_state)
{
  meta_veto = 1;
  state = new_state;
  moda = new_moda;
  goal = new_goal;
  meta_veto = 0;
}
