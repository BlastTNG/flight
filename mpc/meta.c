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

#define DEBUG_META

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
enum goals goal = gl_stop;

uint32_t meta_tk = 0;

static int meta_veto = 0;

/* stop a mode, if necessary; returns non-zero if we needed to stop something */
static int stop_moda(void)
{
  if (moda == md_none)
    return 0;

  switch (goal) {
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
      if (moda == md_running || moda == md_acqcnf)
        return 0;
      break;
  }

#ifdef DEBUG_META
  bprintf(info, "Stop moda: %s for goal %s", moda_string[moda],
      goal_string[goal]);
#endif
  meta_tk = STOP_TK | (moda << MODA_SHIFT);
  return 1;
}

/* Meta's job is to run the goal and mode */
void meta(void)
{
  /* something's going on */
  if (meta_veto || meta_tk)
    return;

  /* stop an old mode, if necessary */
  if (stop_moda())
    return;

  /* now, moda is either be md_none, or one of the allowed modas for this
   * goal */

  /* everyone wants this */
  if (~state & st_drives) {
    meta_tk = st_drives;
    return;
  }

  if (goal == gl_stop) { /* stop stuff */
    if (state & st_config)
      meta_tk = st_config | STOP_TK;
    else if (state & st_active)
      meta_tk = st_active | STOP_TK;
  /* low-level stuff everyone but gl_stop wants */
  } else if (~state & st_active)
    meta_tk = st_active;
  else if (~state & st_mcecom)
    meta_tk = st_mcecom;
  else if (~state & st_config)
    meta_tk = st_config;
  else /* now run the goal */
    switch (goal) {
      case gl_stop: /* nothing to do */
      case gl_ready:
        break;
      case gl_acq:
        if (moda == md_none)
          meta_tk = md_acqcnf << MODA_SHIFT;
        else if (moda == md_acqcnf)
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
    }

#ifdef DEBUG_META
  if (meta_tk) {
    bprintf(info, "M: goal: %s; moda: %s", goal_string[goal],
        moda_string[moda >> MODA_SHIFT]);
    if ((meta_tk & ~STOP_TK) >= md_none)
      bprintf(info, "M: meta_tk: %s %s", (meta_tk & STOP_TK) ? "stop" : "start",
          moda_string[(meta_tk & ~STOP_TK) >> MODA_SHIFT]);
    else 
      bprintf(info, "M: meta_tk: %s 0x%04X",
          (meta_tk & STOP_TK) ? "stop" : "start", meta_tk & ~STOP_TK);
  }
#endif
}

void meta_safe_update(enum goals new_goal, enum modas new_moda,
    unsigned int new_state)
{
  meta_veto = 1;
  state = new_state;
  moda = new_moda;
  goal = new_goal;
  meta_veto = 0;
}
