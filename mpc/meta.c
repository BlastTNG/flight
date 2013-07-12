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

#include "mpc.h"
#include "blast.h"
#include "stdlib.h"

/* the operating mode names */
const char *const mode_string[] = { MODE_STRINGS };

/* the current mode */
unsigned int state = 0;

/* the desired mode */
enum modes goal = op_acq;
int working = 0;

/* mode -> status lookups */

/* Modes that must be active to acheive a goal */
static const int mode_start[op_acq + 1] =
{
  [op_init] = 0,
  [op_ready] = st_drives | st_mcecom,
  [op_tune] = st_drives | st_mcecom | st_config | st_tuning,
  [op_acq] = st_drives | st_mcecom | st_config | st_acqcnf | st_retdat,
};
/* Modes that must be inactive to acheive a goal */
static const int mode_stop[op_acq + 1] =
{
  [op_init] = 0,
  [op_ready] = st_retdat | st_acqcnf,
  [op_tune] = st_retdat | st_acqcnf,
  [op_acq] = st_tuning,
};

enum status start_tk = st_idle;
enum status  stop_tk = st_idle;

/* figure out what to do to flip the desired state */
void try_toggle(enum status st, int stop, enum status *do_start,
    enum status *do_stop)
{
  if (stop) {
    /* nothing to do */
    if (~state & st)
      return;

    switch (st) {
      case st_drives: /* probably shouldn't ever get here ... */
        if (state & st_retdat)
          try_toggle(st_retdat, 1, do_start, do_stop);
        else if (state & st_tuning)
          try_toggle(st_tuning, 1, do_start, do_stop);
        else
          *do_stop = st;
        break;
      case st_acqcnf:
        if (state & st_retdat)
          try_toggle(st_retdat, 1, do_start, do_stop);
        else
          *do_stop = st;
        break;

      case st_idle:
      case st_mcecom:
      case st_config:
      case st_retdat:
      case st_tuning:
        /* always okay */
        *do_stop = st;
        break;
    }
  } else {
    /* nothing to do */
    if (state & st)
      return;

    switch (st) {
      case st_drives:
        /* acquisition must be stopped to intialise a primary */
        if (state & st_retdat)
          try_toggle(st_retdat, 1, do_start, do_stop);
        else 
          *do_start = st;
        break;
      case st_idle:
      case st_mcecom:
        /* always okay */
        *do_start = st;
        break;
      case st_config:
        if (state & st_retdat) /* can't be acquiring data */
          try_toggle(st_retdat, 1, do_start, do_stop);
        else if (~state & st_drives) /* must have a drive */
          try_toggle(st_drives, 0, do_start, do_stop);
        else if (~state & st_mcecom) /* MCE must be alive */
          try_toggle(st_mcecom, 0, do_start, do_stop);
        else
          *do_start = st;
        break;
      case st_acqcnf:
      case st_tuning:
        if (~state & st_config) /* MCE must be configured */
          try_toggle(st_config, 0, do_start, do_stop);
        else
          *do_start = st;
        break;
      case st_retdat:
        if (~state & st_acqcnf) /* acq must be configured */
          try_toggle(st_acqcnf, 0, do_start, do_stop);
        else
          *do_start = st;
        break;
    }
  }

//  bprintf(info, "try_toggle(%04x,%i) = %04x,%04x", st, stop, *do_start, *do_stop);
}

/* Meta's job is to figure out how walk the graph to turn "state" into
 * the value specified by mode_start and mode_stop
 */
void meta(void)
{
  int i;
  unsigned diff;

  /* nothing to do */
  if (((state & mode_start[goal]) == mode_start[goal]) &&
      ((~state & mode_stop[goal]) == mode_stop[goal]))
  {
    return;
  }

  if (!working) {
    bprintf(info, "M: state = 0x%04X", state);
    bprintf(info, "M: goal = 0x%04X/0x%04X (%s)", mode_start[goal],
        mode_stop[goal], mode_string[goal]);

    enum status do_start = st_idle;
    enum status do_stop  = st_idle;

    /* figure out all the things to do */
    diff = (~state & mode_start[goal]) | (state & mode_stop[goal]);
//    bprintf(info, "M: diff = 0x%04X", diff);

    /* find the first thing to do */
    for (i = 0; i < 32; ++i) {
      if (diff & (1 << i)) {
        /* here's a bit that needs fixing, try doing something about it */
        try_toggle(1 << i, (mode_stop[goal] & (1 << i)), &do_start, &do_stop);
        if (do_start != st_idle || do_stop != st_idle)
          break;
      }
    }

    if (do_stop != st_idle) {
      stop_tk = do_stop;
      bprintf(info, "M: STOP %04x", do_stop);
      working = 1;
    } else if (do_start != st_idle) {
      start_tk = do_start;
      bprintf(info, "M: START %04x", do_start);
      working = 1;
    } else {
      /* um.... */
      bprintf(fatal, "M: Unable to walk graph: 0x%04X -> 0x%04X",
          state, mode_start[goal]);
    }
  } else if (start_tk == st_idle && stop_tk == st_idle)
    working = 0;
}
