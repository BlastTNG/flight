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

/* the operating modes */
const char *const mode_string[] = { MODE_STRINGS };

/* the current mode */
unsigned int status = 0;
static unsigned int last_reported_status = 0xFFFF;

/* the desired mode */
static enum modes goal = op_acq;
static enum modes last_reported_goal = -1;
int working = 0;

static const int mode_state[op_acq + 1] =
{
  [op_init] = 0,
  [op_drive] = ST_DRIVE0, /* only need a primary drive */
  [op_ready] = ST_DRIVE0 | ST_MCECMD | ST_CONFIG,
  [op_acq] = ST_DRIVE0 | ST_MCECMD | ST_CONFIG | ST_RETDAT
};

enum modes new_goal = -1;
enum tasks task = tk_idle;

/* Meta's job is to figure out how walk the graph to turn "status" into
 * mode_state[goal]
 */
void meta(void)
{
  int i;
  unsigned diff;

  if (status != last_reported_status)
    bprintf(info, "Director: status = 0x%04X", status);
  last_reported_status = status;

  if (goal != last_reported_goal)
    bprintf(info, "Director: goal = 0x%04X (%s)", mode_state[goal],
        mode_string[goal]);
  last_reported_goal = goal;

  /* nothing to do */
  if (status == mode_state[goal])
    return;

  if (!working) {
    int do_this = -1;
    /* figure out all the things to do */
    diff = status ^ mode_state[goal];

    /* find the first thing to do */
    for (i = 0; i < 32; ++i) {
      if (diff & (1 << i)) {
        do_this = 1 << i;
        break;
      }
    }
    
    switch (do_this) {
      case ST_DRIVE0:
      case ST_DRIVE1:
      case ST_DRIVE2:
        bprintf(info, "Director: new task: drive");
        task = tk_drive;
        break;
      default:
        bprintf(warning, "Director: unable to walk to goal! 0x%04X -> 0x%04X",
            status, mode_state[goal]);
        break;
    }

    working = 1;
  }
}
