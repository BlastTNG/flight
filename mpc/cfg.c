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
#include <libconfig.h>

static config_t expt;

int load_experiment_cfg(void)
{
  char file[1024] = "/data#/mce/current_data/experiment.cfg";
  config_init(&expt);

  /* use the first available */
  if ((drive_map & DRIVE0_MASK) != DRIVE0_UNMAP)
    file[5] = data_drive[0] + '0';
  else if ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP)
    file[5] = data_drive[1] + '0';
  else if ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP)
    file[5] = data_drive[2] + '0';
  else {
    bprintf(err, "No configured drives!");
    goto BAD_EXPT_CNF;
  }

  if (config_read_file(&expt, file) != CONFIG_TRUE) {
    bprintf(err, "Error parsing %s, line %i: %s", file,
        config_error_line(&expt), config_error_text(&expt));
    goto BAD_EXPT_CNF;
  }

  return 0;

BAD_EXPT_CNF:
  config_destroy(&expt);
  return 1;
}
