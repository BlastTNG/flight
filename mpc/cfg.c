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
#include "expcfg_list.h"
#include <libconfig.h>
#include <sys/stat.h>
#include <unistd.h>

/* experiment.cfg needs to be flushed */
static int expt_cfg_dirty = 0;
static config_t expt;

int cfg_set_int(const char *name, int n, int v)
{
  config_setting_t *s;
  int old_v;

  /* get the parameter */
  if ((s = config_lookup(&expt, name)) == NULL) {
    bprintf(err, "unable to find '%s' in experiment.cfg", name);
    return -1;
  }

  /* check type and set */
  switch (config_setting_type(s))
  {
    case CONFIG_TYPE_INT:
      if (n > 0) {
        bprintf(err, "bad index %i trying to set scalar int on %s", n, name);
        return -1;
      }

      /* get */
      old_v = config_setting_get_int(s);

      /* no change */
      if (old_v == v)
        return 0;

      /* set */
      config_setting_set_int(s, v);
      expt_cfg_dirty = 1;
      break;
    case CONFIG_TYPE_ARRAY:
      if (n >= config_setting_length(s)) {
        bprintf(err, "bad index %i > %i trying to set int elem on %s", n,
            config_setting_length(s), name);
        return -1;
      }
      /* get */
      old_v = config_setting_get_int_elem(s, n);

      /* no change */
      if (old_v == v)
        return 0;

      /* set */
      config_setting_set_int_elem(s, n, v);
      expt_cfg_dirty = 1;
      break;
    default:
      bprintf(err, "bad type trying to set int on %s", name);
      return -1;
  }
  return 0;
}

int cfg_set_float(const char *name, int n, double v)
{
  config_setting_t *s;
  double old_v;

  /* get the parameter */
  if ((s = config_lookup(&expt, name)) == NULL) {
    bprintf(err, "unable to find '%s' in experiment.cfg", name);
    return -1;
  }

  /* check type and set */
  switch (config_setting_type(s))
  {
    case CONFIG_TYPE_FLOAT:
      if (n > 0) {
        bprintf(err, "bad index %i trying to set scalar float on %s", n, name);
        return -1;
      }

      /* get */
      old_v = config_setting_get_float(s);

      /* no change */
      if (old_v == v)
        return 0;

      /* set */
      config_setting_set_float(s, v);
      expt_cfg_dirty = 1;
      break;
    case CONFIG_TYPE_ARRAY:
      if (n >= config_setting_length(s)) {
        bprintf(err, "bad index %i > %i trying to set float elem on %s", n,
            config_setting_length(s), name);
        return -1;
      }
      /* get */
      old_v = config_setting_get_float_elem(s, n);

      /* no change */
      if (old_v == v)
        return 0;

      /* set */
      config_setting_set_float_elem(s, n, v);
      expt_cfg_dirty = 1;
      break;
    default:
      bprintf(err, "bad type trying to set float on %s", name);
      return -1;
  }
  return 0;
}

int cfg_set_int_cr(const char *name, int c, int r, int v)
{
  return cfg_set_int(name, c * 41 + r, v);
}

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

int flush_experiment_cfg(void)
{
  int i, fd;
  FILE *stream;

  char xptname[] = "/data#/mce/current_data/experiment.cfg";

  if (!expt_cfg_dirty) /* nothing to do */
    return 0;

  /* write to each physical drive */
  for (i = 0; i < 4; ++i)
    if (slow_dat.df[i] > 0) {
      char tmpname[] = "/data#/mce/current_data/.experiment.cfg.XXXXXX";
      tmpname[5] = i + '0';
      xptname[5] = i + '0';

      fd = mkstemp(tmpname);
      if (fd < 0) {
        bprintf(warning, "unable to create temporary file %s", tmpname);
        continue;
      }
      stream = fdopen(fd, "w");

      config_write(&expt, stream);
      fclose(stream);
      if (rename(tmpname, xptname)) {
        bprintf(warning, "error writing %s", xptname);
        unlink(tmpname);
      }
      chmod(xptname, 0777);
      bprintf(info, "re-wrote %s", xptname);
    }

  return 0;
}

int serialise_experiment_cfg(void)
{
  /* listify experiment.cfg */
}
