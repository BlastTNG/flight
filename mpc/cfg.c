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
#include <string.h>
#include <unistd.h>

static int have_expt_cfg = 0;
/* experiment.cfg needs to be flushed */
static int expt_cfg_dirty = 0;
static config_t expt;

static int deferred_timing = 0;
static int deferred_row_len, deferred_num_rows, deferred_data_rate;

int cfg_get_int(const char *name, int n)
{
  config_setting_t *s;

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
        bprintf(err, "bad index %i trying to get scalar int on %s", n, name);
        return -1;
      }

      /* get */
      return config_setting_get_int(s);
    case CONFIG_TYPE_ARRAY:
      if (n >= config_setting_length(s)) {
        bprintf(err, "bad index %i > %i trying to get int elem on %s", n,
            config_setting_length(s), name);
        return -1;
      }
      /* get */
      return config_setting_get_int_elem(s, n);
  }
  bprintf(err, "bad type trying to set int on %s", name);
  return -1;
}

int cfg_get_int_cr(const char *name, int c, int r)
{
  return cfg_get_int(name, c * 41 + r);
}

int cfg_set_intarr(const char *name, int o, uint32_t *d, int n)
{
  config_setting_t *s;
  int old_v, v, i;

  /* get the parameter */
  if ((s = config_lookup(&expt, name)) == NULL) {
    bprintf(err, "unable to find '%s' in experiment.cfg", name);
    return -1;
  }

  /* check type and set */
  switch (config_setting_type(s))
  {
    case CONFIG_TYPE_ARRAY:
      if (o + n > config_setting_length(s)) {
        bprintf(err, "bad index %i > %i trying to set int array on %s", o + n,
            config_setting_length(s), name);
        return -1;
      }
      /* get */
      for (i = o; i < o + n; ++i) { 
        v = d[i - o];
        old_v = config_setting_get_int_elem(s, i);

        if (old_v != v) {
          /* set */
          config_setting_set_int_elem(s, i, v);
          expt_cfg_dirty = 1;
        }
      }
      break;
    default:
      bprintf(err, "bad type trying to set int array on %s", name);
      return -1;
  }
  return 0;
}

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

int cfg_set_int_cr(const char *name, int c, int r, int v)
{
  return cfg_set_int(name, c * 41 + r, v);
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

static int read_libconfig_file(const char *file, config_t *cfg)
{
  config_init(cfg);

  if (config_read_file(cfg, file) != CONFIG_TRUE) {
    bprintf(err, "Error parsing %s, line %i: %s", file,
        config_error_line(cfg), config_error_text(cfg));
    config_destroy(cfg);
    return 1;
  }

  return 0;
}

int load_experiment_cfg(void)
{
  char file[] = "/data#/mce/current_data/experiment.cfg";

  if (have_expt_cfg)
    return 1;

  /* use the first available */
  if ((drive_map & DRIVE0_MASK) != DRIVE0_UNMAP)
    file[5] = data_drive[0] + '0';
  else if ((drive_map & DRIVE1_MASK) != DRIVE1_UNMAP)
    file[5] = data_drive[1] + '0';
  else if ((drive_map & DRIVE2_MASK) != DRIVE2_UNMAP)
    file[5] = data_drive[2] + '0';
  else {
    bprintf(err, "No configured drives!");
    return 1;
  }

  if (read_libconfig_file(file, &expt))
    return 1;

  bprintf(info, "Read config data from %s\n", file);

  have_expt_cfg = 1;
  /* deal with deferred row len */
  if (deferred_timing) {
    deferred_timing = 0;
    cfg_set_int("row_len", 0, deferred_row_len);
    cfg_set_int("sample_dly", 0, deferred_row_len -
        cfg_get_int("sample_num", 0));
    cfg_set_int("num_rows", 0, deferred_num_rows);
    cfg_set_int("data_rate", 0, deferred_data_rate);
    if (expt_cfg_dirty) {
      flush_experiment_cfg(0);
      state &= ~st_config;
    }
  }
  return 0;
}

int flush_experiment_cfg(int force)
{
  int i, fd;
  FILE *stream;

  char xptname[] = "/data#/mce/current_data/experiment.cfg";

  if (!force && !expt_cfg_dirty) /* nothing to do */
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

  send_mceparam = 1;
  expt_cfg_dirty = 0;

  return 0;
}

/* listify experiment.cfg */
int serialise_experiment_cfg(void)
{
  config_setting_t *s, *dd, *fd;
  int i, c, r, n;

  if (!have_expt_cfg)
    return 0;

  for (i = 0; expcfg_list[i].name; ++i) {
    if ((s = config_lookup(&expt, expcfg_list[i].name)) == NULL) {
      bprintf(err, "unable to find '%s' in experiment.cfg",
          expcfg_list[i].name);
      continue;
    }

    switch (expcfg_list[i].type) {
      case CFGSER_BIT:
        if (expcfg_list[i].count == 0)
          mce_stat[N_EXP_OFF + expcfg_list[i].offset] = 0;

        if (config_setting_get_int(s)) 
          mce_stat[N_EXP_OFF + expcfg_list[i].offset] |=
            (1 << expcfg_list[i].count);
        break;
      case CFGSER_BITARR:
        for (r = 0; r < expcfg_list[i].count; ++r) {
          n = 0;
          for (c = 0; c < expcfg_list[i].count; ++c)
            if (config_setting_get_int_elem(s, r * 16 + c))
              n |= (1 << c);
          mce_stat[N_EXP_OFF + expcfg_list[i].offset + r] = n;
        }
        break;
      case CFGSER_INT:
        for (r = 0; r < expcfg_list[i].count; ++r)
          mce_stat[N_EXP_OFF + expcfg_list[i].offset + r] =
            config_setting_get_int_elem(s, r);
        break;
      case CFGSER_FLOAT:
        for (r = 0; r < expcfg_list[i].count; ++r) {
          mce_stat[N_EXP_OFF + expcfg_list[i].offset + r] =
            65536 * config_setting_get_int_elem(s, r) /
            (expcfg_list[i].max - expcfg_list[i].min) - expcfg_list[i].min;
        }
        break;
    }
  }

  /* update the dead dead detector count */
  n = 0;
  s = config_lookup(&expt, "columns_off");
  dd = config_lookup(&expt, "dead_detectors");
  fd = config_lookup(&expt, "frail_detectors");

  for (c = 0; c < NUM_COL; ++c)
    if (config_setting_get_int_elem(s, c))
      n += NUM_ROW;
    else
      for (i = 0; i < NUM_ROW; ++i)
        /* these masks are full 41x32 sized */
        if (config_setting_get_int_elem(dd, c * 41 + i))
          n++;
        else if (config_setting_get_int_elem(fd, c * 41 + i))
          n++;

  slow_dat.dead_count = n;
  return 0;
}

/* update row_len, num_rows, data_rate -- the tricky part here is making
 * sure we've read expcfg before doing it */
void cfg_update_timing(int row_len, int num_rows, int data_rate)
{
  if (!have_expt_cfg) {
    deferred_timing = 1;
    deferred_row_len = row_len;
    deferred_num_rows = num_rows;
    deferred_data_rate = data_rate;
  } else {
    cfg_set_int("row_len", 0, row_len);
    cfg_set_int("sample_dly", 0, row_len - cfg_get_int("sample_num", 0));
    cfg_set_int("num_rows", 0, num_rows);
    cfg_set_int("data_rate", 0, data_rate);
    if (expt_cfg_dirty) {
      flush_experiment_cfg(0);
      state &= ~st_config;
    }
  }
}

/* copy a parameter between loaded config_ts */
static int cfg_copy_param(config_t *out, const config_t *in, const char *n,
    int vet)
{
  int i;

  /* vetting vector comes from the destination */
  config_setting_t *svet = vet ? config_lookup(out, "columns_off") : NULL;

  /* find the parameters */
  config_setting_t *sout = config_lookup(out, n);
  config_setting_t *sin  = config_lookup(in, n);
  if (sout == NULL || sin == NULL) {
    bprintf(warning, "Can't find for copy settting: %s", n);
    return 1;
  }

  /* checks */
  if (config_setting_type(sin) != CONFIG_TYPE_ARRAY ||
      config_setting_type(sout) != CONFIG_TYPE_ARRAY)
  {
    bprintf(warning, "Bad type in copy for settting: %s", n);
    return 1;
  }

  if (config_setting_length(sin) != config_setting_length(sout)) {
    bprintf(warning, "Length mismatch in copy for settting: %s", n);
    return 1;
  }

  /* copy! */
  for (i = 0; i < config_setting_length(sin); ++i)
    if (!vet || !config_setting_get_int_elem(svet, i))
      config_setting_set_int_elem(sout, i, config_setting_get_int_elem(sin, i));
    else
      config_setting_set_int_elem(sout, i, 0);

  return 0;
}

/* read an archived experiment.cfg file and transfer relevant parameters into
 * the live system */
void cfg_apply_tuning(int n, int force_reconfig)
{
  config_t cfg;
  int d, have_cfg = -1;
  char file[100];

  /* the list of parameters to copy */
  const char *param[] = {"adc_offset_c", "adc_offset_cr", "sa_fb",
    "sa_offset", "sq1_bias", "sq1_bias_off", "sq2_fb", "sq2_fb_set",
    "default_sa_bias", "default_sq2_bias", "default_sq1_bias",
    "default_sq1_bias_off", NULL};

  /* try to read an archive */
  if (tuning_filename("sq1_ramp", "experiment.cfg", n, file) == 0)
    for (d = 0; d < 4; ++d)
      if (slow_dat.df[d]) {
        file[5] = d + '0';
        if (read_libconfig_file(file, &cfg) == 0) {
          have_cfg = d;
          break;
        }
      }

  if (have_cfg == -1) {
    bprintf(warning, "Couldn't read tuning %i", n);
    return;
  }

  /* copy parameters and record */
  int copy_error = 0;
  for (d = 0; param[d]; ++d)
    if (cfg_copy_param(&expt, &cfg, param[d], 0))
      copy_error = 1;

  /* these need to be vetted by columns_off */
  if (cfg_copy_param(&expt, &cfg, "sa_bias", 1))
    copy_error = 1;
  if (cfg_copy_param(&expt, &cfg, "sq2_bias", 1))
    copy_error = 1;

  if (!copy_error) { /* probably means it's completely corrupt, but... meh */
    flush_experiment_cfg(1);

    /* force reconfig */
    if (force_reconfig)
      state &= ~st_config;

    memory.used_tune = n;
    mem_dirty = 1;

    bprintf(info, "copied parameters from %s\n", file);
  }

  config_destroy(&cfg);
  return;
}

/* replace the current experiment cfg with the template */
void cfg_load_template(void)
{
  char file[100]; 
  config_t cfg;
  int row_len, num_rows, data_rate;

  /* we'll get around to it later */
  if (!have_expt_cfg)
    return;

  /* save these */
  row_len = cfg_get_int("row_len", 0);
  num_rows = cfg_get_int("num_rows", 0);
  data_rate = cfg_get_int("data_rate", 0);

  /* this is data drive agnostic ... -ish */
  sprintf(file, "/data/mas/config/experiment_x%i.cfg", nmce + 1);

  config_init(&cfg);

  if (read_libconfig_file(file, &cfg)) {
    bprintf(info, "Unable to load experiment.cfg template.");
    return;
  }

  /* destroy the old one and replace it with the new */
  config_destroy(&expt);
  memcpy(&expt, &cfg, sizeof(cfg));

  /* ignore the sync box parameters from the template */
  cfg_set_int("row_len", 0, row_len);
  cfg_set_int("sample_dly", 0, row_len - cfg_get_int("sample_num", 0));
  cfg_set_int("num_rows", 0, num_rows);
  cfg_set_int("data_rate", 0, data_rate);

  /* rewrite */
  flush_experiment_cfg(1);
}

/* replace the current dead and frail masks with the templates */
#define NLISTS 5
void cfg_load_dead_masks(void)
{
  /* we could get this from the config, but this is much easier */
  const char *list[NLISTS] = {"connection", "jumpers", "multilock", "other",
    "squid1"};
  const char *what[2] = {"dead", "frail"};
  char file[100], setting[50];
  int i, j, k;
  config_setting_t *s;
  int new_count = 0;

  for (i = 0; i < 2; ++i) {
    int mask[32 * 41]; /* masks are full sized */ 
    memset(mask, 0, sizeof(int) * 32 * 41);
    for (j = 0; j < NLISTS; ++j) {
      config_t cfg;
      config_init(&cfg);

      sprintf(file, "/data/mas/config/dead_lists/x%i/%s_%s.cfg", nmce + 1,
          what[i], list[j]);
      if (read_libconfig_file(file, &cfg) == 0) {
        s = config_lookup(&cfg, "mask");
        if (s && config_setting_type(s) == CONFIG_TYPE_ARRAY)
          for (k = 0; k < 32 * 41; ++k)
            if (!mask[k] && config_setting_get_int_elem(s, k)) {
              mask[k] = 1;
              new_count++;
            }
        bprintf(info, "read: %s; dead_count = %i\n", file, new_count);
      }
      config_destroy(&cfg);
    }

    /* now write it -- why doesn't libconfig allow the writing of a whole
     * array? */
    sprintf(setting, "%s_detectors", what[i]);
    s = config_lookup(&expt, setting);
    for (k = 0; k < 32 * 41; ++k)
      config_setting_set_int_elem(s, k, mask[k]);
  }

  /* force save */
  flush_experiment_cfg(1);
}

enum det_types cfg_det_type(int c, int r)
{
  if (cfg_get_int_cr("dead_detectors", c, r))
    return det_dead;
  if (cfg_get_int_cr("frail_detectors", c, r))
    return det_frail;
  return det_healthy;
}

int cfg_frail_pid(char l)
{
  if (l == 'p')
    return cfg_get_int("frail_servo_p", 0);
  else if (l == 'i')
    return cfg_get_int("frail_servo_i", 0);
  return cfg_get_int("frail_servo_d", 0);
}
