/* command_list.h: BLAST command specification file definitions
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "isc_protocol.h"  /* required for constants */

#define N_SCOMMANDS 110        /* total number of single word cmds */
#define N_MCOMMANDS 71         /* total number of multiword commands */
#define MAX_N_PARAMS 6
#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)
#define MAX_30BIT (1073741823.)

#define SIZE_NAME 80
#define SIZE_ABOUT 80
#define SIZE_PARNAME 80

#define N_GROUPS 18

#define GR_POINT        0x00000001
#define GR_BAL          0x00000002
#define GR_BIAS         0x00000004
#define GR_TRIM         0x00000008
#define GR_COOL         0x00000010
#define GR_CALLAMP      0x00000020
#define GR_VETO         0x00000040
#define GR_EHEAT        0x00000080
#define GR_CRYO_HEAT    0x00000100
#define GR_POWER        0x00000200
#define GR_LOCK         0x00000400
#define GR_CRYO_CONTROL 0x00000800
#define GR_GAIN         0x00001000
#define GR_ISC_PARAM    0x00002000
#define GR_ISC_HOUSE    0x00004000
#define GR_MISC         0x00008000
#define GR_OSC_PARAM    0x00010000
#define GR_OSC_HOUSE    0x00020000

#define CONFIRM         0x80000000

extern const char command_list_serial[];
extern const char *GroupNames[N_GROUPS];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  isc_auto_focus,   auto_gyro,          az_off,           az_on,
  balance_allow,    balance_veto,       bias_ac,          bias_dc,
  cal_off,          cal_on,             auto_bdaheat,     charcoal_off,
  charcoal_on,      clock_ext,          clock_int,        coldplate_off,
  coldplate_on,     isc_discard_images, el_off,           el_on,
  elclin_allow,     elclin_veto,        elenc_allow,      elenc_veto,
  fixed,            isc_full_screen,    gps_allow,        gps_veto,
  l_valve_close,    he_valve_on,        he_valve_off,     l_valve_open,
  inner_cool_off,   inner_cool_on,      isc_abort,        isc_allow,
  isc_pause,        isc_reconnect,      isc_run,          isc_shutdown,
  isc_veto,         level_off,          level_on,         mag_allow,
  mag_veto,         outer_cool_off,     outer_cool_on,    osc_auto_focus,
  outer_spare_off,  outer_spare_on,     pin_in,           pot_valve_close,
  pot_valve_off,    pot_valve_on,       ss_off,           ss_on,
  pot_valve_open,   balpump_up,         balpump_off,      balpump_on,
  balpump_down,     sprpump_fwd,        sprpump_off,      sprpump_on,
  sprpump_rev,      ramp,               reset_trims,      isc_save_images,
  stop,             sun_veto,           sun_allow,
  trim_to_isc,      unlock,             lock_off,         xyzzy,
  mcc_halt,         isc_reboot,         isc_cam_cycle,    osc_run,
  osc_shutdown,     osc_reboot,         osc_cam_cycle,    osc_pause,
  osc_abort,        osc_reconnect,      osc_save_images,  osc_discard_images,
  osc_full_screen,  force_el_on,        auto_jfetheat,    fridge_cycle,
  analogue_gyros,   digital_gyros,      gps_off,          gps_on,
  gyro_off,         gyro_on,            isc_off,          isc_on,
  osc_off,          osc_on,             isc_trig_int,     isc_trig_ext,
  osc_trig_int,     osc_trig_ext,       ln_valve_on,      ln_valve_off,
  osc_veto,         osc_allow,          reap
};

struct scom {
  enum singleCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
};

extern struct scom scommands[N_SCOMMANDS];

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,          bal_gain,
  bal_level,         osc_set_focus,     osc_set_aperture, osc_save_period,
  bias1_level,       bias2_level,       bias3_level,      isc_blob_centre,
  box,               osc_pixel_centre,  osc_blob_centre,  isc_gain,
  cal_pulse,         cal_repeat,        cap,              isc_catalogue,
  az_el_trim,        isc_det_set,       drift,            el_gain,
  isc_integrate,     osc_integrate,     osc_det_set,      osc_max_blobs,
  cryo_heat,         heatsw_heat,       inner_level,      isc_offset,
  jfet_heat,         osc_catalogue,     osc_tolerances,   osc_hold_current,
  lock,              isc_max_blobs,     outer_level,      phase,
  pivot_gain,        isc_pixel_centre,  ra_dec_goto,      ra_dec_set,
  roll_gain,         isc_set_aperture,  isc_set_focus,    setpoints,
  bda_heat,          spare_level,       t_gyro_set,       osc_gain,
  t_gyro_gain,       timeout,           isc_tolerances,   vcap,
  vbox,              alice_file,        gyro_override,    isc_hold_current,
  isc_save_period,   back_emf,          osc_offset,       plugh,
  bda_gain,          bda_set,           jfet_set,         isc_foc_off,
  osc_foc_off,       t_gyro_heat,       t_gyro_param
};

struct par {
  char name[SIZE_PARNAME];
  double min;
  double max;
  char type;
  char field[20];
};

struct mcom {
  enum multiCommand command;
  char name[SIZE_NAME];
  char about[SIZE_ABOUT];
  unsigned int group;
  char numparams;
  struct par params[MAX_N_PARAMS];
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];
