/* command_list.c: Spider command specification file
 *
 * This software is copyright (C) 2002-2007 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* COMMANDS IN THIS FILE YOU *MUST*
 * RECOMPILE AND REINSTALL BLASTCMD ON WIDOW!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "command_list.h"
#include "camstruct.h"
#include "motordefs.h"

const char *command_list_serial = "$Revision: 1.10 $";

//these must correspond to #defines in header
const char *GroupNames[N_GROUPS] = {
  "Star Camera Table",   "Miscellaneous",  "Motors",
  "SC Miscellaneous",    "SC Modes",       "SC Parameters", 
  "Pointing Modes", "Pointing Gains", "Pointing Parameters"
};

//echoes as string; makes enum name the command name string
#define COMMAND(x) x, #x

struct scom scommands[N_SCOMMANDS] = {
  //Star Camera commands
  {COMMAND(cam_expose), "Start cam exposure (in triggered mode)", GR_SC_MISC},
  {COMMAND(cam_autofocus), "Camera autofocus mode", GR_SC_MODE},
  {COMMAND(cam_settrig_ext), "Set external cam trigger mode", GR_SC_MODE},
  {COMMAND(cam_force_lens), "Forced mode for cam lens moves", GR_SC_MODE},
  {COMMAND(cam_unforce_lens), "Normal mode for cam lens moves", GR_SC_MODE},
  {COMMAND(pt_set_spin), "Setting pointing mode to spin", GR_PT_MODE},  
  {COMMAND(pt_set_scan), "Setting pointing mode to scan", GR_PT_MODE},  
  {COMMAND(pt_set_point), "Setting pointing mode to point", GR_PT_MODE},  
  {COMMAND(test), "just a test of command system", GR_MISC} 
};

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * l :  parameter is 30 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * d :  parameter is 30 bit renormalised floating point
 * s :  parameter is 7-bit character string
 */
struct mcom mcommands[N_MCOMMANDS] = {
  //Starcam commands
  {COMMAND(cam_any), "Execute arbitrary starcam command", GR_SC_MISC, 1,
    {
      {"Command String", 0, 32, 's', ""}
    }
  },
  {COMMAND(cam_settrig_timed), "Use timed exposure mode", GR_SC_MODE, 1,
    {
      {"Exposure Interval (ms)", 0, MAX_15BIT, 'i', "sc_exp_int"}
    }
  },
  {COMMAND(cam_exp_params), "set starcam exposure commands", GR_SC_PARAM, 1,
    {
      {"Exposure duration (ms)", 40, MAX_15BIT, 'i', "sc_exp_time"}
    }
  },
  {COMMAND(cam_focus_params), "set camera autofocus params", GR_SC_PARAM, 1,
    {
      {"Resolution (number total positions)", 0, MAX_15BIT, 'i', "sc_foc_res"}
    }
  },
  {COMMAND(cam_bad_pix), "Indicate pixel to ignore", GR_SC_MISC | 
    GR_SC_PARAM, 3,
    {
      {"Camera ID (0 or 1)", 0, 1, 'i', ""},
      {"x (0=left)", 0, CAM_WIDTH, 'i', ""},
      {"y (0=top)", 0, CAM_HEIGHT, 'i', ""}
    }
  },
  {COMMAND(cam_blob_params), "set blob finder params", GR_SC_PARAM, 4,
    {
      {"Max number of blobs", 1, MAX_15BIT, 'i', "sc_maxblob"},
      {"Search grid size (pix)", 1, CAM_WIDTH, 'i', "sc_grid"},
      {"Threshold (# sigma*1000)", 0, 100, 'f', "sc_thresh"},
      {"Min blob separation ^2 (pix^2)", 1, CAM_WIDTH, 'i', "sc_mdist"}
    }
  },
  {COMMAND(cam_lens_any), "execute lens command directly", GR_SC_MISC, 1,
    {
      {"Lens command string", 0, 32, 's', ""}
    }
  },
  {COMMAND(cam_lens_move), "move camera lens", GR_SC_MISC, 1,
    {
      {"New position (ticks)", 0, MAX_15BIT, 'i', ""}
    }
  },
  {COMMAND(cam_lens_params), "set starcam lens params", GR_SC_PARAM, 1,
    {
      {"Allowed move error (ticks)", 0, MAX_15BIT, 'i', ""}
    }
  },

  {COMMAND(table_move), "move star camera by relative angle", GR_SC_MISC, 1,
    {
      {"Relative angle (deg)", -360, 360, 'd', "table_move"}
    }
  },
  {COMMAND(table_move_g), "Gains to find move velocity from length",
    GR_TABLE, 1,
    {
      {"P", 0, 100, 'f', "g_table_move"}
    }
  },
  {COMMAND(table_gain), "starcam rotary table gains", GR_TABLE, 2,
    {
      {"Proportional Gain", 0, MAX_15BIT, 'i', "g_p_table"},
      {"Integral Gain",     0, MAX_15BIT, 'i', "g_i_table"},
      {"Derivative Gain",   0, MAX_15BIT, 'i', "g_d_table"}
    }
  },
  //Gondola Pointing Commands
  {COMMAND(pt_spin_gains), "Spin mode gains", GR_PT_GAIN, 4,
   {
     {"SP_R1 RW (prop to vel error)",((-1.0)*SPR1_LIM),SPR1_LIM,'f',"spin_gain_r1"},
     {"SP_R2 RW (prop to gond accel)",((-1.0)*SPR2_LIM),SPR2_LIM,'f',"spin_gain_r2"},
     {"SP_P1 Pivot (prop to vel error)",((-1.0)*SPP1_LIM),SPP1_LIM,'f',"spin_gain_p1"},
     {"SP_P2 Pivot (prop to RW Vel)",((-1.0)*SPP2_LIM),SPP2_LIM,'f',"spin_gain_p2"},
   }
  },
  {COMMAND(pt_scan_gains), "Scan mode gains", GR_PT_GAIN, 4,
   {
     {"SC_R1 RW (prop to vel error)",((-1.0)*SCR1_LIM),SCR1_LIM,'f',"scan_gain_r1"},
     {"SC_R2 RW (prop to gond accel)",((-1.0)*SCR2_LIM),SCR2_LIM,'f',"scan_gain_r2"},
     {"SC_P1 Pivot (prop to vel error)",((-1.0)*SCP1_LIM),SCP1_LIM,'f',"scan_gain_p1"},
     {"SC_P2 Pivot (prop to RW Vel)",((-1.0)*SCP2_LIM),SCP2_LIM,'f',"scan_gain_p2"},
   }
  },
  {COMMAND(pt_spin_vel), "Spin mode velocity", GR_PT_PARAM, 1,
   {
     {"Velocity (dps)",-180,180,'f',"dps_gond_req"},
   }
  },
  {COMMAND(pt_scan_params), "Scan mode parameters", GR_PT_PARAM, 4,
   {
     {"Scan centre (deg)",0,360,'f',"scan_az_centre"},
     {"Scan period (s)",1,60,'f',"scan_az_period"},
     {"Scan width (deg)",0.1,120,'f',"scan_az_width"},
     {"Edge width (constant accel)(deg)",0,120,'f',"scan_az_wcrit"},
   }
  },
  {COMMAND(pt_point_params), "Point mode parameters", GR_PT_PARAM, 2,
   {
     {"Pointing Azimuth (deg)",0,360,'f',"point_az"},
     {"Azimuth drift tolerence (deg)",0,360,'f',"point_tol"},
   }
  }

};
