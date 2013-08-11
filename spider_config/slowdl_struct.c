/* slowdl_struct.c: contains the slow channel lists
 *
 * This software is copyright (C) 2013 University of Toronto
 *
 * This file is part of mcp/pcm licensed under the GNU General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "slowdl.h"

// If encoding is SDL_RAW, then type can be c, s, u, S, or U.
// if encoding is SDL_SCALE, then type can be s, u, or U.
struct SlowDlStruct slowDLList[] = {
  {"time_i_flc", 'U', SDL_RAW},
  {"time_b_flc", 'U', SDL_RAW},
  {"frame_g", 'u', SDL_RAW},
  {"frame_b", 'u', SDL_RAW},  
  {"frame_u", 'u', SDL_RAW},
  {"nblobs_g", 'c', SDL_RAW},
  {"nblobs_b", 'c', SDL_RAW},
  {"nblobs_u", 'c', SDL_RAW},
  {"mapmean_g", 'c', SDL_SCALE, 0.0, 65535.0},
  {"mapmean_b", 'c', SDL_SCALE, 0.0, 65535.0},
  {"mapmean_u", 'c', SDL_SCALE, 0.0, 65535.0},
  {"az_pss", 'c', SDL_SCALE, 0.0, 360.0},
  {"az_dgps", 'c', SDL_SCALE, 0.0, 360.0},
  {"az_mag", 'c', SDL_SCALE, 0.0, 360.0},
  {"el", 'c', SDL_SCALE, 10.0, 60.0},
  {"plover", 'c', SDL_RAW},
  {"count_b_cmd", 'u', SDL_RAW},
  {"count_i_cmd", 'u', SDL_RAW},
  {"last_b_cmd", 'u', SDL_RAW},
  {"last_i_cmd", 'u', SDL_RAW},
  {"i_of_tot", 'c', SDL_SCALE, 0.0, 30.0},
  //{"i_if_tot", 'c', SDL_SCALE, 0.0, 30.0},
  {"v_batt_of_cc", 'c', SDL_SCALE, 18.0, 36.0},
  {"v_batt_if_cc", 'c', SDL_SCALE, 18.0, 36.0},
  {"t_cpu_i_flc", 'c', SDL_SCALE, -10.0, 90.0},
  {"t_cpu_b_flc", 'c', SDL_SCALE, -10.0, 90.0},
  {"i_piv", 'c', SDL_SCALE, 0, 10.0},
  {"i_rw", 'c', SDL_SCALE, 0, 10.0},
  {"vd_mt_botlo_t_hk", 'c', SDL_SCALE, 0.551160, 1.689010},                                        
  {"vd_sft_bottom_t_hk", 'c', SDL_SCALE, 0.090600, 1.694050},                                    
  {"vd_capillary_t_hk", 'c', SDL_SCALE, 0.090620, 1.682190},                                     
  {"vd_vcs1_filter_t_hk", 'c', SDL_SCALE, 0.090620, 1.698120},                                 
  {"vd_vcs2_filter_t_hk", 'c', SDL_SCALE, 0.090620, 1.698120},                                 
  {"vd_pump_x1_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_pump_x2_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_pump_x3_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_pump_x4_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                    
  {"vd_pump_x6_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                    
  {"vd_hsw_x1_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_hsw_x2_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_hsw_x3_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_hsw_x4_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_hsw_x6_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                     
  {"vd_cp_x1_hk", 'c', SDL_SCALE, 0.570700, 1.808540},                                       
  {"vd_cp_x2_hk", 'c', SDL_SCALE, 0.571470, 1.820540},                                           
  {"vd_cp_x3_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                           
  {"vd_cp_x4_hk", 'c', SDL_SCALE, 0.584350, 1.797770},                                             
  {"vd_cp_x6_hk", 'c', SDL_SCALE, 0.584350, 1.797770}, 
  {"vr_still_x1_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_still_x2_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_still_x3_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_still_x4_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_still_x5_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_still_x6_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x1_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x2_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x3_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x4_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x5_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vr_fp_x6_hk", 'c', SDL_LOG, 2E-7, 0.000105},
  {"vd_ssa_x1_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"vd_ssa_x2_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"vd_ssa_x3_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"vd_ssa_x4_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"vd_ssa_x5_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"vd_ssa_x6_hk", 'c', SDL_SCALE, 0.05, 1.8 },
  {"v_cnx_x1_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"v_cnx_x2_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"v_cnx_x3_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"v_cnx_x4_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"v_cnx_x5_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"v_cnx_x6_hk", 'c', SDL_SCALE, 0.0, 5.0},
  {"alive_mpcs", 'c', SDL_RAW},
  {"mce_power", 'c', SDL_RAW},
  {"squid_veto_mpc", 'c', SDL_RAW},

  //{"clamp_count_mce1", 'c', SDL_RAW},
  {"state_mpc1", 's', SDL_RAW},
  {"drive_map_mpc1", 'c', SDL_RAW},
  {"t_mce1", 'c', SDL_SCALE, -20, 90},
  {"t_mcc1", 'c', SDL_SCALE, -20, 90},
  //{"clamp_count_mce2", 'c', SDL_RAW},
  {"state_mpc2", 's', SDL_RAW},
  {"drive_map_mpc2", 'c', SDL_RAW},
  {"t_mce2", 'c', SDL_SCALE, -20, 90},
  {"t_mcc2", 'c', SDL_SCALE, -20, 90},
  //{"clamp_count_mce3", 'c', SDL_RAW},
  {"state_mpc3", 's', SDL_RAW},
  {"drive_map_mpc3", 'c', SDL_RAW},
  {"t_mce3", 'c', SDL_SCALE, -20, 90},
  {"t_mcc3", 'c', SDL_SCALE, -20, 90},
  //{"clamp_count_mce4", 'c', SDL_RAW},
  {"state_mpc4", 's', SDL_RAW},
  {"drive_map_mpc4", 'c', SDL_RAW},
  {"t_mce4", 'c', SDL_SCALE, -20, 90},
  {"t_mcc4", 'c', SDL_SCALE, -20, 90},
  //{"clamp_count_mce5", 'c', SDL_RAW},
  {"state_mpc5", 's', SDL_RAW},
  {"drive_map_mpc5", 'c', SDL_RAW},
  {"t_mce5", 'c', SDL_SCALE, -20, 90},
  {"t_mcc5", 'c', SDL_SCALE, -20, 90},
  //{"clamp_count_mce6", 'c', SDL_RAW},
  {"state_mpc6", 's', SDL_RAW},
  {"drive_map_mpc6", 'c', SDL_RAW},
  {"t_mce6", 'c', SDL_SCALE, -20, 90},
  {"t_mcc6", 'c', SDL_SCALE, -20, 90},
  {""}
};

/*
MT Bot Lo
Cappilary
sft bot
vcs1 filt
vcs2 filt

all inserts
Cond Pt
Still
FPU
SSA
Heat Switch
Pump
4k Plate

*/