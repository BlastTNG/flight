/* slow_dl.c: slow downlink packet specification
 *
 * This software is copyright (C) 2004-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "share/channels.h"
#include "slow_dl.h"
#include "share/blast.h"

/* SLOWDL_FORCE_INT force an integer on numbits between min and max */
/* SLOWDL_U_MASK    masks off all but the lowest numbits and sends those */
/* SLOWDL_TAKE_BIT  takes bit numbits, counting from 1 */
/* "src", type, numbits, min, max */
struct SlowDLStruct SlowDLInfo[SLOWDL_NUM_DATA] = {

    {"lst",           SLOWDL_FORCE_INT, 16,     0, 24},
    {"status_mcc",    SLOWDL_U_MASK,    16},
    {"lat",           SLOWDL_FORCE_INT, 16, -90.0, -30},
    {"lon",           SLOWDL_FORCE_INT, 16,  -180, 360},
    {"alt",           SLOWDL_U_MASK,    16},
    {"disk_free",     SLOWDL_U_MASK,    16},
    {"timeout",       SLOWDL_U_MASK,    16},
    {"plover",        SLOWDL_U_MASK,    16},
    {"time",          SLOWDL_U_MASK,    32},

    {"veto_sensor",   SLOWDL_U_MASK,    16},
    {"ra",            SLOWDL_FORCE_INT, 16,    0, 24},
    {"dec",           SLOWDL_FORCE_INT, 16,  -90, 90},
    {"az",            SLOWDL_FORCE_INT, 16,    0, 360},
    {"el",            SLOWDL_FORCE_INT, 16,   20, 60},
    {"az_sun",        SLOWDL_U_MASK,    16},

    {"i_tot",         SLOWDL_U_MASK, 16},
    {"i_das",         SLOWDL_U_MASK, 16},
    {"i_acs",         SLOWDL_U_MASK, 16},
    {"i_rec",         SLOWDL_U_MASK, 16},
    {"i_trans",       SLOWDL_U_MASK, 16},
    {"i_sc",          SLOWDL_U_MASK, 16},
    {"i_dgps",        SLOWDL_U_MASK, 16},
    {"i_step",        SLOWDL_U_MASK, 16},
    {"i_flc",         SLOWDL_U_MASK, 16},
    {"i_gy",          SLOWDL_U_MASK, 16},
    {"i_el",          SLOWDL_U_MASK, 16},
    {"i_piv",         SLOWDL_U_MASK, 16},
    {"i_rw",          SLOWDL_U_MASK, 16},

    {"v_batt_cc",     SLOWDL_U_MASK, 16},
    {"v_arr_cc",      SLOWDL_U_MASK, 16},

    {"t_wd_flc",      SLOWDL_U_MASK, 16},
    {"t_acs",         SLOWDL_U_MASK, 16},
    {"t_lock",        SLOWDL_U_MASK, 16},
    {"t_port_das",    SLOWDL_U_MASK, 16},
    {"t_port_rec",    SLOWDL_U_MASK, 16},
    {"t_padcdc_rec",  SLOWDL_U_MASK, 16},
    {"t_gy",          SLOWDL_U_MASK, 16},
    {"t_1_bat",       SLOWDL_U_MASK, 16},
    {"t_array",       SLOWDL_U_MASK, 16},
    {"t_prime_sf",    SLOWDL_U_MASK, 16},
    {"t_second_sf",   SLOWDL_U_MASK, 16},
    {"t_chin",        SLOWDL_U_MASK, 16},
    {"t_earth",       SLOWDL_U_MASK, 16},

    {"td_ln",         SLOWDL_FORCE_INT, 16,  2.5,  7.5},
    {"td_lhe",        SLOWDL_FORCE_INT, 16,  2.5,  7.5},
    {"td_vcs_filt",   SLOWDL_FORCE_INT, 16,  2.5,  7.5},
    {"td_charcoal",   SLOWDL_FORCE_INT, 16,  2.5,  7.5},
    {"td_jfet",       SLOWDL_FORCE_INT, 16,  2.5,  7.5},
    {"tr_hwpr",       SLOWDL_FORCE_INT, 16, 1000,  8000},
    {"tr_m4",         SLOWDL_FORCE_INT, 16, 1000,  8000},
    {"tr_300mk_strap",SLOWDL_FORCE_INT, 16, 1000,  8000},

    {"focus_sf",      SLOWDL_U_MASK,    16},

    {"ampl_500_bias", SLOWDL_U_MASK,    16},
    {"ampl_350_bias", SLOWDL_U_MASK,    16},
    {"ampl_250_bias", SLOWDL_U_MASK,    16},
    {"ampl_rox_bias", SLOWDL_U_MASK,    16},
    {"n13_phase",     SLOWDL_U_MASK,    16},
    {"n17_phase",     SLOWDL_U_MASK,    16},
    {"n18_phase",     SLOWDL_U_MASK,    16},
    {"n19_phase",     SLOWDL_U_MASK,    16},
    {"n21_phase",     SLOWDL_U_MASK,    16},
    {"n22_phase",     SLOWDL_U_MASK,    16},
    {"n23_phase",     SLOWDL_U_MASK,    16},
    {"n25_phase",     SLOWDL_U_MASK,    16},
    {"n26_phase",     SLOWDL_U_MASK,    16},
    {"n27_phase",     SLOWDL_U_MASK,    16},
    {"n29_phase",     SLOWDL_U_MASK,    16},
    {"n30_phase",     SLOWDL_U_MASK,    16},
    {"n31_phase",     SLOWDL_U_MASK,    16},

    {"nblobs_sbsc",   SLOWDL_U_MASK,    16},
    {"nblobs_isc",    SLOWDL_U_MASK,    16},
    {"nblobs_osc",    SLOWDL_U_MASK,    16},
    {"mapmean_sbsc",  SLOWDL_U_MASK,    16},
    {"mapmean_isc",   SLOWDL_U_MASK,    16},
    {"mapmean_osc",   SLOWDL_U_MASK,    16},

    {"mask_gy",       SLOWDL_U_MASK,    16},

    {"pot_lock",      SLOWDL_U_MASK,    16}
};

void InitSlowDL(void) {
  int i;
  double dtmp;
  struct NiosStruct *address;
  
  for (i = 0; i < SLOWDL_NUM_DATA; i++) {
    address = GetNiosAddr(SlowDLInfo[i].src);
    SlowDLInfo[i].wide = address->wide;
    SlowDLInfo[i].mindex = ExtractBiPhaseAddr(address)->index;
    SlowDLInfo[i].chnum = ExtractBiPhaseAddr(address)->channel;
  
    SlowDLInfo[i].max = (SlowDLInfo[i].calib_max - address->b) / address->m;
    SlowDLInfo[i].min = (SlowDLInfo[i].calib_min - address->b) / address->m;
    
    if( SlowDLInfo[i].max <  SlowDLInfo[i].min ) {
      dtmp = SlowDLInfo[i].max;
      SlowDLInfo[i].max = SlowDLInfo[i].min;
      SlowDLInfo[i].min = dtmp;
    }
  }
}
