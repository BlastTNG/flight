#include "compressstruct.h"
#include "compressconst.h"

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];

/* Defined in compressstruct.h - included here for reference.
struct fieldStreamStruct {
  char name[256];
  int gain; // x_down = x/gain
  int samples_per_frame; // samples per ~1Hz frame
  int doAverage; // boxcar average before decimation
  int doDifferentiate; // send down derivative, not value
  int bits; // number of bits in stream: 4, 8, 16, 32
  int spikeMode; // SLOW: skip samples  SPIKE: report enlarged data
};
*/
// el_dest_mc, i_el
struct fieldStreamStruct streamList[N_OTH_SETS][MAX_OTH_STREAM_FIELDS] = {
  { // field set 0
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},
  {"framenum",1,1,NOAVG,DX,8,SLOW},
  {"bi0_fifo_size", 1, 5, NOAVG, NODX, 16, SLOW},
  {"bbc_fifo_size", 1, 5, NOAVG, NODX, 16, SLOW},
  {"mce000", 1, 5, NOAVG, NODX, 16, SLOW},
  {"mce001", 1, 5, NOAVG, NODX, 16, SLOW},
  {"timeout_b",1,1,NOAVG,DX,8,SLOW},
  {"timeout_i",1,1,NOAVG,DX,8,SLOW},
  {"ofpch_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ofroll_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"az", 33140, 5, AVG, DX, 8, SLOW},
  {"el", 33140, 5, AVG, DX, 8, SLOW},
  {"ra", 16570, 1, NOAVG, DX, 8, SLOW},
  {"dec", 16570, 1, NOAVG, DX, 8, SLOW},
  {"el_raw_1_enc", 5, 1, AVG, DX, 8, SLOW},
  {"el_raw_2_enc", 5, 1, AVG, DX, 8, SLOW},
  {"az_pss", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_mag", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_dgps", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss1", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss2", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss3", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss4", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss5", 5, 1, NOAVG, DX, 8, SLOW},
  {"az_raw_pss6", 5, 1, NOAVG, DX, 8, SLOW},
  {"step_1_el", 3, 5, NOAVG, NODX, 8, SLOW},
  {"step_2_el", 3, 5, NOAVG, NODX, 8, SLOW},
  {"dac_rw", 256, 5, NOAVG, NODX, 8, SLOW},
  {"dac_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_el", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 5, NOAVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 5, NOAVG, NODX, 8, SLOW},
  {"i_term_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_term_az", 256, 5, NOAVG, NODX, 8, SLOW},
  {"i_v_rw_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_t_rw_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_v_req_az_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_v_rw_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"p_v_az_term_piv", 256, 5, NOAVG, NODX, 8, SLOW},
  {"res_piv", 1, 1, NOAVG, DX, 8, SLOW},
  {"vel_ser_rw", 1, 5, NOAVG, DX, 8, SLOW},
  {"pulse_sc", 1, 100, NOAVG, NODX, 8, SLOW},

  {"ofpch_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofpch_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  
END_OF_STREAM
  },
  
  {  // field set 1
  {"time",1,1,NOAVG,NODX,8,SLOW},
  {"time_usec", 5000, 1, NOAVG, NODX, 8,SLOW},

  {"ofpch_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_1_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofpch_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofyaw_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  {"ofroll_2_gy", 5461, 10, AVG, NODX, 8, SLOW},
  
END_OF_STREAM
  }
};

char *frameList[] = {
  "count_1_el",
  "count_2_el",
  "time",
  "time_sip",
  "time_dgps",
  "lst",
  "parts_sched",
  "frame_thegood",
  "lat",
  "lon",
  "sec_thegood",
  "ra",
  "frame_thebad",
  "sec_thebad",
  "dec",
  "lst_sched",
  "frame_theugly",
  "sec_theugly",
  "time_b_flc",
  "time_i_flc",
  "start_1_cycle",
  "start_2_cycle",
  "start_3_cycle",
  "start_4_cycle",
  "start_5_cycle",
  "start_6_cycle",
  "start_state_1_cycle",
  "start_state_2_cycle",
  "start_state_3_cycle",
  "start_state_4_cycle",
  "start_state_5_cycle",
  "start_state_6_cycle",
  "vr_still_2_hk",
  "vr_mux_2_hk",
  "vr_still_3_hk",
  "vr_mux_3_hk",
  "vr_still_1_hk",
  "vr_mux_1_hk",
  "vr_still_5_hk",
  "vr_mux_5_hk",
  "vr_still_6_hk",
  "vr_mux_6_hk",
  "vr_still_4_hk",
  "vr_mux_4_hk",
  "vd_4k_2_hk",
  "vd_cp_2_hk",
  "vd_pump_2_hk",
  "vd_hsw_2_hk",
  "vd_plate_2_hk",
  "vd_snout_2_hk",
  "vd_eyepiece_2_hk",
  "vd_objective_2_hk",
  "vd_spittoon_2_hk",
  "vd_aux_post_2_hk",
  "vd_stop_2_hk",
  "vd_ssa_2_hk",
  "vd_4k_1_hk",
  "vd_cp_1_hk",
  "vd_pump_1_hk",
  "vd_hsw_1_hk",
  "vd_plate_1_hk",
  "vd_snout_1_hk",
  "vd_eyepiece_1_hk",
  "vd_objective_1_hk",
  "vd_spittoon_1_hk",
  "vd_aux_post_1_hk",
  "vd_stop_1_hk",
  "vd_ssa_1_hk",
  "vd_4k_4_hk",
  "vd_cp_4_hk",
  "vd_pump_4_hk",
  "vd_hsw_4_hk",
  "vd_plate_4_hk",
  "vd_snout_4_hk",
  "vd_eyepiece_4_hk",
  "vd_objective_4_hk",
  "vd_spittoon_4_hk",
  "vd_aux_post_4_hk",
  "vd_stop_4_hk",
  "vd_ssa_4_hk",
  "vd_4k_3_hk",
  "vd_cp_3_hk",
  "vd_pump_3_hk",
  "vd_hsw_3_hk",
  "vd_plate_3_hk",
  "vd_snout_3_hk",
  "vd_eyepiece_3_hk",
  "vd_objective_3_hk",
  "vd_spittoon_3_hk",
  "vd_aux_post_3_hk",
  "vd_stop_3_hk",
  "vd_ssa_3_hk",
  "vd_4k_6_hk",
  "vd_cp_6_hk",
  "vd_pump_6_hk",
  "vd_hsw_6_hk",
  "vd_plate_6_hk",
  "vd_snout_6_hk",
  "vd_eyepiece_6_hk",
  "vd_objective_6_hk",
  "vd_spittoon_6_hk",
  "vd_aux_post_6_hk",
  "vd_stop_6_hk",
  "vd_ssa_6_hk",
  "vd_4k_5_hk",
  "vd_cp_5_hk",
  "vd_pump_5_hk",
  "vd_hsw_5_hk",
  "vd_plate_5_hk",
  "vd_snout_5_hk",
  "vd_eyepiece_5_hk",
  "vd_objective_5_hk",
  "vd_spittoon_5_hk",
  "vd_aux_post_5_hk",
  "vd_stop_5_hk",
  "vd_ssa_5_hk",
  "vd_00_hk",
  "vd_01_hk",
  "vd_02_hk",
  "vd_03_hk",
  "vd_05_hk",
  "vd_06_hk",
  "vd_07_hk",
  "vd_08_hk",
  "vd_09_hk",
  "vd_10_hk",
  "vd_11_hk",
  "vd_12_hk",
  "vd_13_hk",
  "vd_14_hk",
  "vd_15_hk",
  "vd_16_hk",
  "vd_17_hk",
  "mic_time",
  "ph_cnx_2_hk",
  "ph_ntd_2_hk",
  "ph_cnx_1_hk",
  "ph_ntd_1_hk",
  "ph_cnx_3_hk",
  "ph_ntd_3_hk",
  "ph_cnx_4_hk",
  "ph_ntd_4_hk",
  "ph_cnx_6_hk",
  "ph_ntd_6_hk",
  "ph_cnx_5_hk",
  "ph_ntd_5_hk",
  "f_bias_cmd_hk",
  "v_cnx_2_hk",
  "v_ntd_2_hk",
  "heat_strap_2_hk",
  "heat_fplo_2_hk",
  "v_cnx_1_hk",
  "v_ntd_1_hk",
  "heat_strap_1_hk",
  "heat_fplo_1_hk",
  "v_cnx_3_hk",
  "v_ntd_3_hk",
  "heat_strap_3_hk",
  "heat_fplo_3_hk",
  "v_cnx_4_hk",
  "v_ntd_4_hk",
  "heat_strap_4_hk",
  "heat_fplo_4_hk",
  "v_cnx_6_hk",
  "v_ntd_6_hk",
  "heat_strap_6_hk",
  "heat_fplo_6_hk",
  "v_cnx_5_hk",
  "v_ntd_5_hk",
  "heat_strap_5_hk",
  "heat_fplo_5_hk",
  "ifpwr",
  "hwp_bias",
  "phase_00_hwp",
  "phase_01_hwp",
  "phase_02_hwp",
  "phase_03_hwp",
  "phase_04_hwp",
  "phase_05_hwp",
  "phase_06_hwp",
  "phase_07_hwp",
  "phase_08_hwp",
  "phase_09_hwp",
  "phase_10_hwp",
  "phase_11_hwp",
  "phase_12_hwp",
  "phase_13_hwp",
  "phase_14_hwp",
  "phase_15_hwp",
  "phase_16_hwp",
  "phase_17_hwp",
  "phase_18_hwp",
  "phase_19_hwp",
  "phase_20_hwp",
  "phase_21_hwp",
  "phase_22_hwp",
  "phase_23_hwp",
  "phase_24_hwp",
  "vt_1_if",
  "vt_2_if",
  "vt_3_if",
  "vt_4_if",
  "vt_5_if",
  "vt_6_if",
  "vt_7_if",
  "i_mce",
  "i_hk_misc",
  "v_sft_valve_hk",
  "g_com_el",
  "foc_res_thegood",
  "status_eth",
  "timeout_b",
  "az_sun",
  "status_flc",
  "upslot_sched",
  "t_chip_flc",
  "declination_mag",
  "veto_sensor",
  "timeout_i",
  "df_b_flc",
  "alt_sip",
  "mapmean_thegood",
  "lat_sip",
  "lon_sip",
  "lat_dgps",
  "lon_dgps",
  "alt_dgps",
  "speed_dgps",
  "dir_dgps",
  "climb_dgps",
  "att_ok_dgps",
  "g_p_table",
  "g_i_table",
  "g_d_table",
  "n_sat_dgps",
  "df_i_flc",
  "mode_p",
  "x_p",
  "y_p",
  "vel_az_p",
  "del_p",
  "nblobs_thegood",
  "blob00_x_thegood",
  "blob00_y_thegood",
  "blob00_f_thegood",
  "blob00_s_thegood",
  "blob01_x_thegood",
  "blob01_y_thegood",
  "blob01_f_thegood",
  "blob01_s_thegood",
  "blob02_x_thegood",
  "blob02_y_thegood",
  "blob02_f_thegood",
  "blob02_s_thegood",
  "w_p",
  "move_tol_thegood",
  "exp_int_thegood",
  "exp_time_thegood",
  "force_thegood",
  "blob_idx_thegood",
  "blob_idx_thebad",
  "blob_idx_theugly",
  "thresh_thegood",
  "grid_thegood",
  "cal_off_pss1",
  "cal_off_pss2",
  "mdist_thegood",
  "cal_off_pss3",
  "cal_off_pss4",
  "cal_off_pss5",
  "accel_max_az",
  "cal_off_pss6",
  "cal_d_pss1",
  "cal_d_pss2",
  "offset_ofpch_gy",
  "offset_ofroll_gy",
  "offset_ofyaw_gy",
  "az_raw_mag",
  "sigma_mag",
  "az_dgps",
  "sigma_dgps",
  "cal_d_pss3",
  "cal_d_pss4",
  "mapsigma_thegood",
  "cal_d_pss5",
  "az_mag",
  "cal_d_pss6",
  "cal_imin_pss",
  "t_cpu_i_flc",
  "bbc_fifo_size",
  "t_cpu_b_flc",
  "t_mb_flc",
  "mks_hi_sip",
  "mks_med_sip",
  "nblobs_thebad",
  "blob00_x_thebad",
  "blob00_y_thebad",
  "blob00_f_thebad",
  "blob00_s_thebad",
  "blob01_x_thebad",
  "blob01_y_thebad",
  "blob01_f_thebad",
  "blob01_s_thebad",
  "blob02_x_thebad",
  "blob02_y_thebad",
  "blob02_f_thebad",
  "blob02_s_thebad",
  "mapmean_thebad",
  "ra_thegood",
  "dec_thegood",
  "roll_thegood",
  "ra_thebad",
  "foc_res_thebad",
  "dec_thebad",
  "roll_thebad",
  "mapsigma_thebad",
  "move_tol_thebad",
  "exp_int_thebad",
  "exp_time_thebad",
  "force_thebad",
  "ra_theugly",
  "dec_theugly",
  "roll_theugly",
  "thresh_thebad",
  "grid_thebad",
  "mdist_thebad",
  "focpos_thegood",
  "focpos_thebad",
  "focpos_theugly",
  "maxblob_thegood",
  "maxblob_thebad",
  "bi0_fifo_size",
  "plover",
  "ccd_t_thegood",
  "ccd_t_thebad",
  "g_p_heat_gy",
  "g_i_heat_gy",
  "g_d_heat_gy",
  "t_set_gy",
  "trim_pss",
  "az_pss",
  "ra_1_p",
  "dec_1_p",
  "ra_2_p",
  "dec_2_p",
  "ra_3_p",
  "dec_3_p",
  "ra_4_p",
  "dec_4_p",
  "trim_null",
  "trim_mag",
  "trim_dgps",
  "az_raw_dgps",
  "h_p",
  "mks_lo_sip",
  "alt",
  "mode_az_mc",
  "mode_el_mc",
  "dest_az_mc",
  "dest_el_mc",
  "vel_az_mc",
  "vel_el_mc",
  "dir_az_mc",
  "dir_el_mc",
  "slew_veto",
  "sveto_len",
  "dith_el",
  "state_lock",
  "pitch_mag",
  "el_sun",
  "vel_ser_rw",
  "stat_dr_rw",
  "stat_s1_rw",
  "stat_dr_piv",
  "stat_s1_piv",
  "i_ser_rw",
  "res_piv",
  "i_ser_piv",
  "g_v_rw_piv",
  "g_i_rw_piv",
  "g_v_az_piv",
  "g_t_rw_piv",
  "set_rw",
  "vel_dps_az",
  "i_v_rw_term_piv",
  "p_t_rw_term_piv",
  "p_v_req_az_term_piv",
  "vel_ser_piv",
  "g_v_req_az_piv",
  "az_raw_pss1",
  "az_raw_pss2",
  "drive_info_rw",
  "drive_err_cts_rw",
  "drive_info_piv",
  "drive_err_cts_piv",
  "vel_hwp",
  "i_move_hwp",
  "status_hwp",
  "pos_1_hwp",
  "pos_2_hwp",
  "pos_3_hwp",
  "pitch_raw_dgps",
  "roll_raw_dgps",
  "verbose_rw",
  "verbose_piv",
  "p_v_rw_term_piv",
  "p_v_az_term_piv",
  "v_batt_cc1",
  "v_arr_cc1",
  "i_batt_cc1",
  "i_arr_cc1",
  "t_hs_cc1",
  "fault_cc1",
  "alarm_hi_cc1",
  "alarm_lo_cc1",
  "v_targ_cc1",
  "state_cc1",
  "led_cc1",
  "v_batt_cc2",
  "v_arr_cc2",
  "i_batt_cc2",
  "i_arr_cc2",
  "t_hs_cc2",
  "fault_cc2",
  "alarm_hi_cc2",
  "alarm_lo_cc2",
  "v_targ_cc2",
  "state_cc2",
  "led_cc2",
  "azraw_pss",
  "elraw_pss",
  "snr_pss1",
  "snr_pss2",
  "snr_pss3",
  "snr_pss4",
  "snr_pss5",
  "snr_pss6",
  "accel_az",
  "az_raw_pss3",
  "az_raw_pss4",
  "az_cov_dgps",
  "pitch_cov_dgps",
  "roll_cov_dgps",
  "force_theugly",
  "exp_int_theugly",
  "exp_time_theugly",
  "foc_res_theugly",
  "move_tol_theugly",
  "maxblob_theugly",
  "grid_theugly",
  "thresh_theugly",
  "mdist_theugly",
  "mapmean_theugly",
  "mapsigma_theugly",
  "ccd_t_theugly",
  "nblobs_theugly",
  "blob00_x_theugly",
  "blob00_y_theugly",
  "blob00_f_theugly",
  "blob00_s_theugly",
  "blob01_x_theugly",
  "blob01_y_theugly",
  "blob01_f_theugly",
  "blob01_s_theugly",
  "blob02_x_theugly",
  "blob02_y_theugly",
  "blob02_f_theugly",
  "blob02_s_theugly",
  "g_pt_az",
  "pos_4_hwp",
  "pos_5_hwp",
  "pos_6_hwp",
  "rate_tdrss",
  "rate_iridium",
  "az_raw_pss5",
  "az_raw_pss6",
  "el_raw_pss1",
  "el_raw_pss2",
  "el_raw_pss3",
  "el_raw_pss4",
  "el_raw_pss5",
  "el_raw_pss6",
  "insert_last_hk",
  "f_bias_hk",
  "v_heat_last_hk",
  "i_tot",
  "cov_lim_dgps",
  "ant_e_dgps",
  "ant_n_dgps",
  "ant_u_dgps",
  "ants_lim_dgps",
  "frame_int_bbc",
  "rate_ext_bbc",
  "frame_ext_bbc",
  "rate_frame_bbc",
  "rate_samp_adc",
  "enc_cnt_2_hwp",
  "enc_cnt_3_hwp",
  "enc_cnt_4_hwp",
  "enc_cnt_5_hwp",
  "enc_cnt_6_hwp",
  "fset",
  "state_1_cycle",
  "state_2_cycle",
  "state_3_cycle",
  "state_4_cycle",
  "state_5_cycle",
  "state_6_cycle",
  "band_az",
  "is_turn_around",
  "row_len_sync",
  "num_rows_sync",
  "free_run_sync",
  "last_b_cmd",
  "last_i_cmd",
  "count_b_cmd",
  "count_i_cmd",
  "latch0",
  "latch1",
  "switch_gy",
  "switch_grp2",
  "switch_misc",
  "heat_gy",
  "i_trans",
  "i_das",
  "i_acs",
  "i_mcc",
  "i_sc",
  "i_dgps",
  "i_piv",
  "i_el",
  "i_flc",
  "i_rw",
  "i_step",
  "i_gy",
  "t_gy",
  "t_serial",
  "t_mc_piv",
  "t_piv",
  "t_wd_flc",
  "t_port_hexc",
  "t_1_bat",
  "t_2_bat",
  "t_port_back",
  "t_dgps",
  "t_star_front",
  "t_acs",
  "t_dcdc_acs",
  "t_mc_lock",
  "t_lock",
  "t_bsc",
  "t_box_bal",
  "t_pump_bal",
  "t_el",
  "t_array",
  "t_sun",
  "vt_rw",
  "t_earth",
  "t_chin",
  "t_port_pyr",
  "dac2_ampl",
  "dac_piv",
  "mask_gy",
  "control_lock",
  "g_p_az",
  "g_i_az",
  "enc1_offset",
  "enc2_offset",
  "fault_gy",
  "dac_rw",
  "p_term_az",
  "i_term_az",
  "error_az",
  "limit_lock",
  "pitch_piv_clin",
  "roll_piv_clin",
  "t_piv_clin",
  "pitch_of_clin",
  "roll_of_clin",
  "t_of_clin",
  "x_mag",
  "y_mag",
  "z_mag",
  "trig_thegood",
  "trig_thebad",
  "trig_theugly",
  "t_ofpch_gy",
  "trig_s_thegood",
  "trig_l_thegood",
  "trig_s_thebad",
  "trig_l_thebad",
  "trig_s_theugly",
  "trig_l_theugly",
  "v1_1_pss",
  "v2_1_pss",
  "v3_1_pss",
  "v4_1_pss",
  "v1_2_pss",
  "v2_2_pss",
  "v3_2_pss",
  "v4_2_pss",
  "v1_3_pss",
  "v2_3_pss",
  "v3_3_pss",
  "v4_3_pss",
  "v1_4_pss",
  "v2_4_pss",
  "v3_4_pss",
  "v4_4_pss",
  "v1_5_pss",
  "v2_5_pss",
  "v3_5_pss",
  "v4_5_pss",
  "v1_6_pss",
  "v2_6_pss",
  "v3_6_pss",
  "v4_6_pss",
  "heat_t_hk",
  "heat_13_hk",
  "heat_45_hk",
  "heat_26_hk",
  "mce_txmux",
  "data_mode",
  "mic_free",
  "ofyaw_1_gy",
  "ofyaw_2_gy",
  "ofroll_1_gy",
  "ofroll_2_gy",
  "ofpch_2_gy",
  "ofpch_1_gy",
  "enc_table",
  "az",
  "el",
  "vr_ntd1_2_hk",
  "vr_fp_2_hk",
  "vr_strap_2_hk",
  "vr_ntd4_2_hk",
  "vr_ntd3_2_hk",
  "vr_ntd2_2_hk",
  "vr_ntd1_3_hk",
  "vr_fp_3_hk",
  "vr_strap_3_hk",
  "vr_ntd4_3_hk",
  "vr_ntd3_3_hk",
  "vr_ntd2_3_hk",
  "vr_ntd1_1_hk",
  "vr_fp_1_hk",
  "vr_strap_1_hk",
  "vr_ntd4_1_hk",
  "vr_ntd3_1_hk",
  "vr_ntd2_1_hk",
  "vr_ntd1_5_hk",
  "vr_fp_5_hk",
  "vr_strap_5_hk",
  "vr_ntd4_5_hk",
  "vr_ntd3_5_hk",
  "vr_ntd2_5_hk",
  "vr_ntd1_6_hk",
  "vr_fp_6_hk",
  "vr_strap_6_hk",
  "vr_ntd4_6_hk",
  "vr_ntd3_6_hk",
  "vr_ntd2_6_hk",
  "vr_ntd1_4_hk",
  "vr_fp_4_hk",
  "vr_strap_4_hk",
  "vr_ntd4_4_hk",
  "vr_ntd3_4_hk",
  "vr_ntd2_4_hk",
  "vp_01_hk",
  "vp_02_hk",
  "enc_cnt_1_hwp",
  "framenum",
  "ofpch_gy",
  "ofroll_gy",
  "ofyaw_gy",
  "ofaz_gy",
  "vel_req_az",
  "step_1_el",
  "step_2_el",
  "el_raw_1_enc",
  "el_raw_2_enc",
  "vel_ofpch_gy",
  "pulse_sc",
  "dps_table",
  "res_rw",
  ""
};

