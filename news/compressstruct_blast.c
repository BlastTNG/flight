#include "compressstruct.h"

/*
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

struct fieldStreamStruct streamList[] = {
  {"time",1,5,AVG,NODX,8,SLOW},
  {"time_usec", 5000, 2, AVG, NODX, 8,SLOW},
  {"ifel_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifyaw_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"ifroll_gy", 5, 10, AVG, NODX, 8, SLOW},
  {"az", 33140, 10, AVG, DX, 8, SLOW},
  {"el", 33140, 10, AVG, DX, 8, SLOW},
  {"el_enc", 33140, 10, AVG, DX, 8, SLOW},
  {"el_raw_enc", 33140, 10, AVG, DX, 8, SLOW},
  {"el_raw_if_clin", 33140, 10, AVG, DX, 8, SLOW},
  {"el_clin", 33140, 10, AVG, DX, 8, SLOW},
  {"el_isc", 33140, 10, AVG, DX, 8, SLOW},
  {"el_osc", 33140, 10, AVG, DX, 8, SLOW},
  {"az_ss", 33140, 10, AVG, DX, 8, SLOW},
  {"az_dgps", 33140, 10, AVG, DX, 8, SLOW},
  {"az_isc", 33140, 10, AVG, DX, 8, SLOW},
  {"az_mag", 33140, 10, AVG, DX, 8, SLOW},
  {"az_osc", 33140, 10, AVG, DX, 8, SLOW},
  {"az_raw_dgps", 33140, 10, AVG, DX, 8, SLOW},
  {"az_rel_sun_ss", 33140, 10, AVG, DX, 8, SLOW},
  {"az_pss1", 33140, 10, AVG, DX, 8, SLOW},
  {"az_pss2", 33140, 10, AVG, DX, 8, SLOW},
  {"dac_el", 256, 10, AVG, NODX, 8, SLOW},
  {"dac_rw", 256, 10, AVG, NODX, 8, SLOW},
  {"dac_piv", 256, 10, AVG, NODX, 8, SLOW},
  {"i_ser_el", 256, 10, AVG, NODX, 8, SLOW},
  {"i_ser_piv", 100, 10, AVG, NODX, 8, SLOW},
  {"i_ser_rw", 100, 10, AVG, NODX, 8, SLOW},
  {"i_term_az", 256, 10, AVG, NODX, 8, SLOW},
  {"i_term_el", 256, 10, AVG, NODX, 8, SLOW},
  {"p_term_az", 256, 10, AVG, NODX, 8, SLOW},
  {"p_rw_term_piv", 100, 10, AVG, NODX, 8, SLOW},
  {"frict_term_piv", 100, 10, AVG, NODX, 8, SLOW},
  {"res_piv", 256, 10, AVG, NODX, 8, SLOW},
  {"n17c00", 5, 50, AVG, DX, 8, SLOW},
  {"n17c01", 5, 50, AVG, DX, 8, SLOW},
  {"n17c02", 5, 50, AVG, DX, 8, SLOW},
  {"n17c03", 5, 50, AVG, DX, 8, SLOW},
  {"n17c04", 5, 50, AVG, DX, 8, SLOW},
  {"n17c05", 5, 50, AVG, DX, 8, SLOW},
  {"n17c06", 5, 50, AVG, DX, 8, SLOW},
  {"n17c07", 5, 50, AVG, DX, 8, SLOW},
  {"n17c08", 5, 50, AVG, DX, 8, SLOW},
  {"n17c09", 5, 50, AVG, DX, 8, SLOW},
  {"n17c10", 5, 50, AVG, DX, 8, SLOW},
  {"n17c11", 5, 50, AVG, DX, 8, SLOW},
  {"n17c12", 5, 50, AVG, DX, 8, SLOW},
  {"n17c13", 5, 50, AVG, DX, 8, SLOW},
  {"n17c14", 5, 50, AVG, DX, 8, SLOW},
  {"n17c15", 5, 50, AVG, DX, 8, SLOW},
  {"n17c16", 5, 50, AVG, DX, 8, SLOW},
  {"n17c17", 5, 50, AVG, DX, 8, SLOW},
  {"n17c18", 5, 50, AVG, DX, 8, SLOW},
  {"n17c19", 5, 50, AVG, DX, 8, SLOW},
  {"n17c20", 5, 50, AVG, DX, 8, SLOW},
  {"n17c21", 5, 50, AVG, DX, 8, SLOW},
  {"n17c22", 5, 50, AVG, DX, 8, SLOW},
  {"n17c23", 5, 50, AVG, DX, 8, SLOW},
  {"n18c00", 5, 50, AVG, DX, 8, SLOW},
  {"n18c01", 5, 50, AVG, DX, 8, SLOW},
  {"n18c02", 5, 50, AVG, DX, 8, SLOW},
  {"n18c03", 5, 50, AVG, DX, 8, SLOW},
  {"n18c04", 5, 50, AVG, DX, 8, SLOW},
  {"n18c05", 5, 50, AVG, DX, 8, SLOW},
  {"n18c06", 5, 50, AVG, DX, 8, SLOW},
  {"n18c07", 5, 50, AVG, DX, 8, SLOW},
  {"n18c08", 5, 50, AVG, DX, 8, SLOW},
  {"n18c09", 5, 50, AVG, DX, 8, SLOW},
  {"n18c10", 5, 50, AVG, DX, 8, SLOW},
  {"n18c11", 5, 50, AVG, DX, 8, SLOW},
  {"n18c12", 5, 50, AVG, DX, 8, SLOW},
  {"n18c13", 5, 50, AVG, DX, 8, SLOW},
  {"n18c14", 5, 50, AVG, DX, 8, SLOW},
  {"n18c15", 5, 50, AVG, DX, 8, SLOW},
  {"n18c16", 5, 50, AVG, DX, 8, SLOW},
  {"n18c17", 5, 50, AVG, DX, 8, SLOW},
  {"n18c18", 5, 50, AVG, DX, 8, SLOW},
  {"n18c19", 5, 50, AVG, DX, 8, SLOW},
  {"n18c20", 5, 50, AVG, DX, 8, SLOW},
  {"n18c21", 5, 50, AVG, DX, 8, SLOW},
  {"n18c22", 5, 50, AVG, DX, 8, SLOW},
  {"n18c23", 5, 50, AVG, DX, 8, SLOW},
END_OF_STREAM
};

char *frameList[] = {
  //"time",
  //"time_usec",
  "n13_phase",
  "n17_phase",
  "n18_phase",
  "n19_phase",
  "n21_phase",
  "n22_phase",
  "n23_phase",
  "n25_phase",
  "n26_phase",
  "n27_phase",
  "n29_phase",
  "n30_phase",
  "n31_phase",
  "ampl_500_bias",
  "ampl_350_bias",
  "ampl_250_bias",
  "ampl_rox_bias",
  "ampl_x_bias",
  "dig21_das",
  "dig65_das",
  "ramp_ena_bias",
  "ramp_ampl_bias",
  "t_padcdc_rec",
  "t_pauram_rec",
  "t_hkdcdc_rec",
  "t_2_prime",
  "t_strut_bot",
  "t_1_prime",
  "t_1_second",
  "t_2_second",
  "t_strut_side",
  "t_push_plate",
  "t_mot_act",
  "t_rec",
  "t_das",
  "t_if_top_frnt",
  "t_if_top_back",
  "t_if_bot_frnt",
  "t_if_bot_back",
  "t17_das",
  "t18_das",
  "t19_das",
  "t20_das",
  "t21_das",
  "t22_das",
  "t23_das",
  "t24_das",
  "pin_in_lock",
  "fpulse_isc",
  "period_cal",
  "timeout",
  "az_sun",
  "lvdt_low_act",
  "cryostate",
  "t_chip_flc",
  "declination_mag",
  "veto_sensor",
  "level_on_bal",
  "level_off_bal",
  "level_target_bal",
  "alt_sip",
  "mapmean_isc",
  "pitch_dgps",
  "roll_dgps",
  "lat_sip",
  "lon_sip",
  "lat_dgps",
  "lon_dgps",
  "alt_dgps",
  "speed_dgps",
  "dir_dgps",
  "climb_dgps",
  "att_ok_dgps",
  "n_sat_dgps",
  "disk_free",
  "mode_p",
  "x_p",
  "y_p",
  "vel_az_p",
  "del_p",
  "blob_idx_isc",
  "blob00_x_isc",
  "blob00_y_isc",
  "blob00_f_isc",
  "blob00_s_isc",
  "blob01_x_isc",
  "blob01_y_isc",
  "blob01_f_isc",
  "blob01_s_isc",
  "blob02_x_isc",
  "blob02_y_isc",
  "blob02_f_isc",
  "blob02_s_isc",
  "w_p",
  "rtol_isc",
  "apert_isc",
  "maglimit_isc",
  "nrad_isc",
  "mtol_isc",
  "qtol_isc",
  "lrad_isc",
  "thresh_isc",
  "grid_isc",
  "real_trig_osc",
  "mdist_isc",
  "nblobs_isc",
  "foc_off_osc",
  "tol_isc",
  "offset_ifel_gy",
  "offset_ifroll_gy",
  "offset_ifyaw_gy",
  "sigma_mag",
  "sigma_dgps",
  "lvdt_high_act",
  "sigma_isc",
  "pulse_cal",
  "sigma_ss",
  "sigma_clin",
  "spulse_isc",
  "hx_flag_isc",
  "brra_isc",
  "brdec_isc",
  "x_off_isc",
  "gain_osc",
  "i_hold_isc",
  "save_prd_isc",
  "y_off_isc",
  "offset_isc",
  "bbc_fifo_size",
  "t_cpu_flc",
  "t_x_flc",
  "mks_hi_sip",
  "mks_med_sip",
  "blob_idx_osc",
  "blob00_x_osc",
  "blob00_y_osc",
  "blob00_f_osc",
  "blob00_s_osc",
  "blob01_x_osc",
  "blob01_y_osc",
  "blob01_f_osc",
  "blob01_s_osc",
  "blob02_x_osc",
  "blob02_y_osc",
  "blob02_f_osc",
  "blob02_s_osc",
  "mapmean_osc",
  "fpulse_osc",
  "sigma_osc",
  "tol_osc",
  "apert_osc",
  "maglimit_osc",
  "nrad_osc",
  "mtol_osc",
  "qtol_osc",
  "offset_osc",
  "lrad_osc",
  "thresh_osc",
  "grid_osc",
  "real_trig_isc",
  "foc_off_isc",
  "mdist_osc",
  "nblobs_osc",
  "rtol_osc",
  "rd_sigma_osc",
  "spulse_osc",
  "hx_flag_osc",
  "brra_osc",
  "brdec_osc",
  "x_off_osc",
  "i_hold_osc",
  "save_prd_osc",
  "y_off_osc",
  "phase_ss",
  "pref_tp_sf",
  "sun_time_ss",
  "t_cpu_ss",
  "t_hdd_ss",
  "maxblobs_isc",
  "maxblobs_osc",
  "bi0_fifo_size",
  "plover",
  "t_flange_isc",
  "t_lens_isc",
  "t_heat_isc",
  "t_comp_isc",
  "pressure1_isc",
  "t_flange_osc",
  "t_lens_osc",
  "t_heat_osc",
  "t_comp_osc",
  "pressure1_osc",
  "gain_isc",
  "jfet_set_on",
  "jfet_set_off",
  "t_case_ss",
  "cycle_state",
  "trig_type_isc",
  "exposure_isc",
  "trig_type_osc",
  "exposure_osc",
  "fieldrot_isc",
  "fieldrot_osc",
  "g_p_heat_gy",
  "g_i_heat_gy",
  "g_d_heat_gy",
  "t_set_gy",
  "h_age_gy",
  "h_hist_gy",
  "ra_1_p",
  "dec_1_p",
  "ra_2_p",
  "dec_2_p",
  "ra_3_p",
  "dec_3_p",
  "ra_4_p",
  "dec_4_p",
  "trim_clin",
  "trim_enc",
  "trim_null",
  "trim_mag",
  "trim_dgps",
  "trim_ss",
  "gain_bal",
  "h_p",
  "error_isc",
  "rd_sigma_isc",
  "mks_lo_sip",
  "error_osc",
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
  "pot_lock",
  "state_lock",
  "goal_lock",
  "seized_act",
  "x_vel_stage",
  "x_stp_stage",
  "x_str_stage",
  "y_lim_stage",
  "y_stp_stage",
  "y_str_stage",
  "x_lim_stage",
  "y_vel_stage",
  "he4_lev_old",
  "focus_isc",
  "focus_osc",
  "pitch_mag",
  "diskfree_isc",
  "diskfree_osc",
  "off_ifel_gy_isc",
  "off_ifel_gy_osc",
  "off_ifroll_gy_isc",
  "off_ifroll_gy_osc",
  "off_ifyaw_gy_isc",
  "off_ifyaw_gy_osc",
  "pos_lock",
  "pos_0_act",
  "pos_1_act",
  "pos_2_act",
  "enc_0_act",
  "enc_1_act",
  "enc_2_act",
  "goal_sf",
  "focus_sf",
  "maxslew_isc",
  "maxslew_osc",
  "t_port_ss",
  "t_star_ss",
  "v_5_ss",
  "v_12_ss",
  "v_batt_ss",
  "raw_01_ss",
  "raw_02_ss",
  "raw_03_ss",
  "raw_04_ss",
  "raw_05_ss",
  "raw_06_ss",
  "raw_07_ss",
  "raw_08_ss",
  "raw_09_ss",
  "raw_10_ss",
  "raw_11_ss",
  "raw_12_ss",
  "pref_ts_sf",
  "spread_sf",
  "acc_lock",
  "snr_ss",
  "i_move_act",
  "i_hold_act",
  "vel_act",
  "acc_act",
  "i_move_lock",
  "i_hold_lock",
  "vel_lock",
  "g_prime_sf",
  "g_second_sf",
  "step_sf",
  "wait_sf",
  "mode_sf",
  "correction_sf",
  "age_sf",
  "offset_sf",
  "t_prime_sf",
  "t_second_sf",
  "flags_act",
  "lvdt_spread_act",
  "el_sun",
  "mode_cal",
  "minblobs_isc",
  "minblobs_osc",
  "stat_1_rw",
  "stat_2_rw",
  "fault_rw",
  "stat_1_el",
  "stat_2_el",
  "fault_el",
  "stat_dr_piv",
  "stat_s1_piv",
  "t_mc_rw",
  "t_mc_el",
  "max_age_isc",
  "max_age_osc",
  "g_pe_piv",
  "g_pv_piv",
  "set_rw",
  "vel_dps_az",
  "vel_ser_piv",
  "vel_calc_piv",
  "age_isc",
  "age_osc",
  "drive_info_rw",
  "drive_err_cts_rw",
  "drive_info_el",
  "drive_err_cts_el",
  "drive_info_piv",
  "drive_err_cts_piv",
  "vel_hwpr",
  "acc_hwpr",
  "i_move_hwpr",
  "i_hold_hwpr",
  "pos_hwpr",
  "enc_hwpr",
  "mode_bal",
  "pitch_raw_dgps",
  "roll_raw_dgps",
  "verbose_rw",
  "verbose_el",
  "verbose_piv",
  "p_err_term_piv",
  "step_start_bias",
  "step_end_bias",
  "step_n_bias",
  "step_time_bias",
  "step_pul_len_bias",
  "step_array_bias",
  "step_start_phase",
  "step_end_phase",
  "step_nsteps_phase",
  "step_time_phase",
  "step_ena_bias",
  "step_ena_phase",
  "v_batt_cc",
  "v_arr_cc",
  "i_batt_cc",
  "i_arr_cc",
  "t_hs_cc",
  "fault_cc",
  "alarm_hi_cc",
  "alarm_lo_cc",
  "v_targ_cc",
  "state_cc",
  "lvdt_0_act",
  "lvdt_1_act",
  "lvdt_2_act",
  "frict_off_piv",
  "frict_term_uf_piv",
  "az_gy",
  "offset_0_act",
  "offset_1_act",
  "offset_2_act",
  "goal_0_act",
  "goal_1_act",
  "goal_2_act",
  "latch0",
  "latch1",
  "switch_gy",
  "switch_misc",
  "bus_reset_act",
  "i_trans",
  "i_das",
  "i_acs",
  "i_rec",
  "i_sc",
  "i_dgps",
  "i_el",
  "i_piv",
  "i_rw",
  "i_step",
  "i_gy",
  "i_mcc",
  "v_1_sol",
  "v_2_sol",
  "v_3_sol",
  "v_4_sol",
  "v_5_sol",
  "v_6_sol",
  "v_batt",
  "i_sol",
  "i_batt",
  "t_gy",
  "t_serial",
  "t_mc_piv",
  "t_piv",
  "t_mcc",
  "t_charger",
  "t_1_bat",
  "t_2_bat",
  "t_3_bat",
  "t_4_bat",
  "t_array",
  "t_acs",
  "t_dcdc_acs",
  "t_mc_lock",
  "t_lock",
  "t_ext9",
  "t_box_bal",
  "t_pump_bal",
  "t_el",
  "t_shade",
  "t_sun",
  "t_rw",
  "t_earth",
  "t_chin",
  "t_ext8",
  "v_pump_bal",
  "dac2_ampl",
  "mask_gy",
  "g_p_el",
  "g_i_el",
  "g_p_az",
  "g_i_az",
  "fault_gy",
  "p_term_el",
  "error_el",
  "error_az",
  "bits_bal",
  "t_pyr_clin",
  "t_if_clin",
  "ifpm_hall",
  "lvdt_65_act",
  "lvdt_63_act",
  "lvdt_64_act",
  "tr_he3_fridge",
  "tr_m5",
  "tr_m4",
  "tr_hwpr",
  "tr_horn_500",
  "tr_horn_350",
  "tr_horn_250",
  "tr_300mk_strap",
  "tr_he4_pot",
  "tr_optbox_filt",
  "he4_lev",
  "i_charcoal",
  "i_cal_lamp",
  "i_hs_pot",
  "i_hs_char",
  "td_ln",
  "i_300mk",
  "td_vcs_filt",
  "hwpr_enc_cryo",
  "i_jfet",
  "td_hs_charcoal",
  "td_lhe_filt",
  "td_lhe",
  "td_vcs_jfet",
  "td_jfet",
  "td_hs_pot",
  "td_charcoal",
  "td_ln_filt",
  "time_sip",
  "time_dgps",
  "lst",
  "ra_isc",
  "dec_isc",
  "framenum_isc",
  "lat",
  "lon",
  "state_isc",
  "mcpnum_isc",
  "ra",
  "ra_osc",
  "dec_osc",
  "framenum_osc",
  "state_osc",
  "mcpnum_osc",
  "cycle_start",
  "dec",
  "lst_sched",
  ""
};

