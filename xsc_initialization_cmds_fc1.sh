# LAT and LON of the high bay at Penn
blastcmd @192.168.1.3 pos_set 39.9526 284.8110

# xsc_is_new_window_period which valid_period_cs
blastcmd @192.168.1.3 xsc_is_new_window_period 0 1500
blastcmd @192.168.1.3 xsc_is_new_window_period 1 1500

# xsc_exposure_timing which exposure_cs grace_period_s (!!) share_delay_cs
#blastcmd @192.168.1.3 xsc_exposure_timing 0 12 60 200
#blastcmd @192.168.1.3 xsc_exposure_timing 1 12 60 200
blastcmd @192.168.1.3 xsc_exposure_timing 0 12 8 200
blastcmd @192.168.1.3 xsc_exposure_timing 1 12 8 200

# xsc_solver_general which enabled timeout_s
blastcmd @192.168.1.3 xsc_solver_general 0 1 7
blastcmd @192.168.1.3 xsc_solver_general 1 1 7

##### Triggering ######

# xsc_trigger_threshold which enabled max_streaking_px # trigger only when conditions are met
blastcmd @192.168.1.3 xsc_trigger_threshold 0 1 10
blastcmd @192.168.1.3 xsc_trigger_threshold 1 1 10

# xsc_scan_force_trigger which[doesnt apply] enabled # Force triggering at turnaround
blastcmd @192.168.1.3 xsc_scan_force_trigger 0 0

######### Solving and Filters ##########
#  xsc_filter_hor_location which enabled radius_degrees
# blastcmd @192.168.1.3 xsc_filter_hor_location 0 1 20
# blastcmd @192.168.1.3 xsc_filter_hor_location 1 1 20
blastcmd @192.168.1.3 xsc_filter_hor_location 0 0 20
blastcmd @192.168.1.3 xsc_filter_hor_location 1 0 20

#  xsc_filter_eq which enabled radius_degrees
blastcmd @192.168.1.3 xsc_filter_eq_location 0 0 10
blastcmd @192.168.1.3 xsc_filter_eq_location 1 0 10

#  xsc_filter_hor_roll which enabled min_deg max_deg
blastcmd @192.168.1.3 xsc_filter_hor_roll 0 1 43 45
blastcmd @192.168.1.3 xsc_filter_hor_roll 1 1 75 80

#  xsc_filter_el which enabled min_deg max_deg
blastcmd @192.168.1.3 xsc_filter_el 0 1 5 60
blastcmd @192.168.1.3 xsc_filter_el 1 1 5 60

# xsc_blob_finding which snr_thresh max_num_blobs robust_mode_enabled fitting_method
# blastcmd @192.168.1.3 xsc_blob_finding 0 10 15 1 0
# blastcmd @192.168.1.3 xsc_blob_finding 1 10 15 1 0
blastcmd @192.168.1.3 xsc_blob_finding 0 1 15 1 0
blastcmd @192.168.1.3 xsc_blob_finding 1 1 15 1 0

# xsc_pattern_matching which enabled display_star_name match_tol_px iplatescale_min_arcsec iplatescale_max_arcsec platescale_always_fixed platescale_fixed # platescale ~6.4"/pix
blastcmd @192.168.1.3 xsc_pattern_matching 0 1 0 5 6.5 6.8 0 0
blastcmd @192.168.1.3 xsc_pattern_matching 1 1 0 5 6.5 6.8 0 0

#  xsc_filter_matching which error_threshold_arcsec fit_error_thresh_px num_matched 
blastcmd @192.168.1.3 xsc_filter_matching 0 6 10 6
blastcmd @192.168.1.3 xsc_filter_matching 1 6 10 6


